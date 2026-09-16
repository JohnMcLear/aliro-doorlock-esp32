/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure, hardware-free helpers for the ST25R3916 SPI NFC driver.
 *
 * Everything in here is arithmetic on register values and frame bytes. It
 * lives in its own header, with no ESP-IDF dependency, so the host test in
 * firmware/tests/st25r3916_frame_logic_test.cpp can pin down the parts that
 * are easy to get subtly wrong and hard to see on a bench: the transmit
 * length register split, the 10-bit FIFO count, ATS timing fields, the
 * no-response timer range switch, and the Aliro ECP frame bytes.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace st25r3916 {

// NUM_TX_BYTES1 (0x22) holds ntx[12:5]. NUM_TX_BYTES2 (0x23) holds ntx[4:0]
// in bits 7:3 and the number of extra bits (nbtx) in bits 2:0.
//
// The split point is bit 5, not bit 8. A driver that writes (len >> 8) into
// NUM_TX_BYTES1 transmits only (len % 32) bytes for any frame of 32 bytes or
// more, which silently truncates every full-size ISO-DEP I-block.
inline void encode_num_tx(uint16_t bytes, uint8_t extra_bits, uint8_t &reg1, uint8_t &reg2)
{
    reg1 = static_cast<uint8_t>((bytes >> 5) & 0xFF);
    reg2 = static_cast<uint8_t>(((bytes & 0x1F) << 3) | (extra_bits & 0x07));
}

// FIFO_STATUS1 holds fifo_b[7:0]; FIFO_STATUS2 bits 7:6 hold fifo_b[9:8].
inline uint16_t decode_fifo_count(uint8_t status1, uint8_t status2)
{
    return static_cast<uint16_t>(status1 | ((status2 & 0xC0) << 2));
}

// Number of valid bits in the last FIFO byte (0 means a complete byte).
inline uint8_t decode_fifo_last_bits(uint8_t status2)
{
    return static_cast<uint8_t>((status2 >> 1) & 0x07);
}

// ISO/IEC 14443-4 frame waiting time in milliseconds, using the same
// arithmetic as M5Unit-NFC's isodep::fwi_to_ms so this driver keeps the
// timeouts the audited m5nfc component used. FWI 15 is RFU and yields 0.
inline uint32_t fwi_to_ms(uint8_t fwi)
{
    if (fwi >= 15) {
        return 0;
    }
    const uint8_t fwi_eff = (fwi == 0) ? 1 : fwi;
    const float base_us = (4096.0f / 13.56e6f) * 1e6f;
    const float fwt_us = base_us * static_cast<float>(1u << fwi_eff);
    const uint32_t fwt_ms = static_cast<uint32_t>(fwt_us / 1000.0f);
    return fwt_ms != 0 ? fwt_ms : 1;
}

// Start-up frame guard time after the ATS, rounded up to whole milliseconds.
// SFGT = (256 * 16 / fc) * 2^SFGI. SFGI 0 means no extra guard time and 15
// is RFU, which ISO/IEC 14443-4 says to treat as 0.
inline uint32_t sfgi_to_ms(uint8_t sfgi)
{
    if (sfgi == 0 || sfgi >= 15) {
        return 0;
    }
    const uint64_t numerator_us = 4096ull * 1000000ull * (1ull << sfgi);
    const uint64_t fc = 13560000ull;
    const uint64_t us = (numerator_us + fc - 1) / fc;
    return static_cast<uint32_t>((us + 999) / 1000);
}

struct AtsInfo {
    uint16_t fsc = 32;    // PICC max frame size, ISO default when T0 is absent
    uint8_t fwi = 4;      // ISO default when TB is absent
    uint8_t sfgi = 0;
    bool has_tb = false;
};

inline uint16_t fsci_to_fsc(uint8_t fsci)
{
    static const uint16_t kTable[9] = {16, 24, 32, 40, 48, 64, 96, 128, 256};
    return fsci < 9 ? kTable[fsci] : 256;
}

// Parse TL, T0 and the optional TA/TB/TC interface bytes of an ATS.
// Returns false if the ATS is shorter than its own TL byte claims, or claims
// interface bytes that are not there.
inline bool parse_ats(const uint8_t *ats, size_t ats_len, AtsInfo &info)
{
    info = AtsInfo{};
    if (ats == nullptr || ats_len < 1 || ats[0] < 1 || ats[0] > ats_len) {
        return false;
    }
    const size_t tl = ats[0];
    if (tl < 2) {
        return true;  // TL only: all defaults apply
    }
    const uint8_t t0 = ats[1];
    info.fsc = fsci_to_fsc(t0 & 0x0F);
    size_t offset = 2;
    if (t0 & 0x10) {  // TA present
        if (offset >= tl) {
            return false;
        }
        ++offset;
    }
    if (t0 & 0x20) {  // TB present: FWI in the high nibble, SFGI in the low
        if (offset >= tl) {
            return false;
        }
        info.has_tb = true;
        info.fwi = static_cast<uint8_t>(ats[offset] >> 4);
        info.sfgi = static_cast<uint8_t>(ats[offset] & 0x0F);
        ++offset;
    }
    if (t0 & 0x40) {  // TC present
        if (offset >= tl) {
            return false;
        }
        ++offset;
    }
    return true;
}

// RATS parameter byte: FSDI in the high nibble, CID in the low.
inline uint8_t rats_param(uint8_t fsdi, uint8_t cid)
{
    return static_cast<uint8_t>(((fsdi & 0x0F) << 4) | (cid & 0x0F));
}

// No-response timer. One step is 64/fc (about 4.72 us) or, with nrt_step set,
// 4096/fc (about 302 us). The register is 16 bits. Rounds up so the timer is
// never shorter than asked for; saturates at the 4096/fc maximum (~19.8 s).
inline void nrt_from_ms(uint32_t ms, bool &step_4096fc, uint16_t &value)
{
    const uint64_t fc = 13560000ull;
    uint64_t steps = (static_cast<uint64_t>(ms) * fc + (64000ull - 1)) / 64000ull;
    if (steps <= 0xFFFFull) {
        step_4096fc = false;
        value = static_cast<uint16_t>(steps == 0 ? 1 : steps);
        return;
    }
    steps = (static_cast<uint64_t>(ms) * fc + (4096000ull - 1)) / 4096000ull;
    step_4096fc = true;
    value = static_cast<uint16_t>(steps > 0xFFFFull ? 0xFFFF : steps);
}

// Aliro Enhanced Contactless Polling frame, the 16 bytes before CRC-A.
// Byte-for-byte the same as patch 0005 for the m5nfc component: the fixed
// 6A 02 CB 02 06 20 42 20 prefix followed by GroupIdentifier[0..7].
constexpr size_t kEcpIdentifierSize = 8;
constexpr size_t kEcpFrameSize = 16;
constexpr uint8_t kAliroEcpPrefix[8] = {0x6A, 0x02, 0xCB, 0x02, 0x06, 0x20, 0x42, 0x20};

inline void build_ecp_frame(const uint8_t *identifier, uint8_t out[kEcpFrameSize])
{
    std::memcpy(out, kAliroEcpPrefix, sizeof(kAliroEcpPrefix));
    if (identifier != nullptr) {
        std::memcpy(out + sizeof(kAliroEcpPrefix), identifier, kEcpIdentifierSize);
    } else {
        std::memset(out + sizeof(kAliroEcpPrefix), 0, kEcpIdentifierSize);
    }
}

// Anticollision with a split byte. The reader transmits `valid_bits` bits of
// the byte at the split position; the ST25R3916 writes the PICC's answer into
// the FIFO starting at that same byte, with the bits the reader sent cleared.
// Recombine the reader's low bits with the PICC's high bits.
inline uint8_t merge_split_byte(uint8_t received, uint8_t sent, uint8_t valid_bits)
{
    if (valid_bits == 0) {
        return received;
    }
    const uint8_t low_mask = static_cast<uint8_t>((1u << valid_bits) - 1u);
    return static_cast<uint8_t>((sent & low_mask) | (received & static_cast<uint8_t>(~low_mask)));
}

inline uint8_t bcc(const uint8_t *data, size_t len)
{
    uint8_t value = 0;
    for (size_t i = 0; i < len; ++i) {
        value ^= data[i];
    }
    return value;
}

// SEL_PAR (NVB) for an anticollision frame of `bytes` whole bytes plus `bits`.
inline uint8_t sel_par(uint8_t bytes, uint8_t bits)
{
    return static_cast<uint8_t>(((bytes << 4) & 0xF0) | (bits & 0x0F));
}

// The next anticollision prefix after a collision.
//
// COLLISION_STATUS (0x20) holds c_byte in bits 7:4 and c_bit in bits 3:1. The
// position is absolute within the anticollision frame, which starts with SEL
// and NVB, so c_byte 2..5 are the four UID bytes of this cascade level and
// c_byte 6 is the BCC. Bits below the collision were received before it and
// are valid; bits at and above it are not, so this clears them rather than
// leaving whatever the previous round put in the buffer. The colliding bit is
// then driven to 1, which is the branch rfalNfcaPollerSingleCollisionResolution
// takes.
//
// A collision in the BCC means all four UID bytes are already known, so the
// next round sends them whole and no bit is forced.
struct AnticollisionPrefix {
    bool valid = false;
    bool forced = false;       // a colliding bit was driven to 1
    uint8_t forced_byte = 0;   // frame byte holding that bit
    uint8_t forced_bit = 0;    // bit index within it
    uint8_t bytes = 0;  // whole bytes of the next frame, SEL and NVB included
    uint8_t bits = 0;   // valid bits of the trailing partial byte, 0 for none
};

inline AnticollisionPrefix next_anticollision_prefix(uint8_t collision_status, uint8_t *frame)
{
    AnticollisionPrefix next;
    const uint8_t c_byte = static_cast<uint8_t>((collision_status >> 4) & 0x0F);
    const uint8_t c_bit = static_cast<uint8_t>((collision_status >> 1) & 0x07);
    if (c_byte < 2 || c_byte > 6) {
        return next;  // SEL/NVB is our own transmission; past the BCC is off-frame
    }
    next.valid = true;
    if (c_byte == 6) {
        next.bytes = 6;
        next.bits = 0;
        return next;
    }
    const uint8_t known = static_cast<uint8_t>((1u << c_bit) - 1u);
    frame[c_byte] = static_cast<uint8_t>((frame[c_byte] & known) | (1u << c_bit));
    next.forced = true;
    next.forced_byte = c_byte;
    next.forced_bit = c_bit;
    if (c_bit == 7) {
        next.bytes = static_cast<uint8_t>(c_byte + 1);
        next.bits = 0;
    } else {
        next.bytes = c_byte;
        next.bits = static_cast<uint8_t>(c_bit + 1);
    }
    return next;
}

// ---------------------------------------------------------------------------
// Multi-tag enumeration: the full ISO/IEC 14443-3 anticollision tree walk.
//
// next_anticollision_prefix() alone is rfalNfcaPollerSingleCollisionResolution:
// at a collision it takes the branch with a 1 and never comes back for the 0.
// With two credentials on the reader that resolves the same PICC on every poll
// and the other one is invisible, which on a door is a card that never works
// rather than a card that works intermittently.
//
// The walk below is the standard depth-first search. Every collision has two
// branches; we follow the 1 and push the 0 on a stack. When a PICC has been
// fully selected we HLTA it (so it leaves the ACTIVE state, where it would
// ignore the next WUPA) and pop a branch. Each round starts with WUPA, not
// REQA: a PICC that was halted, or that dropped out of the previous round
// because the prefix did not match it, answers WUPA only (ISO/IEC 14443-3 6.3,
// and Mifare Classic parks itself in HALT after a SELECT that is not for it).
// Waking everybody again is harmless because the saved prefix excludes the
// PICCs already found.
//
// A branch pushed at cascade level 2 or 3 also records the UID parts of the
// levels above it, because the reader has to re-SELECT its way back down to
// that level before the saved prefix means anything. That is the bug listed in
// the Yondoor driver's notes as "CL1 collision state must be saved before CL2":
// here each stack entry carries its own path, so CL2 cannot clobber CL1.
//
// Ported from John McLear's ESPHome ST25R3916 driver
// (esphome_st25r_yondoor/components/st25r/st25r.cpp) and its
// memory/multitag_anticol.md, with the brute-force prefix counter replaced by
// an explicit branch stack.
// ---------------------------------------------------------------------------

constexpr uint8_t kMaxUidBytes = 10;
constexpr uint8_t kCascadeLevels = 3;
constexpr uint8_t kAnticollisionFrameSize = 7;  // SEL, NVB, 4 UID bytes, BCC
constexpr uint8_t kMaxPendingBranches = 8;
constexpr uint8_t kMaxEnumerationRounds = 12;
constexpr uint8_t kSelectCommand[kCascadeLevels] = {0x93, 0x95, 0x97};
constexpr uint8_t kSakCascadeBit = 0x04;     // UID is not complete
constexpr uint8_t kSakIso144434Bit = 0x20;   // PICC supports ISO/IEC 14443-4
constexpr uint8_t kCascadeTag = 0x88;
constexpr uint8_t kNvbComplete = 0x70;       // SELECT: 7 whole bytes

struct EnumeratedTag {
    uint8_t uid[kMaxUidBytes] = {};
    uint8_t uid_len = 0;
    uint8_t sak = 0;
};

inline bool tag_is_iso_dep(const EnumeratedTag &tag)
{
    return (tag.sak & kSakIso144434Bit) != 0;
}

enum class AnticolOutcome : uint8_t {
    kResolved,   // a single PICC answered; frame now holds 4 UID bytes and BCC
    kCollision,  // two or more PICCs answered
    kFailed,     // timeout, framing error, or a length that cannot be a UID
};

struct AnticolReply {
    AnticolOutcome outcome = AnticolOutcome::kFailed;
    uint8_t rx_len = 0;            // bytes the PICC put at frame + bytes
    uint8_t collision_status = 0;  // COLLISION_STATUS, only for kCollision
};

// One anticollision branch that has not been explored yet.
struct SearchBranch {
    uint8_t level = 0;                            // cascade level of the branch
    uint8_t path[kCascadeLevels - 1][4] = {};     // UID parts of the levels above
    uint8_t frame[kAnticollisionFrameSize] = {};  // partial frame at `level`
    uint8_t bytes = 2;
    uint8_t bits = 0;
};

struct EnumerationStats {
    uint8_t rounds = 0;
    uint8_t collisions = 0;
    uint8_t branches_pushed = 0;
    bool hit_tag_cap = false;
    bool hit_branch_cap = false;
    bool hit_round_cap = false;
};

// Rebuild the SELECT frames that walk down to a UID already known. Returns the
// number of cascade levels, or 0 for a length that is not 4, 7 or 10.
inline uint8_t build_select_frames(const uint8_t *uid, uint8_t uid_len,
                                   uint8_t frames[kCascadeLevels][kAnticollisionFrameSize])
{
    uint8_t levels = 0;
    if (uid_len == 4) {
        levels = 1;
    } else if (uid_len == 7) {
        levels = 2;
    } else if (uid_len == 10) {
        levels = 3;
    } else {
        return 0;
    }
    uint8_t consumed = 0;
    for (uint8_t level = 0; level < levels; ++level) {
        uint8_t part[4] = {};
        if (level + 1 < levels) {
            part[0] = kCascadeTag;
            std::memcpy(part + 1, uid + consumed, 3);
            consumed = static_cast<uint8_t>(consumed + 3);
        } else {
            std::memcpy(part, uid + consumed, 4);
            consumed = static_cast<uint8_t>(consumed + 4);
        }
        frames[level][0] = kSelectCommand[level];
        frames[level][1] = kNvbComplete;
        std::memcpy(frames[level] + 2, part, 4);
        frames[level][6] = bcc(part, 4);
    }
    return levels;
}

// The cascade tag is not part of the UID: each incomplete level contributes its
// last three bytes, the final level all four.
inline uint8_t assemble_uid(const uint8_t parts[kCascadeLevels][4], uint8_t levels, uint8_t *uid)
{
    uint8_t len = 0;
    for (uint8_t level = 0; level < levels; ++level) {
        if (level + 1 < levels) {
            std::memcpy(uid + len, parts[level] + 1, 3);
            len = static_cast<uint8_t>(len + 3);
        } else {
            std::memcpy(uid + len, parts[level], 4);
            len = static_cast<uint8_t>(len + 4);
        }
    }
    return len;
}

// Anticollision at one cascade level, starting from the prefix in
// frame/bytes/bits. Every collision on the way pushes its 0 branch.
//
// `Ops` is the hardware seam, so the walk can be tested on the host:
//   bool       wupa();
//   void       begin_anticollision(uint8_t level);
//   void       end_anticollision(uint8_t level);
//   AnticolReply anticollision(uint8_t level, uint8_t *frame, uint8_t bytes, uint8_t bits);
//   bool       select(uint8_t level, uint8_t *frame, uint8_t &sak);
//   void       halt();
template <typename Ops>
bool resolve_anticollision_level(Ops &ops, uint8_t level, uint8_t *frame, uint8_t &bytes, uint8_t &bits,
                                 const uint8_t path[kCascadeLevels][4], SearchBranch *pending,
                                 uint8_t &pending_count, EnumerationStats &stats)
{
    ops.begin_anticollision(level);
    bool resolved = false;
    uint8_t previous_bytes = 0;
    uint8_t previous_bits = 0;
    for (int iteration = 0; iteration < 32; ++iteration) {
        if (iteration > 0 && bytes == previous_bytes && bits == previous_bits) {
            // The prefix did not grow, so the same frame would go out again.
            // Only a BCC collision does this, and it cannot be split: two PICCs
            // are claiming the same four UID bytes. Give the subtree up rather
            // than spending 32 anticollision timeouts on it.
            break;
        }
        previous_bytes = bytes;
        previous_bits = bits;
        frame[0] = kSelectCommand[level];
        frame[1] = sel_par(bytes, bits);
        // The FIFO returns the PICC's answer with the bits we transmitted
        // cleared, so keep our own copy of the split byte to OR back in.
        const uint8_t sent_split = bits ? frame[bytes] : 0;

        const AnticolReply reply = ops.anticollision(level, frame, bytes, bits);
        if (bits) {
            frame[bytes] = merge_split_byte(frame[bytes], sent_split, bits);
        }

        if (reply.outcome == AnticolOutcome::kCollision) {
            const AnticollisionPrefix next = next_anticollision_prefix(reply.collision_status, frame);
            if (!next.valid) {
                break;
            }
            ++stats.collisions;
            if (next.forced) {
                // The 1 branch is the one we follow now; remember the 0 branch,
                // with the frame truncated so nothing above the prefix leaks in.
                if (pending_count < kMaxPendingBranches) {
                    SearchBranch &branch = pending[pending_count];
                    branch = SearchBranch{};
                    branch.level = level;
                    for (uint8_t i = 0; i + 1 < kCascadeLevels; ++i) {
                        std::memcpy(branch.path[i], path[i], 4);
                    }
                    std::memcpy(branch.frame, frame, sizeof(branch.frame));
                    branch.frame[next.forced_byte] =
                        static_cast<uint8_t>(branch.frame[next.forced_byte] & ~(1u << next.forced_bit));
                    for (uint8_t i = static_cast<uint8_t>(next.bits ? next.bytes + 1 : next.bytes);
                         i < kAnticollisionFrameSize; ++i) {
                        branch.frame[i] = 0;
                    }
                    branch.bytes = next.bytes;
                    branch.bits = next.bits;
                    ++pending_count;
                    ++stats.branches_pushed;
                } else {
                    stats.hit_branch_cap = true;
                }
            }
            bytes = next.bytes;
            bits = next.bits;
            continue;
        }
        if (reply.outcome != AnticolOutcome::kResolved || bytes + reply.rx_len != kAnticollisionFrameSize) {
            break;
        }
        resolved = true;
        break;
    }
    ops.end_anticollision(level);
    return resolved && bcc(frame + 2, 4) == frame[6];
}

// Enumerate every PICC in the field. The caller has already polled, so the
// first round runs on that poll and a field with a single PICC costs exactly
// the frames it cost before: WUPA, anticollision, SELECT, and nothing else.
// Returns the number of tags written to `tags`; the last one is left selected.
template <typename Ops>
uint8_t enumerate_tags(Ops &ops, EnumeratedTag *tags, uint8_t capacity, EnumerationStats &stats)
{
    SearchBranch pending[kMaxPendingBranches];
    uint8_t pending_count = 0;
    uint8_t count = 0;

    SearchBranch branch;  // the root: cascade level 1, no prefix
    bool need_poll = false;

    while (count < capacity) {
        if (stats.rounds >= kMaxEnumerationRounds) {
            stats.hit_round_cap = true;
            break;
        }
        ++stats.rounds;
        if (need_poll && !ops.wupa()) {
            break;
        }
        need_poll = true;

        uint8_t parts[kCascadeLevels][4] = {};
        bool ok = true;
        // Walk back down to the branch's cascade level. Those levels are fully
        // known, so they need a SELECT and no anticollision.
        for (uint8_t level = 0; level < branch.level && ok; ++level) {
            uint8_t frame[kAnticollisionFrameSize] = {};
            frame[0] = kSelectCommand[level];
            frame[1] = kNvbComplete;
            std::memcpy(frame + 2, branch.path[level], 4);
            frame[6] = bcc(branch.path[level], 4);
            uint8_t sak = 0;
            ok = ops.select(level, frame, sak) && (sak & kSakCascadeBit) != 0;
            std::memcpy(parts[level], branch.path[level], 4);
        }

        uint8_t frame[kAnticollisionFrameSize] = {};
        std::memcpy(frame, branch.frame, sizeof(frame));
        uint8_t bytes = branch.bytes;
        uint8_t bits = branch.bits;
        uint8_t level = branch.level;
        uint8_t sak = 0;
        bool complete = false;

        while (ok && level < kCascadeLevels) {
            if (!resolve_anticollision_level(ops, level, frame, bytes, bits, parts, pending, pending_count, stats)) {
                ok = false;
                break;
            }
            frame[1] = kNvbComplete;
            if (!ops.select(level, frame, sak)) {
                ok = false;
                break;
            }
            std::memcpy(parts[level], frame + 2, 4);
            if ((sak & kSakCascadeBit) == 0) {
                complete = true;
                break;
            }
            ++level;
            std::memset(frame, 0, sizeof(frame));
            bytes = 2;
            bits = 0;
        }

        if (complete) {
            EnumeratedTag &tag = tags[count];
            tag = EnumeratedTag{};
            tag.uid_len = assemble_uid(parts, static_cast<uint8_t>(level + 1), tag.uid);
            tag.sak = sak;
            ++count;
        }

        if (pending_count == 0) {
            break;
        }
        if (count >= capacity) {
            stats.hit_tag_cap = true;
            break;
        }
        // A PICC that has been selected sits in ACTIVE and ignores WUPA. HLTA
        // parks it in HALT, where the next round's WUPA wakes it again; it then
        // drops out on the first prefix byte that is not its own. HLTA to a
        // PICC that is only part-way through its cascade is an error frame it
        // answers by returning to IDLE, which WUPA also wakes, so this is safe
        // whether or not the round got as far as a SELECT.
        ops.halt();
        branch = pending[--pending_count];
    }
    return count;
}

} // namespace st25r3916
