// Host test for firmware/components/st25r3916_spi/src/st25r3916_frame_logic.h.
//
//   c++ -std=gnu++17 -Wall -Wextra -Werror -I firmware/components/st25r3916_spi/src
//       firmware/tests/st25r3916_frame_logic_test.cpp -o /tmp/frame_logic_test
//   /tmp/frame_logic_test
//
// scripts/build_release.sh runs it for variants that use the SPI driver.

#include "st25r3916_frame_logic.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace st25r3916;

namespace {

int g_failures = 0;

#define EXPECT_EQ(actual, expected)                                                                   \
    do {                                                                                              \
        const long long a_ = static_cast<long long>(actual);                                          \
        const long long e_ = static_cast<long long>(expected);                                        \
        if (a_ != e_) {                                                                               \
            std::printf("FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, #actual, a_, e_); \
            ++g_failures;                                                                             \
        }                                                                                             \
    } while (0)

void test_num_tx_bytes_split_at_bit_5()
{
    uint8_t r1 = 0;
    uint8_t r2 = 0;
    encode_num_tx(2, 0, r1, r2);
    EXPECT_EQ(r1, 0x00);
    EXPECT_EQ(r2, 0x10);

    // 16-byte Aliro ECP frame
    encode_num_tx(16, 0, r1, r2);
    EXPECT_EQ(r1, 0x00);
    EXPECT_EQ(r2, 0x80);

    // 32 bytes is where a (len >> 8) split breaks: it would send 0 bytes.
    encode_num_tx(32, 0, r1, r2);
    EXPECT_EQ(r1, 0x01);
    EXPECT_EQ(r2, 0x00);

    // A full FSC-256 I-block without CRC: 254 bytes.
    encode_num_tx(254, 0, r1, r2);
    EXPECT_EQ((r1 << 5) | (r2 >> 3), 254);

    // Anticollision: SEL, NVB and 1 whole UID byte plus 3 bits.
    encode_num_tx(3, 3, r1, r2);
    EXPECT_EQ(r1, 0x00);
    EXPECT_EQ(r2, (3 << 3) | 3);

    // Largest FIFO load.
    encode_num_tx(512, 0, r1, r2);
    EXPECT_EQ((r1 << 5) | (r2 >> 3), 512);
}

void test_fifo_count_uses_high_bits()
{
    EXPECT_EQ(decode_fifo_count(0x05, 0x00), 5);
    EXPECT_EQ(decode_fifo_count(0x02, 0x40), 258);  // FSD 256 frame plus CRC
    EXPECT_EQ(decode_fifo_count(0xFF, 0xC0), 1023);
    EXPECT_EQ(decode_fifo_last_bits(0x0A), 5);
    EXPECT_EQ(decode_fifo_last_bits(0x01), 0);
}

void test_fwi_matches_m5nfc()
{
    // m5 isodep::fwi_to_ms truncates (4096/fc * 2^fwi) to whole ms.
    EXPECT_EQ(fwi_to_ms(0), 0 + 1);   // FWI 0 treated as 1: 0.604 ms -> 1 ms floor
    EXPECT_EQ(fwi_to_ms(4), 4);       // 4.83 ms
    EXPECT_EQ(fwi_to_ms(8), 77);      // 77.3 ms
    EXPECT_EQ(fwi_to_ms(14), 4949);   // 4949 ms
    EXPECT_EQ(fwi_to_ms(15), 0);
}

void test_sfgt_rounds_up()
{
    EXPECT_EQ(sfgi_to_ms(0), 0);
    EXPECT_EQ(sfgi_to_ms(1), 1);    // 0.604 ms
    EXPECT_EQ(sfgi_to_ms(4), 5);    // 4.83 ms
    EXPECT_EQ(sfgi_to_ms(15), 0);   // RFU
}

void test_parse_ats()
{
    AtsInfo info;
    // iPhone-style ATS: TL=6, T0=0x78 (TA, TB, TC present, FSCI 8), TA, TB=0x81, TC, one historical byte
    const uint8_t iphone[] = {0x06, 0x78, 0x77, 0x81, 0x02, 0x80};
    EXPECT_EQ(parse_ats(iphone, sizeof(iphone), info), true);
    EXPECT_EQ(info.fsc, 256);
    EXPECT_EQ(info.has_tb, true);
    EXPECT_EQ(info.fwi, 8);
    EXPECT_EQ(info.sfgi, 1);

    // TL only
    const uint8_t minimal[] = {0x01};
    EXPECT_EQ(parse_ats(minimal, sizeof(minimal), info), true);
    EXPECT_EQ(info.fsc, 32);
    EXPECT_EQ(info.fwi, 4);

    // T0 without TB: FWI default stays 4
    const uint8_t no_tb[] = {0x03, 0x15, 0x77};
    EXPECT_EQ(parse_ats(no_tb, sizeof(no_tb), info), true);
    EXPECT_EQ(info.fsc, 64);
    EXPECT_EQ(info.has_tb, false);
    EXPECT_EQ(info.fwi, 4);

    // T0 claims TB, but TL ends first
    const uint8_t truncated[] = {0x02, 0x20};
    EXPECT_EQ(parse_ats(truncated, sizeof(truncated), info), false);

    // TL longer than the bytes received
    const uint8_t short_read[] = {0x05, 0x78};
    EXPECT_EQ(parse_ats(short_read, sizeof(short_read), info), false);
}

void test_rats_param()
{
    EXPECT_EQ(rats_param(8, 0), 0x80);  // m5nfc: FSDI 8, CID 0
    EXPECT_EQ(rats_param(5, 1), 0x51);
}

void test_nrt_range_switch()
{
    bool step = true;
    uint16_t value = 0;
    nrt_from_ms(12, step, value);  // ECP window
    EXPECT_EQ(step, false);
    EXPECT_EQ(value, 2543);        // ceil(12 ms / 4.7198 us)

    nrt_from_ms(309, step, value);  // largest in 64/fc steps
    EXPECT_EQ(step, false);

    nrt_from_ms(310, step, value);
    EXPECT_EQ(step, true);

    nrt_from_ms(5000, step, value);  // WTX ceiling
    EXPECT_EQ(step, true);
    EXPECT_EQ(value, 16553);          // ceil(5000 ms / 302.06 us)

    nrt_from_ms(60000, step, value);  // saturates
    EXPECT_EQ(step, true);
    EXPECT_EQ(value, 0xFFFF);
}

void test_ecp_frame_bytes()
{
    const uint8_t group_id[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    const uint8_t expected[16] = {0x6A, 0x02, 0xCB, 0x02, 0x06, 0x20, 0x42, 0x20,
                                  0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint8_t frame[16] = {};
    build_ecp_frame(group_id, frame);
    for (int i = 0; i < 16; ++i) {
        EXPECT_EQ(frame[i], expected[i]);
    }
    build_ecp_frame(nullptr, frame);
    for (int i = 8; i < 16; ++i) {
        EXPECT_EQ(frame[i], 0);
    }
}

void test_anticollision_helpers()
{
    // Reader sent the low 3 bits (101b); the PICC's answer fills the rest.
    EXPECT_EQ(merge_split_byte(0xA8, 0x05, 3), 0xAD);
    EXPECT_EQ(merge_split_byte(0x42, 0xFF, 0), 0x42);
    EXPECT_EQ(sel_par(2, 0), 0x20);
    EXPECT_EQ(sel_par(7, 0), 0x70);
    EXPECT_EQ(sel_par(4, 5), 0x45);
    const uint8_t uid[4] = {0x82, 0x81, 0xA4, 0x41};
    EXPECT_EQ(bcc(uid, 4), 0x82 ^ 0x81 ^ 0xA4 ^ 0x41);
}

void test_next_anticollision_prefix()
{
    // The case seen on the STEVAL bench with two NTAGs in the field:
    // COLLISION_STATUS 0x40 is frame byte 4 (the second UID byte of CL1),
    // bit 0. The next frame carries SEL, NVB and two whole UID bytes plus one
    // bit, and that bit is driven to 1.
    uint8_t frame[7] = {0x93, 0x20, 0x88, 0x04, 0x00, 0x00, 0x00};
    AnticollisionPrefix next = next_anticollision_prefix(0x40, frame);
    EXPECT_EQ(next.valid, 1);
    EXPECT_EQ(next.bytes, 4);
    EXPECT_EQ(next.bits, 1);
    EXPECT_EQ(frame[4], 0x01);
    EXPECT_EQ(sel_par(next.bytes, next.bits), 0x41);

    // Bits at and above the collision are not valid data. Whatever the
    // previous round left in the buffer must not survive into the prefix, or
    // the reader transmits UID bits no PICC ever sent.
    uint8_t stale[7] = {0x93, 0x20, 0x88, 0x04, 0xFF, 0xFF, 0xFF};
    next = next_anticollision_prefix(0x44, stale);  // byte 4, bit 2
    EXPECT_EQ(next.valid, 1);
    EXPECT_EQ(next.bytes, 4);
    EXPECT_EQ(next.bits, 3);
    EXPECT_EQ(stale[4], 0x07);  // bits 0 and 1 kept, bit 2 forced to 1

    // A collision in the top bit of a byte rolls the prefix on to whole bytes.
    uint8_t top[7] = {0x93, 0x20, 0x11, 0x00, 0x00, 0x00, 0x00};
    next = next_anticollision_prefix(0x2E, top);  // byte 2, bit 7
    EXPECT_EQ(next.valid, 1);
    EXPECT_EQ(next.bytes, 3);
    EXPECT_EQ(next.bits, 0);
    EXPECT_EQ(top[2], 0x91);
    EXPECT_EQ(sel_par(next.bytes, next.bits), 0x30);

    // A collision in the BCC means all four UID bytes are already known.
    uint8_t in_bcc[7] = {0x93, 0x20, 0x88, 0x04, 0x1A, 0xA7, 0x00};
    next = next_anticollision_prefix(0x60, in_bcc);
    EXPECT_EQ(next.valid, 1);
    EXPECT_EQ(next.bytes, 6);
    EXPECT_EQ(next.bits, 0);
    EXPECT_EQ(in_bcc[6], 0x00);  // the BCC byte is left alone
    EXPECT_EQ(sel_par(next.bytes, next.bits), 0x60);

    // SEL and NVB are our own transmission, and there is nothing past the BCC.
    uint8_t any[7] = {0x93, 0x20, 0, 0, 0, 0, 0};
    EXPECT_EQ(next_anticollision_prefix(0x10, any).valid, 0);
    EXPECT_EQ(next_anticollision_prefix(0x70, any).valid, 0);
    EXPECT_EQ(next_anticollision_prefix(0xF0, any).valid, 0);
}

// ---------------------------------------------------------------------------
// Multi-tag enumeration
//
// The tests below drive enumerate_tags() against a simulated population of
// PICCs that behaves the way the bench does:
//
//  - the FIFO answer starts at the frame byte the prefix ended in, with the
//    bits the reader transmitted cleared, so the split byte has to be merged
//    back (Yondoor bug 1);
//  - COLLISION_STATUS is an absolute frame position, byte 2 being the first UID
//    byte, so no offset correction is needed here (Yondoor bug 2: the ESPHome
//    driver subtracts 16 bits because its buffer is UID-relative, this one's
//    is not);
//  - bits at and above the collision come back as garbage, so the walk must
//    keep only the bits below it;
//  - a PICC that does not match an anticollision prefix, or a SELECT, parks
//    itself in HALT and answers WUPA only, never REQA (Yondoor bug 3);
//  - a PICC in ACTIVE ignores WUPA until it has been halted.
// ---------------------------------------------------------------------------

struct SimTag {
    uint8_t uid[10] = {};
    uint8_t uid_len = 0;
    uint8_t sak = 0;
    uint8_t bad_bcc = 0;  // XOR into the BCC of the last cascade level
    enum State : uint8_t { kIdle, kReady, kActive, kHalt } state = kIdle;
    uint8_t level = 0;

    uint8_t levels() const { return uid_len == 4 ? 1 : (uid_len == 7 ? 2 : 3); }

    // The 4 UID bytes plus BCC this PICC answers with at cascade level `l`.
    void part(uint8_t l, uint8_t out[5]) const
    {
        const uint8_t n = levels();
        if (l + 1 < n) {
            out[0] = 0x88;
            out[1] = uid[l * 3 + 0];
            out[2] = uid[l * 3 + 1];
            out[3] = uid[l * 3 + 2];
        } else {
            const uint8_t base = static_cast<uint8_t>(l * 3);
            out[0] = uid[base + 0];
            out[1] = uid[base + 1];
            out[2] = uid[base + 2];
            out[3] = uid[base + 3];
        }
        out[4] = static_cast<uint8_t>(out[0] ^ out[1] ^ out[2] ^ out[3]);
        if (l + 1 == n) {
            out[4] = static_cast<uint8_t>(out[4] ^ bad_bcc);
        }
    }
};

struct SimField {
    SimTag tags[8];
    uint8_t tag_count = 0;
    unsigned wupa_calls = 0;
    unsigned anticol_calls = 0;
    unsigned select_calls = 0;
    unsigned halt_calls = 0;

    void add(const char *uid_hex, uint8_t sak, uint8_t bad_bcc = 0)
    {
        SimTag &t = tags[tag_count++];
        uint8_t len = 0;
        for (const char *p = uid_hex; p[0] && p[1]; p += 2) {
            unsigned byte = 0;
            std::sscanf(p, "%2x", &byte);
            t.uid[len++] = static_cast<uint8_t>(byte);
        }
        t.uid_len = len;
        t.sak = sak;
        t.bad_bcc = bad_bcc;
    }

    bool wupa()
    {
        ++wupa_calls;
        bool any = false;
        for (uint8_t i = 0; i < tag_count; ++i) {
            SimTag &t = tags[i];
            if (t.state == SimTag::kIdle || t.state == SimTag::kHalt || t.state == SimTag::kReady) {
                t.state = SimTag::kReady;
                t.level = 0;
                any = true;
            }
        }
        return any;
    }

    void begin_anticollision(uint8_t) {}
    void end_anticollision(uint8_t) {}

    static bool matches(const uint8_t part[5], const uint8_t *frame, uint8_t bytes, uint8_t bits)
    {
        for (uint8_t i = 2; i < bytes; ++i) {
            if (part[i - 2] != frame[i]) {
                return false;
            }
        }
        if (bits != 0) {
            const uint8_t mask = static_cast<uint8_t>((1u << bits) - 1u);
            if ((part[bytes - 2] & mask) != (frame[bytes] & mask)) {
                return false;
            }
        }
        return true;
    }

    AnticolReply anticollision(uint8_t level, uint8_t *frame, uint8_t bytes, uint8_t bits)
    {
        ++anticol_calls;
        AnticolReply reply;
        uint8_t parts[8][5] = {};
        uint8_t responders = 0;
        for (uint8_t i = 0; i < tag_count; ++i) {
            SimTag &t = tags[i];
            if (t.state != SimTag::kReady || t.level != level) {
                continue;
            }
            uint8_t part[5] = {};
            t.part(level, part);
            if (!matches(part, frame, bytes, bits)) {
                t.state = SimTag::kHalt;  // Mifare Classic parks in HALT, not IDLE
                continue;
            }
            std::memcpy(parts[responders++], part, 5);
        }
        if (responders == 0) {
            reply.outcome = AnticolOutcome::kFailed;
            return reply;
        }

        // First bit position, LSB first within a byte, where the answers differ.
        int c_byte = -1;
        int c_bit = -1;
        for (uint8_t b = bytes; b < 7 && c_byte < 0; ++b) {
            for (uint8_t bit = (b == bytes ? bits : 0); bit < 8; ++bit) {
                const uint8_t value = static_cast<uint8_t>((parts[0][b - 2] >> bit) & 1);
                for (uint8_t r = 1; r < responders; ++r) {
                    if (((parts[r][b - 2] >> bit) & 1) != value) {
                        c_byte = b;
                        c_bit = bit;
                        break;
                    }
                }
                if (c_byte >= 0) {
                    break;
                }
            }
        }

        if (c_byte < 0) {
            for (uint8_t i = bytes; i < 7; ++i) {
                frame[i] = parts[0][i - 2];
            }
            if (bits != 0) {
                frame[bytes] = static_cast<uint8_t>(frame[bytes] & ~((1u << bits) - 1u));
            }
            reply.outcome = AnticolOutcome::kResolved;
            reply.rx_len = static_cast<uint8_t>(7 - bytes);
            return reply;
        }

        // Garbage at and above the collision, the agreed bits below it, and the
        // bits we transmitted ourselves cleared.
        for (uint8_t i = bytes; i < 7; ++i) {
            frame[i] = 0xFF;
        }
        for (uint8_t b = bytes; b <= static_cast<uint8_t>(c_byte); ++b) {
            const uint8_t first = (b == bytes) ? bits : 0;
            const uint8_t last = (b == c_byte) ? static_cast<uint8_t>(c_bit) : 8;
            for (uint8_t bit = first; bit < last; ++bit) {
                const uint8_t value = static_cast<uint8_t>((parts[0][b - 2] >> bit) & 1);
                frame[b] = static_cast<uint8_t>((frame[b] & ~(1u << bit)) | (value << bit));
            }
        }
        if (bits != 0) {
            frame[bytes] = static_cast<uint8_t>(frame[bytes] & ~((1u << bits) - 1u));
        }
        reply.outcome = AnticolOutcome::kCollision;
        reply.collision_status = static_cast<uint8_t>((c_byte << 4) | (c_bit << 1));
        return reply;
    }

    bool select(uint8_t level, uint8_t *frame, uint8_t &sak)
    {
        ++select_calls;
        bool matched = false;
        for (uint8_t i = 0; i < tag_count; ++i) {
            SimTag &t = tags[i];
            if (t.state != SimTag::kReady || t.level != level) {
                continue;
            }
            uint8_t part[5] = {};
            t.part(level, part);
            if (std::memcmp(part, frame + 2, 5) != 0) {
                t.state = SimTag::kHalt;
                continue;
            }
            if (level + 1 < t.levels()) {
                t.level = static_cast<uint8_t>(level + 1);
                sak = 0x04;
            } else {
                t.state = SimTag::kActive;
                sak = t.sak;
            }
            matched = true;
        }
        return matched;
    }

    void halt()
    {
        ++halt_calls;
        for (uint8_t i = 0; i < tag_count; ++i) {
            if (tags[i].state != SimTag::kIdle) {
                tags[i].state = SimTag::kHalt;
            }
        }
    }
};

bool uid_is(const EnumeratedTag &tag, const char *hex)
{
    char text[2 * 10 + 1] = {};
    for (uint8_t i = 0; i < tag.uid_len; ++i) {
        std::snprintf(text + 2 * i, sizeof(text) - 2 * i, "%02X", tag.uid[i]);
    }
    return std::strcmp(text, hex) == 0;
}

bool found(const EnumeratedTag *tags, uint8_t count, const char *hex)
{
    for (uint8_t i = 0; i < count; ++i) {
        if (uid_is(tags[i], hex)) {
            return true;
        }
    }
    return false;
}

void test_enumerate_single_tag_costs_nothing_extra()
{
    // The single-tag case has to be the frame sequence the driver already sent:
    // the caller's poll, one anticollision, one SELECT. No extra WUPA, no HLTA.
    SimField field;
    field.add("DEA30D00", 0x08);
    field.wupa();  // the caller's poll
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 1);
    EXPECT_EQ(uid_is(tags[0], "DEA30D00"), true);
    EXPECT_EQ(tags[0].uid_len, 4);
    EXPECT_EQ(tags[0].sak, 0x08);
    EXPECT_EQ(tag_is_iso_dep(tags[0]), false);
    EXPECT_EQ(stats.rounds, 1);
    EXPECT_EQ(stats.collisions, 0);
    EXPECT_EQ(field.wupa_calls, 0);
    EXPECT_EQ(field.anticol_calls, 1);
    EXPECT_EQ(field.select_calls, 1);
    EXPECT_EQ(field.halt_calls, 0);
}

void test_enumerate_seven_byte_uid_needs_cl2()
{
    SimField field;
    field.add("041AA7675F6180", 0x00);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 1);
    EXPECT_EQ(tags[0].uid_len, 7);
    EXPECT_EQ(uid_is(tags[0], "041AA7675F6180"), true);
    EXPECT_EQ(stats.rounds, 1);
    EXPECT_EQ(stats.collisions, 0);
    EXPECT_EQ(field.anticol_calls, 2);  // CL1 and CL2
    EXPECT_EQ(field.select_calls, 2);
    EXPECT_EQ(field.halt_calls, 0);
}

void test_enumerate_two_tags_colliding_at_cl1()
{
    // Two 4-byte UIDs that differ in the first UID byte.
    SimField field;
    field.add("DEA30D00", 0x08);
    field.add("DFA30D00", 0x20);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 2);
    EXPECT_EQ(found(tags, count, "DEA30D00"), true);
    EXPECT_EQ(found(tags, count, "DFA30D00"), true);
    EXPECT_EQ(stats.rounds, 2);
    EXPECT_EQ(stats.collisions, 1);
    EXPECT_EQ(stats.branches_pushed, 1);
    EXPECT_EQ(field.wupa_calls, 1);  // one per extra round
    EXPECT_EQ(field.halt_calls, 1);
    // The ISO-DEP tag is the one the Aliro exchange wants.
    for (uint8_t i = 0; i < count; ++i) {
        EXPECT_EQ(tag_is_iso_dep(tags[i]), uid_is(tags[i], "DFA30D00"));
    }
}

void test_enumerate_the_bench_pair()
{
    // The two cards on the STEVAL bench. Both are 7-byte UIDs, so both cascade,
    // and they collide at CL1 in the second UID byte: 0xDB against 0x1A differ
    // in bit 0, which is COLLISION_STATUS 0x40, frame byte 4 bit 0. The branch
    // with the 1 is 04DB..., which is the only UID the old single-branch
    // resolution ever reported.
    //
    // This is the case the Yondoor notes call "CL1 collision state must be
    // saved before CL2": the pending CL1 branch has to survive the whole CL2
    // excursion for the first card.
    SimField field;
    field.add("04DB70605F6180", 0x00);
    field.add("041AA7675F6180", 0x00);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 2);
    EXPECT_EQ(uid_is(tags[0], "04DB70605F6180"), true);  // the 1 branch first
    EXPECT_EQ(uid_is(tags[1], "041AA7675F6180"), true);
    EXPECT_EQ(stats.rounds, 2);
    EXPECT_EQ(stats.collisions, 1);
    EXPECT_EQ(stats.hit_round_cap, false);
    EXPECT_EQ(stats.hit_tag_cap, false);
}

void test_enumerate_collision_at_cl2_replays_cl1()
{
    // Same manufacturer bytes, so CL1 is identical and the collision is at CL2.
    // The pending branch is a CL2 branch, and reaching it again means selecting
    // CL1 first: the branch has to carry the CL1 UID part with it.
    SimField field;
    field.add("04DB7011223344", 0x00);
    field.add("04DB7055667788", 0x00);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 2);
    EXPECT_EQ(found(tags, count, "04DB7011223344"), true);
    EXPECT_EQ(found(tags, count, "04DB7055667788"), true);
    EXPECT_EQ(stats.rounds, 2);
    EXPECT_EQ(stats.collisions, 1);
    // Round 2 re-selects CL1 from the branch's saved path, then anticollides at
    // CL2: two SELECTs in round 1, two in round 2.
    EXPECT_EQ(field.select_calls, 4);
}

void test_enumerate_collision_in_the_bcc()
{
    // A BCC collision cannot be split: the four UID bytes are already known and
    // no bit can be driven, so there is no second branch to push. The subtree is
    // abandoned without hanging, and the other card is still found.
    SimField field;
    field.add("DEA30D00", 0x08);
    field.add("DEA30D00", 0x08, 0x5A);  // same UID, corrupt BCC
    field.add("5EA30D00", 0x08);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 1);
    EXPECT_EQ(found(tags, count, "5EA30D00"), true);
    EXPECT_EQ(stats.hit_round_cap, false);
    // The BCC collision is at frame byte 6, where next_anticollision_prefix
    // forces no bit, so it pushes no branch. Only the UID-byte collision does.
    EXPECT_EQ(stats.branches_pushed, 1);
    // One collision on the first UID byte, then the BCC collision twice: the
    // second one does not grow the prefix, so the subtree is given up there.
    EXPECT_EQ(stats.collisions, 3);
    EXPECT_EQ(stats.rounds, 2);
}

void test_enumerate_stops_at_the_cap()
{
    SimField field;
    field.add("01A30D00", 0x08);
    field.add("03A30D00", 0x08);
    field.add("05A30D00", 0x08);
    field.add("07A30D00", 0x08);
    field.add("09A30D00", 0x08);
    field.wupa();
    field.wupa_calls = 0;

    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    const uint8_t count = enumerate_tags(field, tags, 4, stats);
    EXPECT_EQ(count, 4);
    EXPECT_EQ(stats.hit_tag_cap, true);
    EXPECT_EQ(stats.rounds <= kMaxEnumerationRounds, true);
    // Nothing was written past the caller's buffer.
    EXPECT_EQ(stats.hit_round_cap, false);
}

void test_enumerate_empty_field()
{
    SimField field;
    EnumeratedTag tags[4] = {};
    EnumerationStats stats;
    EXPECT_EQ(enumerate_tags(field, tags, 4, stats), 0);
    EXPECT_EQ(stats.rounds, 1);
}

void test_select_frames_round_trip()
{
    uint8_t frames[kCascadeLevels][kAnticollisionFrameSize] = {};
    const uint8_t uid7[7] = {0x04, 0xDB, 0x70, 0x60, 0x5F, 0x61, 0x80};
    EXPECT_EQ(build_select_frames(uid7, 7, frames), 2);
    EXPECT_EQ(frames[0][0], 0x93);
    EXPECT_EQ(frames[0][1], 0x70);
    EXPECT_EQ(frames[0][2], 0x88);  // cascade tag
    EXPECT_EQ(frames[0][3], 0x04);
    EXPECT_EQ(frames[0][6], 0x88 ^ 0x04 ^ 0xDB ^ 0x70);
    EXPECT_EQ(frames[1][0], 0x95);
    EXPECT_EQ(frames[1][2], 0x60);
    EXPECT_EQ(frames[1][6], 0x60 ^ 0x5F ^ 0x61 ^ 0x80);

    const uint8_t uid4[4] = {0xDE, 0xA3, 0x0D, 0x00};
    EXPECT_EQ(build_select_frames(uid4, 4, frames), 1);
    EXPECT_EQ(frames[0][2], 0xDE);
    EXPECT_EQ(build_select_frames(uid4, 5, frames), 0);

    uint8_t parts[kCascadeLevels][4] = {{0x88, 0x04, 0xDB, 0x70}, {0x60, 0x5F, 0x61, 0x80}, {0, 0, 0, 0}};
    uint8_t uid[10] = {};
    EXPECT_EQ(assemble_uid(parts, 2, uid), 7);
    for (int i = 0; i < 7; ++i) {
        EXPECT_EQ(uid[i], uid7[i]);
    }
}

} // namespace

int main()
{
    test_num_tx_bytes_split_at_bit_5();
    test_fifo_count_uses_high_bits();
    test_fwi_matches_m5nfc();
    test_sfgt_rounds_up();
    test_parse_ats();
    test_rats_param();
    test_nrt_range_switch();
    test_ecp_frame_bytes();
    test_anticollision_helpers();
    test_next_anticollision_prefix();
    test_select_frames_round_trip();
    test_enumerate_single_tag_costs_nothing_extra();
    test_enumerate_seven_byte_uid_needs_cl2();
    test_enumerate_two_tags_colliding_at_cl1();
    test_enumerate_the_bench_pair();
    test_enumerate_collision_at_cl2_replays_cl1();
    test_enumerate_collision_in_the_bcc();
    test_enumerate_stops_at_the_cap();
    test_enumerate_empty_field();
    if (g_failures != 0) {
        std::printf("st25r3916 frame logic: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    std::printf("st25r3916 frame logic: all checks passed\n");
    return EXIT_SUCCESS;
}
