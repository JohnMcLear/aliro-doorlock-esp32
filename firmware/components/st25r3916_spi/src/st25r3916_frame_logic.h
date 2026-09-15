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

} // namespace st25r3916
