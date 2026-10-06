/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-free helpers for the ST25R300 SPI NFC driver, kept apart so the
 * host test in firmware/tests/st25r300_logic_test.cpp can pin them down.
 *
 * The interesting part is slow-card tolerance. A Java Card applet doing
 * AES-GCM in software (VivoKey's Aliro ring applet on NXP J3R180/J3R452) can
 * take 10 to 30 seconds over one transaction. The card keeps the reader
 * waiting by sending S(WTX) requests, often one per frame waiting time, so a
 * single APDU can see hundreds of them. The shared ISO-DEP layer
 * (third_party/yondoor_isodep/isodep_apdu.h, copied verbatim and not to be
 * edited here) gives up after 8 in a row, which suits phones and would kill a
 * slow card after well under a second. So this driver absorbs S(WTX) below
 * that layer, bounded by a wall-clock budget per APDU instead of a count, and
 * the ISO-DEP layer only ever sees the frame that finally carries data.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace st25r300 {

// TX_FRAME1 holds ntx[12:5]; TX_FRAME2 holds ntx[4:0] in bits 7:3 and the
// number of extra bits in 2:0. Same split as the ST25R3916's NUM_TX_BYTES.
inline void encode_tx_frame(uint16_t bytes, uint8_t extra_bits, uint8_t &reg1, uint8_t &reg2)
{
    reg1 = static_cast<uint8_t>((bytes >> 5) & 0xFF);
    reg2 = static_cast<uint8_t>(((bytes & 0x1F) << 3) | (extra_bits & 0x07));
}

// FIFO_STATUS1 holds fifo_b<7:0>; fifo_b<8> is bit 6 of FIFO_STATUS2. This is
// NOT the ST25R3916 layout (bits 7:6 there), so the 3916 helper must not be
// reused: it would read a 256-byte frame as 0 or 512.
inline uint16_t decode_fifo_count(uint8_t status1, uint8_t status2)
{
    return static_cast<uint16_t>(status1 | ((status2 & 0x40) << 2));
}

// fifo_lb<2:0> in bits 3:1: valid bits in the last byte, 0 for a whole byte.
inline uint8_t decode_fifo_last_bits(uint8_t status2)
{
    return static_cast<uint8_t>((status2 >> 1) & 0x07);
}

// NRT_CONF1 value for the step chosen by st25r3916::nrt_from_ms(). The timer
// arithmetic is identical on the two chips (64/fc or 4096/fc steps, 16 bits,
// 309 ms or 19.8 s range, DS14655 5.8.2); only the register differs.
inline uint8_t nrt_conf1(bool step_4096fc)
{
    return step_4096fc ? 0x01 : 0x00;  // nrt_emv stays 0: RXS stops the timer
}

// ISO/IEC 14443-4 S(WTX) request: PCB 1111 x010 (bit 3 is the CID flag, which
// this reader never sets but a card could still echo), one INF byte.
inline bool is_swtx_request(const uint8_t *raw, uint8_t raw_len)
{
    return raw != nullptr && raw_len >= 2 && (raw[0] & 0xF7) == 0xF2;
}

// WTXM is the low six bits; 0 and 60..63 are RFU. Treat 0 as 1 and clamp the
// RFU top end to 59, the largest legal multiplier.
inline uint8_t wtxm_from_inf(uint8_t inf)
{
    uint8_t wtxm = static_cast<uint8_t>(inf & 0x3F);
    if (wtxm == 0) {
        wtxm = 1;
    }
    if (wtxm > 59) {
        wtxm = 59;
    }
    return wtxm;
}

// How long to wait after granting a WTX: FWT x WTXM, but never above the
// ceiling, and never below the plain FWT.
inline uint32_t wtx_wait_ms(uint32_t fwt_ms, uint8_t wtxm, uint32_t ceiling_ms)
{
    if (ceiling_ms < fwt_ms) {
        ceiling_ms = fwt_ms;
    }
    const uint64_t wait = static_cast<uint64_t>(fwt_ms) * (wtxm ? wtxm : 1);
    return wait > ceiling_ms ? ceiling_ms : static_cast<uint32_t>(wait);
}

struct WtxPolicy {
    uint32_t fwt_ms = 100;          // frame waiting time for a normal frame
    uint32_t wtx_ceiling_ms = 5000; // longest single wait after a WTX grant
    uint32_t budget_ms = 30000;     // wall clock for the whole APDU
    uint32_t max_wtx = 2000;        // backstop against a card that never stops
};

struct FrameStats {
    uint32_t frames = 0;        // frames transmitted, WTX responses included
    uint32_t wtx = 0;           // S(WTX) requests granted
    uint8_t max_wtxm = 0;
    uint32_t longest_wait_ms = 0;
    bool budget_exceeded = false;
    bool wtx_limit = false;
};

// Send one ISO-DEP frame and absorb any run of S(WTX) requests.
//
//   raw_tx(const uint8_t *frame, uint8_t len, uint8_t *raw, uint8_t &raw_len,
//          uint32_t timeout_ms) -> bool
//       one frame out, one frame back (PCB included, CRC stripped)
//   elapsed_ms() -> uint32_t
//       wall clock since the APDU began
//   on_wtx(uint32_t count, uint8_t wtxm, uint32_t wait_ms, uint32_t elapsed_ms)
//       progress hook, called once per granted WTX
//
// Returns true with the first frame that is not a WTX request in raw/raw_len.
// The S(WTX) response echoes the request's PCB and WTXM, as ISO/IEC 14443-4
// 7.3 requires.
template <typename RawTx, typename Elapsed, typename OnWtx>
bool send_frame_absorbing_wtx(const uint8_t *frame, uint8_t frame_len, uint8_t *raw, uint8_t &raw_len,
                              const WtxPolicy &policy, FrameStats &stats, RawTx &raw_tx, Elapsed &elapsed_ms,
                              OnWtx &on_wtx)
{
    uint8_t grant[2] = {};
    const uint8_t *out = frame;
    uint8_t out_len = frame_len;
    uint32_t timeout_ms = policy.fwt_ms;

    while (true) {
        if (elapsed_ms() > policy.budget_ms) {
            stats.budget_exceeded = true;
            return false;
        }
        raw_len = 0;
        ++stats.frames;
        if (!raw_tx(out, out_len, raw, raw_len, timeout_ms)) {
            return false;
        }
        if (!is_swtx_request(raw, raw_len)) {
            return true;
        }
        if (stats.wtx >= policy.max_wtx) {
            stats.wtx_limit = true;
            return false;
        }
        ++stats.wtx;
        const uint8_t wtxm = wtxm_from_inf(raw[1]);
        if (wtxm > stats.max_wtxm) {
            stats.max_wtxm = wtxm;
        }
        timeout_ms = wtx_wait_ms(policy.fwt_ms, wtxm, policy.wtx_ceiling_ms);
        if (timeout_ms > stats.longest_wait_ms) {
            stats.longest_wait_ms = timeout_ms;
        }
        on_wtx(stats.wtx, wtxm, timeout_ms, elapsed_ms());
        grant[0] = raw[0];
        grant[1] = static_cast<uint8_t>(raw[1] & 0x3F);
        out = grant;
        out_len = sizeof(grant);
    }
}

} // namespace st25r300
