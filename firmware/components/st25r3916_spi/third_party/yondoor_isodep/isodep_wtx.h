#pragma once

// ISO 14443-4 S(WTX) handling and the small-frame APDU path.
//
// History, because the shape of this file is now misleading on its own:
// this originally held the whole ISO-DEP state machine, which was one I-Block
// out, one I-Block back, with S(WTX) waits absorbed in between. That is all
// the driver needed for NDEF and short select/read APDUs.
//
// Aliro needs block chaining in both directions, so the state machine moved to
// isodep_apdu.h and grew. What remains here is the original entry point, kept
// because callers and tests use it, now implemented as a thin wrapper, plus
// the CRC strip helper that the ST25R300 transport needs.
//
// The wrapper's behaviour is unchanged for everything that worked before, with
// one deliberate improvement: an APDU longer than a single frame used to be
// rejected outright and now chains.
//
// Contract, unchanged:
//
//  * `tx` is a callable with signature:
//        bool tx(const uint8_t *frame, uint8_t frame_len,
//                uint8_t *raw_resp, uint8_t &raw_len)
//    It returns true on success and writes the raw frame received from the
//    PICC (including the leading PCB) into raw_resp/raw_len.
//
//  * `block_number_io` is the caller's persistent block-number bit, toggled on
//    a successful exchange per the ISO 14443-4 alternation rule.
//
//  * `resp` / `resp_len` receive the response payload with PCBs stripped. The
//    caller must provide at least 63 bytes, which is the bound this wrapper
//    enforces.
//
//  * S(WTX) requests are acknowledged and the wait re-entered, up to 8 times.
//    Android HCE cold start has been observed to issue 1-3 in succession on
//    first tap.
//
//  * S(DESELECT), a stray R(NAK) or a truncated frame all return false.
//
// Why a template: avoids any function-pointer indirection in the production
// path. The compiler inlines the call to `tx` against `transceive_`.

#include "isodep_apdu.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace esphome {
namespace st25r {

static constexpr uint8_t kIsodepMaxWtx = kIsodepMaxWtxWaits;
static constexpr uint8_t kIsodepFrameMax = 64;

/// Single-frame-oriented APDU exchange, bounded to a 63-byte response.
/// Prefer isodep_exchange_apdu() directly when the response can be larger,
/// which for anything Aliro touches it can.
template <typename Tx>
inline bool isodep_process_loop(const uint8_t *apdu, size_t apdu_len,
                                uint8_t *resp, uint8_t &resp_len,
                                uint8_t &block_number_io,
                                Tx &tx) {
  size_t len = 0;
  const bool ok = isodep_exchange_apdu(apdu, apdu_len, resp,
                                       kIsodepFrameMax - 1, len,
                                       block_number_io, kIsodepFrameMax, tx);
  resp_len = ok ? static_cast<uint8_t>(len) : 0;
  return ok;
}

// Strip the trailing 2 bytes (the ISO 14443 CRC trailer) from a chip-FIFO
// response. The ST25R300's RX_CRC bit only validates the CRC; it does not
// remove it from the FIFO. Without this, every with_crc=true read leaks two
// garbage bytes that callers misinterpret as part of the payload (the ISO-DEP
// SW1 SW2 in particular).
//
// Returns true if the resulting length is still valid (>= 0). Specifically:
//  * with_crc=false → no-op, returns true, *len unchanged.
//  * with_crc=true,  *len >= 2 → *len -= 2, returns true.
//  * with_crc=true,  *len == 1 → returns false (truncated, can't strip).
//  * with_crc=true,  *len == 0 → no-op, returns true (nothing to strip,
//                                  caller will treat as "no response").
//
// Extracted into a free function purely so the strip rule can be unit-tested
// without instantiating an ST25R300.
inline bool strip_trailing_crc(uint8_t & /*ref to mutate*/ len, bool with_crc) {
  if (!with_crc) return true;
  if (len == 0) return true;
  if (len < 2) return false;
  len = static_cast<uint8_t>(len - 2);
  return true;
}

}  // namespace st25r
}  // namespace esphome
