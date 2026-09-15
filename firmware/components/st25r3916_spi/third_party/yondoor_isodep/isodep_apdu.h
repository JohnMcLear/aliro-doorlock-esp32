#pragma once

// ISO 14443-4 (ISO-DEP) APDU exchange with block chaining in both directions.
//
// Why this exists
// ---------------
// isodep_wtx.h handled the single-frame case: one I-Block out, one I-Block
// back, with S(WTX) waits in between. That covers NDEF reads and short
// select/read APDUs, which is everything this driver needed until now.
//
// It does not cover Aliro. An Aliro transaction carries reader certificates,
// EXCHANGE payloads and step-up ENVELOPEs that run to hundreds or thousands of
// bytes, well past any single ISO-DEP frame. Both directions have to chain:
// the reader splits a long command across several I-Blocks, and the phone
// splits a long response back the same way.
//
// Espressif's Aliro SDK lists "Extended or Chaining APDU in NFC Transport" as
// a supported feature and its session config carries an `extended_apdu` flag,
// so the transport underneath it is expected to do this.
//
// Contract
// --------
//  * `tx` is a callable with signature:
//        bool tx(const uint8_t *frame, uint8_t frame_len,
//                uint8_t *raw_resp, uint8_t &raw_len)
//    It returns true on success and writes the raw frame received from the
//    PICC (including the leading PCB) into raw_resp/raw_len. A single frame is
//    never more than 255 bytes, so uint8_t lengths are correct here even though
//    the assembled APDU either side can be much longer. It is taken by
//    reference: a stateful callable passed by value would have its state
//    silently discarded between frames.
//
//  * `block_number_io` is the caller's persistent block-number bit, toggled per
//    ISO 14443-4 rule B: toggle on receipt of an I-Block or R(ACK) carrying the
//    current block number, before sending anything else.
//
//  * `fsc` is the PICC's maximum frame size in bytes, taken from the ATS. The
//    largest payload we can put in one frame is fsc - 3: one PCB byte and the
//    two CRC bytes. We never use DID or NAD, so nothing further is deducted.
//
//  * `resp_capacity` bounds the assembled response. Overrun returns false
//    rather than truncating, because a silently short APDU response is far
//    worse than a failed exchange.
//
// Block numbering, and why receive is permissive
// ----------------------------------------------
// We drive our own block number strictly by the toggle rule. We do not reject
// a received I-Block whose block number disagrees, we log it and carry on.
// Real phones are the target here, and both Android HCE and iOS have been
// observed to be looser about this than the standard reads. Being strict buys
// conformance we cannot use and costs taps that should have worked.
//
// Deliberately not implemented
// ----------------------------
//  * DID and NAD. Single tag in the field, no addressing needed.
//  * R(NAK) retransmission. A failed frame fails the exchange and the caller
//    starts again, which for a door reader means the user taps twice. Retry
//    logic that hides a genuinely bad RF path is worse than a second tap.
//  * S(DESELECT) is recognised and treated as a hard stop, not answered.

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace esphome {
namespace st25r {

// Up to this many S(WTX) waits in a row before giving up. Android HCE cold
// start has been seen to issue 1-3 in succession on first tap; an Aliro
// step-up phase does elliptic curve work on the phone and may ask for more.
static constexpr uint8_t kIsodepMaxWtxWaits = 8;

// Guard against a PICC that chains without ever terminating.
static constexpr uint16_t kIsodepMaxBlocks = 256;

// Largest single frame we build or receive. The transceive layer below us
// carries uint8_t lengths, so 255 is the hard ceiling regardless of FSC.
static constexpr size_t kIsodepMaxFrame = 256;

// PCB bit patterns, ISO 14443-4 section 7.1.
static constexpr uint8_t kPcbIBlockBase = 0x02;   // 000x xx1x
static constexpr uint8_t kPcbChaining = 0x10;
static constexpr uint8_t kPcbRAckBase = 0xA2;     // 101x xx1x, bit 4 clear
static constexpr uint8_t kPcbSWtxResponse = 0xF2;
static constexpr uint8_t kPcbSDeselect = 0xC2;

/// FSC lookup from the 4-bit FSCI in ATS byte T0. ISO 14443-4 table 5.
/// Values above 8 are RFU and are clamped to 256.
inline uint16_t isodep_fsc_from_fsci(uint8_t fsci) {
  static const uint16_t kTable[9] = {16, 24, 32, 40, 48, 64, 96, 128, 256};
  return fsci < 9 ? kTable[fsci] : 256;
}

/// Pull FSC out of an ATS. Returns the ISO default of 32 bytes when the ATS is
/// too short to carry T0, which is legal and means "assume the minimum".
inline uint16_t isodep_parse_ats_fsc(const uint8_t *ats, uint8_t ats_len) {
  if (ats == nullptr || ats_len < 2) {
    return 32;
  }
  return isodep_fsc_from_fsci(static_cast<uint8_t>(ats[1] & 0x0F));
}

/// Largest INF payload that fits one frame at this FSC: PCB plus two CRC bytes
/// come off the top. Also clamped so we never build a frame the uint8_t-length
/// transceive layer beneath cannot describe.
inline size_t isodep_max_payload(uint16_t fsc) {
  size_t p = (fsc > 3) ? static_cast<size_t>(fsc - 3) : 1;
  if (p > kIsodepMaxFrame - 1) {
    p = kIsodepMaxFrame - 1;
  }
  return p;
}

/// Result of one raw frame classification, so the send and receive phases can
/// share the S(WTX) handling without duplicating the PCB decode.
enum class IsodepFrameKind : uint8_t {
  IBlock,
  IBlockChaining,
  RAck,
  SWtxRequest,
  Deselect,
  Unusable,
};

inline IsodepFrameKind isodep_classify(const uint8_t *raw, uint8_t raw_len) {
  if (raw == nullptr || raw_len < 1) {
    return IsodepFrameKind::Unusable;
  }
  const uint8_t pcb = raw[0];

  // S-Block: bits 7,6 = 11.
  if ((pcb & 0xC0) == 0xC0) {
    // S(WTX) request: bits 5,4 = 11, one INF byte carrying WTXM.
    if ((pcb & 0x30) == 0x30 && raw_len >= 2) {
      return IsodepFrameKind::SWtxRequest;
    }
    if ((pcb & 0x30) == 0x00) {
      return IsodepFrameKind::Deselect;
    }
    return IsodepFrameKind::Unusable;
  }

  // R-Block: bits 7,6 = 10. Bit 4 clear is ACK, set is NAK.
  if ((pcb & 0xC0) == 0x80) {
    return (pcb & kPcbChaining) ? IsodepFrameKind::Unusable : IsodepFrameKind::RAck;
  }

  // I-Block: bits 7,6 = 00.
  return (pcb & kPcbChaining) ? IsodepFrameKind::IBlockChaining : IsodepFrameKind::IBlock;
}

/// Send one frame and absorb any run of S(WTX) requests, returning the first
/// frame that is not a wait. Returns false if the PICC keeps asking for time
/// past kIsodepMaxWtxWaits, or the transceive fails, or it deselects.
template <typename Tx>
inline bool isodep_send_and_settle(const uint8_t *frame, uint8_t frame_len,
                                   uint8_t *raw, uint8_t &raw_len,
                                   IsodepFrameKind &kind, Tx &tx) {
  // Two bytes is the largest frame this ever rewrites in place (an S(WTX)
  // grant), but it starts as a copy of whatever the caller built.
  uint8_t out[kIsodepMaxFrame];
  uint8_t out_len = frame_len;
  std::memcpy(out, frame, frame_len);

  for (uint8_t wait = 0; wait <= kIsodepMaxWtxWaits; ++wait) {
    raw_len = 0;
    if (!tx(out, out_len, raw, raw_len)) {
      return false;
    }
    kind = isodep_classify(raw, raw_len);
    if (kind != IsodepFrameKind::SWtxRequest) {
      return kind != IsodepFrameKind::Unusable && kind != IsodepFrameKind::Deselect;
    }
    // Grant the wait by echoing WTXM back in an S(WTX) response.
    out[0] = kPcbSWtxResponse;
    out[1] = raw[1];
    out_len = 2;
  }
  return false;
}

/// Full APDU exchange with chaining in both directions.
///
/// On success `resp` holds the assembled response INF with every PCB stripped,
/// and `resp_len` its length. `block_number_io` is left where the next exchange
/// should pick it up.
template <typename Tx>
inline bool isodep_exchange_apdu(const uint8_t *apdu, size_t apdu_len,
                                 uint8_t *resp, size_t resp_capacity, size_t &resp_len,
                                 uint8_t &block_number_io, uint16_t fsc, Tx &tx) {
  resp_len = 0;
  if (apdu == nullptr || apdu_len == 0 || resp == nullptr) {
    return false;
  }

  const size_t max_payload = isodep_max_payload(fsc);
  uint8_t frame[kIsodepMaxFrame];
  uint8_t raw[kIsodepMaxFrame];
  uint8_t raw_len = 0;
  IsodepFrameKind kind = IsodepFrameKind::Unusable;

  // ── Send phase. Every fragment but the last carries the chaining bit and is
  // acknowledged with an R(ACK); the last one is answered with the response.
  size_t sent = 0;
  while (true) {
    const size_t remaining = apdu_len - sent;
    const size_t chunk = remaining > max_payload ? max_payload : remaining;
    const bool more = (chunk < remaining);

    frame[0] = static_cast<uint8_t>(kPcbIBlockBase | (block_number_io & 0x01) |
                                    (more ? kPcbChaining : 0));
    std::memcpy(frame + 1, apdu + sent, chunk);

    if (!isodep_send_and_settle(frame, static_cast<uint8_t>(chunk + 1), raw, raw_len, kind, tx)) {
      return false;
    }
    sent += chunk;

    if (!more) {
      break;  // raw now holds the first frame of the response.
    }

    // Mid-chain the PICC must acknowledge and nothing else.
    if (kind != IsodepFrameKind::RAck) {
      return false;
    }
    block_number_io ^= 1;
  }

  // ── Receive phase. `raw` already holds the first response frame.
  for (uint16_t block = 0; block < kIsodepMaxBlocks; ++block) {
    if (kind != IsodepFrameKind::IBlock && kind != IsodepFrameKind::IBlockChaining) {
      return false;
    }
    if (raw_len < 1) {
      return false;
    }

    const size_t payload = static_cast<size_t>(raw_len - 1);
    if (resp_len + payload > resp_capacity) {
      return false;  // Refuse to truncate an APDU response.
    }
    std::memcpy(resp + resp_len, raw + 1, payload);
    resp_len += payload;

    block_number_io ^= 1;

    if (kind == IsodepFrameKind::IBlock) {
      // A complete single-frame response still has to be long enough to carry
      // a status word. Chained fragments are allowed to be shorter than that.
      return resp_len >= 2;
    }

    // More to come: acknowledge with R(ACK) at the freshly toggled number.
    frame[0] = static_cast<uint8_t>(kPcbRAckBase | (block_number_io & 0x01));
    if (!isodep_send_and_settle(frame, 1, raw, raw_len, kind, tx)) {
      return false;
    }
  }
  return false;  // PICC chained without end.
}

}  // namespace st25r
}  // namespace esphome
