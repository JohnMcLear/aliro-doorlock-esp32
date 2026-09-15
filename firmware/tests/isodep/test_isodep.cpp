/*
 * Unit tests for isodep_apdu.h and the isodep_wtx.h wrapper.
 *
 * These cover the ISO 14443-4 block chaining that Aliro needs and the driver
 * did not previously have: a command split across several I-Blocks with
 * R(ACK)s between them, a response split the same way, S(WTX) waits arriving
 * at any point, and the failure modes we deliberately do not paper over.
 *
 * The transceive callable is scripted, so none of this needs a bus, a chip or
 * a tag. Frames the code under test sends are recorded and asserted on, which
 * matters more than the payload here: getting a PCB or a block number wrong is
 * exactly the kind of bug that shows up as an intermittent failed tap months
 * later.
 *
 * Compile & run:
 *   make -C tests/unit run
 * or directly:
 *   g++ -std=c++11 -I../../components/st25r test_isodep.cpp -o test_isodep && ./test_isodep
 */

#include "isodep_wtx.h"
#include "isodep_apdu.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace esphome::st25r;

// ── Minimal test framework, same shape as test_crypto1.cpp ───────────────────

static int g_pass = 0;
static int g_fail = 0;
static const char *g_test_name = "";

#define BEGIN_TEST(name)  do { g_test_name = (name); } while(0)
#define END_TEST()        do { printf("  PASS: %s\n", g_test_name); g_pass++; } while(0)

#define ASSERT_EQ(actual, expected)                                          \
  do {                                                                       \
    long long a_ = (long long)(actual), e_ = (long long)(expected);          \
    if (a_ != e_) {                                                          \
      printf("  FAIL: %s\n    %s:%d: %s == %lld, expected %lld\n",           \
             g_test_name, __FILE__, __LINE__, #actual, a_, e_);              \
      g_fail++;                                                              \
      return;                                                                \
    }                                                                        \
  } while(0)

#define ASSERT_TRUE(cond)                                                    \
  do {                                                                       \
    if (!(cond)) {                                                           \
      printf("  FAIL: %s\n    %s:%d: expected true: %s\n",                   \
             g_test_name, __FILE__, __LINE__, #cond);                        \
      g_fail++;                                                              \
      return;                                                                \
    }                                                                        \
  } while(0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_BYTES(actual, actual_len, expected, expected_len)             \
  do {                                                                       \
    ASSERT_EQ((actual_len), (expected_len));                                 \
    if (std::memcmp((actual), (expected), (expected_len)) != 0) {            \
      printf("  FAIL: %s\n    %s:%d: byte mismatch in %s\n",                 \
             g_test_name, __FILE__, __LINE__, #actual);                      \
      g_fail++;                                                              \
      return;                                                                \
    }                                                                        \
  } while(0)

// ── Scripted PICC ────────────────────────────────────────────────────────────

using Frame = std::vector<uint8_t>;

/// Replays a fixed list of responses and records every frame it was given.
/// A response of length 0 makes the transceive itself fail, which models a
/// timeout rather than a protocol error.
struct ScriptedPicc {
  std::vector<Frame> responses;
  std::vector<Frame> sent;
  size_t index = 0;

  bool operator()(const uint8_t *frame, uint8_t frame_len,
                  uint8_t *raw_resp, uint8_t &raw_len) {
    sent.emplace_back(frame, frame + frame_len);
    if (index >= responses.size()) {
      raw_len = 0;
      return false;
    }
    const Frame &r = responses[index++];
    if (r.empty()) {
      raw_len = 0;
      return false;
    }
    std::memcpy(raw_resp, r.data(), r.size());
    raw_len = static_cast<uint8_t>(r.size());
    return true;
  }
};

/// Build an I-Block: payload prefixed with a PCB.
static Frame iblock(uint8_t block_num, bool chaining, const std::vector<uint8_t> &payload) {
  Frame f;
  f.push_back(static_cast<uint8_t>(0x02 | (block_num & 1) | (chaining ? 0x10 : 0x00)));
  f.insert(f.end(), payload.begin(), payload.end());
  return f;
}

static Frame rack(uint8_t block_num) {
  return Frame{static_cast<uint8_t>(0xA2 | (block_num & 1))};
}

static Frame swtx(uint8_t wtxm) {
  return Frame{0xF2, wtxm};
}

/// A predictable filler so reassembly errors show up as wrong values, not
/// merely wrong lengths.
static std::vector<uint8_t> ramp(size_t n, uint8_t start = 0) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>(start + i);
  }
  return v;
}

// ── FSC helpers ──────────────────────────────────────────────────────────────

static void test_fsc_table() {
  BEGIN_TEST("FSC lookup covers the ISO 14443-4 table and clamps RFU values");
  ASSERT_EQ(isodep_fsc_from_fsci(0), 16);
  ASSERT_EQ(isodep_fsc_from_fsci(2), 32);
  ASSERT_EQ(isodep_fsc_from_fsci(5), 64);
  ASSERT_EQ(isodep_fsc_from_fsci(7), 128);
  ASSERT_EQ(isodep_fsc_from_fsci(8), 256);
  // 9-15 are RFU. Clamping up is the safe direction: we would rather send a
  // frame the PICC can hold than refuse to talk to it.
  ASSERT_EQ(isodep_fsc_from_fsci(9), 256);
  ASSERT_EQ(isodep_fsc_from_fsci(15), 256);
  END_TEST();
}

static void test_ats_parsing() {
  BEGIN_TEST("ATS FSC parsing, including the legal short-ATS default");
  const uint8_t ats_128[] = {0x05, 0x77, 0x80, 0x02, 0x00};  // T0 low nibble 7
  ASSERT_EQ(isodep_parse_ats_fsc(ats_128, sizeof(ats_128)), 128);

  const uint8_t ats_32[] = {0x02, 0x02};
  ASSERT_EQ(isodep_parse_ats_fsc(ats_32, sizeof(ats_32)), 32);

  // TL only, or nothing at all: ISO says assume the minimum.
  const uint8_t ats_short[] = {0x01};
  ASSERT_EQ(isodep_parse_ats_fsc(ats_short, sizeof(ats_short)), 32);
  ASSERT_EQ(isodep_parse_ats_fsc(nullptr, 0), 32);
  END_TEST();
}

static void test_max_payload() {
  BEGIN_TEST("Max payload deducts PCB and CRC, and never exceeds one frame");
  ASSERT_EQ(isodep_max_payload(32), 29);
  ASSERT_EQ(isodep_max_payload(128), 125);
  // FSC 256 would allow 253, which fits. The clamp exists for the degenerate
  // cases below it, not for this one.
  ASSERT_EQ(isodep_max_payload(256), 253);
  ASSERT_EQ(isodep_max_payload(3), 1);
  ASSERT_EQ(isodep_max_payload(0), 1);
  END_TEST();
}

// ── Single frame ─────────────────────────────────────────────────────────────

static void test_single_frame_exchange() {
  BEGIN_TEST("Single frame out, single frame back, block number toggles once");
  ScriptedPicc picc;
  picc.responses = {iblock(0, false, {0x6F, 0x00, 0x90, 0x00})};

  const uint8_t apdu[] = {0x00, 0xA4, 0x04, 0x00};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));

  const uint8_t expect[] = {0x6F, 0x00, 0x90, 0x00};
  ASSERT_BYTES(resp, resp_len, expect, sizeof(expect));
  ASSERT_EQ(block, 1);

  ASSERT_EQ(picc.sent.size(), 1);
  ASSERT_EQ(picc.sent[0][0], 0x02);  // I-Block, block 0, no chaining
  ASSERT_EQ(picc.sent[0].size(), sizeof(apdu) + 1);
  END_TEST();
}

static void test_block_number_alternates_across_calls() {
  BEGIN_TEST("Block number alternates across consecutive exchanges");
  ScriptedPicc picc;
  picc.responses = {iblock(0, false, {0x90, 0x00}), iblock(1, false, {0x90, 0x00})};

  const uint8_t apdu[] = {0x00, 0xB0, 0x00, 0x00};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));
  ASSERT_EQ(block, 1);
  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));
  ASSERT_EQ(block, 0);

  ASSERT_EQ(picc.sent[0][0], 0x02);
  ASSERT_EQ(picc.sent[1][0], 0x03);
  END_TEST();
}

static void test_response_needs_status_word() {
  BEGIN_TEST("An unchained response shorter than a status word is rejected");
  ScriptedPicc picc;
  picc.responses = {iblock(0, false, {0x90})};

  const uint8_t apdu[] = {0x00, 0xB0};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  END_TEST();
}

// ── S(WTX) ───────────────────────────────────────────────────────────────────

static void test_wtx_is_granted_and_echoed() {
  BEGIN_TEST("S(WTX) is granted with the WTXM echoed back, then the exchange continues");
  ScriptedPicc picc;
  picc.responses = {swtx(0x05), swtx(0x0A), iblock(0, false, {0x90, 0x00})};

  const uint8_t apdu[] = {0x80, 0xAA};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));
  ASSERT_EQ(resp_len, 2);

  // Frame 0 is the command; 1 and 2 are the WTX grants carrying WTXM back.
  ASSERT_EQ(picc.sent.size(), 3);
  ASSERT_EQ(picc.sent[1][0], 0xF2);
  ASSERT_EQ(picc.sent[1][1], 0x05);
  ASSERT_EQ(picc.sent[2][0], 0xF2);
  ASSERT_EQ(picc.sent[2][1], 0x0A);
  // The block number moves once for the exchange, not once per wait.
  ASSERT_EQ(block, 1);
  END_TEST();
}

static void test_endless_wtx_gives_up() {
  BEGIN_TEST("A PICC that only ever asks for more time is eventually abandoned");
  ScriptedPicc picc;
  for (int i = 0; i < 40; ++i) {
    picc.responses.push_back(swtx(0x01));
  }

  const uint8_t apdu[] = {0x80, 0xAA};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  ASSERT_TRUE(picc.sent.size() <= kIsodepMaxWtxWaits + 2);
  END_TEST();
}

static void test_wtx_during_receive_chain() {
  BEGIN_TEST("S(WTX) arriving midway through a chained response is handled");
  ScriptedPicc picc;
  picc.responses = {
      iblock(0, true, ramp(4, 0x10)),
      swtx(0x03),
      iblock(1, false, ramp(4, 0x20)),
  };

  const uint8_t apdu[] = {0x80, 0xBB};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));
  ASSERT_EQ(resp_len, 8);
  ASSERT_EQ(resp[0], 0x10);
  ASSERT_EQ(resp[4], 0x20);
  END_TEST();
}

// ── Chaining, send direction ─────────────────────────────────────────────────

static void test_send_chaining() {
  BEGIN_TEST("A long command splits into chained I-Blocks acknowledged by R(ACK)");
  // FSC 32 gives 29 payload bytes per frame. 70 bytes is three frames: 29, 29, 12.
  ScriptedPicc picc;
  picc.responses = {rack(0), rack(1), iblock(0, false, {0x90, 0x00})};

  std::vector<uint8_t> apdu = ramp(70);
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu.data(), apdu.size(), resp, sizeof(resp),
                                   resp_len, block, 32, picc));
  ASSERT_EQ(resp_len, 2);
  ASSERT_EQ(picc.sent.size(), 3);

  // First two carry the chaining bit and alternate block numbers 0 then 1;
  // the last drops chaining. That final block number is 0 again.
  ASSERT_EQ(picc.sent[0][0], 0x12);
  ASSERT_EQ(picc.sent[1][0], 0x13);
  ASSERT_EQ(picc.sent[2][0], 0x02);
  ASSERT_EQ(picc.sent[0].size(), 30);
  ASSERT_EQ(picc.sent[1].size(), 30);
  ASSERT_EQ(picc.sent[2].size(), 13);

  // Reassemble what the PICC actually received and check nothing was dropped
  // or duplicated at a fragment boundary.
  std::vector<uint8_t> got;
  for (const Frame &f : picc.sent) {
    got.insert(got.end(), f.begin() + 1, f.end());
  }
  ASSERT_BYTES(got.data(), got.size(), apdu.data(), apdu.size());
  END_TEST();
}

static void test_send_chaining_exact_multiple() {
  BEGIN_TEST("A command that is an exact multiple of the frame size does not send an empty tail");
  // FSC 32, payload 29. 58 bytes is exactly two frames.
  ScriptedPicc picc;
  picc.responses = {rack(0), iblock(0, false, {0x90, 0x00})};

  std::vector<uint8_t> apdu = ramp(58);
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu.data(), apdu.size(), resp, sizeof(resp),
                                   resp_len, block, 32, picc));
  ASSERT_EQ(picc.sent.size(), 2);
  ASSERT_EQ(picc.sent[0][0], 0x12);  // chained
  ASSERT_EQ(picc.sent[1][0], 0x03);  // final, block 1, no chaining
  END_TEST();
}

static void test_send_chaining_requires_ack() {
  BEGIN_TEST("An I-Block where an R(ACK) was due aborts the exchange");
  ScriptedPicc picc;
  picc.responses = {iblock(0, false, {0x90, 0x00})};

  std::vector<uint8_t> apdu = ramp(70);
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu.data(), apdu.size(), resp, sizeof(resp),
                                    resp_len, block, 32, picc));
  END_TEST();
}

// ── Chaining, receive direction ──────────────────────────────────────────────

static void test_receive_chaining() {
  BEGIN_TEST("A chained response is reassembled and each fragment acknowledged");
  ScriptedPicc picc;
  picc.responses = {
      iblock(0, true, ramp(20, 0x00)),
      iblock(1, true, ramp(20, 0x20)),
      iblock(0, false, ramp(6, 0x40)),
  };

  const uint8_t apdu[] = {0x80, 0xCA, 0x00, 0x00};
  uint8_t resp[128];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                   resp_len, block, 128, picc));
  ASSERT_EQ(resp_len, 46);
  ASSERT_EQ(resp[0], 0x00);
  ASSERT_EQ(resp[20], 0x20);
  ASSERT_EQ(resp[40], 0x40);

  // One command frame then two R(ACK)s at the toggled block numbers.
  ASSERT_EQ(picc.sent.size(), 3);
  ASSERT_EQ(picc.sent[0][0], 0x02);
  ASSERT_EQ(picc.sent[1].size(), 1);
  ASSERT_EQ(picc.sent[1][0], 0xA3);
  ASSERT_EQ(picc.sent[2].size(), 1);
  ASSERT_EQ(picc.sent[2][0], 0xA2);
  END_TEST();
}

static void test_receive_chaining_respects_capacity() {
  BEGIN_TEST("A response larger than the buffer fails rather than truncating");
  ScriptedPicc picc;
  picc.responses = {
      iblock(0, true, ramp(30)),
      iblock(1, true, ramp(30)),
      iblock(0, false, ramp(30)),
  };

  const uint8_t apdu[] = {0x80, 0xCA};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  END_TEST();
}

static void test_both_directions_chain() {
  BEGIN_TEST("A long command and a long response chain in the same exchange");
  ScriptedPicc picc;
  picc.responses = {
      rack(0),
      iblock(1, true, ramp(29, 0x50)),
      iblock(0, false, ramp(10, 0x70)),
  };

  std::vector<uint8_t> apdu = ramp(50, 0x80);
  uint8_t resp[128];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_exchange_apdu(apdu.data(), apdu.size(), resp, sizeof(resp),
                                   resp_len, block, 32, picc));
  ASSERT_EQ(resp_len, 39);
  ASSERT_EQ(resp[0], 0x50);
  ASSERT_EQ(resp[29], 0x70);
  END_TEST();
}

// ── Failure modes we choose not to paper over ────────────────────────────────

static void test_deselect_is_terminal() {
  BEGIN_TEST("S(DESELECT) ends the exchange rather than being answered");
  ScriptedPicc picc;
  picc.responses = {Frame{0xC2}};

  const uint8_t apdu[] = {0x00, 0xA4};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  ASSERT_EQ(picc.sent.size(), 1);
  END_TEST();
}

static void test_nak_is_not_retried() {
  BEGIN_TEST("R(NAK) fails the exchange, deliberately, rather than retransmitting");
  ScriptedPicc picc;
  picc.responses = {Frame{0xB2}};

  const uint8_t apdu[] = {0x00, 0xA4};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  END_TEST();
}

static void test_transceive_failure_propagates() {
  BEGIN_TEST("A timeout from the bus layer fails the exchange");
  ScriptedPicc picc;
  picc.responses = {Frame{}};

  const uint8_t apdu[] = {0x00, 0xA4};
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  END_TEST();
}

static void test_empty_apdu_rejected() {
  BEGIN_TEST("An empty APDU is rejected without touching the bus");
  ScriptedPicc picc;
  uint8_t resp[64];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(nullptr, 0, resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  ASSERT_EQ(picc.sent.size(), 0);
  END_TEST();
}

static void test_endless_picc_chaining_terminates() {
  BEGIN_TEST("A PICC that chains forever is abandoned rather than looping");
  ScriptedPicc picc;
  for (int i = 0; i < 400; ++i) {
    picc.responses.push_back(iblock(static_cast<uint8_t>(i & 1), true, {0x00}));
  }

  const uint8_t apdu[] = {0x00, 0xA4};
  static uint8_t resp[8192];
  size_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_exchange_apdu(apdu, sizeof(apdu), resp, sizeof(resp),
                                    resp_len, block, 128, picc));
  END_TEST();
}

// ── The wrapper the rest of the driver still calls ───────────────────────────

static void test_wrapper_single_frame() {
  BEGIN_TEST("isodep_process_loop still behaves for the single-frame case");
  ScriptedPicc picc;
  picc.responses = {iblock(0, false, {0x03, 0x90, 0x00})};

  const uint8_t apdu[] = {0x00, 0xB0, 0x00, 0x00, 0x03};
  uint8_t resp[64];
  uint8_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_process_loop(apdu, sizeof(apdu), resp, resp_len, block, picc));
  ASSERT_EQ(resp_len, 3);
  ASSERT_EQ(resp[0], 0x03);
  ASSERT_EQ(block, 1);
  END_TEST();
}

static void test_wrapper_bounds_response() {
  BEGIN_TEST("isodep_process_loop refuses a response past its 63-byte bound");
  ScriptedPicc picc;
  picc.responses = {
      iblock(0, true, ramp(40)),
      iblock(1, false, ramp(40)),
  };

  const uint8_t apdu[] = {0x00, 0xB0};
  uint8_t resp[64];
  uint8_t resp_len = 0xFF;
  uint8_t block = 0;

  ASSERT_FALSE(isodep_process_loop(apdu, sizeof(apdu), resp, resp_len, block, picc));
  // On failure the length is cleared, so a caller that ignores the return
  // value at least does not read stale bytes.
  ASSERT_EQ(resp_len, 0);
  END_TEST();
}

static void test_wrapper_now_chains_long_commands() {
  BEGIN_TEST("isodep_process_loop chains a long command instead of rejecting it");
  // The old implementation returned false for any APDU past 63 bytes.
  ScriptedPicc picc;
  picc.responses = {rack(0), iblock(0, false, {0x90, 0x00})};

  std::vector<uint8_t> apdu = ramp(100);
  uint8_t resp[64];
  uint8_t resp_len = 0;
  uint8_t block = 0;

  ASSERT_TRUE(isodep_process_loop(apdu.data(), apdu.size(), resp, resp_len, block, picc));
  ASSERT_EQ(picc.sent.size(), 2);
  ASSERT_EQ(resp_len, 2);
  END_TEST();
}

// ── Runner ───────────────────────────────────────────────────────────────────

int main() {
  printf("ISO-DEP APDU chaining tests\n");

  test_fsc_table();
  test_ats_parsing();
  test_max_payload();

  test_single_frame_exchange();
  test_block_number_alternates_across_calls();
  test_response_needs_status_word();

  test_wtx_is_granted_and_echoed();
  test_endless_wtx_gives_up();
  test_wtx_during_receive_chain();

  test_send_chaining();
  test_send_chaining_exact_multiple();
  test_send_chaining_requires_ack();

  test_receive_chaining();
  test_receive_chaining_respects_capacity();
  test_both_directions_chain();

  test_deselect_is_terminal();
  test_nak_is_not_retried();
  test_transceive_failure_propagates();
  test_empty_apdu_rejected();
  test_endless_picc_chaining_terminates();

  test_wrapper_single_frame();
  test_wrapper_bounds_response();
  test_wrapper_now_chains_long_commands();

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
