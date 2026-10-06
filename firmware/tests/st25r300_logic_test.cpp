// Host test for firmware/components/st25r300_spi/src/st25r300_logic.h.
//
//   c++ -std=gnu++17 -Wall -Wextra -Werror -I firmware/components/st25r300_spi/src
//       firmware/tests/st25r300_logic_test.cpp -o /tmp/st25r300_logic_test
//   /tmp/st25r300_logic_test
//
// scripts/build_release.sh runs it for variants that use the ST25R300 driver.
// The part that matters most is the S(WTX) loop: a Java Card applet doing
// AES-GCM in software keeps the reader waiting for tens of seconds with
// hundreds of WTX requests, and the shared ISO-DEP layer would give up after 8.

#include "st25r300_logic.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace st25r300;

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

void test_tx_frame_split()
{
    uint8_t r1 = 0;
    uint8_t r2 = 0;
    encode_tx_frame(0, 7, r1, r2);  // WUPA short frame, as the ESPHome driver writes it
    EXPECT_EQ(r1, 0x00);
    EXPECT_EQ(r2, 0x07);
    encode_tx_frame(2, 0, r1, r2);  // HLTA, ESPHome writes TX_FRAME2 = 0x10
    EXPECT_EQ(r1, 0x00);
    EXPECT_EQ(r2, 0x10);
    encode_tx_frame(255, 0, r1, r2);
    EXPECT_EQ(r1, 0x07);
    EXPECT_EQ(r2, 0xF8);
    encode_tx_frame(2, 3, r1, r2);  // anticollision with a split byte
    EXPECT_EQ(r2, 0x13);
}

void test_fifo_count_ninth_bit_is_bit_6()
{
    EXPECT_EQ(decode_fifo_count(0x00, 0x40), 256);
    EXPECT_EQ(decode_fifo_count(0xFF, 0x00), 255);
    // Bit 7 is not part of the count on this chip (it is on the ST25R3916).
    EXPECT_EQ(decode_fifo_count(0x10, 0x80), 0x10);
    EXPECT_EQ(decode_fifo_last_bits(0x0E), 7);
    EXPECT_EQ(decode_fifo_last_bits(0x41), 0);
}

void test_swtx_recognition()
{
    const uint8_t wtx[2] = {0xF2, 0x01};
    const uint8_t wtx_cid[3] = {0xFA, 0x00, 0x05};
    const uint8_t deselect[1] = {0xC2};
    const uint8_t iblock[3] = {0x02, 0x90, 0x00};
    EXPECT_EQ(is_swtx_request(wtx, 2), true);
    EXPECT_EQ(is_swtx_request(wtx, 1), false);
    EXPECT_EQ(is_swtx_request(wtx_cid, 3), true);
    EXPECT_EQ(is_swtx_request(deselect, 1), false);
    EXPECT_EQ(is_swtx_request(iblock, 3), false);
    EXPECT_EQ(wtxm_from_inf(0x00), 1);
    EXPECT_EQ(wtxm_from_inf(0xC5), 5);  // power level bits ignored
    EXPECT_EQ(wtxm_from_inf(0x3F), 59);
    EXPECT_EQ(wtx_wait_ms(77, 1, 5000), 77);
    EXPECT_EQ(wtx_wait_ms(77, 59, 5000), 4543);
    EXPECT_EQ(wtx_wait_ms(309, 59, 5000), 5000);
    EXPECT_EQ(wtx_wait_ms(6000, 2, 5000), 6000);  // never below the FWT itself
}

// A scripted card: answers each frame from a queue and advances a fake clock.
struct FakeCard {
    std::vector<std::vector<uint8_t>> replies;
    std::vector<std::vector<uint8_t>> sent;
    std::vector<uint32_t> timeouts;
    size_t next = 0;
    uint32_t clock_ms = 0;
    uint32_t ms_per_frame = 0;

    bool operator()(const uint8_t *frame, uint8_t len, uint8_t *raw, uint8_t &raw_len, uint32_t timeout_ms)
    {
        sent.emplace_back(frame, frame + len);
        timeouts.push_back(timeout_ms);
        clock_ms += ms_per_frame;
        if (next >= replies.size()) {
            return false;
        }
        const auto &r = replies[next++];
        std::memcpy(raw, r.data(), r.size());
        raw_len = static_cast<uint8_t>(r.size());
        return true;
    }
};

void test_absorbs_hundreds_of_wtx_within_budget()
{
    FakeCard card;
    card.ms_per_frame = 77;
    for (int i = 0; i < 300; ++i) {
        card.replies.push_back({0xF2, 0x01});
    }
    card.replies.push_back({0x02, 0x90, 0x00});
    auto elapsed = [&card]() { return card.clock_ms; };
    uint32_t wtx_seen = 0;
    auto on_wtx = [&wtx_seen](uint32_t, uint8_t, uint32_t, uint32_t) { ++wtx_seen; };

    WtxPolicy policy;
    policy.fwt_ms = 77;
    policy.budget_ms = 30000;
    FrameStats stats;
    const uint8_t frame[3] = {0x02, 0x80, 0x80};
    uint8_t raw[256] = {};
    uint8_t raw_len = 0;
    EXPECT_EQ(send_frame_absorbing_wtx(frame, sizeof(frame), raw, raw_len, policy, stats, card, elapsed, on_wtx),
              true);
    EXPECT_EQ(raw_len, 3);
    EXPECT_EQ(raw[0], 0x02);
    EXPECT_EQ(stats.wtx, 300);
    EXPECT_EQ(wtx_seen, 300);
    EXPECT_EQ(stats.frames, 301);
    EXPECT_EQ(stats.budget_exceeded, false);
    // Every grant echoes PCB and WTXM, and waits FWT x WTXM.
    EXPECT_EQ(card.sent[1].size(), 2u);
    EXPECT_EQ(card.sent[1][0], 0xF2);
    EXPECT_EQ(card.sent[1][1], 0x01);
    EXPECT_EQ(card.timeouts[0], 77);
    EXPECT_EQ(card.timeouts[1], 77);
}

void test_budget_ends_an_endless_card()
{
    FakeCard card;
    card.ms_per_frame = 1000;
    for (int i = 0; i < 100; ++i) {
        card.replies.push_back({0xF2, 0x0A});
    }
    auto elapsed = [&card]() { return card.clock_ms; };
    auto on_wtx = [](uint32_t, uint8_t, uint32_t, uint32_t) {};
    WtxPolicy policy;
    policy.fwt_ms = 100;
    policy.budget_ms = 10000;
    FrameStats stats;
    const uint8_t frame[2] = {0x03, 0x00};
    uint8_t raw[256] = {};
    uint8_t raw_len = 0;
    EXPECT_EQ(send_frame_absorbing_wtx(frame, sizeof(frame), raw, raw_len, policy, stats, card, elapsed, on_wtx),
              false);
    EXPECT_EQ(stats.budget_exceeded, true);
    EXPECT_EQ(stats.wtx_limit, false);
    EXPECT_EQ(stats.max_wtxm, 10);
    EXPECT_EQ(card.timeouts[1], 1000);  // 100 ms x WTXM 10
}

void test_wtx_count_backstop()
{
    FakeCard card;
    for (int i = 0; i < 20; ++i) {
        card.replies.push_back({0xF2, 0x01});
    }
    auto elapsed = [&card]() { return card.clock_ms; };
    auto on_wtx = [](uint32_t, uint8_t, uint32_t, uint32_t) {};
    WtxPolicy policy;
    policy.max_wtx = 5;
    FrameStats stats;
    const uint8_t frame[2] = {0x02, 0x00};
    uint8_t raw[256] = {};
    uint8_t raw_len = 0;
    EXPECT_EQ(send_frame_absorbing_wtx(frame, sizeof(frame), raw, raw_len, policy, stats, card, elapsed, on_wtx),
              false);
    EXPECT_EQ(stats.wtx_limit, true);
    EXPECT_EQ(stats.wtx, 5);
}

void test_lost_frame_fails_without_retrying()
{
    FakeCard card;  // no replies at all
    auto elapsed = [&card]() { return card.clock_ms; };
    auto on_wtx = [](uint32_t, uint8_t, uint32_t, uint32_t) {};
    WtxPolicy policy;
    FrameStats stats;
    const uint8_t frame[2] = {0x02, 0x00};
    uint8_t raw[256] = {};
    uint8_t raw_len = 0;
    EXPECT_EQ(send_frame_absorbing_wtx(frame, sizeof(frame), raw, raw_len, policy, stats, card, elapsed, on_wtx),
              false);
    EXPECT_EQ(stats.frames, 1);
    EXPECT_EQ(card.sent.size(), 1u);
}

} // namespace

int main()
{
    test_tx_frame_split();
    test_fifo_count_ninth_bit_is_bit_6();
    test_swtx_recognition();
    test_absorbs_hundreds_of_wtx_within_budget();
    test_budget_ends_an_endless_card();
    test_wtx_count_backstop();
    test_lost_frame_fails_without_retrying();
    if (g_failures != 0) {
        std::printf("st25r300 logic: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    std::printf("st25r300 logic: all checks passed\n");
    return EXIT_SUCCESS;
}
