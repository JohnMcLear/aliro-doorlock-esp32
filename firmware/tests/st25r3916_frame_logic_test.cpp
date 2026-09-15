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
    if (g_failures != 0) {
        std::printf("st25r3916 frame logic: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    std::printf("st25r3916 frame logic: all checks passed\n");
    return EXIT_SUCCESS;
}
