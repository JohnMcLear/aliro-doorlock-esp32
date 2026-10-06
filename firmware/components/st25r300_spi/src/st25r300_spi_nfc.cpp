/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25R300 NFC-A poller over SPI, exposing the m5nfc interface that
 * esp-matter's Aliro door lock delegate calls. See include/m5nfc.h.
 *
 * Structure follows ../st25r3916_spi/src/st25r3916_spi_nfc.cpp, so the two
 * drivers can be read side by side. What changes is the chip layer:
 *
 *  - Register sequences, init order and the NFC-A analog profile come from
 *    John McLear's ESPHome ST25R300 driver (esphome_st25r/components/st25r300,
 *    MIT), verified on hardware for ISO 14443A UID, NDEF and ISO-DEP, and its
 *    notes st25r300_init_sequence.md and st25r300_rx_protocol1_guide.md.
 *  - The ST25R300 has no WUPA command: WUPA is a 7-bit frame from the FIFO.
 *  - RX_PROTOCOL1 differs per phase (WUPA 0x38, anticollision 0x39, data 0x3C).
 *  - CLEAR_FIFO, never STOP_ALL, before a transmission: STOP_ALL turns the
 *    receiver off and the answer is lost.
 *  - With rx_crc set the chip checks the CRC but leaves both bytes in the FIFO.
 *  - FIFO is 256 bytes; the 9th count bit is FIFO_STATUS2 bit 6.
 *
 * Shared without copying, from ../st25r3916_spi: the chip-independent frame
 * logic (anticollision tree walk, ATS parsing, ECP frame, NRT arithmetic) and
 * the verbatim ISO-DEP chaining header in third_party/yondoor_isodep.
 *
 * Slow-card tolerance (S(WTX) absorbed under a wall-clock budget instead of a
 * count of 8) lives in st25r300_logic.h; see the comment there.
 *
 * STATUS: compiles; NOT YET RUN ON HARDWARE. The ESPHome register sequences
 * it reuses are proven; this port of them is not.
 */

#include <m5nfc.h>
#include <st25r300_nfc.h>

#include "isodep_apdu.h"
#include "st25r3916_frame_logic.h"
#include "st25r300_hal.h"
#include "st25r300_logic.h"
#include "st25r300_regs.h"

#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstring>

using namespace st25r300;
using st25r3916::AnticolOutcome;
using st25r3916::AnticolReply;
using st25r3916::AtsInfo;
using st25r3916::EnumeratedTag;
using st25r3916::EnumerationStats;

namespace {

constexpr char kTag[] = "st25r300";

constexpr uint8_t kIsoDepFsdi = CONFIG_ST25R300_ISODEP_FSDI;
constexpr uint32_t kNrtMaxMs = 19800;

// Aliro ECP, identical to the 3916 driver and to patch 0005.
constexpr uint32_t kEcpResponseTimeoutMs = 12;
constexpr uint32_t kEcpSettleTimeMs = 5;

// NFC-A timeouts, measured by the chip's no-response timer from the end of our
// transmission. The ESPHome driver uses ~5 ms for every NFC-A frame.
constexpr uint32_t kWupaTimeoutMs = 5;
constexpr uint32_t kAnticollisionTimeoutMs = 8;
constexpr uint32_t kSelectTimeoutMs = 8;
constexpr uint32_t kRatsTimeoutMs = 10;
constexpr uint32_t kDeselectTimeoutMs = 8;

constexpr uint8_t kMaxTagsPerPoll = 4;

constexpr int64_t kHealthCheckIntervalUs = 5 * 1000 * 1000;
constexpr int64_t kReinitBackoffUs = 5 * 1000 * 1000;
constexpr int64_t kFirstStatusLogUs = 30 * 1000 * 1000;
constexpr int64_t kStatusLogIntervalUs = 10 * 60 * 1000 * 1000LL;
constexpr int64_t kCollisionLogIntervalUs = 5 * 1000 * 1000;
constexpr int64_t kWtxProgressLogIntervalUs = 2 * 1000 * 1000;

constexpr uint32_t kIrqUsed = kIrqRxErr | kIrqTxe | kIrqRxs | kIrqRxe | kIrqCol | kIrqNre | kIrqDct | kIrqOsc;

enum class RxStatus : uint8_t {
    kOk,
    kTimeout,
    kCollision,
    kFraming,
    kParity,
    kCrc,
    kIncomplete,
    kOverflow,
    kIo,
};

const char *rx_status_name(RxStatus s)
{
    switch (s) {
    case RxStatus::kOk: return "ok";
    case RxStatus::kTimeout: return "timeout";
    case RxStatus::kCollision: return "collision";
    case RxStatus::kFraming: return "framing";
    case RxStatus::kParity: return "parity";
    case RxStatus::kCrc: return "crc";
    case RxStatus::kIncomplete: return "incomplete-byte";
    case RxStatus::kOverflow: return "overflow";
    case RxStatus::kIo: return "io";
    }
    return "?";
}

enum class FrameKind : uint8_t {
    kShort,   // 7-bit WUPA
    kAnticol, // bit-oriented anticollision frame
    kData,    // whole bytes with CRC both ways
    kDataNoResponse,
};

struct TxRxRequest {
    const uint8_t *tx = nullptr;
    uint16_t tx_bytes = 0;
    uint8_t tx_bits = 0;
    FrameKind kind = FrameKind::kData;
    uint32_t timeout_ms = 10;
};

// Chip and link state. Owned by the nfc_detect task.
bool g_chip_ready = false;
uint8_t g_ic_identity = 0;
uint8_t g_rx_frame[kFifoDepth];
uint8_t g_nrt_shadow[3] = {0xFF, 0xFF, 0xFF};

bool g_active = false;
bool g_active_iso_dep = false;
uint8_t g_block_number = 0;
uint16_t g_fsc = 32;
uint32_t g_ats_fwt_ms = 0;

int64_t g_last_health_us = 0;
int64_t g_next_reinit_us = 0;
int64_t g_next_status_log_us = kFirstStatusLogUs;
int64_t g_next_collision_log_us = 0;
uint32_t g_iso_dep_rotor = 0;

struct Stats {
    uint32_t polls = 0;
    uint32_t tags = 0;
    uint32_t iso_dep = 0;
    uint32_t rx_errors = 0;
    uint32_t apdu_failures = 0;
    uint32_t reinits = 0;
    uint32_t collisions = 0;
    uint32_t enumerated = 0;
    uint32_t multi_tag_polls = 0;
    uint32_t tag_cap_hits = 0;
    uint32_t wtx_total = 0;
} g_stats;

// Run-time knobs, written from the serial task and read by the NFC task.
std::atomic<uint32_t> g_apdu_budget_ms{CONFIG_ST25R300_APDU_BUDGET_MS};
std::atomic<uint32_t> g_fwt_floor_ms{CONFIG_ST25R300_FWT_FLOOR_MS};
std::atomic<uint32_t> g_wtx_ceiling_ms{CONFIG_ST25R300_WTX_CEILING_MS};
std::atomic<uint32_t> g_apdu_attempts{CONFIG_ST25R300_APDU_ATTEMPTS};
std::atomic<bool> g_frame_trace{false};

st25r300_apdu_stats_t g_last_apdu = {};
portMUX_TYPE g_last_apdu_lock = portMUX_INITIALIZER_UNLOCKED;

// ECP state is written from the Matter task and read by the NFC task.
std::array<uint8_t, st25r3916::kEcpFrameSize> g_aliro_ecp_frame = {
    0x6A, 0x02, 0xCB, 0x02, 0x06, 0x20, 0x42, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
bool g_aliro_ecp_enabled = false;
portMUX_TYPE g_aliro_ecp_lock = portMUX_INITIALIZER_UNLOCKED;

HalConfig hal_config()
{
    HalConfig c = {};
    c.spi_host = CONFIG_ST25R300_SPI_HOST;
    c.pin_sclk = CONFIG_ST25R300_SPI_PIN_SCLK;
    c.pin_miso = CONFIG_ST25R300_SPI_PIN_MISO;
    c.pin_mosi = CONFIG_ST25R300_SPI_PIN_MOSI;
    c.pin_cs = CONFIG_ST25R300_SPI_PIN_CS;
    c.pin_irq = CONFIG_ST25R300_SPI_PIN_IRQ;
    c.pin_rst = CONFIG_ST25R300_SPI_PIN_RST;
    c.spi_freq_hz = CONFIG_ST25R300_SPI_FREQ_HZ;
    return c;
}

void delay_ms(uint32_t ms)
{
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(ms)));
}

uint32_t effective_fwt_ms()
{
    const uint32_t floor_ms = std::min<uint32_t>(g_fwt_floor_ms.load(), kNrtMaxMs);
    return std::max<uint32_t>(g_ats_fwt_ms ? g_ats_fwt_ms : 1, floor_ms);
}

void trace_frame(const char *direction, const uint8_t *data, size_t len, uint32_t t_ms)
{
    if (!g_frame_trace.load()) {
        return;
    }
    char line[3 * 32 + 1];
    for (size_t offset = 0; offset < len || offset == 0; offset += 32) {
        const size_t chunk = std::min<size_t>(32, len - offset);
        size_t pos = 0;
        line[0] = '\0';
        for (size_t i = 0; i < chunk; ++i) {
            pos += static_cast<size_t>(snprintf(line + pos, sizeof(line) - pos, "%02X ", data[offset + i]));
        }
        ESP_LOGI(kTag, "FRAME %s t=%" PRIu32 " ms len=%u %s%s", direction, t_ms, static_cast<unsigned>(len),
                 offset ? "+" : "", line);
        if (len == 0) {
            break;
        }
    }
}

esp_err_t set_no_response_timer(uint32_t ms)
{
    bool step_4096 = false;
    uint16_t value = 0;
    st25r3916::nrt_from_ms(ms, step_4096, value);
    const uint8_t want[3] = {nrt_conf1(step_4096), static_cast<uint8_t>(value >> 8),
                             static_cast<uint8_t>(value & 0xFF)};
    const uint8_t regs[3] = {kRegNrtConf1, kRegNrtConf2, kRegNrtConf3};
    for (int i = 0; i < 3; ++i) {
        if (g_nrt_shadow[i] == want[i]) {
            continue;
        }
        esp_err_t err = write_reg(regs[i], want[i]);
        if (err != ESP_OK) {
            g_nrt_shadow[i] = 0xFF;
            return err;
        }
        g_nrt_shadow[i] = want[i];
    }
    return ESP_OK;
}

RxStatus transceive(const TxRxRequest &req, uint8_t *rx, size_t rx_capacity, size_t &rx_len)
{
    rx_len = 0;

    uint8_t tx_prot = kTxProtData;
    uint8_t rx_prot = kRxProtData;
    bool rx_crc = true;
    bool expect_response = true;
    switch (req.kind) {
    case FrameKind::kShort:
        tx_prot = kTxProtShortFrame;
        rx_prot = kRxProtShortFrame;
        rx_crc = false;
        break;
    case FrameKind::kAnticol:
        tx_prot = kTxProtAnticol;
        // DS14655 table 44: rx_nbtx must be set when a bit-oriented
        // anticollision answer is expected. The ESPHome driver never sends a
        // split byte, so this path is from the datasheet, not from a bench run.
        rx_prot = static_cast<uint8_t>(kRxProtAnticol | (req.tx_bits ? kRxProtRxNbtx : 0));
        rx_crc = false;
        break;
    case FrameKind::kData:
        break;
    case FrameKind::kDataNoResponse:
        expect_response = false;
        break;
    }

    if (write_reg(kRegTxProtocol1, tx_prot) != ESP_OK || write_reg(kRegRxProtocol1, rx_prot) != ESP_OK ||
        set_no_response_timer(std::min<uint32_t>(req.timeout_ms, kNrtMaxMs)) != ESP_OK) {
        return RxStatus::kIo;
    }
    // CLEAR_FIFO, not STOP_ALL: STOP_ALL disables the receiver and the answer
    // is lost (ESPHome st25r300 notes). CLEAR_RX_GAIN resets AGC and squelch.
    if (command(kCmdClearFifo) != ESP_OK || command(kCmdClearRxGain) != ESP_OK) {
        return RxStatus::kIo;
    }
    irq_clear(kIrqUsed);

    uint8_t f1 = 0;
    uint8_t f2 = 0;
    const uint16_t tx_bytes = req.kind == FrameKind::kShort ? 0 : req.tx_bytes;
    const uint8_t tx_bits = req.kind == FrameKind::kShort ? 7 : req.tx_bits;
    encode_tx_frame(tx_bytes, tx_bits, f1, f2);
    const size_t fifo_bytes = static_cast<size_t>(tx_bytes) + (tx_bits ? 1u : 0u);
    // TX_FRAME before the FIFO data, then transmit (DS14655 5.16.6).
    if (write_reg(kRegTxFrame1, f1) != ESP_OK || write_reg(kRegTxFrame2, f2) != ESP_OK ||
        fifo_write(req.tx, fifo_bytes) != ESP_OK || command(kCmdTransmitData) != ESP_OK) {
        return RxStatus::kIo;
    }

    const uint32_t tx_timeout_ms = 20 + (req.tx_bytes / 8u);
    if (irq_wait(kIrqTxe, tx_timeout_ms) == 0) {
        ESP_LOGW(kTag, "no TXE after %" PRIu32 " ms", tx_timeout_ms);
        return RxStatus::kIo;
    }
    if (!expect_response) {
        return RxStatus::kOk;
    }

    // The chip's NRT is authoritative; the software deadline is a backstop in
    // case its interrupt is lost.
    uint32_t hit = irq_wait(kIrqRxs | kIrqRxe | kIrqNre, req.timeout_ms + 50);
    if ((hit & (kIrqRxs | kIrqRxe)) == 0) {
        return RxStatus::kTimeout;
    }
    if ((hit & kIrqRxe) == 0) {
        // A full 256-byte frame takes about 25 ms on air at 106 kbit/s.
        hit |= irq_wait(kIrqRxe, 50 + static_cast<uint32_t>(rx_capacity / 8));
        if ((hit & kIrqRxe) == 0) {
            ++g_stats.rx_errors;
            return RxStatus::kFraming;
        }
    }
    const uint32_t errors = irq_take(kIrqRxErr | kIrqCol) | (hit & (kIrqRxErr | kIrqCol));

    uint8_t st1 = 0;
    uint8_t st2 = 0;
    if (read_reg(kRegFifoStatus1, st1) != ESP_OK || read_reg(kRegFifoStatus2, st2) != ESP_OK) {
        return RxStatus::kIo;
    }
    const uint16_t count = decode_fifo_count(st1, st2);
    const uint8_t last_bits = decode_fifo_last_bits(st2);
    if ((st2 & kFifoStatus2Ovr) || count > sizeof(g_rx_frame)) {
        ++g_stats.rx_errors;
        return RxStatus::kOverflow;
    }
    if (count > 0 && fifo_read(g_rx_frame, count) != ESP_OK) {
        return RxStatus::kIo;
    }

    RxStatus status = RxStatus::kOk;
    if (errors & kIrqRxErr) {
        uint8_t st3 = 0;
        (void)read_reg(kRegStaticStatus3, st3);
        if (st3 & kStatic3Crc) {
            status = RxStatus::kCrc;
        } else if (st3 & kStatic3Par) {
            status = RxStatus::kParity;
        } else {
            status = RxStatus::kFraming;
        }
    } else if (errors & kIrqCol) {
        status = RxStatus::kCollision;
    } else if (last_bits != 0 && req.kind != FrameKind::kAnticol) {
        status = RxStatus::kIncomplete;
    } else if ((st2 & kFifoStatus2NpLb) && req.kind != FrameKind::kAnticol) {
        status = RxStatus::kFraming;
    }

    // rx_crc checks the CRC but leaves it in the FIFO (ESPHome finding, unit
    // tested there as strip_trailing_crc). Strip it here.
    size_t payload = count;
    if (rx_crc && status == RxStatus::kOk) {
        if (count < 2) {
            status = RxStatus::kFraming;
            payload = 0;
        } else {
            payload = count - 2;
        }
    }
    if (payload > rx_capacity) {
        ++g_stats.rx_errors;
        return RxStatus::kOverflow;  // never truncate a frame silently
    }
    if (payload > 0) {
        std::memcpy(rx, g_rx_frame, payload);
    }
    rx_len = payload;
    if (status != RxStatus::kOk && status != RxStatus::kCollision) {
        ++g_stats.rx_errors;
    }
    return status;
}

bool send_aliro_ecp()
{
    std::array<uint8_t, st25r3916::kEcpFrameSize> frame = {};
    bool enabled = false;
    portENTER_CRITICAL(&g_aliro_ecp_lock);
    frame = g_aliro_ecp_frame;
    enabled = g_aliro_ecp_enabled;
    portEXIT_CRITICAL(&g_aliro_ecp_lock);
    if (!enabled) {
        return false;
    }
    // A broadcast: CRC-A appended by the chip, reply neither expected nor used.
    TxRxRequest req;
    req.tx = frame.data();
    req.tx_bytes = static_cast<uint16_t>(frame.size());
    req.kind = FrameKind::kData;
    req.timeout_ms = kEcpResponseTimeoutMs;
    uint8_t response[1] = {};
    size_t response_len = 0;
    (void)transceive(req, response, sizeof(response), response_len);
    return true;
}

// WUPA, not REQA: every activation ends in HLTA or DESELECT, and a halted
// PICC answers WUPA only (ISO/IEC 14443-3 6.3). Same reasoning as the 3916
// driver. The ST25R300 sends it as a 7-bit frame from the FIFO.
bool poll_a(uint16_t &atqa)
{
    static constexpr uint8_t kWupa[1] = {0x52};
    TxRxRequest req;
    req.tx = kWupa;
    req.kind = FrameKind::kShort;
    req.timeout_ms = kWupaTimeoutMs;
    uint8_t rx[2] = {};
    size_t rx_len = 0;
    const RxStatus status = transceive(req, rx, sizeof(rx), rx_len);
    if (status == RxStatus::kTimeout || rx_len == 0) {
        return false;
    }
    atqa = static_cast<uint16_t>((rx_len > 1 ? (rx[1] << 8) : 0) | rx[0]);
    return status == RxStatus::kOk || status == RxStatus::kCollision || status == RxStatus::kParity ||
        status == RxStatus::kFraming || status == RxStatus::kCrc;
}

void report_collision(uint8_t level, uint8_t collision_display)
{
    const int64_t now = esp_timer_get_time();
    if (now < g_next_collision_log_us) {
        return;
    }
    g_next_collision_log_us = now + kCollisionLogIntervalUs;
    ESP_LOGI(kTag,
             "NFC-A CL%u collision at frame byte %u bit %u (COLLISION_DISPLAY=0x%02x): more than one tag is in "
             "the field.",
             level + 1, static_cast<unsigned>((collision_display >> 4) & 0x0F),
             static_cast<unsigned>((collision_display >> 1) & 0x07), collision_display);
}

// The hardware seam st25r3916::enumerate_tags() drives. COLLISION_DISPLAY
// (0x38) has the same c_byte/c_bit layout as the 3916's COLLISION_STATUS, so
// the tree walk applies unchanged; the ESPHome base class relies on the same
// equivalence for both chips.
struct PollerOps {
    bool wupa()
    {
        uint16_t atqa = 0;
        return poll_a(atqa);
    }

    void begin_anticollision(uint8_t) {}
    void end_anticollision(uint8_t) {}

    AnticolReply anticollision(uint8_t level, uint8_t *frame, uint8_t bytes, uint8_t bits)
    {
        TxRxRequest req;
        req.tx = frame;
        req.tx_bytes = bytes;
        req.tx_bits = bits;
        req.kind = FrameKind::kAnticol;
        req.timeout_ms = kAnticollisionTimeoutMs;
        size_t rx_len = 0;
        const RxStatus status =
            transceive(req, frame + bytes, st25r3916::kAnticollisionFrameSize - bytes, rx_len);

        AnticolReply reply;
        if (status == RxStatus::kCollision) {
            uint8_t coll = 0;
            if (read_reg(kRegCollisionDisplay, coll) != ESP_OK) {
                return reply;
            }
            ++g_stats.collisions;
            report_collision(level, coll);
            reply.outcome = AnticolOutcome::kCollision;
            reply.collision_status = coll;
            return reply;
        }
        if (status != RxStatus::kOk) {
            ESP_LOGD(kTag, "anticollision CL%u failed: %s rx_len=%u", level + 1, rx_status_name(status),
                     static_cast<unsigned>(rx_len));
            return reply;
        }
        reply.outcome = AnticolOutcome::kResolved;
        reply.rx_len = static_cast<uint8_t>(rx_len);
        return reply;
    }

    bool select(uint8_t level, uint8_t *frame, uint8_t &sak)
    {
        TxRxRequest req;
        req.tx = frame;
        req.tx_bytes = st25r3916::kAnticollisionFrameSize;
        req.kind = FrameKind::kData;
        req.timeout_ms = kSelectTimeoutMs;
        uint8_t sak_buf[1] = {};
        size_t sak_len = 0;
        const RxStatus status = transceive(req, sak_buf, sizeof(sak_buf), sak_len);
        if (status != RxStatus::kOk || sak_len != 1) {
            ESP_LOGD(kTag, "SELECT CL%u failed: %s len=%u", level + 1, rx_status_name(status),
                     static_cast<unsigned>(sak_len));
            return false;
        }
        sak = sak_buf[0];
        return true;
    }

    void halt()
    {
        static constexpr uint8_t kHlta[2] = {0x50, 0x00};
        TxRxRequest req;
        req.tx = kHlta;
        req.tx_bytes = sizeof(kHlta);
        req.kind = FrameKind::kDataNoResponse;
        uint8_t rx[1] = {};
        size_t rx_len = 0;
        (void)transceive(req, rx, sizeof(rx), rx_len);
        delay_ms(2);
    }
};

bool reselect_tag(PollerOps &ops, const EnumeratedTag &tag)
{
    uint8_t frames[st25r3916::kCascadeLevels][st25r3916::kAnticollisionFrameSize] = {};
    const uint8_t levels = st25r3916::build_select_frames(tag.uid, tag.uid_len, frames);
    if (levels == 0) {
        return false;
    }
    ops.halt();
    if (!ops.wupa()) {
        return false;
    }
    uint8_t sak = 0;
    for (uint8_t level = 0; level < levels; ++level) {
        if (!ops.select(level, frames[level], sak)) {
            return false;
        }
        const bool cascade = (sak & st25r3916::kSakCascadeBit) != 0;
        if (cascade != (level + 1 < levels)) {
            return false;
        }
    }
    return sak == tag.sak;
}

const char *format_uid(const uint8_t *uid, uint8_t len, char *out, size_t out_size)
{
    size_t pos = 0;
    out[0] = '\0';
    for (uint8_t i = 0; i < len && pos + 3 <= out_size; ++i) {
        pos += static_cast<size_t>(snprintf(out + pos, out_size - pos, "%02X", uid[i]));
    }
    return out;
}

bool rats(uint16_t atqa, uint8_t sak, const char *uid_str)
{
    const uint8_t rats_cmd[2] = {0xE0, st25r3916::rats_param(kIsoDepFsdi, 0)};
    TxRxRequest req;
    req.tx = rats_cmd;
    req.tx_bytes = sizeof(rats_cmd);
    req.kind = FrameKind::kData;
    req.timeout_ms = kRatsTimeoutMs;
    uint8_t ats[64] = {};
    size_t ats_len = 0;
    const RxStatus status = transceive(req, ats, sizeof(ats), ats_len);
    AtsInfo info;
    if (status != RxStatus::kOk || ats_len < 1 || !st25r3916::parse_ats(ats, ats_len, info)) {
        ESP_LOGW(kTag, "RATS failed: %s ats_len=%u (ATQA=%04x SAK=%02x UID=%s)", rx_status_name(status),
                 static_cast<unsigned>(ats_len), atqa, sak, uid_str);
        return false;
    }

    g_ats_fwt_ms = st25r3916::fwi_to_ms(info.fwi);
    g_fsc = info.fsc;
    g_block_number = 0;

    const uint32_t sfgt_ms = st25r3916::sfgi_to_ms(info.sfgi);
    if (sfgt_ms > 0) {
        delay_ms(sfgt_ms);
    }
    char ats_hex[3 * 20 + 1] = {};
    size_t pos = 0;
    for (size_t i = 0; i < ats_len && i < 20; ++i) {
        pos += static_cast<size_t>(snprintf(ats_hex + pos, sizeof(ats_hex) - pos, "%02X", ats[i]));
    }
    ESP_LOGI(kTag,
             "ATS %s len=%u FSC=%u FSDI=%u FWI=%u (ATS FWT %" PRIu32 " ms, in force %" PRIu32
             " ms) SFGT=%" PRIu32 " ms",
             ats_hex, static_cast<unsigned>(ats_len), g_fsc, kIsoDepFsdi, info.fwi, g_ats_fwt_ms,
             effective_fwt_ms(), sfgt_ms);
    return true;
}

uint8_t sense_rf()
{
    irq_clear(kIrqDct);
    if (command(kCmdSenseRf) != ESP_OK) {
        return 0;
    }
    (void)irq_wait(kIrqDct, 10);
    uint8_t value = 0;
    (void)read_reg(kRegSenseRf, value);
    return value;
}

struct RegValue {
    uint8_t reg;
    uint8_t value;
};

// NFC-A 106 kbit/s profile, exactly as the ESPHome ST25R300 driver's
// reset_chip() writes it (RFAL ST25R500 analogConfigTbl, Poll NFC-A 106).
// Factory defaults for the correlator do NOT receive an ATQA.
const RegValue kAnalogProfile[] = {
    {kRegGeneralConf, 0x00},   // differential antenna, no RFO2
    {kRegProtocol1, 0x01},     // om = NFC-A poller
    {kRegTxProtocol1, kTxProtData},
    {kRegRxProtocol1, kRxProtData},
    {kRegRxAnalog1, 0x73},     // dig_clk_dly 7, hpf_ctrl 3
    {kRegRxAnalog2, 0x22},     // afe_gain_rw 2, afe_gain_td 2
    {kRegRxAnalog3, 0x85},
    {kRegRxDigital, 0xCC},
    {kRegCorr1, 0xF8},
    {kRegCorr2, 0x2E},
    {kRegCorr3, 0x0F},         // start_wait 15
    {kRegCorr4, 0x88},
    {kRegCorr5, 0x32},         // dis_agc_noise_meas, required by afe_gain_rw != 0
    {kRegCorr6, 0x20},
    {kRegTxDriver, static_cast<uint8_t>((15 - CONFIG_ST25R300_RF_POWER) & 0x0F)},  // d_res
    {kRegTxMod1, 0x70},        // am_mod 7; must stay out of TX_DRIVER for NFC-A
};

esp_err_t write_and_check_profile()
{
    for (const auto &rv : kAnalogProfile) {
        ESP_RETURN_ON_ERROR(write_reg(rv.reg, rv.value), kTag, "write 0x%02x failed", rv.reg);
    }
    // Read back. The ESPHome driver never did this, so a mismatch is reported
    // rather than treated as fatal: some bits may legitimately read back
    // differently, and a working reader should not be refused on a guess.
    for (const auto &rv : kAnalogProfile) {
        uint8_t got = 0;
        ESP_RETURN_ON_ERROR(read_reg(rv.reg, got), kTag, "read 0x%02x failed", rv.reg);
        if (got != rv.value) {
            ESP_LOGW(kTag, "register 0x%02x reads 0x%02x, wrote 0x%02x", rv.reg, got, rv.value);
        }
    }
    return ESP_OK;
}

esp_err_t spi_self_test()
{
    constexpr uint8_t patterns[] = {0xA5, 0x5A, 0xFF, 0x00};
    for (uint8_t p : patterns) {
        uint8_t got1 = 0;
        uint8_t got2 = 0;
        ESP_RETURN_ON_ERROR(write_reg(kRegNrtConf2, p), kTag, "SPI write failed");
        ESP_RETURN_ON_ERROR(write_reg(kRegNrtConf3, static_cast<uint8_t>(~p)), kTag, "SPI write failed");
        ESP_RETURN_ON_ERROR(read_reg(kRegNrtConf2, got1), kTag, "SPI read failed");
        ESP_RETURN_ON_ERROR(read_reg(kRegNrtConf3, got2), kTag, "SPI read failed");
        if (got1 != p || got2 != static_cast<uint8_t>(~p)) {
            ESP_LOGE(kTag, "SPI self-test: wrote %02x/%02x, read %02x/%02x", p, static_cast<uint8_t>(~p), got1, got2);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    g_nrt_shadow[0] = g_nrt_shadow[1] = g_nrt_shadow[2] = 0xFF;
    return ESP_OK;
}

esp_err_t read_identity(uint8_t &identity)
{
    identity = 0;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 5; ++attempt) {
        err = read_reg(kRegIcIdentity, identity);
        if (err == ESP_OK && (identity & kIcTypeMask) == kIcTypeSt25r300) {
            return ESP_OK;
        }
        delay_ms(20);
    }
    return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
}

// Field on, the ESPHome way: tx without rx, NFC field on (initial RF collision
// avoidance), then everything on.
esp_err_t field_on()
{
    ESP_RETURN_ON_ERROR(write_reg(kRegOperation, kOpEn | kOpVdddrEn | kOpTxEn), kTag, "OPERATION write failed");
    delay_ms(5);
    irq_clear(kIrqDct);
    ESP_RETURN_ON_ERROR(command(kCmdFieldOn), kTag, "field-on command failed");
    delay_ms(10);
    ESP_RETURN_ON_ERROR(write_reg(kRegOperation, kOpAllOn), kTag, "OPERATION write failed");
    delay_ms(10);
    return ESP_OK;
}

esp_err_t chip_configure()
{
    g_chip_ready = false;
    g_active = false;
    g_active_iso_dep = false;

    hal_reset_pulse();

    // Identity before SET_DEFAULT: if the bus is wrong, do not write anything.
    uint8_t identity = 0;
    esp_err_t err = read_identity(identity);
    if (err != ESP_OK) {
        ESP_LOGE(kTag,
                 "no ST25R300 on SPI: IC_IDENTITY=0x%02x (%s). Expect 0xB0-0xB7. Check SCLK=%d MISO=%d MOSI=%d "
                 "CS=%d RST=%d, VIO 3V3, VTX 5 V and GND. An ST25R3916 here would need the st25r3916_spi build.",
                 identity, esp_err_to_name(err), CONFIG_ST25R300_SPI_PIN_SCLK, CONFIG_ST25R300_SPI_PIN_MISO,
                 CONFIG_ST25R300_SPI_PIN_MOSI, CONFIG_ST25R300_SPI_PIN_CS, CONFIG_ST25R300_SPI_PIN_RST);
        return err;
    }
    ESP_LOGI(kTag, "ST25R300 identified over SPI: IC_IDENTITY=0x%02x (revision %u)", identity, identity & kIcRevMask);

    ESP_RETURN_ON_ERROR(command(kCmdSetDefault), kTag, "SET_DEFAULT failed");
    delay_ms(10);
    ESP_RETURN_ON_ERROR(read_identity(identity), kTag, "chip lost after SET_DEFAULT");
    g_ic_identity = identity;

    // Oscillator on, confirmed by its interrupt and by STATUS1 osc_ok.
    ESP_RETURN_ON_ERROR(irq_set_enabled(kIrqUsed), kTag, "IRQ mask write failed");
    irq_clear(0xFFFFFFFF);
    const uint32_t edges_before = irq_edge_count();
    ESP_RETURN_ON_ERROR(write_reg(kRegOperation, kOpEn), kTag, "OPERATION write failed");
    const uint32_t osc = irq_wait(kIrqOsc, 50);
    uint8_t status1 = 0;
    (void)read_reg(kRegStatus1, status1);
    if (!(status1 & kStatus1OscOk)) {
        ESP_LOGE(kTag, "oscillator did not start (STATUS1=0x%02x)", status1);
        return ESP_ERR_TIMEOUT;
    }
    if (CONFIG_ST25R300_SPI_PIN_IRQ < 0) {
        ESP_LOGI(kTag, "oscillator stable; no IRQ line configured, polling the IRQ registers");
    } else if (osc != 0 && irq_edge_count() > edges_before) {
        ESP_LOGI(kTag, "oscillator stable; IRQ line on GPIO %d verified", CONFIG_ST25R300_SPI_PIN_IRQ);
    } else {
        ESP_LOGW(kTag, "oscillator stable but no edge on IRQ GPIO %d; falling back to per-tick polling",
                 CONFIG_ST25R300_SPI_PIN_IRQ);
    }

    // Register read-back only once the oscillator runs: in power-down the chip answers
    // reads (IC_IDENTITY works) but register writes do not stick, which made this test
    // fail on the first bench run (6 Oct: wrote a5/5a, read 00/00). The ESPHome driver
    // never tests before enabling the oscillator.
    ESP_RETURN_ON_ERROR(spi_self_test(), kTag, "SPI register read-back failed at %d Hz", CONFIG_ST25R300_SPI_FREQ_HZ);

    // VDDDR regulator, then tx/rx on, then regulator adjustment.
    ESP_RETURN_ON_ERROR(write_reg(kRegOperation, kOpEn | kOpVdddrEn), kTag, "OPERATION write failed");
    delay_ms(1);
    ESP_RETURN_ON_ERROR(write_reg(kRegOperation, kOpAllOn), kTag, "OPERATION write failed");
    irq_clear(kIrqDct);
    ESP_RETURN_ON_ERROR(command(kCmdAdjustRegulators), kTag, "ADJUST_REGULATORS failed");
    if (irq_wait(kIrqDct, 20) == 0) {
        ESP_LOGW(kTag, "no direct-command-terminated interrupt after ADJUST_REGULATORS");
    }

    ESP_RETURN_ON_ERROR(write_and_check_profile(), kTag, "NFC-A profile write failed");
    ESP_RETURN_ON_ERROR(set_no_response_timer(kWupaTimeoutMs), kTag, "NRT write failed");

    ESP_RETURN_ON_ERROR(field_on(), kTag, "field on failed");

    uint8_t operation = 0;
    ESP_RETURN_ON_ERROR(read_reg(kRegOperation, operation), kTag, "OPERATION read failed");
    if ((operation & kOpAllOn) != kOpAllOn) {
        ESP_LOGE(kTag, "RF field did not come on: OPERATION=0x%02x", operation);
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t sense = sense_rf();
    ESP_LOGI(kTag, "RF field on: OPERATION=0x%02x SENSE_RF=%u", operation, sense);
    ESP_LOGI(kTag,
             "NFC-A poller ready: 106 kbit/s, FSDI=%u, SPI %d Hz, APDU budget %" PRIu32 " ms, FWT floor %" PRIu32
             " ms, WTX ceiling %" PRIu32 " ms, %" PRIu32 " attempt(s) per APDU",
             kIsoDepFsdi, CONFIG_ST25R300_SPI_FREQ_HZ, g_apdu_budget_ms.load(), g_fwt_floor_ms.load(),
             g_wtx_ceiling_ms.load(), g_apdu_attempts.load());

    g_chip_ready = true;
    g_last_health_us = esp_timer_get_time();
    return ESP_OK;
}

void log_status(const char *reason)
{
    ESP_LOGI(kTag,
             "NFC poller %s: polls=%" PRIu32 " tags=%" PRIu32 " enumerated=%" PRIu32 " multi_tag_polls=%" PRIu32
             " iso_dep=%" PRIu32 " rx_errors=%" PRIu32 " collisions=%" PRIu32 " apdu_failures=%" PRIu32
             " wtx=%" PRIu32 " reinits=%" PRIu32 " spi_errors=%" PRIu32 " irq_edges=%" PRIu32 " sense_rf=%u",
             reason, g_stats.polls, g_stats.tags, g_stats.enumerated, g_stats.multi_tag_polls, g_stats.iso_dep,
             g_stats.rx_errors, g_stats.collisions, g_stats.apdu_failures, g_stats.wtx_total, g_stats.reinits,
             spi_error_count(), irq_edge_count(), sense_rf());
}

void publish_last_apdu(const st25r300_apdu_stats_t &stats)
{
    portENTER_CRITICAL(&g_last_apdu_lock);
    g_last_apdu = stats;
    portEXIT_CRITICAL(&g_last_apdu_lock);
}

} // namespace

extern "C" {

void st25r300_set_apdu_budget_ms(uint32_t ms)
{
    g_apdu_budget_ms = std::max<uint32_t>(ms, 100);
}
uint32_t st25r300_get_apdu_budget_ms(void)
{
    return g_apdu_budget_ms.load();
}
void st25r300_set_fwt_floor_ms(uint32_t ms)
{
    g_fwt_floor_ms = std::min<uint32_t>(std::max<uint32_t>(ms, 1), kNrtMaxMs);
}
uint32_t st25r300_get_fwt_floor_ms(void)
{
    return g_fwt_floor_ms.load();
}
void st25r300_set_wtx_ceiling_ms(uint32_t ms)
{
    g_wtx_ceiling_ms = std::min<uint32_t>(std::max<uint32_t>(ms, 1), kNrtMaxMs);
}
uint32_t st25r300_get_wtx_ceiling_ms(void)
{
    return g_wtx_ceiling_ms.load();
}
void st25r300_set_apdu_attempts(uint8_t attempts)
{
    g_apdu_attempts = std::min<uint32_t>(std::max<uint32_t>(attempts, 1), 5);
}
uint8_t st25r300_get_apdu_attempts(void)
{
    return static_cast<uint8_t>(g_apdu_attempts.load());
}
void st25r300_set_frame_trace(bool on)
{
    g_frame_trace = on;
}
bool st25r300_get_frame_trace(void)
{
    return g_frame_trace.load();
}
void st25r300_get_last_apdu_stats(st25r300_apdu_stats_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&g_last_apdu_lock);
    *out = g_last_apdu;
    portEXIT_CRITICAL(&g_last_apdu_lock);
}

} // extern "C"

esp_err_t m5nfc_set_ecp_identifier(const uint8_t *identifier, size_t identifier_len)
{
    const bool clear = identifier == nullptr && identifier_len == 0;
    if (!clear && (!identifier || identifier_len != st25r3916::kEcpIdentifierSize)) {
        return ESP_ERR_INVALID_ARG;
    }
    std::array<uint8_t, st25r3916::kEcpFrameSize> frame = {};
    st25r3916::build_ecp_frame(clear ? nullptr : identifier, frame.data());
    portENTER_CRITICAL(&g_aliro_ecp_lock);
    g_aliro_ecp_frame = frame;
    g_aliro_ecp_enabled = !clear;
    portEXIT_CRITICAL(&g_aliro_ecp_lock);
    return ESP_OK;
}

esp_err_t m5nfc_init(void)
{
    const HalConfig config = hal_config();
    ESP_LOGI(kTag, "ST25R300 SPI reader: host %d SCLK=%d MISO=%d MOSI=%d CS=%d IRQ=%d RST=%d at %d Hz",
             config.spi_host, config.pin_sclk, config.pin_miso, config.pin_mosi, config.pin_cs, config.pin_irq,
             config.pin_rst, config.spi_freq_hz);
    ESP_RETURN_ON_ERROR(hal_init(config), kTag, "SPI/IRQ/RST setup failed");
    return chip_configure();
}

void m5nfc_update(void)
{
    const int64_t now = esp_timer_get_time();

    if (!g_chip_ready) {
        if (now >= g_next_reinit_us) {
            ++g_stats.reinits;
            ESP_LOGW(kTag, "reinitializing reader (attempt %" PRIu32 ")", g_stats.reinits);
            if (chip_configure() != ESP_OK) {
                g_next_reinit_us = now + kReinitBackoffUs;
            }
        }
        return;
    }

    if (now - g_last_health_us >= kHealthCheckIntervalUs) {
        g_last_health_us = now;
        uint8_t identity = 0;
        uint8_t operation = 0;
        const bool read_ok =
            read_reg(kRegIcIdentity, identity) == ESP_OK && read_reg(kRegOperation, operation) == ESP_OK;
        if (!read_ok || identity != g_ic_identity || (operation & kOpAllOn) != kOpAllOn) {
            ESP_LOGE(kTag, "reader health check failed: IC_IDENTITY=0x%02x (expected 0x%02x) OPERATION=0x%02x",
                     identity, g_ic_identity, operation);
            g_chip_ready = false;
            g_next_reinit_us = now;
            return;
        }
    }

    if (now >= g_next_status_log_us) {
        log_status(g_next_status_log_us == kFirstStatusLogUs ? "running" : "status");
        g_next_status_log_us = now + kStatusLogIntervalUs;
    }
}

m5nfc_activation_result_t m5nfc_activate(void)
{
    if (!g_chip_ready) {
        return M5NFC_ACTIVATION_NONE;
    }
    g_active = false;
    g_active_iso_dep = false;
    ++g_stats.polls;

    if (send_aliro_ecp()) {
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(kEcpSettleTimeMs)));
    }

    uint16_t atqa = 0;
    if (!poll_a(atqa)) {
        return M5NFC_ACTIVATION_NONE;
    }

    PollerOps ops;
    EnumeratedTag tags[kMaxTagsPerPoll] = {};
    EnumerationStats enumeration;
    const uint8_t tag_count = st25r3916::enumerate_tags(ops, tags, kMaxTagsPerPoll, enumeration);
    if (tag_count == 0) {
        ESP_LOGD(kTag, "ATQA=%04x seen but anticollision/SELECT did not complete", atqa);
        return M5NFC_ACTIVATION_NONE;
    }
    g_stats.enumerated += tag_count;
    char uid_str[2 * st25r3916::kMaxUidBytes + 1];

    if (tag_count > 1) {
        ++g_stats.multi_tag_polls;
        if (enumeration.hit_tag_cap) {
            ++g_stats.tag_cap_hits;
        }
        char line[kMaxTagsPerPoll * 40] = {};
        size_t pos = 0;
        for (uint8_t i = 0; i < tag_count && pos < sizeof(line); ++i) {
            format_uid(tags[i].uid, tags[i].uid_len, uid_str, sizeof(uid_str));
            pos += static_cast<size_t>(snprintf(line + pos, sizeof(line) - pos, "%s[%u] UID=%s SAK=%02x",
                                                i ? " " : "", i, uid_str, tags[i].sak));
        }
        ESP_LOGI(kTag, "NFC-A field holds %u tags%s: %s", tag_count,
                 enumeration.hit_tag_cap ? " (cap reached, there may be more)" : "", line);
    }

    // Prefer an ISO-DEP tag, rotating between several so none is shut out.
    uint8_t chosen = 0;
    uint8_t iso_dep_count = 0;
    for (uint8_t i = 0; i < tag_count; ++i) {
        if (st25r3916::tag_is_iso_dep(tags[i])) {
            ++iso_dep_count;
        }
    }
    if (iso_dep_count > 0) {
        const uint8_t pick = iso_dep_count > 1 ? static_cast<uint8_t>(g_iso_dep_rotor++ % iso_dep_count) : 0;
        uint8_t seen = 0;
        for (uint8_t i = 0; i < tag_count; ++i) {
            if (!st25r3916::tag_is_iso_dep(tags[i])) {
                continue;
            }
            if (seen == pick) {
                chosen = i;
                break;
            }
            ++seen;
        }
    }

    const uint8_t *uid = tags[chosen].uid;
    const uint8_t uid_len = tags[chosen].uid_len;
    const uint8_t sak = tags[chosen].sak;
    format_uid(uid, uid_len, uid_str, sizeof(uid_str));

    if (chosen + 1 != tag_count) {
        if (!reselect_tag(ops, tags[chosen])) {
            ESP_LOGW(kTag, "could not reselect UID=%s out of %u tags in the field", uid_str, tag_count);
            return M5NFC_ACTIVATION_NONE;
        }
    }
    ++g_stats.tags;

    if (!(sak & 0x20)) {
        g_active = true;
        ESP_LOGI(kTag, "NFC-A tag selected without ISO-DEP ATQA=%04x SAK=%02x UID=%s", atqa, sak, uid_str);
        return M5NFC_ACTIVATION_NON_ISO_DEP;
    }

    g_active = true;
    if (!rats(atqa, sak, uid_str)) {
        return M5NFC_ACTIVATION_NONE;
    }
    g_active_iso_dep = true;
    ++g_stats.iso_dep;
    ESP_LOGI(kTag, "NFC-A ISO-DEP activated ATQA=%04x SAK=%02x UID=%s", atqa, sak, uid_str);
    return M5NFC_ACTIVATION_ISO_DEP;
}

void m5nfc_deactivate(void)
{
    if (!g_chip_ready) {
        return;
    }
    const bool iso_dep = g_active_iso_dep;
    g_active = false;
    g_active_iso_dep = false;

    TxRxRequest req;
    uint8_t rx[4] = {};
    size_t rx_len = 0;
    if (iso_dep) {
        static constexpr uint8_t kDeselect[1] = {0xC2};
        req.tx = kDeselect;
        req.tx_bytes = sizeof(kDeselect);
        req.kind = FrameKind::kData;
        req.timeout_ms = kDeselectTimeoutMs;
    } else {
        static constexpr uint8_t kHlta[2] = {0x50, 0x00};
        req.tx = kHlta;
        req.tx_bytes = sizeof(kHlta);
        req.kind = FrameKind::kDataNoResponse;
    }
    (void)transceive(req, rx, sizeof(rx), rx_len);
    delay_ms(2);
}

esp_err_t m5nfc_message_exchange(const uint8_t *command_apdu, size_t command_len, uint8_t *response,
                                 size_t *response_len)
{
    ESP_RETURN_ON_FALSE(command_apdu && response && response_len, ESP_ERR_INVALID_ARG, kTag, "invalid APDU buffer");
    ESP_RETURN_ON_FALSE(command_len > 0 && command_len <= UINT16_MAX && *response_len <= UINT16_MAX,
                        ESP_ERR_INVALID_SIZE, kTag, "APDU buffer too large");
    ESP_RETURN_ON_FALSE(g_chip_ready && g_active_iso_dep, ESP_ERR_INVALID_STATE, kTag, "no ISO-DEP tag active");

    const int64_t apdu_start_us = esp_timer_get_time();
    WtxPolicy policy;
    policy.fwt_ms = effective_fwt_ms();
    policy.wtx_ceiling_ms = g_wtx_ceiling_ms.load();
    policy.budget_ms = g_apdu_budget_ms.load();
    policy.max_wtx = CONFIG_ST25R300_MAX_WTX;
    const unsigned max_attempts = std::max<uint32_t>(1, g_apdu_attempts.load());
    const uint8_t ins = command_len > 1 ? command_apdu[1] : 0;

    ESP_LOGD(kTag, "APDU TX len=%u cla=%02x ins=%02x FWT=%" PRIu32 " ms", static_cast<unsigned>(command_len),
             command_apdu[0], ins, policy.fwt_ms);

    st25r300_apdu_stats_t summary = {};
    summary.fwt_ms = policy.fwt_ms;
    FrameStats frame_stats;
    RxStatus last_status = RxStatus::kOk;
    int64_t next_progress_us = apdu_start_us + kWtxProgressLogIntervalUs;

    auto elapsed = [apdu_start_us]() -> uint32_t {
        return static_cast<uint32_t>((esp_timer_get_time() - apdu_start_us) / 1000);
    };
    auto raw_tx = [&last_status, &elapsed](const uint8_t *frame, uint8_t frame_len, uint8_t *raw, uint8_t &raw_len,
                                          uint32_t timeout_ms) -> bool {
        trace_frame(">", frame, frame_len, elapsed());
        TxRxRequest req;
        req.tx = frame;
        req.tx_bytes = frame_len;
        req.kind = FrameKind::kData;
        req.timeout_ms = timeout_ms;
        size_t n = 0;
        // FSD 256 includes the two CRC bytes, so a legal frame is at most 254.
        last_status = transceive(req, raw, 254, n);
        if (last_status != RxStatus::kOk || n < 1) {
            if (g_frame_trace.load()) {
                ESP_LOGI(kTag, "FRAME < t=%" PRIu32 " ms no frame: %s after waiting up to %" PRIu32 " ms",
                         elapsed(), rx_status_name(last_status), timeout_ms);
            }
            return false;
        }
        raw_len = static_cast<uint8_t>(n);
        trace_frame("<", raw, n, elapsed());
        return true;
    };
    auto on_wtx = [&next_progress_us, ins](uint32_t count, uint8_t wtxm, uint32_t wait_ms, uint32_t elapsed_ms) {
        ++g_stats.wtx_total;
        if (g_frame_trace.load()) {
            ESP_LOGI(kTag, "WTX #%" PRIu32 " WTXM=%u: granting, waiting up to %" PRIu32 " ms (t=%" PRIu32 " ms)",
                     count, wtxm, wait_ms, elapsed_ms);
            return;
        }
        const int64_t now = esp_timer_get_time();
        if (now >= next_progress_us) {
            next_progress_us = now + kWtxProgressLogIntervalUs;
            ESP_LOGI(kTag, "card still working on INS %02x: %" PRIu32 " ms so far, %" PRIu32 " WTX granted", ins,
                     elapsed_ms, count);
        }
    };
    auto tx = [&](const uint8_t *frame, uint8_t frame_len, uint8_t *raw, uint8_t &raw_len) -> bool {
        return send_frame_absorbing_wtx(frame, frame_len, raw, raw_len, policy, frame_stats, raw_tx, elapsed,
                                        on_wtx);
    };

    for (unsigned attempt = 1; attempt <= max_attempts; ++attempt) {
        const uint8_t block_before = g_block_number;
        size_t received = 0;
        summary.attempts = static_cast<uint8_t>(attempt);
        if (esphome::st25r::isodep_exchange_apdu(command_apdu, command_len, response, *response_len, received,
                                                 g_block_number, g_fsc, tx)) {
            *response_len = received;
            summary.ok = true;
            break;
        }
        g_block_number = block_before;
        ++g_stats.apdu_failures;
        ESP_LOGW(kTag,
                 "APDU INS %02x attempt %u/%u failed after %" PRIu32 " ms: last frame %s, %" PRIu32
                 " WTX granted%s%s (FWT %" PRIu32 " ms, budget %" PRIu32 " ms)",
                 ins, attempt, max_attempts, elapsed(), rx_status_name(last_status), frame_stats.wtx,
                 frame_stats.budget_exceeded ? ", APDU BUDGET EXCEEDED" : "",
                 frame_stats.wtx_limit ? ", WTX LIMIT HIT" : "", policy.fwt_ms, policy.budget_ms);
        if (frame_stats.budget_exceeded) {
            break;  // the budget covers every attempt together
        }
    }

    summary.elapsed_ms = elapsed();
    summary.frames = frame_stats.frames;
    summary.wtx = frame_stats.wtx;
    summary.max_wtxm = frame_stats.max_wtxm;
    summary.longest_wait_ms = frame_stats.longest_wait_ms;
    summary.budget_exceeded = frame_stats.budget_exceeded;
    summary.wtx_limit = frame_stats.wtx_limit;
    summary.last_frame = rx_status_name(last_status);
    publish_last_apdu(summary);

    if (summary.ok) {
        ESP_LOGD(kTag, "APDU RX len=%u in %" PRIu32 " ms, %" PRIu32 " frames, %" PRIu32 " WTX",
                 static_cast<unsigned>(*response_len), summary.elapsed_ms, summary.frames, summary.wtx);
        return ESP_OK;
    }
    return summary.budget_exceeded ? ESP_ERR_TIMEOUT : ESP_FAIL;
}
