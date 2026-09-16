/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25R3916 / ST25R3916B NFC-A poller over SPI, exposing the m5nfc interface
 * that esp-matter's Aliro door lock delegate calls. See include/m5nfc.h.
 *
 * Sources for the register sequences:
 *  - ST RFAL (rfal_rfst25r3916.cpp, rfal_nfca.cpp, analog config table) for
 *    the transceive state machine, interrupt handling, anticollision and the
 *    NFC-A 106 kbit/s analog profile.
 *  - John McLear's ESPHome ST25R3916 driver, verified on the STEVAL-MB17149B,
 *    for the bring-up order, the antenna tuning search and the hard rules in
 *    its DEVELOPMENT_STATE.md: RX_CONF3 = 0x00 on the B chip, and
 *    RESET_RX_GAIN before every transceive.
 *  - The m5nfc component at 0d00697 plus patch 0005 for the behaviour the door
 *    lock was verified against: FSDI 8, CID off, an FWT floor of
 *    CONFIG_ST25R3916_TRANSCEIVE_TIMEOUT_MS, a WTX ceiling of 5 s, three APDU
 *    attempts, DESELECT or HLTA on deactivate, and the Aliro ECP frame with
 *    its 12 ms response window and 5 ms settle before the polling command.
 *    Polling itself is WUPA, not m5nfc's REQA: see poll_a().
 */

#include <m5nfc.h>

#include "isodep_apdu.h"
#include "st25r3916_frame_logic.h"
#include "st25r3916_hal.h"
#include "st25r3916_regs.h"

#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>

using namespace st25r3916;

namespace {

constexpr char kTag[] = "st25r3916";

// ISO-DEP policy carried over from m5nfc.
constexpr uint8_t kIsoDepFsdi = 8;  // FSD 256
constexpr uint32_t kWtxMaxMs = 5000;
constexpr unsigned kApduMaxAttempts = 3;

// Aliro ECP, identical to patch 0005.
constexpr uint32_t kEcpResponseTimeoutMs = 12;
constexpr uint32_t kEcpSettleTimeMs = 5;

// NFC-A timeouts. These are the chip's no-response timer, measured from the
// end of our transmission, so a missing tag costs exactly this long.
constexpr uint32_t kWupaTimeoutMs = 5;
constexpr uint32_t kAnticollisionTimeoutMs = 8;
constexpr uint32_t kSelectTimeoutMs = 8;
constexpr uint32_t kRatsTimeoutMs = 10;
constexpr uint32_t kDeselectTimeoutMs = 8;

// Mask receive timer after our own transmission, in 64/fc steps. RFAL uses
// FDT(listen, NFC-A poller) 1172/fc less its 276/fc adjustment: 896/64 = 14.
constexpr uint8_t kMaskReceiveTimer = 14;

// RFAL NFC-A 106 kbit/s correlator setting, and the anticollision variant with
// corr_s6 cleared so a collision is detected at a different level from data.
constexpr uint8_t kCorrConf1Data = 0x51;
constexpr uint8_t kCorrConf1Anticollision = 0x11;

// Housekeeping in m5nfc_update().
constexpr int64_t kHealthCheckIntervalUs = 5 * 1000 * 1000;
constexpr int64_t kReinitBackoffUs = 5 * 1000 * 1000;
constexpr int64_t kFirstStatusLogUs = 30 * 1000 * 1000;
constexpr int64_t kStatusLogIntervalUs = 10 * 60 * 1000 * 1000LL;
constexpr int64_t kCollisionLogIntervalUs = 5 * 1000 * 1000;

constexpr uint32_t kIrqUsed =
    kIrqTxe | kIrqRxs | kIrqRxe | kIrqCol | kIrqNre | kIrqDct | kIrqErrors;

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

struct TxRxRequest {
    const uint8_t *tx = nullptr;   // nullptr with direct_cmd set: REQA/WUPA
    uint16_t tx_bytes = 0;         // whole bytes
    uint8_t tx_bits = 0;           // extra bits in a trailing partial byte
    uint8_t direct_cmd = 0;        // kCmdTransmitWupa etc. when tx == nullptr
    bool tx_crc = true;
    bool rx_crc = true;            // chip checks CRC; driver strips it
    bool antcol = false;
    bool expect_response = true;
    uint32_t timeout_ms = 10;
};

// Chip and link state. Owned by the nfc_detect task, see st25r3916_hal.h.
bool g_chip_ready = false;
uint8_t g_ic_identity = 0;
uint8_t g_aux_shadow = 0;
uint8_t g_rx_frame[kFifoDepth];

bool g_active = false;
bool g_active_iso_dep = false;
uint8_t g_block_number = 0;
uint16_t g_fsc = 32;
uint32_t g_fwt_ms = CONFIG_ST25R3916_TRANSCEIVE_TIMEOUT_MS;

int64_t g_last_health_us = 0;
int64_t g_next_reinit_us = 0;
int64_t g_next_status_log_us = kFirstStatusLogUs;

struct Stats {
    uint32_t polls = 0;
    uint32_t tags = 0;
    uint32_t iso_dep = 0;
    uint32_t rx_errors = 0;
    uint32_t apdu_failures = 0;
    uint32_t reinits = 0;
    uint32_t collisions = 0;
} g_stats;

int64_t g_next_collision_log_us = 0;

// ECP state is written from the Matter task and read by the NFC task.
std::array<uint8_t, kEcpFrameSize> g_aliro_ecp_frame = {
    0x6A, 0x02, 0xCB, 0x02, 0x06, 0x20, 0x42, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
bool g_aliro_ecp_enabled = false;
portMUX_TYPE g_aliro_ecp_lock = portMUX_INITIALIZER_UNLOCKED;

HalConfig hal_config()
{
    HalConfig c = {};
    c.spi_host = CONFIG_ST25R3916_SPI_HOST;
    c.pin_sclk = CONFIG_ST25R3916_SPI_PIN_SCLK;
    c.pin_miso = CONFIG_ST25R3916_SPI_PIN_MISO;
    c.pin_mosi = CONFIG_ST25R3916_SPI_PIN_MOSI;
    c.pin_cs = CONFIG_ST25R3916_SPI_PIN_CS;
    c.pin_irq = CONFIG_ST25R3916_SPI_PIN_IRQ;
    c.spi_freq_hz = CONFIG_ST25R3916_SPI_FREQ_HZ;
    return c;
}

void delay_ms(uint32_t ms)
{
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(ms)));
}

esp_err_t set_aux(uint8_t value)
{
    if (value == g_aux_shadow) {
        return ESP_OK;
    }
    esp_err_t err = write_reg(kRegAux, value);
    if (err == ESP_OK) {
        g_aux_shadow = value;
    }
    return err;
}

esp_err_t set_no_response_timer(uint32_t ms)
{
    bool step_4096 = false;
    uint16_t value = 0;
    nrt_from_ms(ms, step_4096, value);
    // Keep nrt_emv off: in normal mode the timer is a pure "no answer started"
    // deadline, and RXS stops it.
    esp_err_t err = write_reg(kRegTimerEmvControl, step_4096 ? kTimerEmvNrtStep4096 : 0x00);
    if (err == ESP_OK) {
        err = write_reg(kRegNoResponseTimer1, static_cast<uint8_t>(value >> 8));
    }
    if (err == ESP_OK) {
        err = write_reg(kRegNoResponseTimer2, static_cast<uint8_t>(value & 0xFF));
    }
    return err;
}

// Run one direct command that ends with a "direct command terminated" IRQ and
// read its A/D result.
bool command_and_read_ad(uint8_t cmd, uint8_t &result)
{
    irq_clear(kIrqDct);
    if (command(cmd) != ESP_OK) {
        return false;
    }
    if (irq_wait(kIrqDct, 100) == 0) {
        return false;
    }
    return read_reg(kRegAdResult, result) == ESP_OK;
}

RxStatus transceive(const TxRxRequest &req, uint8_t *rx, size_t rx_capacity, size_t &rx_len, uint8_t *rx_last_bits)
{
    rx_len = 0;
    if (rx_last_bits) {
        *rx_last_bits = 0;
    }

    // RFAL rfalPrepareTransceive: STOP resets the receive logic and clears the
    // FIFO, then RESET_RX_GAIN puts AGC and squelch back to their start state.
    if (command(kCmdStop) != ESP_OK || command(kCmdResetRxGain) != ESP_OK) {
        return RxStatus::kIo;
    }
    const uint8_t aux = static_cast<uint8_t>((g_aux_shadow & ~kAuxNoCrcRx) | (req.rx_crc ? 0 : kAuxNoCrcRx));
    if (write_reg(kRegIso14443aNfc, req.antcol ? kIso14443aAntcl : 0x00) != ESP_OK || set_aux(aux) != ESP_OK ||
        set_no_response_timer(req.timeout_ms) != ESP_OK) {
        return RxStatus::kIo;
    }
    irq_clear(kIrqUsed);

    uint8_t ntx1 = 0;
    uint8_t ntx2 = 0;
    if (req.tx != nullptr) {
        encode_num_tx(req.tx_bytes, req.tx_bits, ntx1, ntx2);
        const size_t fifo_bytes = req.tx_bytes + (req.tx_bits ? 1u : 0u);
        if (write_reg(kRegNumTxBytes1, ntx1) != ESP_OK || write_reg(kRegNumTxBytes2, ntx2) != ESP_OK ||
            fifo_write(req.tx, fifo_bytes) != ESP_OK ||
            command(req.tx_crc ? kCmdTransmitWithCrc : kCmdTransmitWithoutCrc) != ESP_OK) {
            return RxStatus::kIo;
        }
    } else {
        // RFAL: clear nbtx before REQA/WUPA or the chip reports a parity error.
        if (write_reg(kRegNumTxBytes2, 0x00) != ESP_OK || command(req.direct_cmd) != ESP_OK) {
            return RxStatus::kIo;
        }
    }

    // 106 kbit/s is about 9.4 ms per 100 bytes on air, parity included.
    const uint32_t tx_timeout_ms = 20 + (req.tx_bytes / 8u);
    if (irq_wait(kIrqTxe, tx_timeout_ms) == 0) {
        ESP_LOGW(kTag, "no TXE after %" PRIu32 " ms", tx_timeout_ms);
        return RxStatus::kIo;
    }
    if (!req.expect_response) {
        return RxStatus::kOk;
    }

    // The chip's no-response timer is authoritative. The software deadline is a
    // backstop in case its interrupt is lost.
    uint32_t hit = irq_wait(kIrqRxs | kIrqRxe | kIrqNre, req.timeout_ms + 50);
    if ((hit & (kIrqRxs | kIrqRxe)) == 0) {
        return RxStatus::kTimeout;
    }
    if ((hit & kIrqRxe) == 0) {
        // RFAL waits RFAL_NORXE_TOUT (50 ms) for RXE after RXS: ST25R3916
        // errata, a corrupted frame can raise RXS without RXE. A full 258-byte
        // frame takes about 25 ms on air.
        hit |= irq_wait(kIrqRxe, 50 + static_cast<uint32_t>(rx_capacity / 8));
        if ((hit & kIrqRxe) == 0) {
            ++g_stats.rx_errors;
            return RxStatus::kFraming;
        }
    }
    const uint32_t errors = irq_take(kIrqErrors | kIrqCol);

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
    if (errors & (kIrqErr1 | kIrqErr2)) {
        status = RxStatus::kFraming;
    } else if (errors & kIrqPar) {
        status = RxStatus::kParity;
    } else if (errors & kIrqCrc) {
        status = RxStatus::kCrc;
    } else if (errors & kIrqCol) {
        status = RxStatus::kCollision;
    } else if (last_bits != 0 && !req.antcol) {
        status = RxStatus::kIncomplete;
    } else if ((st2 & kFifoStatus2NpLb) && !req.antcol) {
        status = RxStatus::kFraming;
    }

    // With the chip's CRC check on, the two CRC bytes still land in the FIFO
    // (RFAL strips them unless asked to keep them). Strip them here.
    size_t payload = count;
    if (req.rx_crc && status == RxStatus::kOk) {
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
    if (rx_last_bits) {
        *rx_last_bits = last_bits;
    }
    if (status != RxStatus::kOk && status != RxStatus::kCollision) {
        ++g_stats.rx_errors;
    }
    return status;
}

bool send_aliro_ecp()
{
    std::array<uint8_t, kEcpFrameSize> frame = {};
    bool enabled = false;
    portENTER_CRITICAL(&g_aliro_ecp_lock);
    frame = g_aliro_ecp_frame;
    enabled = g_aliro_ecp_enabled;
    portEXIT_CRITICAL(&g_aliro_ecp_lock);
    if (!enabled) {
        return false;
    }

    // ECP is a broadcast; a reply is neither expected nor used. Transmit the
    // 16 bytes with CRC-A appended by the chip, then give a phone the same
    // 12 ms response window the m5nfc path gave it.
    TxRxRequest req;
    req.tx = frame.data();
    req.tx_bytes = static_cast<uint16_t>(frame.size());
    req.tx_crc = true;
    req.rx_crc = true;
    req.timeout_ms = kEcpResponseTimeoutMs;
    uint8_t response[1] = {};
    size_t response_len = 0;
    (void)transceive(req, response, sizeof(response), response_len, nullptr);
    return true;
}

// Poll for a PICC with WUPA, not REQA.
//
// m5nfc_deactivate() ends every successful activation with HLTA (or DESELECT,
// which also leaves the PICC halted). ISO/IEC 14443-3 6.3 says a PICC in the
// HALT state answers WUPA only: REQA wakes IDLE PICCs and nothing else. With
// REQA the reader therefore reads each credential exactly once and is then
// deaf to it until it physically leaves the field and comes back, so a tap
// that needs a second attempt can never get one. On the bench that showed up
// as two reads in 302 polls; with WUPA the same card reads on every poll.
bool poll_a(uint16_t &atqa)
{
    TxRxRequest req;
    req.direct_cmd = kCmdTransmitWupa;
    req.tx_crc = false;
    req.rx_crc = false;
    req.timeout_ms = kWupaTimeoutMs;
    uint8_t rx[2] = {};
    size_t rx_len = 0;
    const RxStatus status = transceive(req, rx, sizeof(rx), rx_len, nullptr);
    if (status == RxStatus::kTimeout || rx_len == 0) {
        return false;
    }
    // Several tags answering at once still means a tag is there; anticollision
    // sorts them out. Keep the value, even if it is only approximate.
    atqa = static_cast<uint16_t>((rx_len > 1 ? (rx[1] << 8) : 0) | rx[0]);
    return status == RxStatus::kOk || status == RxStatus::kCollision || status == RxStatus::kParity ||
        status == RxStatus::kFraming || status == RxStatus::kCrc;
}

// A collision means two or more PICCs answered the same anticollision frame,
// so the UID this poll ends up reporting is one of several. Say so, throttled,
// because silently picking a branch looks exactly like a corrupted UID from
// outside the driver.
void report_collision(uint8_t level, uint8_t collision_status)
{
    const int64_t now = esp_timer_get_time();
    if (now < g_next_collision_log_us) {
        return;
    }
    g_next_collision_log_us = now + kCollisionLogIntervalUs;
    ESP_LOGI(kTag,
             "NFC-A CL%u collision at frame byte %u bit %u (COLLISION_STATUS=0x%02x): more than one tag is in the "
             "field. Taking the branch with a 1; the UID reported is one of them, not the only one.",
             level + 1, static_cast<unsigned>((collision_status >> 4) & 0x0F),
             static_cast<unsigned>((collision_status >> 1) & 0x07), collision_status);
}

// One cascade level of anticollision and SELECT, following
// rfalNfcaPollerSingleCollisionResolution: on a collision, take the tag that
// has a 1 at the collision bit.
bool select_cascade_level(uint8_t level, uint8_t uid_part[4], uint8_t &sak)
{
    constexpr uint8_t kSelCmd[3] = {0x93, 0x95, 0x97};
    uint8_t frame[7] = {kSelCmd[level], 0x00, 0, 0, 0, 0, 0};
    uint8_t bytes = 2;
    uint8_t bits = 0;

    (void)write_reg(kRegCorrConf1, kCorrConf1Anticollision);
    bool resolved = false;
    for (int iteration = 0; iteration < 32; ++iteration) {
        frame[1] = sel_par(bytes, bits);
        const uint8_t sent_split = bits ? frame[bytes] : 0;

        TxRxRequest req;
        req.tx = frame;
        req.tx_bytes = bytes;
        req.tx_bits = bits;
        req.tx_crc = false;
        req.rx_crc = false;
        req.antcol = true;
        req.timeout_ms = kAnticollisionTimeoutMs;
        size_t rx_len = 0;
        const RxStatus status = transceive(req, frame + bytes, sizeof(frame) - bytes, rx_len, nullptr);
        if (bits) {
            frame[bytes] = merge_split_byte(frame[bytes], sent_split, bits);
        }

        if (status == RxStatus::kCollision) {
            uint8_t coll = 0;
            if (read_reg(kRegCollisionStatus, coll) != ESP_OK) {
                break;
            }
            const AnticollisionPrefix next = next_anticollision_prefix(coll, frame);
            if (!next.valid) {
                ESP_LOGD(kTag, "anticollision: collision position out of range (0x%02x)", coll);
                break;
            }
            ++g_stats.collisions;
            report_collision(level, coll);
            bytes = next.bytes;
            bits = next.bits;
            continue;
        }
        if (status != RxStatus::kOk || bytes + rx_len != sizeof(frame)) {
            ESP_LOGD(kTag, "anticollision CL%u failed: %s rx_len=%u", level + 1, rx_status_name(status),
                     static_cast<unsigned>(rx_len));
            break;
        }
        resolved = true;
        break;
    }
    (void)write_reg(kRegCorrConf1, kCorrConf1Data);
    if (!resolved) {
        return false;
    }
    if (bcc(frame + 2, 4) != frame[6]) {
        ESP_LOGD(kTag, "anticollision CL%u: BCC mismatch", level + 1);
        return false;
    }

    frame[1] = 0x70;  // SELECT: 7 whole bytes
    TxRxRequest req;
    req.tx = frame;
    req.tx_bytes = sizeof(frame);
    req.tx_crc = true;
    req.rx_crc = true;
    req.timeout_ms = kSelectTimeoutMs;
    uint8_t sak_buf[1] = {};
    size_t sak_len = 0;
    const RxStatus status = transceive(req, sak_buf, sizeof(sak_buf), sak_len, nullptr);
    if (status != RxStatus::kOk || sak_len != 1) {
        ESP_LOGD(kTag, "SELECT CL%u failed: %s len=%u", level + 1, rx_status_name(status),
                 static_cast<unsigned>(sak_len));
        return false;
    }
    std::memcpy(uid_part, frame + 2, 4);
    sak = sak_buf[0];
    return true;
}

bool select_a(uint8_t *uid, uint8_t &uid_len, uint8_t &sak)
{
    uid_len = 0;
    for (uint8_t level = 0; level < 3; ++level) {
        uint8_t part[4] = {};
        if (!select_cascade_level(level, part, sak)) {
            return false;
        }
        if (sak & 0x04) {  // UID not complete: part[0] is the cascade tag
            std::memcpy(uid + uid_len, part + 1, 3);
            uid_len = static_cast<uint8_t>(uid_len + 3);
            continue;
        }
        std::memcpy(uid + uid_len, part, 4);
        uid_len = static_cast<uint8_t>(uid_len + 4);
        return true;
    }
    return false;
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
    const uint8_t rats_cmd[2] = {0xE0, rats_param(kIsoDepFsdi, 0)};
    TxRxRequest req;
    req.tx = rats_cmd;
    req.tx_bytes = sizeof(rats_cmd);
    req.timeout_ms = kRatsTimeoutMs;
    uint8_t ats[64] = {};
    size_t ats_len = 0;
    const RxStatus status = transceive(req, ats, sizeof(ats), ats_len, nullptr);
    AtsInfo info;
    if (status != RxStatus::kOk || ats_len < 1 || !parse_ats(ats, ats_len, info)) {
        ESP_LOGW(kTag, "RATS failed: %s ats_len=%u (ATQA=%04x SAK=%02x UID=%s)", rx_status_name(status),
                 static_cast<unsigned>(ats_len), atqa, sak, uid_str);
        return false;
    }

    const uint32_t ats_fwt_ms = fwi_to_ms(info.fwi);
    g_fwt_ms = std::max<uint32_t>(ats_fwt_ms ? ats_fwt_ms : 1, CONFIG_ST25R3916_TRANSCEIVE_TIMEOUT_MS);
    g_fsc = info.fsc;
    g_block_number = 0;

    const uint32_t sfgt_ms = sfgi_to_ms(info.sfgi);
    if (sfgt_ms > 0) {
        delay_ms(sfgt_ms);
    }
    ESP_LOGI(kTag, "ATS len=%u FSCI=%u ISO-DEP FSC=%u FSDI=%u FWT=%" PRIu32 " ms SFGT=%" PRIu32 " ms",
             static_cast<unsigned>(ats_len), ats_len > 1 ? (ats[1] & 0x0F) : 2, g_fsc, kIsoDepFsdi, g_fwt_ms,
             sfgt_ms);
    return true;
}

void measure_field(uint8_t &amplitude)
{
    amplitude = 0;
    (void)command_and_read_ad(kCmdMeasureAmplitude, amplitude);
}

// Antenna tuning search, from the Yondoor driver (RFAL AN5322 approach):
// hill-climb ANT_TUNE_A/B for the largest field amplitude. Worth about 10 mm
// of range on the STEVAL-MB17149B's varicap-tuned antenna.
void tune_antenna(uint8_t &best_a, uint8_t &best_b, uint8_t &best_amp)
{
    uint8_t a = 0x80;
    uint8_t b = 0x40;
    int step = 16;
    (void)write_reg(kRegAntTuneA, a);
    (void)write_reg(kRegAntTuneB, b);
    measure_field(best_amp);
    best_a = a;
    best_b = b;

    for (int iteration = 0; iteration < 20 && step > 0; ++iteration) {
        const int dirs[4][2] = {{step, 0}, {-step, 0}, {0, step}, {0, -step}};
        bool improved = false;
        for (const auto &d : dirs) {
            const int na = a + d[0];
            const int nb = b + d[1];
            if (na < 0 || na > 255 || nb < 0 || nb > 255) {
                continue;
            }
            (void)write_reg(kRegAntTuneA, static_cast<uint8_t>(na));
            (void)write_reg(kRegAntTuneB, static_cast<uint8_t>(nb));
            uint8_t amp = 0;
            measure_field(amp);
            if (amp > best_amp) {
                best_amp = amp;
                best_a = static_cast<uint8_t>(na);
                best_b = static_cast<uint8_t>(nb);
                improved = true;
            }
        }
        if (improved) {
            a = best_a;
            b = best_b;
        } else {
            step /= 2;
        }
    }
    (void)write_reg(kRegAntTuneA, best_a);
    (void)write_reg(kRegAntTuneB, best_b);
}

struct RegValue {
    uint8_t reg;
    uint8_t value;
};

// NFC-A 106 kbit/s poller profile. RFAL chip-init, poll-common and NFC-A
// TX/RX 106 analog config, which is also what the Yondoor driver writes.
// RX_CONF3 stays 0x00: lf_en=1 (0xE2) blocks every ATQA on the ST25R3916B's
// tuned antenna (Yondoor DEVELOPMENT_STATE.md, bug 5).
const RegValue kAnalogProfile[] = {
    {kRegIoConf1, 0x07},                  // MCU_CLK and LF clock off
    {kRegMode, kModeIso14443a},           // NFC-A initiator, OOK
    {kRegBitRate, 0x00},                  // 106 kbit/s both ways
    {kRegIso14443aNfc, 0x00},
    {kRegPassiveTarget, 0x50},            // fdel = 5
    {kRegRxConf1, 0x08},
    {kRegRxConf2, 0x2D},
    {kRegRxConf3, 0x00},
    {kRegRxConf4, 0x00},
    {kRegMaskRxTimer, kMaskReceiveTimer},
    {kRegTxDriver, 0x00},                 // d_res 0: lowest driver resistance
    {kRegPtMod, 0x51},
    {kRegFieldThresholdActv, 0x11},
    {kRegFieldThresholdDeactv, 0x00},
    {kRegEmdSupConf, 0x40},               // rx_start_emv
    {kRegAuxMod, 0x10},                   // lm_dri
    {kRegResAmMod, 0x80},
    {kRegOvershootConf1, 0x40},
    {kRegOvershootConf2, 0x03},
    {kRegUndershootConf1, 0x40},
    {kRegUndershootConf2, 0x03},
    {kRegCorrConf1, kCorrConf1Data},
    {kRegCorrConf2, 0x00},
};

esp_err_t write_and_verify_profile()
{
    for (const auto &rv : kAnalogProfile) {
        ESP_RETURN_ON_ERROR(write_reg(rv.reg, rv.value), kTag, "write 0x%02x failed", rv.reg);
    }
    // Read everything back. A space-B write that went to the wrong space would
    // show up here as a space-A register holding a space-B value.
    int mismatches = 0;
    for (const auto &rv : kAnalogProfile) {
        uint8_t got = 0;
        ESP_RETURN_ON_ERROR(read_reg(rv.reg, got), kTag, "read 0x%02x failed", rv.reg);
        if (got != rv.value) {
            ESP_LOGE(kTag, "register %s 0x%02x reads 0x%02x, wrote 0x%02x", (rv.reg & kSpaceB) ? "B" : "A",
                     rv.reg & 0x3F, got, rv.value);
            ++mismatches;
        }
    }
    return mismatches == 0 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t spi_self_test()
{
    constexpr uint8_t patterns[] = {0xA5, 0x5A, 0xFF, 0x00};
    for (uint8_t p : patterns) {
        uint8_t got1 = 0;
        uint8_t got2 = 0;
        ESP_RETURN_ON_ERROR(write_reg(kRegNoResponseTimer1, p), kTag, "SPI write failed");
        ESP_RETURN_ON_ERROR(write_reg(kRegNoResponseTimer2, static_cast<uint8_t>(~p)), kTag, "SPI write failed");
        ESP_RETURN_ON_ERROR(read_reg(kRegNoResponseTimer1, got1), kTag, "SPI read failed");
        ESP_RETURN_ON_ERROR(read_reg(kRegNoResponseTimer2, got2), kTag, "SPI read failed");
        if (got1 != p || got2 != static_cast<uint8_t>(~p)) {
            ESP_LOGE(kTag, "SPI self-test: wrote %02x/%02x, read %02x/%02x", p, static_cast<uint8_t>(~p), got1, got2);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_OK;
}

// RFAL st25r3916PerformCollisionAvoidance(INITIAL_RF_COLLISION) followed by
// st25r3916TxRxOn.
esp_err_t field_on()
{
    ESP_RETURN_ON_ERROR(write_reg(kRegFieldOnGt, 0x00), kTag, "FIELD_ON_GT write failed");
    ESP_RETURN_ON_ERROR(modify_reg(kRegOpControl, kOpControlEnFdMask, kOpControlEnFdManualCa), kTag,
                        "OP_CONTROL write failed");
    ESP_RETURN_ON_ERROR(modify_reg(kRegAux, kAuxNfcNMask, 0x00), kTag, "AUX write failed");
    ESP_RETURN_ON_ERROR(irq_set_enabled(kIrqUsed | kIrqCac | kIrqCat | kIrqApon), kTag, "IRQ mask write failed");
    irq_clear(kIrqCac | kIrqCat | kIrqApon | kIrqEon | kIrqEof);

    ESP_RETURN_ON_ERROR(command(kCmdInitialRfCollision), kTag, "field-on command failed");
    uint32_t hit = irq_wait(kIrqCac | kIrqApon, 20);
    esp_err_t result = ESP_OK;
    if (hit & kIrqCac) {
        ESP_LOGE(kTag, "field on refused: another RF field is present (collision avoidance)");
        result = ESP_ERR_INVALID_STATE;
    } else if (hit & kIrqApon) {
        if (irq_wait(kIrqCat, 20) == 0) {
            ESP_LOGW(kTag, "field on: no guard-time interrupt after APON");
        }
    } else {
        ESP_LOGE(kTag, "field on: no APON or CAC interrupt from collision avoidance");
        result = ESP_ERR_TIMEOUT;
    }

    (void)modify_reg(kRegOpControl, kOpControlEnFdMask, kOpControlEnFdAuto);
    (void)irq_take(kIrqEon | kIrqEof);
    (void)irq_set_enabled(kIrqUsed);
    if (result != ESP_OK) {
        return result;
    }
    ESP_RETURN_ON_ERROR(modify_reg(kRegOpControl, 0, kOpControlRxEn | kOpControlTxEn), kTag, "field on failed");
    return ESP_OK;
}

esp_err_t read_identity(uint8_t &identity)
{
    identity = 0;
    esp_err_t err = ESP_FAIL;
    // A warm reboot can catch the chip mid-transaction; retry briefly.
    for (int attempt = 0; attempt < 5; ++attempt) {
        err = read_reg(kRegIcIdentity, identity);
        const uint8_t type = identity & kIcTypeMask;
        if (err == ESP_OK && (type == kIcTypeSt25r3916b || type == kIcTypeSt25r3916)) {
            return ESP_OK;
        }
        delay_ms(20);
    }
    return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
}

esp_err_t chip_configure()
{
    g_chip_ready = false;
    g_active = false;
    g_active_iso_dep = false;

    uint8_t identity = 0;
    esp_err_t err = read_identity(identity);
    if (err != ESP_OK) {
        ESP_LOGE(kTag,
                 "no ST25R3916 on SPI: IC_IDENTITY=0x%02x (%s). Expect 0x30-0x37 for ST25R3916B. Check SCLK=%d "
                 "MISO=%d MOSI=%d CS=%d and 3V3/GND",
                 identity, esp_err_to_name(err), CONFIG_ST25R3916_SPI_PIN_SCLK, CONFIG_ST25R3916_SPI_PIN_MISO,
                 CONFIG_ST25R3916_SPI_PIN_MOSI, CONFIG_ST25R3916_SPI_PIN_CS);
        return err;
    }
    const uint8_t type = identity & kIcTypeMask;
    ESP_LOGI(kTag, "%s identified over SPI: IC_IDENTITY=0x%02x (type 0x%02x, revision %u)",
             type == kIcTypeSt25r3916b ? "ST25R3916B" : "ST25R3916", identity, type, identity & kIcRevMask);
    if (type != kIcTypeSt25r3916b) {
        ESP_LOGW(kTag, "this build is tuned for the ST25R3916B (STEVAL-MB17149B); the analog profile is the "
                       "RFAL default and should also work on ST25R3916, but that is not verified");
    }

    // Leave whatever a previous firmware was doing, then reset registers.
    (void)command(kCmdStop);
    (void)write_reg(kRegOpControl, 0x00);
    ESP_RETURN_ON_ERROR(command(kCmdSetDefault), kTag, "SET_DEFAULT failed");
    delay_ms(10);
    g_aux_shadow = 0;
    ESP_RETURN_ON_ERROR(read_identity(identity), kTag, "chip lost after SET_DEFAULT");
    g_ic_identity = identity;

    ESP_RETURN_ON_ERROR(write_reg(kRegIoConf2, kIoConf2MisoPd1 | kIoConf2MisoPd2 | kIoConf2AatEn), kTag,
                        "IO_CONF2 write failed");
    ESP_RETURN_ON_ERROR(spi_self_test(), kTag, "SPI register read-back failed at %d Hz",
                        CONFIG_ST25R3916_SPI_FREQ_HZ);

    // Oscillator on, confirmed by its interrupt. This is also the first check
    // that the IRQ wire works.
    ESP_RETURN_ON_ERROR(irq_set_enabled(kIrqOsc), kTag, "IRQ mask write failed");
    irq_clear(0xFFFFFFFF);
    const uint32_t edges_before = irq_edge_count();
    ESP_RETURN_ON_ERROR(write_reg(kRegOpControl, kOpControlEn), kTag, "OP_CONTROL write failed");
    const uint32_t osc = irq_wait(kIrqOsc, 100);
    uint8_t aux_display = 0;
    (void)read_reg(kRegAuxDisplay, aux_display);
    if (!(aux_display & kAuxDisplayOscOk)) {
        ESP_LOGE(kTag, "oscillator did not start (AUX_DISPLAY=0x%02x)", aux_display);
        return ESP_ERR_TIMEOUT;
    }
    if (osc != 0 && irq_edge_count() > edges_before) {
        ESP_LOGI(kTag, "oscillator stable; IRQ line on GPIO %d verified", CONFIG_ST25R3916_SPI_PIN_IRQ);
    } else {
        ESP_LOGW(kTag, "oscillator stable but no edge on IRQ GPIO %d; falling back to per-tick polling",
                 CONFIG_ST25R3916_SPI_PIN_IRQ);
    }

    ESP_RETURN_ON_ERROR(irq_set_enabled(kIrqUsed), kTag, "IRQ mask write failed");
    irq_clear(0xFFFFFFFF);

    // Supply: measure VDD, set sup3V to match, then let the chip pick its
    // regulator voltage (RFAL st25r3916Initialize / st25r3916AdjustRegulators).
    (void)modify_reg(kRegRegulatorControl, kRegulatorControlMpsvMask, 0x00);
    uint8_t vdd_raw = 0;
    if (!command_and_read_ad(kCmdMeasureVdd, vdd_raw)) {
        ESP_LOGE(kTag, "VDD measurement did not complete");
        return ESP_ERR_TIMEOUT;
    }
    const uint32_t vdd_mv = vdd_raw * 23u + ((vdd_raw * 4u + 5u) / 10u);
    const bool supply_3v3 = vdd_mv < 3600;
    ESP_RETURN_ON_ERROR(modify_reg(kRegIoConf2, kIoConf2Sup3v, supply_3v3 ? kIoConf2Sup3v : 0), kTag,
                        "IO_CONF2 write failed");
    (void)modify_reg(kRegRegulatorControl, 0, kRegulatorControlRegS);
    (void)modify_reg(kRegRegulatorControl, kRegulatorControlRegS, 0);
    uint8_t unused = 0;
    (void)command_and_read_ad(kCmdAdjustRegulators, unused);
    ESP_LOGI(kTag, "VDD %" PRIu32 " mV, %s supply mode", vdd_mv, supply_3v3 ? "3.3 V" : "5 V");

    err = write_and_verify_profile();
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "NFC-A analog profile did not read back; refusing to poll with a misconfigured chip");
        return err;
    }
    ESP_RETURN_ON_ERROR(write_reg(kRegAux, 0x00), kTag, "AUX write failed");  // correlator receiver
    g_aux_shadow = 0x00;

    // Field on, the RFAL rfalFieldOnAndStartGT way: the initial RF collision
    // avoidance command checks for someone else's field, switches ours on and
    // reports it with the APON interrupt; only then are tx_en and rx_en set.
    err = field_on();
    if (err != ESP_OK) {
        return err;
    }
    delay_ms(5);  // ISO/IEC 14443 guard time before the first command

#if CONFIG_ST25R3916_SPI_ANTENNA_TUNING
    uint8_t tune_a = 0;
    uint8_t tune_b = 0;
    uint8_t tuned_amp = 0;
    tune_antenna(tune_a, tune_b, tuned_amp);
    ESP_LOGI(kTag, "antenna tuning: ANT_TUNE_A=0x%02x ANT_TUNE_B=0x%02x amplitude=%u", tune_a, tune_b, tuned_amp);
#else
    (void)write_reg(kRegAntTuneA, 0x80);
    (void)write_reg(kRegAntTuneB, 0x40);
#endif

    // AUX_DISPLAY tx_on/rx_on are live activity flags on this chip, not a
    // latched "field is on" state: on the STEVAL-MB17149B they read 0 while
    // idle with tx_en set. The evidence for the field is the APON interrupt
    // above and tx_en reading back in OP_CONTROL.
    uint8_t amplitude = 0;
    measure_field(amplitude);
    uint8_t op_control = 0;
    ESP_RETURN_ON_ERROR(read_reg(kRegOpControl, op_control), kTag, "OP_CONTROL read failed");
    if ((op_control & (kOpControlEn | kOpControlRxEn | kOpControlTxEn)) !=
        (kOpControlEn | kOpControlRxEn | kOpControlTxEn)) {
        ESP_LOGE(kTag, "RF field did not come on: OP_CONTROL=0x%02x", op_control);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(kTag, "RF field on: collision avoidance APON received, OP_CONTROL=0x%02x amplitude=%u", op_control,
             amplitude);
    ESP_LOGI(kTag, "NFC-A poller ready: 106 kbit/s, RX_CONF3=0x00, FSDI=%u, SPI %d Hz", kIsoDepFsdi,
             CONFIG_ST25R3916_SPI_FREQ_HZ);

    g_chip_ready = true;
    g_last_health_us = esp_timer_get_time();
    return ESP_OK;
}

void log_status(const char *reason)
{
    uint8_t amplitude = 0;
    measure_field(amplitude);
    ESP_LOGI(kTag,
             "NFC poller %s: polls=%" PRIu32 " tags=%" PRIu32 " iso_dep=%" PRIu32 " rx_errors=%" PRIu32
             " collisions=%" PRIu32 " apdu_failures=%" PRIu32 " reinits=%" PRIu32 " spi_errors=%" PRIu32
             " irq_edges=%" PRIu32 " amplitude=%u",
             reason, g_stats.polls, g_stats.tags, g_stats.iso_dep, g_stats.rx_errors, g_stats.collisions,
             g_stats.apdu_failures, g_stats.reinits, spi_error_count(), irq_edge_count(), amplitude);
}

} // namespace

esp_err_t m5nfc_set_ecp_identifier(const uint8_t *identifier, size_t identifier_len)
{
    const bool clear = identifier == nullptr && identifier_len == 0;
    if (!clear && (!identifier || identifier_len != kEcpIdentifierSize)) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, kEcpFrameSize> frame = {};
    build_ecp_frame(clear ? nullptr : identifier, frame.data());
    portENTER_CRITICAL(&g_aliro_ecp_lock);
    g_aliro_ecp_frame = frame;
    g_aliro_ecp_enabled = !clear;
    portEXIT_CRITICAL(&g_aliro_ecp_lock);
    return ESP_OK;
}

esp_err_t m5nfc_init(void)
{
    const HalConfig config = hal_config();
    ESP_LOGI(kTag, "ST25R3916 SPI reader: host %d SCLK=%d MISO=%d MOSI=%d CS=%d IRQ=%d at %d Hz", config.spi_host,
             config.pin_sclk, config.pin_miso, config.pin_mosi, config.pin_cs, config.pin_irq, config.spi_freq_hz);
    ESP_RETURN_ON_ERROR(hal_init(config), kTag, "SPI/IRQ setup failed");
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
        uint8_t op_control = 0;
        const uint8_t field_bits = kOpControlEn | kOpControlRxEn | kOpControlTxEn;
        const bool read_ok = read_reg(kRegIcIdentity, identity) == ESP_OK &&
            read_reg(kRegOpControl, op_control) == ESP_OK;
        if (!read_ok || identity != g_ic_identity || (op_control & field_bits) != field_bits) {
            // A brown-out or a chip reset shows up as a lost register state.
            ESP_LOGE(kTag, "reader health check failed: IC_IDENTITY=0x%02x (expected 0x%02x) OP_CONTROL=0x%02x",
                     identity, g_ic_identity, op_control);
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

    uint8_t uid[10] = {};
    uint8_t uid_len = 0;
    uint8_t sak = 0;
    if (!select_a(uid, uid_len, sak)) {
        ESP_LOGD(kTag, "ATQA=%04x seen but anticollision/SELECT did not complete", atqa);
        return M5NFC_ACTIVATION_NONE;
    }
    ++g_stats.tags;
    char uid_str[2 * sizeof(uid) + 1];
    format_uid(uid, uid_len, uid_str, sizeof(uid_str));

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
        req.timeout_ms = kDeselectTimeoutMs;
    } else {
        static constexpr uint8_t kHlta[2] = {0x50, 0x00};
        req.tx = kHlta;
        req.tx_bytes = sizeof(kHlta);
        req.expect_response = false;
    }
    (void)transceive(req, rx, sizeof(rx), rx_len, nullptr);
    delay_ms(2);
}

esp_err_t m5nfc_message_exchange(const uint8_t *command_apdu, size_t command_len, uint8_t *response,
                                 size_t *response_len)
{
    ESP_RETURN_ON_FALSE(command_apdu && response && response_len, ESP_ERR_INVALID_ARG, kTag, "invalid APDU buffer");
    ESP_RETURN_ON_FALSE(command_len > 0 && command_len <= UINT16_MAX && *response_len <= UINT16_MAX,
                        ESP_ERR_INVALID_SIZE, kTag, "APDU buffer too large");
    ESP_RETURN_ON_FALSE(g_chip_ready && g_active_iso_dep, ESP_ERR_INVALID_STATE, kTag, "no ISO-DEP tag active");

    ESP_LOGI(kTag, "APDU TX len=%u cla=%02x ins=%02x", static_cast<unsigned>(command_len), command_apdu[0],
             command_len > 1 ? command_apdu[1] : 0);
    ESP_LOG_BUFFER_HEX_LEVEL(kTag, command_apdu, command_len, ESP_LOG_DEBUG);

    RxStatus last_status = RxStatus::kOk;
    auto tx = [&last_status](const uint8_t *frame, uint8_t frame_len, uint8_t *raw, uint8_t &raw_len) -> bool {
        uint32_t timeout_ms = g_fwt_ms;
        if (frame_len == 2 && frame[0] == 0xF2) {
            // Our S(WTX) response: the PICC asked for WTXM times the FWT.
            const uint32_t wtxm = std::max<uint32_t>(1, frame[1] & 0x3F);
            const uint32_t ceiling = std::max(kWtxMaxMs, g_fwt_ms);
            timeout_ms = static_cast<uint32_t>(
                std::min<uint64_t>(static_cast<uint64_t>(g_fwt_ms) * wtxm, ceiling));
        }
        TxRxRequest req;
        req.tx = frame;
        req.tx_bytes = frame_len;
        req.timeout_ms = timeout_ms;
        size_t n = 0;
        // FSD is 256 including the two CRC bytes, so a legal frame is at most
        // 254 bytes here, which also fits the uint8_t length below.
        last_status = transceive(req, raw, 254, n, nullptr);
        if (last_status != RxStatus::kOk || n < 1) {
            return false;
        }
        raw_len = static_cast<uint8_t>(n);
        return true;
    };

    for (unsigned attempt = 1; attempt <= kApduMaxAttempts; ++attempt) {
        const uint8_t block_before = g_block_number;
        size_t received = 0;
        if (esphome::st25r::isodep_exchange_apdu(command_apdu, command_len, response, *response_len, received,
                                                 g_block_number, g_fsc, tx)) {
            *response_len = received;
            ESP_LOGI(kTag, "APDU RX len=%u", static_cast<unsigned>(received));
            ESP_LOG_BUFFER_HEX_LEVEL(kTag, response, received, ESP_LOG_DEBUG);
            if (attempt > 1) {
                ESP_LOGI(kTag, "APDU exchange succeeded after %u attempts", attempt);
            }
            return ESP_OK;
        }
        g_block_number = block_before;
        ++g_stats.apdu_failures;
        ESP_LOGW(kTag, "APDU exchange attempt %u/%u failed (last frame: %s, FWT=%" PRIu32 " ms)", attempt,
                 kApduMaxAttempts, rx_status_name(last_status), g_fwt_ms);
    }
    return ESP_FAIL;
}
