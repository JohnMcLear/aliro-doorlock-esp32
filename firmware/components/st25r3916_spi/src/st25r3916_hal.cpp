/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "st25r3916_hal.h"

#include "st25r3916_regs.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <cstring>

namespace st25r3916 {
namespace {

constexpr char kTag[] = "st25r3916";

// One prefix byte (space-B access or FIFO opcode), one address byte, and a
// full FIFO worth of payload.
constexpr size_t kSpiBufferSize = 2 + kFifoDepth;

spi_device_handle_t g_spi = nullptr;
uint8_t *g_tx = nullptr;
uint8_t *g_rx = nullptr;
int g_pin_irq = -1;
SemaphoreHandle_t g_irq_sem = nullptr;
volatile uint32_t g_irq_edges = 0;
uint32_t g_pending_irqs = 0;
uint32_t g_spi_errors = 0;

void IRAM_ATTR irq_isr(void *)
{
    g_irq_edges = g_irq_edges + 1;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(g_irq_sem, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t transfer(size_t len)
{
    spi_transaction_t t = {};
    t.length = len * 8;
    t.rxlength = len * 8;
    t.tx_buffer = g_tx;
    t.rx_buffer = g_rx;
    esp_err_t err = spi_device_polling_transmit(g_spi, &t);
    if (err != ESP_OK) {
        ++g_spi_errors;
    }
    return err;
}

// Space A only: the chip auto-increments the address on a multi-byte read.
esp_err_t read_regs_a(uint8_t reg, uint8_t *out, size_t count)
{
    g_tx[0] = static_cast<uint8_t>(kSpiRead | (reg & 0x3F));
    std::memset(g_tx + 1, 0, count);
    esp_err_t err = transfer(1 + count);
    if (err == ESP_OK) {
        std::memcpy(out, g_rx + 1, count);
    }
    return err;
}

void drain_irq_registers()
{
    // Reading IRQ_MAIN..IRQ_TARGET clears them and releases the line. If a new
    // event lands while the read is in progress the line stays high without a
    // fresh rising edge, so read again until the line is low.
    for (int attempt = 0; attempt < 4; ++attempt) {
        uint8_t regs[4] = {};
        if (read_regs_a(kRegIrqMain, regs, sizeof(regs)) != ESP_OK) {
            return;
        }
        g_pending_irqs |= static_cast<uint32_t>(regs[0]) | (static_cast<uint32_t>(regs[1]) << 8) |
            (static_cast<uint32_t>(regs[2]) << 16) | (static_cast<uint32_t>(regs[3]) << 24);
        if (g_pin_irq < 0 || gpio_get_level(static_cast<gpio_num_t>(g_pin_irq)) == 0) {
            return;
        }
    }
}

} // namespace

esp_err_t hal_init(const HalConfig &config)
{
    if (g_spi != nullptr) {
        return ESP_OK;
    }

    g_tx = static_cast<uint8_t *>(heap_caps_malloc(kSpiBufferSize, MALLOC_CAP_DMA));
    g_rx = static_cast<uint8_t *>(heap_caps_malloc(kSpiBufferSize, MALLOC_CAP_DMA));
    if (g_tx == nullptr || g_rx == nullptr) {
        heap_caps_free(g_tx);
        heap_caps_free(g_rx);
        g_tx = g_rx = nullptr;
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus = {};
    bus.mosi_io_num = config.pin_mosi;
    bus.miso_io_num = config.pin_miso;
    bus.sclk_io_num = config.pin_sclk;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = static_cast<int>(kSpiBufferSize);
    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(config.spi_host), &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    // ST25R3916 SPI: CPOL 0, CPHA 1 (mode 1), MSB first, active-low CS.
    spi_device_interface_config_t dev = {};
    dev.mode = 1;
    dev.clock_speed_hz = config.spi_freq_hz;
    dev.spics_io_num = config.pin_cs;
    dev.queue_size = 1;
    dev.cs_ena_pretrans = 1;
    dev.cs_ena_posttrans = 1;
    err = spi_bus_add_device(static_cast<spi_host_device_t>(config.spi_host), &dev, &g_spi);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    g_irq_sem = xSemaphoreCreateBinary();
    if (g_irq_sem == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    if (config.pin_irq >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << config.pin_irq;
        io.mode = GPIO_MODE_INPUT;
        // The chip drives IRQ push-pull. The pull-down only matters when the
        // wire is missing: the line then reads low and the init self-test
        // reports it instead of seeing a floating input.
        io.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.intr_type = GPIO_INTR_POSEDGE;
        err = gpio_config(&io);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        err = gpio_isr_handler_add(static_cast<gpio_num_t>(config.pin_irq), irq_isr, nullptr);
        if (err != ESP_OK) {
            return err;
        }
        g_pin_irq = config.pin_irq;
    }
    return ESP_OK;
}

esp_err_t read_reg(uint8_t reg, uint8_t &value)
{
    size_t n = 0;
    if (reg & kSpaceB) {
        g_tx[n++] = kSpiSpaceBAccess;
    }
    g_tx[n++] = static_cast<uint8_t>(kSpiRead | (reg & 0x3F));
    g_tx[n++] = 0x00;
    esp_err_t err = transfer(n);
    if (err == ESP_OK) {
        value = g_rx[n - 1];
    }
    return err;
}

esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    size_t n = 0;
    if (reg & kSpaceB) {
        g_tx[n++] = kSpiSpaceBAccess;
    }
    g_tx[n++] = static_cast<uint8_t>(reg & 0x3F);
    g_tx[n++] = value;
    return transfer(n);
}

esp_err_t modify_reg(uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t value = 0;
    esp_err_t err = read_reg(reg, value);
    if (err != ESP_OK) {
        return err;
    }
    const uint8_t next = static_cast<uint8_t>((value & ~clear_mask) | set_mask);
    return next == value ? ESP_OK : write_reg(reg, next);
}

esp_err_t command(uint8_t cmd)
{
    g_tx[0] = cmd;
    return transfer(1);
}

esp_err_t fifo_write(const uint8_t *data, size_t len)
{
    if (len == 0 || len > kFifoDepth) {
        return ESP_ERR_INVALID_SIZE;
    }
    g_tx[0] = kSpiFifoLoad;
    std::memcpy(g_tx + 1, data, len);
    return transfer(1 + len);
}

esp_err_t fifo_read(uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    if (len > kFifoDepth) {
        return ESP_ERR_INVALID_SIZE;
    }
    g_tx[0] = kSpiFifoRead;
    std::memset(g_tx + 1, 0, len);
    esp_err_t err = transfer(1 + len);
    if (err == ESP_OK) {
        std::memcpy(data, g_rx + 1, len);
    }
    return err;
}

esp_err_t irq_set_enabled(uint32_t mask)
{
    // A set bit in the mask registers disables that interrupt.
    esp_err_t err = write_reg(kRegIrqMaskMain, static_cast<uint8_t>(~mask & 0xFF));
    if (err == ESP_OK) {
        err = write_reg(kRegIrqMaskTimerNfc, static_cast<uint8_t>(~(mask >> 8) & 0xFF));
    }
    if (err == ESP_OK) {
        err = write_reg(kRegIrqMaskErrorWup, static_cast<uint8_t>(~(mask >> 16) & 0xFF));
    }
    if (err == ESP_OK) {
        err = write_reg(kRegIrqMaskTarget, static_cast<uint8_t>(~(mask >> 24) & 0xFF));
    }
    return err;
}

void irq_clear(uint32_t mask)
{
    drain_irq_registers();
    g_pending_irqs &= ~mask;
}

uint32_t irq_take(uint32_t mask)
{
    drain_irq_registers();
    const uint32_t hit = g_pending_irqs & mask;
    g_pending_irqs &= ~hit;
    return hit;
}

uint32_t irq_wait(uint32_t mask, uint32_t timeout_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
    while (true) {
        const uint32_t hit = irq_take(mask);
        if (hit != 0) {
            return hit;
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= deadline_us) {
            return 0;
        }
        const int64_t remaining_ms = (deadline_us - now_us + 999) / 1000;
        TickType_t ticks = static_cast<TickType_t>((remaining_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
        if (g_pin_irq < 0) {
            ticks = 1;  // no interrupt line: poll once per tick
        }
        (void)xSemaphoreTake(g_irq_sem, std::max<TickType_t>(1, ticks));
    }
}

uint32_t irq_edge_count(void)
{
    return g_irq_edges;
}

uint32_t spi_error_count(void)
{
    return g_spi_errors;
}

} // namespace st25r3916
