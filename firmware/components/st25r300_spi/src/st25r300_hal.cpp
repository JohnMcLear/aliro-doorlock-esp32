/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Same structure as the ST25R3916 HAL in ../st25r3916_spi, with the ST25R300
 * SPI encoding (DS14655 table 5) and its three IRQ registers.
 */

#include "st25r300_hal.h"

#include "st25r300_regs.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstring>

namespace st25r300 {
namespace {

constexpr char kTag[] = "st25r300";

// One opcode byte and a full FIFO worth of payload.
constexpr size_t kSpiBufferSize = 1 + kFifoDepth;

spi_device_handle_t g_spi = nullptr;
uint8_t *g_tx = nullptr;
uint8_t *g_rx = nullptr;
int g_pin_irq = -1;
int g_pin_rst = -1;
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

void drain_irq_registers()
{
    // Reading IRQ_STATUS1..3 clears them and releases the line. If an event
    // lands mid-read the line stays high without a fresh rising edge, so read
    // again until it is low.
    for (int attempt = 0; attempt < 4; ++attempt) {
        uint8_t r1 = 0;
        uint8_t r2 = 0;
        uint8_t r3 = 0;
        if (read_reg(kRegIrqStatus1, r1) != ESP_OK || read_reg(kRegIrqStatus2, r2) != ESP_OK ||
            read_reg(kRegIrqStatus3, r3) != ESP_OK) {
            return;
        }
        g_pending_irqs |= static_cast<uint32_t>(r1) | (static_cast<uint32_t>(r2) << 8) |
            (static_cast<uint32_t>(r3) << 16);
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

    if (config.pin_rst >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << config.pin_rst;
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            return err;
        }
        // Low = run. Driven before anything else so the chip is not held in
        // reset by a floating line while SPI comes up.
        (void)gpio_set_level(static_cast<gpio_num_t>(config.pin_rst), 0);
        g_pin_rst = config.pin_rst;
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

    // ST25R300 SPI: CPOL 0, CPHA 1 (mode 1), MSB first, active-low CS. Same as
    // the ESPHome st25r300_spi component (CLOCK_PHASE_TRAILING).
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
        // The chip drives IRQ high when an unmasked interrupt is pending. The
        // pull-down only matters when the wire is missing: the line then reads
        // low and the init check reports it instead of seeing a floating input.
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

void hal_reset_pulse(void)
{
    if (g_pin_rst < 0) {
        return;
    }
    // Timings from the ESPHome driver's setup(): 10 ms in reset, 10 ms after.
    (void)gpio_set_level(static_cast<gpio_num_t>(g_pin_rst), 1);
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
    (void)gpio_set_level(static_cast<gpio_num_t>(g_pin_rst), 0);
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
    g_pending_irqs = 0;
}

esp_err_t read_reg(uint8_t reg, uint8_t &value)
{
    g_tx[0] = static_cast<uint8_t>(kSpiRead | (reg & 0x7F));
    g_tx[1] = 0x00;
    esp_err_t err = transfer(2);
    if (err == ESP_OK) {
        value = g_rx[1];
    }
    return err;
}

esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    g_tx[0] = static_cast<uint8_t>(reg & 0x7F);
    g_tx[1] = value;
    return transfer(2);
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
    // A set bit in a mask register disables that interrupt.
    esp_err_t err = write_reg(kRegIrqMask1, static_cast<uint8_t>(~mask & 0xFF));
    if (err == ESP_OK) {
        err = write_reg(kRegIrqMask2, static_cast<uint8_t>(~(mask >> 8) & 0xFF));
    }
    if (err == ESP_OK) {
        err = write_reg(kRegIrqMask3, static_cast<uint8_t>(~(mask >> 16) & 0xFF));
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

} // namespace st25r300
