/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI, interrupt-line and reset-line access to an ST25R300.
 *
 * Threading: the door lock drives the reader from one FreeRTOS task
 * (nfc_detect), after m5nfc_init() has run to completion on the main task.
 * None of these functions is re-entrant and none takes a lock.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <esp_err.h>

namespace st25r300 {

struct HalConfig {
    int spi_host;
    int pin_sclk;
    int pin_miso;
    int pin_mosi;
    int pin_cs;
    int pin_irq;  // -1: poll the IRQ registers once per tick
    int pin_rst;  // -1: no reset line. RESET is active high on this chip.
    int spi_freq_hz;
};

esp_err_t hal_init(const HalConfig &config);

// Pulse RESET high then low. The chip leaves reset in power-down mode with
// every register at its default. No-op without a reset line.
void hal_reset_pulse(void);

esp_err_t read_reg(uint8_t reg, uint8_t &value);
esp_err_t write_reg(uint8_t reg, uint8_t value);
esp_err_t command(uint8_t cmd);
esp_err_t fifo_write(const uint8_t *data, size_t len);
esp_err_t fifo_read(uint8_t *data, size_t len);

// Interrupts, in the 24-bit layout of st25r300_regs.h. Reading the IRQ status
// registers clears them, so every read is folded into a pending set and bits
// are only consumed when a caller asks for them.
esp_err_t irq_set_enabled(uint32_t mask);
void irq_clear(uint32_t mask);
uint32_t irq_wait(uint32_t mask, uint32_t timeout_ms);
uint32_t irq_take(uint32_t mask);
uint32_t irq_edge_count(void);

uint32_t spi_error_count(void);

} // namespace st25r300
