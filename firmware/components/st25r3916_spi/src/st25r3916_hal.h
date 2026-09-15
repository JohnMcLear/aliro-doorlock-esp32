/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI and interrupt-line access to an ST25R3916 / ST25R3916B.
 *
 * Threading: the door lock drives the reader from one FreeRTOS task
 * (nfc_detect), after m5nfc_init() has run to completion on the main task.
 * None of these functions is re-entrant and none takes a lock. Do not call
 * them from a second task without adding one.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <esp_err.h>

namespace st25r3916 {

struct HalConfig {
    int spi_host;
    int pin_sclk;
    int pin_miso;
    int pin_mosi;
    int pin_cs;
    int pin_irq;
    int spi_freq_hz;
};

esp_err_t hal_init(const HalConfig &config);

// Register access. A register with kSpaceB set is reached through the 0xFB
// space-B prefix in the same chip-select frame.
esp_err_t read_reg(uint8_t reg, uint8_t &value);
esp_err_t write_reg(uint8_t reg, uint8_t value);
esp_err_t modify_reg(uint8_t reg, uint8_t clear_mask, uint8_t set_mask);
esp_err_t command(uint8_t cmd);
esp_err_t fifo_write(const uint8_t *data, size_t len);
esp_err_t fifo_read(uint8_t *data, size_t len);

// Interrupts. Reading the chip's IRQ registers clears them, so every read is
// folded into a pending set and bits are only consumed when a caller asks
// for them.

// Unmask exactly `mask` (32-bit RFAL layout: IRQ_MAIN, IRQ_TIMER_NFC,
// IRQ_ERROR_WUP, IRQ_TARGET from low byte to high), mask everything else.
esp_err_t irq_set_enabled(uint32_t mask);
// Drop pending bits in `mask` after draining the chip registers.
void irq_clear(uint32_t mask);
// Wait until any bit in `mask` is pending or `timeout_ms` passes. Returns and
// consumes the pending bits in `mask`; other pending bits stay pending.
uint32_t irq_wait(uint32_t mask, uint32_t timeout_ms);
// Consume and return whatever bits in `mask` are pending right now.
uint32_t irq_take(uint32_t mask);
// Number of interrupt-line edges the GPIO ISR has seen since boot.
uint32_t irq_edge_count(void);

// Count of SPI transactions that returned an error since boot.
uint32_t spi_error_count(void);

} // namespace st25r3916
