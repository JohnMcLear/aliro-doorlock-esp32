// Board pin map for an ESP32-C6 devkit wired to an ST STEVAL-MB17149B
// (ST25R3916B) over SPI.
//
// This header is copied into the door-lock example as
// main/aliro_board_config.h by scripts/build_release.sh. The audited
// firmware source includes it and reads the RGB macros below.
//
// The NFC reader pins are consumed by the st25r3916_spi component through
// Kconfig, set in firmware/overlay/sdkconfig.release.c6-steval-thread. They
// are repeated here so the whole board is described in one place, and the
// build script checks the header, the overlay and variants.json agree.
//
// Wiring verified on the bench board (Yondoor DEVELOPMENT_STATE.md and
// tests/test-steval-mb17149-b-c6.yaml). The example YAMLs in that repo
// swap MOSI/MISO and CS/IRQ; they are wrong for this board.

#pragma once

#define ALIRO_BOARD_ID "c6-steval"
#define ALIRO_BOARD_LABEL "ESP32-C6 devkit + STEVAL-MB17149B (ST25R3916B, SPI)"

// ST25R3916B SPI
#define ALIRO_BOARD_NFC_SPI_SCLK_GPIO 19
#define ALIRO_BOARD_NFC_SPI_MISO_GPIO 10
#define ALIRO_BOARD_NFC_SPI_MOSI_GPIO 18
#define ALIRO_BOARD_NFC_SPI_CS_GPIO 6
#define ALIRO_BOARD_NFC_IRQ_GPIO 7

// On-board WS2812 RGB LED.
// Espressif's ESP32-C6-DevKitC-1 and -DevKitM-1 wire it to GPIO 8 with no
// power-enable pin. GPIO 8 is also a boot strapping pin; the WS2812 input
// does not pull it, so boot mode is unaffected.
#define ALIRO_BOARD_RGB_DATA_GPIO 8
#define ALIRO_BOARD_HAS_RGB_POWER 0

// BOOT button (shared with the ROM download strap on GPIO 9)
#define ALIRO_BOARD_USER_BUTTON_GPIO 9
#define ALIRO_BOARD_USER_BUTTON_ACTIVE_LOW 1

// Lock actuator output, driven high while unlocked.
//
// GPIO 2 is the LED on John's ST25R prototyping board, which his ESPHome test
// configs drive alongside GPIO 15 on every tag read. It is an indicator, not
// the relay, but it proves the lock state reaches a pin.
//
// The relay looks like it is on GPIO 19, which on this board is the reader's
// SPI clock. Nothing can drive it until the reader moves, so that is a wiring
// decision rather than a firmware one. GPIO 15, 16 and 13 were tried and none
// clicked the relay.
#define ALIRO_BOARD_LOCK_GPIO 2
#define ALIRO_BOARD_LOCK_ACTIVE_HIGH 1
