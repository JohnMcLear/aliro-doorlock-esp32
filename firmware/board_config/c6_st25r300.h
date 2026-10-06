// Board pin map for an ESP32-C6-DevKitC-1 wired to one of Lauri's 40 x 40 mm
// ST25R300 reader boards over its 9-pin PicoBlade (CS, MOSI, SCLK, MISO, RST,
// IRQ, VTX 5 V, VIO 3V3, GND).
//
// This header is copied into the door-lock example as
// main/aliro_board_config.h by scripts/build_release.sh. The NFC pins are
// consumed by the st25r300_spi component through Kconfig, set in
// firmware/overlay/sdkconfig.release.c6-st25r300-wifi-ringtest; they are
// repeated here so the whole board is described in one place, and the build
// script checks the header, the overlay and variants.json agree.
//
// SPI pins are the ones the exterior ESPHome prototype runs the ST25R300 on
// (doorplan esphome_external: CLK 19, MISO 20, MOSI 21, CS 22). RST and IRQ
// are new: GPIO 18 and 23 sit next to the SPI pins on the devkit header and
// avoid GPIO 6/7 (old STEVAL wiring), 8 (RGB LED and strap), 9 (BOOT), 12/13
// (USB), 15 (strap) and 16/17 (UART0).

#pragma once

#define ALIRO_BOARD_ID "c6-st25r300"
#define ALIRO_BOARD_LABEL "ESP32-C6 devkit + ST25R300 reader board (SPI)"

// ST25R300 SPI
#define ALIRO_BOARD_NFC_SPI_SCLK_GPIO 19
#define ALIRO_BOARD_NFC_SPI_MISO_GPIO 20
#define ALIRO_BOARD_NFC_SPI_MOSI_GPIO 21
#define ALIRO_BOARD_NFC_SPI_CS_GPIO 22
#define ALIRO_BOARD_NFC_IRQ_GPIO 23
#define ALIRO_BOARD_NFC_RST_GPIO 18

// On-board WS2812 RGB LED on GPIO 8, no power-enable pin.
#define ALIRO_BOARD_RGB_DATA_GPIO 8
#define ALIRO_BOARD_HAS_RGB_POWER 0

// BOOT button (shared with the ROM download strap on GPIO 9)
#define ALIRO_BOARD_USER_BUTTON_GPIO 9
#define ALIRO_BOARD_USER_BUTTON_ACTIVE_LOW 1

// Lock output, driven high while unlocked. Nothing is attached on the test
// rig; GPIO 2 lets a meter or an LED show that a successful tap acted.
#define ALIRO_BOARD_LOCK_GPIO 2
#define ALIRO_BOARD_LOCK_ACTIVE_HIGH 1
