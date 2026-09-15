/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * NFC reader interface used by esp-matter's Aliro door lock delegate.
 *
 * The door lock talks to its reader only through these six functions. The
 * M5Stack build gets them from the managed `m5nfc` component (M5 Unit NFC over
 * I2C) plus firmware/patches/0005-add-m5nfc-aliro-ecp.patch. This component
 * provides the same functions, with the same names, types and semantics, for
 * an ST25R3916 or ST25R3916B wired directly to SPI. Keeping the names means
 * aliro_door_lock_delegate.cpp and the audited patches 0003, 0004, 0006, 0007
 * and 0008 apply unchanged, and the build script's delegate source contract
 * still checks the real call sites.
 *
 * The header name is kept for that reason only; nothing here depends on M5Stack
 * hardware or libraries.
 */

#pragma once

#include <esp_err.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M5NFC_ACTIVATION_NONE = 0,
    M5NFC_ACTIVATION_NON_ISO_DEP,
    M5NFC_ACTIVATION_ISO_DEP,
} m5nfc_activation_result_t;

/* Bring up SPI, verify the chip identity, configure NFC-A polling and turn the
 * RF field on. Fails if no ST25R3916/ST25R3916B answers on the configured bus. */
esp_err_t m5nfc_init(void);

/* Set the 8-byte Aliro GroupIdentifier used in the Enhanced Contactless Polling
 * frame, or clear it with (NULL, 0). Safe to call from any task. */
esp_err_t m5nfc_set_ecp_identifier(const uint8_t *identifier, size_t identifier_len);

/* Periodic housekeeping, called once per poll loop iteration: chip health
 * check, recovery after a lost chip, and a low-rate status log. */
void m5nfc_update(void);

/* Send ECP (when an identifier is set), then REQA, anticollision and SELECT.
 * For a tag that advertises ISO 14443-4, also RATS. */
m5nfc_activation_result_t m5nfc_activate(void);

/* S(DESELECT) for an ISO-DEP tag, HLTA otherwise. */
void m5nfc_deactivate(void);

/* One command APDU out, one response APDU back, over ISO-DEP with block
 * chaining in both directions and S(WTX) handling. On entry *response_len is
 * the capacity of `response`; on success it is the response length. */
esp_err_t m5nfc_message_exchange(const uint8_t *command, size_t command_len, uint8_t *response, size_t *response_len);

#ifdef __cplusplus
}
#endif
