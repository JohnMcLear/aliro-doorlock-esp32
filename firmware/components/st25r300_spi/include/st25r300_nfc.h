/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25R300 driver extensions beyond the m5nfc interface: run-time tuning for
 * slow cards and per-APDU diagnostics. Used by the ring applet test mode
 * (firmware/patches/0027-ring-applet-test-mode.patch); everything here has a
 * Kconfig default, so a build that never calls these behaves the same.
 *
 * Setters may be called from any task. They take effect at the next APDU.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t elapsed_ms;       /* wall clock for the whole APDU */
    uint32_t frames;           /* frames sent, WTX grants and chaining included */
    uint32_t wtx;              /* S(WTX) requests granted */
    uint8_t max_wtxm;          /* largest WTX multiplier the card asked for */
    uint32_t longest_wait_ms;  /* longest single wait after a WTX grant */
    uint32_t fwt_ms;           /* frame waiting time in force */
    uint8_t attempts;          /* attempts used, 1 when the first one worked */
    bool ok;
    bool budget_exceeded;      /* gave up on the per-APDU wall-clock budget */
    bool wtx_limit;            /* gave up on the WTX count backstop */
    const char *last_frame;    /* status of the last frame: "ok", "timeout", "crc"... */
} st25r300_apdu_stats_t;

/* Wall-clock budget for one APDU, WTX waits included. */
void st25r300_set_apdu_budget_ms(uint32_t ms);
uint32_t st25r300_get_apdu_budget_ms(void);

/* Minimum frame waiting time, whatever the card's ATS claims. For cards that
 * compute without sending S(WTX). Capped at 19800 ms, the NRT limit. */
void st25r300_set_fwt_floor_ms(uint32_t ms);
uint32_t st25r300_get_fwt_floor_ms(void);

/* Longest single wait after granting a WTX. */
void st25r300_set_wtx_ceiling_ms(uint32_t ms);
uint32_t st25r300_get_wtx_ceiling_ms(void);

/* How many times an APDU is attempted. Retrying a slow card mid-computation
 * usually makes things worse, so the ring test mode uses 1. */
void st25r300_set_apdu_attempts(uint8_t attempts);
uint8_t st25r300_get_apdu_attempts(void);

/* Log every ISO-DEP frame in hex, with timestamps, and every WTX grant. */
void st25r300_set_frame_trace(bool on);
bool st25r300_get_frame_trace(void);

/* Statistics for the most recent m5nfc_message_exchange(). */
void st25r300_get_last_apdu_stats(st25r300_apdu_stats_t *out);

#ifdef __cplusplus
}
#endif
