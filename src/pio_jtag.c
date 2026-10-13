/*
 * Copyright (c) 2026 Dan Maio & Antigravity AI
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RP2040 PIO Hardware Acceleration for JTAG (CMSIS-DAP)
 */

#include "pio_jtag.h"
#include "DAP_config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "jtag.pio.h"

pio_jtag_t g_pio_jtag = {
    .pio = NULL,
    .sm = 0,
    .offset = 0,
    .enabled = false,
    .clock_hz = DAP_DEFAULT_SWJ_CLOCK,
    .pin_tck = PROBE_PIN_SWCLK_TCK,
    .pin_tdi = PROBE_PIN_TDI,
    .pin_tdo = PROBE_PIN_TDO,
    .pin_tms = PROBE_PIN_SWDIO_TMS,
    .pin_trst = PROBE_PIN_nTRST,
    .pin_reset = PROBE_PIN_nRESET
};

bool pio_jtag_init(uint32_t clock_hz) {
    if (g_pio_jtag.enabled) {
        pio_jtag_set_clock(clock_hz);
        return true;
    }

    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio = pio1;
        sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) {
            return false;
        }
    }

    if (!pio_can_add_program(pio, &jtag_tdo_program)) {
        pio_sm_unclaim(pio, (uint)sm);
        return false;
    }

    uint offset = pio_add_program(pio, &jtag_tdo_program);

    pio_sm_config c = jtag_tdo_program_get_default_config(offset);
    sm_config_set_out_pins(&c, g_pio_jtag.pin_tdi, 1);
    sm_config_set_set_pins(&c, g_pio_jtag.pin_tms, 1);
    sm_config_set_in_pins(&c, g_pio_jtag.pin_tdo);
    sm_config_set_sideset_pins(&c, g_pio_jtag.pin_tck);

    // LSB first for CMSIS-DAP. TDI: no autopull, the program pulls each
    // byte with `pull ifempty` (threshold 8); TDO: 8-bit autopush.
    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_in_shift(&c, true, true, 8);

    // Initial frequency divider (4 PIO instructions per JTAG TCK period)
    if (clock_hz == 0) {
        clock_hz = DAP_DEFAULT_SWJ_CLOCK;
    }
    float div = (float)clock_get_hz(clk_sys) / (4.0f * (float)clock_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 65535.0f) div = 65535.0f;
    sm_config_set_clkdiv(&c, div);

    // Initial pin levels: TCK, TDI and TMS high
    uint32_t outs = (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi)
                  | (1u << g_pio_jtag.pin_tms);
    pio_sm_set_pins_with_mask(pio, (uint)sm, outs, outs);

    // Direction mask: TCK, TDI and TMS outputs, TDO input
    pio_sm_set_pindirs_with_mask(pio, (uint)sm, outs,
        outs | (1u << g_pio_jtag.pin_tdo));

    pio_gpio_init(pio, g_pio_jtag.pin_tdi);
    pio_gpio_init(pio, g_pio_jtag.pin_tck);
    pio_gpio_init(pio, g_pio_jtag.pin_tms);

    // TDO input with pull-up
    gpio_init(g_pio_jtag.pin_tdo);
    gpio_set_dir(g_pio_jtag.pin_tdo, GPIO_IN);
    gpio_pull_up(g_pio_jtag.pin_tdo);

    // Synchronous JTAG bypass to reduce input delay
    hw_set_bits(&pio->input_sync_bypass, 1u << g_pio_jtag.pin_tdo);

    // nTRST output high
    gpio_init(g_pio_jtag.pin_trst);
    gpio_set_dir(g_pio_jtag.pin_trst, GPIO_OUT);
    gpio_put(g_pio_jtag.pin_trst, 1);

    // nRESET open-drain high (input with pull-up)
    gpio_init(g_pio_jtag.pin_reset);
    gpio_set_dir(g_pio_jtag.pin_reset, GPIO_IN);
    gpio_pull_up(g_pio_jtag.pin_reset);
    gpio_put(g_pio_jtag.pin_reset, 0);

    pio_sm_init(pio, (uint)sm, offset, &c);
    pio_sm_set_enabled(pio, (uint)sm, true);

    g_pio_jtag.pio = pio;
    g_pio_jtag.sm = (uint)sm;
    g_pio_jtag.offset = offset;
    g_pio_jtag.pins_pio = true;
    g_pio_jtag.enabled = true;
    g_pio_jtag.clock_hz = clock_hz;

    return true;
}

void pio_jtag_deinit(void) {
    if (!g_pio_jtag.enabled) return;

    pio_sm_set_enabled(g_pio_jtag.pio, g_pio_jtag.sm, false);
    pio_remove_program(g_pio_jtag.pio, &jtag_tdo_program, g_pio_jtag.offset);
    pio_sm_unclaim(g_pio_jtag.pio, g_pio_jtag.sm);

    // Restore pins to SIO High-Z
    gpio_init(g_pio_jtag.pin_tck);
    gpio_set_dir(g_pio_jtag.pin_tck, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tck);

    gpio_init(g_pio_jtag.pin_tdi);
    gpio_set_dir(g_pio_jtag.pin_tdi, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tdi);

    gpio_init(g_pio_jtag.pin_tdo);
    gpio_set_dir(g_pio_jtag.pin_tdo, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tdo);

    gpio_init(g_pio_jtag.pin_tms);
    gpio_set_dir(g_pio_jtag.pin_tms, GPIO_IN);
    gpio_disable_pulls(g_pio_jtag.pin_tms);

    g_pio_jtag.pins_pio = false;
    g_pio_jtag.enabled = false;
}

void pio_jtag_pins_to_sio(void) {
    if (!g_pio_jtag.enabled || !g_pio_jtag.pins_pio) return;

    // The PIO is idle between commands (TCK high, stalled on `pull`):
    // give SIO the levels the pins have now, then switch them over, so
    // no pin glitches.
    uint32_t mask = (1u << g_pio_jtag.pin_tck) | (1u << g_pio_jtag.pin_tdi)
                  | (1u << g_pio_jtag.pin_tms);
    uint32_t level = gpio_get_all() & mask;
    gpio_put_masked(mask, level);
    gpio_set_dir_out_masked(mask);
    gpio_set_function(g_pio_jtag.pin_tck, GPIO_FUNC_SIO);
    gpio_set_function(g_pio_jtag.pin_tdi, GPIO_FUNC_SIO);
    gpio_set_function(g_pio_jtag.pin_tms, GPIO_FUNC_SIO);
    g_pio_jtag.pins_pio = false;
}

void pio_jtag_pins_to_pio(void) {
    if (!g_pio_jtag.enabled || g_pio_jtag.pins_pio) return;

    // TCK: the PIO's side-set level is high while it waits on `pull`, as
    // the bit-banging paths leave it. TDI is set before each rising edge,
    // TMS by every entry header while TCK is still high.
    pio_gpio_init(g_pio_jtag.pio, g_pio_jtag.pin_tck);
    pio_gpio_init(g_pio_jtag.pio, g_pio_jtag.pin_tdi);
    pio_gpio_init(g_pio_jtag.pio, g_pio_jtag.pin_tms);
    g_pio_jtag.pins_pio = true;
}

bool pio_jtag_is_enabled(void) {
    return g_pio_jtag.enabled;
}

void pio_jtag_set_clock(uint32_t clock_hz) {
    if (clock_hz == 0) {
        clock_hz = DAP_DEFAULT_SWJ_CLOCK;
    }
    g_pio_jtag.clock_hz = clock_hz;
    if (!g_pio_jtag.enabled) return;

    float div = (float)clock_get_hz(clk_sys) / (4.0f * (float)clock_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 65535.0f) div = 65535.0f;

    pio_sm_set_clkdiv(g_pio_jtag.pio, g_pio_jtag.sm, div);
}

// Header word of one entry for the PIO: TCK count - 1 and the TMS level.
static inline uint32_t entry_header(uint32_t info, uint32_t *bits) {
    uint32_t n = info & JTAG_SEQUENCE_TCK;
    if (n == 0U) {
        n = 64U;
    }
    *bits = n;
    return (n - 1U) | (((info & JTAG_SEQUENCE_TMS) != 0U) ? 0x80U : 0U);
}

uint32_t pio_jtag_sequences(uint32_t count, const uint8_t *request, uint8_t *response) {
    PIO pio = g_pio_jtag.pio;
    uint sm = g_pio_jtag.sm;
    io_wo_32 *txf = &pio->txf[sm];
    const io_ro_32 *rxf = &pio->rxf[sm];

    // A bit-banging path may have taken TCK/TDI/TMS since the last command.
    pio_jtag_pins_to_pio();

    // Two cursors walk the same request: the TX one feeds headers and TDI
    // bytes, the RX one sorts the TDO words that come back. The PIO runs
    // entry after entry; the CPU only keeps both FIFOs moving.
    const uint8_t *tp = request;
    uint32_t t_left = count;        // entries whose header is not sent yet
    uint32_t t_bytes = 0;           // TDI bytes of the current entry still to send

    const uint8_t *rp = request;
    uint32_t r_left = count;        // entries whose TDO is not sorted yet
    uint32_t r_words = 0;           // RX words of the current entry still to come
    uint32_t r_rem = 0;             // bits in the entry's last, partial word
    bool r_capture = false;
    uint8_t *out = response;

    while (t_left || t_bytes || r_left || r_words) {
        if (!pio_sm_is_tx_fifo_full(pio, sm)) {
            if (t_bytes != 0U) {
                *txf = *tp++;
                t_bytes--;
            } else if (t_left != 0U) {
                uint32_t bits;
                *txf = entry_header(*tp++, &bits);
                t_bytes = (bits + 7U) / 8U;
                t_left--;
            }
        }

        if (r_words == 0U && r_left != 0U) {
            uint32_t bits;
            uint32_t info = *rp;
            (void)entry_header(info, &bits);
            r_capture = (info & JTAG_SEQUENCE_TDO) != 0U;
            r_words = bits / 8U + 1U;   // full bytes, then the last word
            r_rem = bits % 8U;
            rp += 1U + (bits + 7U) / 8U;
            r_left--;
        }

        if (r_words != 0U && !pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t val = *rxf;
            r_words--;
            if (r_capture) {
                // The ISR shifts right: a full byte sits in bits 31..24,
                // the last r bits in bits 31..32-r; an empty last word
                // (r = 0) carries no data.
                if (r_words != 0U) {
                    *out++ = (uint8_t)(val >> 24);
                } else if (r_rem != 0U) {
                    *out++ = (uint8_t)(val >> (32U - r_rem));
                }
            }
        }
    }

    return ((uint32_t)(tp - request) << 16) | (uint32_t)(out - response);
}

void pio_jtag_sequence(uint32_t info, const uint8_t *tdi, uint8_t *tdo) {
    // One entry, laid out as in a DAP_JTAG_Sequence request.
    uint8_t entry[1U + 8U];
    uint32_t bits;
    (void)entry_header(info, &bits);
    entry[0] = (uint8_t)info;
    for (uint32_t i = 0U; i < (bits + 7U) / 8U; i++) {
        entry[1U + i] = tdi[i];
    }
    (void)pio_jtag_sequences(1U, entry, tdo);
}
