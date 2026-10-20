/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Status LED: on while the probe is idle, blinking at 10 Hz while TCK/SWCLK
 * is in use.
 *
 * The DAP thread (core 1) only stamps led_tck_last_us; the LED is driven by
 * an SDK repeating timer, whose callback runs in the timer IRQ on core 0. A
 * thread would not do: during a transfer the "usb" thread spins on core 0
 * above normal priority, and the "dap" thread sleeps when the host is idle,
 * which would leave the LED frozen in either state.
 */

#include <pico/stdlib.h>

#include "board_pico_config.h"
#include "led.h"

// Toggle period while TCK is in use: 50 ms on, 50 ms off.
#define LED_BLINK_MS        50

// How long after the last clocking command the LED keeps blinking, so that
// a single command still gives a visible blink.
#define LED_TCK_HOLD_US     100000u

volatile uint32_t led_tck_last_us;

static repeating_timer_t led_timer;

static bool led_timer_cb(repeating_timer_t *rt)
{
    (void) rt;

    if ((time_us_32() - led_tck_last_us) < LED_TCK_HOLD_US)
        gpio_xor_mask(1u << PROBE_USB_CONNECTED_LED);
    else
        gpio_put(PROBE_USB_CONNECTED_LED, 1);
    return true;
}

void led_init(void)
{
    // Idle at start: no activity within the hold time.
    led_tck_last_us = time_us_32() - LED_TCK_HOLD_US;

    gpio_init(PROBE_USB_CONNECTED_LED);
    gpio_set_dir(PROBE_USB_CONNECTED_LED, GPIO_OUT);
    gpio_put(PROBE_USB_CONNECTED_LED, 1);

    add_repeating_timer_ms(LED_BLINK_MS, led_timer_cb, NULL, &led_timer);
}
