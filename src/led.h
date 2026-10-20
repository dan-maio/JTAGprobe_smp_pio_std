/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Status LED (PROBE_USB_CONNECTED_LED, the Pico's GP25): on all the time,
 * blinking rapidly while the DAP commands clock TCK/SWCLK.
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include <hardware/timer.h>

// Time of the last DAP command that clocked TCK/SWCLK (1 MHz timer).
extern volatile uint32_t led_tck_last_us;

// Start the LED: on, and the timer that blinks it ("main" thread, core 0).
void led_init(void);

// A DAP command is about to clock TCK/SWCLK ("dap" thread, core 1).
static inline void led_tck_activity(void)
{
    led_tck_last_us = time_us_32();
}

#endif /* LED_H_ */
