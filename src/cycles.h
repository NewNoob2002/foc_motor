/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: the DWT cycle counter (CYCCNT), one count per HCLK cycle (170 MHz).
 * A J-Link disconnect can stop it (TRCENA/CYCCNTENA cleared, seen in stage 4
 * run 1), so callers check cycles_start() before trusting a measurement.
 */

#ifndef FOC_MOTOR_CYCLES_H
#define FOC_MOTOR_CYCLES_H

#include <stdbool.h>
#include <stdint.h>

#include <soc.h>

#define CYCLES_PER_US 170U

/* Starts the counter if it is stopped. Returns true if it was already running. */
static inline bool cycles_start(void) {
    bool running = (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;

    if (!running) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return running;
}

/* Wraps every 2^32 cycles (25 s); differences of uint32_t are correct across one wrap. */
static inline uint32_t cycles_now(void) {
    return DWT->CYCCNT;
}

#endif /* FOC_MOTOR_CYCLES_H */
