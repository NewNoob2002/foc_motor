/* SPDX-License-Identifier: Apache-2.0 */
#include "sincos.h"

#include <stm32g4xx_ll_bus.h>

/** Configure a fixed q1.31 cosine/sine transaction with unity modulus. */
void sincos_init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
    LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_CORDIC);
    LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_CORDIC);
    /* RM0440 CORDIC functional description and CSR PRECISION: four iterations
     * per clock, so six clocks give 24 iterations (angular residual about
     * 2^-23 rad, comfortably below 1e-6; internal precision is 24 bits).
     * COSINE, SCALE=0, NARGS=0 (one angle, reset modulus defaults to unity),
     * NRES=1 (cos then sin), ARGSIZE=RESSIZE=0 (q1.31).
     * Bit encodings and RCC reset/clock bits are supplied by the STM32 LL headers.
     */
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
}
