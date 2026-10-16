/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_SINCOS_H
#define FOC_MOTOR_SINCOS_H

#include <stdint.h>
#include <stm32g4xx_ll_cordic.h>

/** Enable and configure the exclusive CORDIC before the boot bench and PWM ISR. */
void sincos_init(void);

/**
 * Compute sine/cosine for 0..4095 electrical counts (4096 = 2*pi).
 * Only the FOC ISR may call this after it starts; the boot bench is the sole
 * earlier user. CORDIC has one shared argument/result state and is not reentrant.
 * RM0440, CORDIC functional description (zero-overhead mode) and WDATA/RDATA:
 * the first blocking RDATA read stalls the bus until ready; COSINE returns
 * cosine first, sine second. Register definitions come from stm32g4xx_ll_cordic.h.
 */
static inline __attribute__((always_inline)) void sincos_e_count(uint32_t e_count, float* s, float* c) {
    int32_t angle = (int32_t)(e_count << 20U);
    CORDIC->WDATA = (uint32_t)angle;
    int32_t cosine = (int32_t)CORDIC->RDATA;
    int32_t sine = (int32_t)CORDIC->RDATA;
    *c = (float)cosine * 0x1p-31f;
    *s = (float)sine * 0x1p-31f;
}

#endif /* FOC_MOTOR_SINCOS_H */
