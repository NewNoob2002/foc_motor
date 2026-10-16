/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_SINCOS_H
#define FOC_MOTOR_SINCOS_H

#include <math.h>
#include <stdint.h>

/** Provide the BSP init API without a peripheral in host tests. */
static inline void sincos_init(void) {}

/** Match the BSP signed q1.31 angle mapping using host libm. */
static inline void sincos_e_count(uint32_t e_count, float* s, float* c) {
    int32_t angle = (int32_t)(e_count << 20U);
    float theta = (float)angle * (0x1p-31f * 3.14159265358979323846f);
    *s = sinf(theta);
    *c = cosf(theta);
}

#endif /* FOC_MOTOR_SINCOS_H */
