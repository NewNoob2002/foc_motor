/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_FOC_CTRL_MOTION_H
#define FOC_MOTOR_FOC_CTRL_MOTION_H

#include <stdint.h>

#include "foc_ctrl.h"

/** Fixed ISR-owned speed window, also used by the synthetic boot benchmark. */
typedef struct {
    uint32_t previous;
    uint32_t periods;
    int32_t delta;
    int32_t counts;
} foc_ctrl_speed_t;

/** Convert a raw encoder count to the calibrated electrical frame modulo 4096. */
static inline uint32_t foc_ctrl_rotor_angle(uint32_t enc, int32_t direction, uint32_t offset) {
    uint32_t raw = ((enc & 4095U) * 7U) & 4095U;
    uint32_t directed = direction < 0 ? 0U - raw : raw;

    return (directed - offset) & 4095U;
}

/** Accumulate shortest signed encoder increments and publish one count estimate per 200 periods. */
static inline void foc_ctrl_speed_step(foc_ctrl_speed_t* speed, uint32_t enc) {
    int32_t delta = (int32_t)((enc - speed->previous + 2048U) & 4095U) - 2048;

    speed->previous = enc;
    speed->delta += delta;
    speed->periods++;
    if (speed->periods == FOC_CTRL_SPEED_PERIODS) {
        speed->counts = speed->delta;
        speed->periods = 0U;
        speed->delta = 0;
    }
}

#endif /* FOC_MOTOR_FOC_CTRL_MOTION_H */
