/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_FOC_CTRL_MOTION_H
#define FOC_MOTOR_FOC_CTRL_MOTION_H

#include <stdint.h>

#include "foc_speed.h"

/** Convert a raw encoder count to the calibrated electrical frame modulo 4096. */
static inline uint32_t foc_ctrl_rotor_angle(uint32_t enc, int32_t direction, uint32_t offset) {
    uint32_t raw = ((enc & 4095U) * 7U) & 4095U;
    uint32_t directed = direction < 0 ? 0U - raw : raw;

    return (directed - offset) & 4095U;
}

#endif /* FOC_MOTOR_FOC_CTRL_MOTION_H */
