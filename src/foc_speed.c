/* SPDX-License-Identifier: Apache-2.0 */
#include "foc_speed.h"

/** Reset every ring bucket and scalar before accepting encoder increments. */
void foc_ctrl_speed_reset(foc_ctrl_speed_t* speed, uint32_t enc) {
    speed->previous = enc;
    speed->periods = 0U;
    speed->index = 0U;
    speed->delta = 0;
    speed->counts = 0;
    speed->trip_counts = 0;
    /* Volatile halfword stores prevent GCC from turning this bounded reset
     * into picolibc's byte-loop memset on mode entry in the ISR.
     */
    volatile int16_t* ring = speed->ring;
    for (uint32_t k = 0U; k < FOC_CTRL_SPD_WINDOW; k++) {
        ring[k] = 0;
    }
}

/** Publish a rolling sum every millisecond and the trip window every ten milliseconds. */
bool foc_ctrl_speed_step(foc_ctrl_speed_t* speed, uint32_t enc) {
    int32_t delta = (int32_t)((enc - speed->previous + 2048U) & 4095U) - 2048;

    speed->previous = enc;
    speed->delta += delta;
    speed->periods++;
    if (speed->periods != FOC_CTRL_SPD_DIV) {
        return false;
    }
    speed->counts += speed->delta - speed->ring[speed->index];
    speed->ring[speed->index] = (int16_t)speed->delta;
    speed->periods = 0U;
    speed->delta = 0;
    speed->index++;
    if (speed->index == FOC_CTRL_SPD_WINDOW) {
        speed->index = 0U;
        speed->trip_counts = speed->counts;
    }
    return true;
}

/** Initialize fixed gains and symmetric integral/output limits without aggregate copies. */
void foc_speed_loop_reset(foc_speed_loop_t* loop) {
    loop->pi.kp = FOC_CTRL_SPD_KP;
    loop->pi.ki = FOC_CTRL_SPD_KI;
    loop->pi.out_min = -FOC_CTRL_SPD_IQ_MAX_A;
    loop->pi.out_max = FOC_CTRL_SPD_IQ_MAX_A;
    loop->pi.integral = 0.0f;
    loop->ref = 0.0f;
    loop->iq = 0.0f;
}

/** Apply a bounded reference increment and reuse the current-loop integral-clamping PI. */
void foc_speed_loop_update(foc_speed_loop_t* loop, float target, float rpm, bool step) {
    float delta = target - loop->ref;
    const float increment = FOC_CTRL_SPD_RAMP_RPM_S * 0.001f;

    if (!step) {
        if (delta > increment) {
            delta = increment;
        } else if (delta < -increment) {
            delta = -increment;
        }
    }
    loop->ref += delta;
    loop->iq = foc_pi_step(&loop->pi, loop->ref - rpm, 0.001f);
}
