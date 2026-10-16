/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_FOC_SPEED_H
#define FOC_MOTOR_FOC_SPEED_H

#include <stdbool.h>
#include <stdint.h>

#include "foc/foc_pi.h"

#define FOC_CTRL_SPEED_PERIODS     200U
#define FOC_CTRL_SPD_DIV           20U
#define FOC_CTRL_SPD_WINDOW        10U
#define FOC_CTRL_SPD_COUNTS_TO_RPM 1.46484375f /* 60 / (4096 * 0.01) */
#define FOC_CTRL_SPD_IQ_MAX_A      0.5f
#define FOC_CTRL_SPD_MAX_RPM       200
#define FOC_CTRL_SPD_RAMP_RPM_S    400.0f
#define FOC_CTRL_J_EST             1.5e-5f
#define FOC_CTRL_SPD_KT            0.0217f
#define FOC_CTRL_SPD_BW_HZ         10.0f
/* J is an assumption, not a measured inertia. With ws = 2*pi*10,
 * kp = J*ws/Kt in A/(rad/s), then multiply by 2*pi/60 for A/rpm.
 * ki = kp*ws/4 puts the PI zero at ws/4: about 4.55 mA/rpm and
 * 71.44 mA/(rpm s). All gain arithmetic is evaluated at compile time.
 */
#define FOC_CTRL_SPD_WS            (6.28318530718f * FOC_CTRL_SPD_BW_HZ)
#define FOC_CTRL_SPD_KP            (FOC_CTRL_J_EST * FOC_CTRL_SPD_WS / FOC_CTRL_SPD_KT * (6.28318530718f / 60.0f))
#define FOC_CTRL_SPD_KI            (FOC_CTRL_SPD_KP * FOC_CTRL_SPD_WS * 0.25f)

/** ISR-owned rolling 10 ms encoder window, refreshed at 1 kHz.
 * The rectangular window has about 5 ms effective delay and a resolution
 * of 1.46484375 rpm/count. Startup zero-pads the ten 1 ms buckets.
 * counts is raw encoder direction; convert to the calibrated frame for PI.
 */
typedef struct {
    uint32_t previous;
    uint32_t periods;
    uint32_t index;
    int32_t delta;
    int32_t counts;
    int32_t trip_counts; /* Published every 10 ms, preserving the trip cadence. */
    int16_t ring[FOC_CTRL_SPD_WINDOW];
} foc_ctrl_speed_t;

/** Pure numeric speed PI state; iq holds between 1 ms updates. */
typedef struct {
    foc_pi_t pi;
    float ref;
    float iq;
} foc_speed_loop_t;

/** Reset the estimator without a byte-loop memory clear. */
void foc_ctrl_speed_reset(foc_ctrl_speed_t* speed, uint32_t enc);

/** Accumulate a shortest modulo-4096 delta; return true on each twentieth period. */
bool foc_ctrl_speed_step(foc_ctrl_speed_t* speed, uint32_t enc);

/** Reset the PI, ramp and held current reference with compile-time gains. */
void foc_speed_loop_reset(foc_speed_loop_t* loop);

/** Run one 1 ms ramp/PI update; step bypasses the 400 rpm/s reference ramp. */
void foc_speed_loop_update(foc_speed_loop_t* loop, float target, float rpm, bool step);

#endif /* FOC_MOTOR_FOC_SPEED_H */
