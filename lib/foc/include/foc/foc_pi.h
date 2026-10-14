/* SPDX-License-Identifier: Apache-2.0 */

/*
 * PI regulator with output saturation and anti-windup.
 *
 * Contract checked by tests/host/foc/test_pi.c:
 * - output = kp * error + integral, saturated to [out_min, out_max].
 * - integral is stored in output units: each step adds ki * error * dt_s.
 * - Anti-windup: while the output is saturated, the integral must not keep
 *   growing in the saturated direction, so the output leaves the limit on
 *   the first step after the error changes sign.
 * - foc_pi_reset() clears the integral and keeps gains and limits.
 *
 * Preconditions (the caller validates them): finite inputs, dt_s > 0,
 * out_min < out_max. Limits may change between calls (e.g. a voltage limit
 * that follows the DC bus).
 */

#ifndef FOC_PI_H
#define FOC_PI_H

typedef struct {
    float kp; /* output unit per error unit */
    float ki; /* output unit per (error unit * second) */
    float out_min;
    float out_max;
    float integral; /* state, output unit */
} foc_pi_t;

void foc_pi_reset(foc_pi_t* pi);

float foc_pi_step(foc_pi_t* pi, float error, float dt_s);

#endif /* FOC_PI_H */
