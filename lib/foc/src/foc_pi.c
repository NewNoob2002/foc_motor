/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_pi.h"

static inline void clampf(float* x, float lo, float hi) {
    if (*x < lo) {
        *x = lo;
    } else if (*x > hi) {
        *x = hi;
    }
}

void foc_pi_reset(foc_pi_t* pi) {
    pi->integral = 0.0f;
}

float foc_pi_step(foc_pi_t* pi, float error, float dt_s) {
    /*
     * Anti-windup by clamping the integral to the output limits. After a long
     * saturation the integral sits at the limit, so recovery is slower than
     * with conditional integration when kp * error is large.
     */
    pi->integral += pi->ki * error * dt_s;
    clampf(&pi->integral, pi->out_min, pi->out_max);
    float output = pi->kp * error + pi->integral;
    clampf(&output, pi->out_min, pi->out_max);
    return output;
}
