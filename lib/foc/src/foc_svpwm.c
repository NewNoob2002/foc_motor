/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_svpwm.h"
#include <math.h> /* sqrtf */
#include "foc/foc_transform.h"

bool foc_svpwm(foc_ab_t v_ab, float vbus, foc_abc_t* duty) {
    bool limited = false;
    foc_abc_t v_a_b_c;
    foc_abc_t v_a_b_c_shifted;
    if (!(vbus > 0.0f)) {
        duty->a = 0.5f;
        duty->b = 0.5f;
        duty->c = 0.5f;
        return true;
    }
    float mag = sqrtf(v_ab.alpha * v_ab.alpha + v_ab.beta * v_ab.beta);
    if (mag > vbus / sqrtf(3.0f)) {
        limited = true;
        v_ab.alpha *= (vbus / sqrtf(3.0f)) / mag;
        v_ab.beta *= (vbus / sqrtf(3.0f)) / mag;
    }

    v_a_b_c = foc_inv_clarke(v_ab);

    float hi = v_a_b_c.a > v_a_b_c.b ? v_a_b_c.a : v_a_b_c.b;
    hi = hi > v_a_b_c.c ? hi : v_a_b_c.c;

    float lo = v_a_b_c.a < v_a_b_c.b ? v_a_b_c.a : v_a_b_c.b;
    lo = lo < v_a_b_c.c ? lo : v_a_b_c.c;

    float v_offset = 0.5f * (hi + lo);
    v_a_b_c_shifted.a = v_a_b_c.a - v_offset;
    v_a_b_c_shifted.b = v_a_b_c.b - v_offset;
    v_a_b_c_shifted.c = v_a_b_c.c - v_offset;

    duty->a = 0.5f + v_a_b_c_shifted.a / vbus;
    duty->b = 0.5f + v_a_b_c_shifted.b / vbus;
    duty->c = 0.5f + v_a_b_c_shifted.c / vbus;
    return limited;
}
