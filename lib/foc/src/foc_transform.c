/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_transform.h"

#define FOC_SQRT3_F     1.7320508f
#define FOC_INV_SQRT3_F 0.57735027f

foc_ab_t foc_clarke(foc_abc_t abc) {
    /* Using all three phases (not just a and b) cancels common mode. */
    foc_ab_t ab = {
        .alpha = (2.0f * abc.a - abc.b - abc.c) * (1.0f / 3.0f),
        .beta = (abc.b - abc.c) * FOC_INV_SQRT3_F,
    };

    return ab;
}

foc_abc_t foc_inv_clarke(foc_ab_t ab) {
    float half_sqrt3_beta = 0.5f * FOC_SQRT3_F * ab.beta;
    foc_abc_t abc = {
        .a = ab.alpha,
        .b = -0.5f * ab.alpha + half_sqrt3_beta,
        .c = -0.5f * ab.alpha - half_sqrt3_beta,
    };

    return abc;
}

foc_dq_t foc_park(foc_ab_t ab, foc_sincos_t sc) {
    foc_dq_t dq = {
        .d = ab.alpha * sc.cos_theta + ab.beta * sc.sin_theta,
        .q = -ab.alpha * sc.sin_theta + ab.beta * sc.cos_theta,
    };

    return dq;
}

foc_ab_t foc_inv_park(foc_dq_t dq, foc_sincos_t sc) {
    foc_ab_t ab = {
        .alpha = dq.d * sc.cos_theta - dq.q * sc.sin_theta,
        .beta = dq.d * sc.sin_theta + dq.q * sc.cos_theta,
    };

    return ab;
}
