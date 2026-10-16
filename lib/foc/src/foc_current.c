/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_current.h"

#include <math.h>

#include "foc/foc_svpwm.h"

#define FOC_INV_SQRT3_F 0.57735027f

/** Clamp a normalized phase current without out-of-line min/max calls. */
static inline float phase_comp(float i, float inv_band, float vdt) {
    float sign = i * inv_band;
    if (sign > 1.0f) {
        sign = 1.0f;
    } else if (sign < -1.0f) {
        sign = -1.0f;
    }
    return vdt * sign;
}

/** Construct a smooth per-phase compensation vector with a precomputed reciprocal. */
foc_ab_t foc_deadtime_comp_inv(foc_abc_t i_abc, float vdt, float inv_band) {
    foc_abc_t v;
    v.a = phase_comp(i_abc.a, inv_band, vdt);
    v.b = phase_comp(i_abc.b, inv_band, vdt);
    v.c = phase_comp(i_abc.c, inv_band, vdt);
    return foc_clarke(v);
}

/** Run transforms and capped PIs without modulation; return false for invalid limits. */
static bool current_pi_limit(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus,
                             float dt_s, float v_lim, foc_current_out_t* out);

/** Reserve PI headroom, add feed-forward, then cap the complete applied voltage. */
void foc_current_step_ff(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus, float dt_s,
                         foc_ab_t feedforward, float total_lim, foc_current_out_t* out) {
    float pi_lim = total_lim - sqrtf(feedforward.alpha * feedforward.alpha + feedforward.beta * feedforward.beta);
    if (pi_lim < 0.2f) {
        pi_lim = 0.2f;
    }
    if (!(total_lim > 0.0f)) {
        pi_lim = 0.0f;
    }
    if (!current_pi_limit(cc, i_abc, sc, i_ref, vbus, dt_s, pi_lim, out)) {
        return;
    }
    foc_ab_t total = foc_inv_park(out->v_dq, sc);
    total.alpha += feedforward.alpha;
    total.beta += feedforward.beta;
    float length_sq = total.alpha * total.alpha + total.beta * total.beta;
    if (length_sq > total_lim * total_lim) {
        float scale = total_lim / sqrtf(length_sq);
        total.alpha *= scale;
        total.beta *= scale;
        out->limited = true;
    }
    out->v_dq = foc_park(total, sc);
    out->limited = foc_svpwm(total, vbus, &out->duty) || out->limited;
}

void foc_current_step(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus, float dt_s,
                      foc_current_out_t* out) {
    out->i_dq = foc_park(foc_clarke(i_abc), sc);
    if (!(vbus > 0.0f)) {
        /* No PI step: a zero or inverted limit range would break the PI precondition. */
        out->v_dq = (foc_dq_t){.d = 0.0f, .q = 0.0f};
        out->duty = (foc_abc_t){.a = 0.5f, .b = 0.5f, .c = 0.5f};
        out->limited = true;
        return;
    }

    float v_lim = vbus * FOC_INV_SQRT3_F;

    cc->pi_d.out_min = -v_lim;
    cc->pi_d.out_max = v_lim;
    cc->pi_q.out_min = -v_lim;
    cc->pi_q.out_max = v_lim;
    out->v_dq.d = foc_pi_step(&cc->pi_d, i_ref.d - out->i_dq.d, dt_s);
    out->v_dq.q = foc_pi_step(&cc->pi_q, i_ref.q - out->i_dq.q, dt_s);
    out->limited = foc_svpwm(foc_inv_park(out->v_dq, sc), vbus, &out->duty);
}

/** Compute measured dq and bounded PI voltage, leaving modulation to the caller. */
static bool current_pi_limit(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus,
                             float dt_s, float v_lim, foc_current_out_t* out) {
    out->i_dq = foc_park(foc_clarke(i_abc), sc);
    if (!(vbus > 0.0f) || !(v_lim > 0.0f)) {
        out->v_dq.d = 0.0f;
        out->v_dq.q = 0.0f;
        out->duty.a = 0.5f;
        out->duty.b = 0.5f;
        out->duty.c = 0.5f;
        out->limited = true;
        return false;
    }
    float axis_lim = vbus * FOC_INV_SQRT3_F;

    if (axis_lim > v_lim) {
        axis_lim = v_lim;
    }
    cc->pi_d.out_min = -axis_lim;
    cc->pi_d.out_max = axis_lim;
    cc->pi_q.out_min = -axis_lim;
    cc->pi_q.out_max = axis_lim;
    out->v_dq.d = foc_pi_step(&cc->pi_d, i_ref.d - out->i_dq.d, dt_s);
    out->v_dq.q = foc_pi_step(&cc->pi_q, i_ref.q - out->i_dq.q, dt_s);
    bool limited = fabsf(out->v_dq.d) >= axis_lim || fabsf(out->v_dq.q) >= axis_lim;
    float length_sq = out->v_dq.d * out->v_dq.d + out->v_dq.q * out->v_dq.q;

    if (length_sq > v_lim * v_lim) {
        float scale = v_lim / sqrtf(length_sq);

        out->v_dq.d *= scale;
        out->v_dq.q *= scale;
        limited = true;
    }
    out->limited = limited;
    return true;
}

/** Execute the current loop with an explicit applied-vector cap and matching PI bounds. */
void foc_current_step_lim(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus, float dt_s,
                          float v_lim, foc_current_out_t* out) {
    if (current_pi_limit(cc, i_abc, sc, i_ref, vbus, dt_s, v_lim, out)) {
        out->limited = foc_svpwm(foc_inv_park(out->v_dq, sc), vbus, &out->duty) || out->limited;
    }
}
