/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_current.h"

#include "foc/foc_svpwm.h"

#define FOC_INV_SQRT3_F 0.57735027f

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
