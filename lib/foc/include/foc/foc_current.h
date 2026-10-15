/* SPDX-License-Identifier: Apache-2.0 */

/*
 * One step of the field-oriented current loop:
 * Clarke -> Park -> PI on d and q -> inverse Park -> SVPWM.
 *
 * Contract checked by tests/host/foc/test_current.c:
 * - Same result as calling foc_clarke, foc_park, foc_pi_step (d then q),
 *   foc_inv_park and foc_svpwm in that order.
 * - Each PI output is limited to +-vbus/sqrt(3) (the SVPWM linear range);
 *   the step overwrites out_min/out_max, the caller sets kp, ki and the
 *   integral. SVPWM may still shorten the (v_d, v_q) vector.
 * - If vbus is not > 0 (including NaN): i_dq is still computed, the PIs are
 *   not stepped (integrals unchanged), v_dq is 0, all duties are 0.5 and
 *   limited is true.
 *
 * Preconditions (the caller validates them): finite currents, sin/cos of
 * the same angle, dt_s > 0. With two shunts on A and C the caller
 * reconstructs b = -(a + c).
 */

#ifndef FOC_CURRENT_H
#define FOC_CURRENT_H

#include <stdbool.h>

#include "foc/foc_pi.h"
#include "foc/foc_transform.h"

typedef struct {
    foc_pi_t pi_d; /* V per A of d error */
    foc_pi_t pi_q; /* V per A of q error */
} foc_current_t;

typedef struct {
    foc_dq_t i_dq;  /* A, measured */
    foc_dq_t v_dq;  /* V, PI outputs */
    foc_abc_t duty; /* high-side duty, [0, 1] */
    bool limited;   /* SVPWM shortened the vector, or vbus not > 0 */
} foc_current_out_t;

void foc_current_step(foc_current_t* cc, foc_abc_t i_abc, foc_sincos_t sc, foc_dq_t i_ref, float vbus, float dt_s,
                      foc_current_out_t* out);

#endif /* FOC_CURRENT_H */
