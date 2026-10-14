/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Clarke and Park transforms.
 *
 * Conventions (frozen for this project, see docs/study/foc_study_guide.md):
 * - SI units: amperes or volts in, same unit out; angles in radians.
 * - Positive phase sequence is A -> B -> C.
 * - Clarke is amplitude-invariant: a balanced set with peak X gives an
 *   alpha/beta vector of length X. The alpha axis is aligned with phase A.
 * - theta is the electrical angle of the d axis measured from alpha;
 *   q leads d by +90 degrees.
 * - Callers pass sin/cos instead of theta so the board can use the CORDIC
 *   unit while this module stays free of hardware and libm dependencies.
 */

#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

typedef struct {
    float a;
    float b;
    float c;
} foc_abc_t;

typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

typedef struct {
    float d;
    float q;
} foc_dq_t;

typedef struct {
    float sin_theta;
    float cos_theta;
} foc_sincos_t;

/*
 * Rejects any common-mode (zero-sequence) component. With two shunts on
 * phases A and C, reconstruct b = -(a + c) before calling.
 */
foc_ab_t foc_clarke(foc_abc_t abc);

/* Returns a zero-sequence-free three-phase set. */
foc_abc_t foc_inv_clarke(foc_ab_t ab);

foc_dq_t foc_park(foc_ab_t ab, foc_sincos_t sc);

foc_ab_t foc_inv_park(foc_dq_t dq, foc_sincos_t sc);

#endif /* FOC_TRANSFORM_H */
