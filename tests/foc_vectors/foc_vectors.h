/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Fixed test vectors for one current-loop step, shared by the host
 * comparison (tests/host/foc/vec_compare.c) and the Stage 5 firmware
 * (src/foc_bench.c). Pure C11 + libm, like lib/foc.
 *
 * The inputs are decimal literals in a const table, so the host compiler and
 * the ARM compiler turn them into the same float bits; any difference in the
 * outputs comes from the arithmetic (FMA contraction, libm sinf/cosf), not
 * from the inputs.
 */

#ifndef FOC_VECTORS_H
#define FOC_VECTORS_H

#include <stdbool.h>
#include <stdint.h>

#include "foc/foc_transform.h"

#define FOC_VECTORS_COUNT 64U

/* 20 kHz current loop. */
#define FOC_VECTORS_DT_S  50e-6f

/* PI gains for both axes; with up to 20 A of error kp alone saturates the output. */
#define FOC_VECTORS_KP    1.5f    /* V/A */
#define FOC_VECTORS_KI    3000.0f /* V/(A*s) */

typedef struct {
    float ia;     /* A, phase A shunt */
    float ic;     /* A, phase C shunt */
    float theta;  /* rad, electrical angle of the d axis */
    float id_ref; /* A */
    float iq_ref; /* A */
    float vbus;   /* V, > 0 */
    float int_d;  /* V, d-axis PI integral before the step */
    float int_q;  /* V, q-axis PI integral before the step */
} foc_vector_in_t;

typedef struct {
    foc_dq_t i_dq;  /* A, measured current after Clarke and Park */
    foc_dq_t v_dq;  /* V, PI outputs */
    foc_dq_t v_int; /* V, PI integrals after the step */
    foc_abc_t duty; /* SVPWM high-side duty, [0, 1] */
    bool limited;   /* SVPWM scaled the vector down */
} foc_vector_out_t;

/*
 * Result line: FOC_VECTORS_TAG, the index, the FOC_VECTORS_OUT_FLOATS fields
 * as 8-digit hex float bits, then limited (0/1), separated by single spaces.
 * The firmware prints it; vec_compare finds it anywhere in a log line.
 */
#define FOC_VECTORS_TAG        "foc_vec"

/* Number of float fields in foc_vector_out_t, in the order of foc_vector_fields(). */
#define FOC_VECTORS_OUT_FLOATS 9U

extern const foc_vector_in_t foc_vectors_in[FOC_VECTORS_COUNT];

/*
 * One current-loop step: b = -(a + c), Clarke, sinf/cosf, Park, a PI per
 * axis limited to +-vbus/sqrt(3), inverse Park, SVPWM.
 */
void foc_vector_step(const foc_vector_in_t* in, foc_vector_out_t* out);

/* Copies the float outputs into fields[FOC_VECTORS_OUT_FLOATS] (i_d, i_q, v_d, v_q, int_d, int_q, duty a, b, c). */
void foc_vector_fields(const foc_vector_out_t* out, float* fields);

#endif /* FOC_VECTORS_H */
