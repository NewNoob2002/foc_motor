/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Compares the Stage 5 firmware results (FOC_VECTORS_TAG lines in a UART
 * log) with the same vectors computed on the host.
 *
 *   vec_compare <log file>   compare; exit 1 on a missing vector, a
 *                            different SVPWM limit flag or an error above
 *                            the tolerance
 *   vec_compare --selftest   check the parser and a few hand-computed
 *                            results (registered with CTest)
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "foc_test.h"
#include "foc_vectors.h"

/*
 * Tolerances. Inputs are bit-identical on both sides, so differences come
 * only from rounding (instruction choice, libm sinf/cosf): a few float ULPs.
 * 1e-4 A or V is far below anything measurable on the board; duty 1e-5 is
 * 0.04 TIM1 counts at ARR 4250.
 */
#define TOL_PHYS     1e-4
#define TOL_DUTY     1e-5

#define LINE_MAX_LEN 512

static const char* const field_names[FOC_VECTORS_OUT_FLOATS] = {
    "i_d", "i_q", "v_d", "v_q", "int_d", "int_q", "duty_a", "duty_b", "duty_c",
};

typedef struct {
    unsigned int index;
    float fields[FOC_VECTORS_OUT_FLOATS];
    unsigned int limited;
} vec_line_t;

static float bits_float(uint32_t u) {
    float f;

    (void)memcpy(&f, &u, sizeof(f));
    return f;
}

static uint32_t float_bits(float f) {
    uint32_t u;

    (void)memcpy(&u, &f, sizeof(u));
    return u;
}

/* Without the locale table lookup of isxdigit(). */
static bool is_hex_digit(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

/* Parses one unsigned field and the single space before it; false on anything else. */
static bool next_field(const char** p, int base, unsigned long max, unsigned long* value) {
    char* end;

    if (**p != ' ' || !is_hex_digit((*p)[1])) {
        return false;
    }
    errno = 0;
    *value = strtoul(*p + 1, &end, base);
    if (errno != 0 || *value > max) {
        return false;
    }
    *p = end;
    return true;
}

/* Returns true if the line holds a well-formed result line. */
static bool parse_line(const char* line, vec_line_t* v) {
    const char* p = strstr(line, FOC_VECTORS_TAG " ");
    unsigned long value;

    if (p == NULL) {
        return false;
    }
    p += strlen(FOC_VECTORS_TAG);
    if (!next_field(&p, 10, FOC_VECTORS_COUNT - 1U, &value)) {
        return false;
    }
    v->index = (unsigned int)value;
    for (size_t k = 0; k < FOC_VECTORS_OUT_FLOATS; k++) {
        if (!next_field(&p, 16, UINT32_MAX, &value)) {
            return false;
        }
        v->fields[k] = bits_float((uint32_t)value);
    }
    if (!next_field(&p, 10, 1U, &value)) {
        return false;
    }
    v->limited = (unsigned int)value;
    /* End of line, or the colour reset (ESC [ 0 m) that the Zephyr log adds. */
    return *p == '\0' || *p == '\r' || *p == '\n' || *p == '\x1b';
}

/* Same format as print_vector() in src/foc_bench.c. */
static void format_line(char* buf, size_t len, unsigned int index, const foc_vector_out_t* out) {
    float f[FOC_VECTORS_OUT_FLOATS];

    foc_vector_fields(out, f);
    (void)snprintf(buf, len,
                   "[00:00:00.000,000] <inf> foc_bench: " FOC_VECTORS_TAG
                   " %u %08x %08x %08x %08x %08x %08x %08x %08x %08x %u",
                   index, float_bits(f[0]), float_bits(f[1]), float_bits(f[2]), float_bits(f[3]), float_bits(f[4]),
                   float_bits(f[5]), float_bits(f[6]), float_bits(f[7]), float_bits(f[8]), out->limited ? 1U : 0U);
}

static int selftest(void) {
    foc_vector_out_t out;
    char buf[LINE_MAX_LEN];
    vec_line_t v;

    for (unsigned int i = 0; i < FOC_VECTORS_COUNT; i++) {
        float f[FOC_VECTORS_OUT_FLOATS];

        foc_vector_step(&foc_vectors_in[i], &out);
        foc_vector_fields(&out, f);
        format_line(buf, sizeof(buf), i, &out);
        if (!parse_line(buf, &v)) {
            CHECK(false);
            continue;
        }
        CHECK(v.index == i);
        CHECK(v.limited == (out.limited ? 1U : 0U));
        for (size_t k = 0; k < FOC_VECTORS_OUT_FLOATS; k++) {
            CHECK(float_bits(v.fields[k]) == float_bits(f[k]));
            if (k >= 6U) {
                CHECK(f[k] >= 0.0f && f[k] <= 1.0f);
            }
        }
    }
    CHECK(!parse_line("foc_vec 64 0 0 0 0 0 0 0 0 0 0", &v));
    CHECK(!parse_line("foc_vec 1 0 0 0", &v));
    CHECK(!parse_line("foc_vec 1 0 0 0 0 0 0 0 0 0 2", &v));
    CHECK(!parse_line("foc_vec 1 0 0 0 0 0 0 0 0 0 0 junk", &v));
    CHECK(parse_line("<inf> foc_bench: foc_vec 1 0 0 0 0 0 0 0 0 0 0\x1b[0m\r\n", &v));

    /* Row 0: nothing in, zero voltage out. */
    foc_vector_step(&foc_vectors_in[0], &out);
    CHECK_NEAR(out.duty.a, 0.5, 1e-7);
    CHECK_NEAR(out.duty.b, 0.5, 1e-7);
    CHECK_NEAR(out.duty.c, 0.5, 1e-7);
    CHECK(!out.limited);

    /* Row 1: a = 1, b = c = -0.5 at theta 0 is 1 A on d; the references match, so no PI output. */
    foc_vector_step(&foc_vectors_in[1], &out);
    CHECK_NEAR(out.i_dq.d, 1.0, 1e-6);
    CHECK_NEAR(out.i_dq.q, 0.0, 1e-6);
    CHECK_NEAR(out.v_dq.d, 0.0, 1e-6);
    CHECK_NEAR(out.v_dq.q, 0.0, 1e-6);

    /* Row 3: 10 A of q error, kp * 10 = 15 V > 24 / sqrt(3) = 13.86 V, so v_q sits at the limit. */
    foc_vector_step(&foc_vectors_in[3], &out);
    CHECK_NEAR(out.v_dq.q, 24.0 / sqrt(3.0), 1e-5);

    /* Row 4: both axes saturate at 10 V bus; the vector is longer than the hexagon's circle. */
    foc_vector_step(&foc_vectors_in[4], &out);
    CHECK(out.limited);

    /* Row 5: integrals of +-20 V are clamped to +-24 / sqrt(3). */
    foc_vector_step(&foc_vectors_in[5], &out);
    CHECK_NEAR(out.v_int.d, 24.0 / sqrt(3.0), 1e-5);
    CHECK_NEAR(out.v_int.q, -24.0 / sqrt(3.0), 1e-5);

    return foc_test_result("vec_compare selftest");
}

static int compare(const char* path) {
    FILE* fp = fopen(path, "r");
    char line[LINE_MAX_LEN];
    bool seen[FOC_VECTORS_COUNT] = {false};
    double max_err[FOC_VECTORS_OUT_FLOATS] = {0.0};
    unsigned int max_at[FOC_VECTORS_OUT_FLOATS] = {0U};
    unsigned int differ[FOC_VECTORS_OUT_FLOATS] = {0U};
    unsigned int n_seen = 0U;
    unsigned int n_limited_bad = 0U;
    bool ok = true;

    if (fp == NULL) {
        perror(path);
        return 1;
    }
    while (fgets(line, sizeof(line), fp) != NULL) {
        vec_line_t v;
        foc_vector_out_t out;
        float host[FOC_VECTORS_OUT_FLOATS];

        if (!parse_line(line, &v)) {
            continue;
        }
        if (seen[v.index]) {
            fprintf(stderr, "vector %u appears twice; using the first\n", v.index);
            continue;
        }
        seen[v.index] = true;
        n_seen++;
        foc_vector_step(&foc_vectors_in[v.index], &out);
        foc_vector_fields(&out, host);
        if (v.limited != (out.limited ? 1U : 0U)) {
            fprintf(stderr, "vector %u: limited target %u host %d\n", v.index, v.limited, out.limited ? 1 : 0);
            n_limited_bad++;
        }
        for (size_t k = 0; k < FOC_VECTORS_OUT_FLOATS; k++) {
            double err = fabs((double)v.fields[k] - (double)host[k]);

            if (float_bits(v.fields[k]) != float_bits(host[k])) {
                differ[k]++;
            }
            if (!(err <= max_err[k])) {
                max_err[k] = err;
                max_at[k] = v.index;
            }
        }
    }
    (void)fclose(fp);

    printf("%u of %u vectors found, SVPWM limit flag differs in %u\n", n_seen, FOC_VECTORS_COUNT, n_limited_bad);
    printf("%-7s %12s %6s %9s %10s\n", "field", "max |err|", "at", "not exact", "tolerance");
    for (size_t k = 0; k < FOC_VECTORS_OUT_FLOATS; k++) {
        double tol = (k >= 6U) ? TOL_DUTY : TOL_PHYS;
        bool pass = max_err[k] <= tol;

        printf("%-7s %12.3g %6u %9u %10g %s\n", field_names[k], max_err[k], max_at[k], differ[k], tol,
               pass ? "ok" : "FAIL");
        ok = ok && pass;
    }
    ok = ok && n_seen == FOC_VECTORS_COUNT && n_limited_bad == 0U;
    printf("%s\n", ok ? "PASS: target matches host" : "FAIL");
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "--selftest") == 0) {
        return selftest();
    }
    if (argc == 2) {
        return compare(argv[1]);
    }
    fprintf(stderr, "usage: %s <log file> | --selftest\n", argv[0]);
    return 2;
}
