/* SPDX-License-Identifier: Apache-2.0 */

/* Minimal check helpers; each test executable returns non-zero on failure. */

#ifndef FOC_TEST_H
#define FOC_TEST_H

#include <math.h>
#include <stdio.h>

static int foc_test_failures;

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            foc_test_failures++;                                                                                       \
            fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);                                   \
        }                                                                                                              \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                                                              \
    do {                                                                                                               \
        double act_ = (double)(actual);                                                                                \
        double exp_ = (double)(expected);                                                                              \
        if (!(fabs(act_ - exp_) <= (double)(tol))) {                                                                   \
            foc_test_failures++;                                                                                       \
            fprintf(stderr, "%s:%d: %s = %.9g, expected %.9g +- %g\n", __FILE__, __LINE__, #actual, act_, exp_,        \
                    (double)(tol));                                                                                    \
        }                                                                                                              \
    } while (0)

static inline int foc_test_result(const char* name) {
    if (foc_test_failures != 0) {
        fprintf(stderr, "%s: %d check(s) failed\n", name, foc_test_failures);
        return 1;
    }
    printf("%s: all checks passed\n", name);
    return 0;
}

/* Fixed-seed LCG so random-vector failures are reproducible. */
static inline float foc_test_rand(unsigned int* state, float lo, float hi) {
    *state = *state * 1664525u + 1013904223u;
    return lo + (hi - lo) * (float)(*state >> 8) * (1.0f / 16777216.0f);
}

#define FOC_TEST_PI 3.14159265358979323846

#endif /* FOC_TEST_H */
