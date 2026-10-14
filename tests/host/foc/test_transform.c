/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Oracle: physical vectors with known answers (balanced cosine sets, axis
 * unit vectors), not a copy of the formulas under test.
 */

#include "foc/foc_transform.h"
#include "foc_test.h"

#define TOL 1e-5

static foc_sincos_t sincos_of(double theta) {
    foc_sincos_t sc = {.sin_theta = (float)sin(theta), .cos_theta = (float)cos(theta)};

    return sc;
}

/* Positive-sequence balanced set: peak 1, phase A at angle theta. */
static foc_abc_t balanced(double theta, double peak) {
    foc_abc_t abc = {
        .a = (float)(peak * cos(theta)),
        .b = (float)(peak * cos(theta - 2.0 * FOC_TEST_PI / 3.0)),
        .c = (float)(peak * cos(theta + 2.0 * FOC_TEST_PI / 3.0)),
    };

    return abc;
}

static void test_clarke_zero(void) {
    foc_ab_t ab = foc_clarke((foc_abc_t){0});

    CHECK_NEAR(ab.alpha, 0.0, TOL);
    CHECK_NEAR(ab.beta, 0.0, TOL);
}

static void test_clarke_balanced_set_is_rotating_vector(void) {
    for (int i = 0; i <= 72; i++) {
        double theta = 2.0 * FOC_TEST_PI * i / 72.0;
        foc_ab_t ab = foc_clarke(balanced(theta, 2.5));

        /* Amplitude-invariant: vector length equals phase peak. */
        CHECK_NEAR(ab.alpha, 2.5 * cos(theta), TOL);
        CHECK_NEAR(ab.beta, 2.5 * sin(theta), TOL);
    }
}

static void test_clarke_rejects_common_mode(void) {
    foc_abc_t abc = balanced(0.7, 1.0);
    foc_ab_t ref = foc_clarke(abc);

    abc.a += 0.3f;
    abc.b += 0.3f;
    abc.c += 0.3f;
    foc_ab_t shifted = foc_clarke(abc);

    CHECK_NEAR(shifted.alpha, ref.alpha, TOL);
    CHECK_NEAR(shifted.beta, ref.beta, TOL);
}

static void test_two_shunt_reconstruction(void) {
    /* The board measures A and C only. */
    foc_abc_t full = balanced(1.9, 3.0);
    foc_abc_t two_shunt = {.a = full.a, .b = -(full.a + full.c), .c = full.c};
    foc_ab_t ref = foc_clarke(full);
    foc_ab_t ab = foc_clarke(two_shunt);

    CHECK_NEAR(ab.alpha, ref.alpha, TOL);
    CHECK_NEAR(ab.beta, ref.beta, TOL);
}

static void test_inv_clarke_round_trip(void) {
    unsigned int seed = 12345u;

    for (int i = 0; i < 1000; i++) {
        foc_ab_t ab = {foc_test_rand(&seed, -10.0f, 10.0f), foc_test_rand(&seed, -10.0f, 10.0f)};
        foc_abc_t abc = foc_inv_clarke(ab);
        foc_ab_t back = foc_clarke(abc);

        CHECK_NEAR(abc.a + abc.b + abc.c, 0.0, 1e-4);
        CHECK_NEAR(back.alpha, ab.alpha, 1e-4);
        CHECK_NEAR(back.beta, ab.beta, 1e-4);
    }
}

static void test_park_axes(void) {
    /* theta = 0: d is alpha, q is beta. */
    foc_dq_t dq = foc_park((foc_ab_t){.alpha = 0.0f, .beta = 1.0f}, sincos_of(0.0));

    CHECK_NEAR(dq.d, 0.0, TOL);
    CHECK_NEAR(dq.q, 1.0, TOL);

    /* theta = +90 deg: d axis now points along beta. */
    dq = foc_park((foc_ab_t){.alpha = 0.0f, .beta = 1.0f}, sincos_of(FOC_TEST_PI / 2.0));
    CHECK_NEAR(dq.d, 1.0, TOL);
    CHECK_NEAR(dq.q, 0.0, TOL);

    /* theta = 180 deg: alpha maps to -d. */
    dq = foc_park((foc_ab_t){.alpha = 1.0f, .beta = 0.0f}, sincos_of(FOC_TEST_PI));
    CHECK_NEAR(dq.d, -1.0, TOL);
    CHECK_NEAR(dq.q, 0.0, TOL);
}

static void test_park_tracks_rotating_vector(void) {
    /*
	 * A balanced current at angle theta + 90 deg is pure q current when
	 * observed at theta. This is what Id = 0, Iq > 0 control produces.
	 */
    for (int i = 0; i <= 72; i++) {
        double theta = -2.0 * FOC_TEST_PI + 6.0 * FOC_TEST_PI * i / 72.0;
        foc_ab_t ab = foc_clarke(balanced(theta + FOC_TEST_PI / 2.0, 1.5));
        foc_dq_t dq = foc_park(ab, sincos_of(theta));

        CHECK_NEAR(dq.d, 0.0, TOL);
        CHECK_NEAR(dq.q, 1.5, TOL);
    }
}

static void test_park_round_trip(void) {
    unsigned int seed = 777u;

    for (int i = 0; i < 1000; i++) {
        foc_ab_t ab = {foc_test_rand(&seed, -10.0f, 10.0f), foc_test_rand(&seed, -10.0f, 10.0f)};
        foc_sincos_t sc = sincos_of(foc_test_rand(&seed, -10.0f, 10.0f));
        foc_ab_t back = foc_inv_park(foc_park(ab, sc), sc);

        CHECK_NEAR(back.alpha, ab.alpha, 1e-4);
        CHECK_NEAR(back.beta, ab.beta, 1e-4);
    }
}

int main(void) {
    test_clarke_zero();
    test_clarke_balanced_set_is_rotating_vector();
    test_clarke_rejects_common_mode();
    test_two_shunt_reconstruction();
    test_inv_clarke_round_trip();
    test_park_axes();
    test_park_tracks_rotating_vector();
    test_park_round_trip();

    return foc_test_result("test_transform");
}
