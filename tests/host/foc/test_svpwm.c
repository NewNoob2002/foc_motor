/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"
#include "foc_test.h"

#define TOL  1e-4
#define VBUS 12.0f

static double vmax_linear(float vbus) {
    return vbus / sqrt(3.0);
}

static void check_duty_range(foc_abc_t d) {
    CHECK(d.a >= 0.0f && d.a <= 1.0f);
    CHECK(d.b >= 0.0f && d.b <= 1.0f);
    CHECK(d.c >= 0.0f && d.c <= 1.0f);
}

/* Line-to-line voltages the bridge applies, expressed back in alpha/beta. */
static foc_ab_t applied(foc_abc_t d, float vbus) {
    foc_abc_t v = {d.a * vbus, d.b * vbus, d.c * vbus};

    return foc_clarke(v);
}

static void test_zero_vector(void) {
    foc_abc_t d;

    CHECK(!foc_svpwm((foc_ab_t){0}, VBUS, &d));
    CHECK_NEAR(d.a, 0.5, TOL);
    CHECK_NEAR(d.b, 0.5, TOL);
    CHECK_NEAR(d.c, 0.5, TOL);
}

static void test_linear_range_sweep(void) {
    /* 0.999 of the limit at every angle, including sector boundaries. */
    double mag = 0.999 * vmax_linear(VBUS);

    for (int i = 0; i <= 360; i++) {
        double theta = 2.0 * FOC_TEST_PI * i / 360.0;
        foc_ab_t v = {(float)(mag * cos(theta)), (float)(mag * sin(theta))};
        foc_abc_t d;

        CHECK(!foc_svpwm(v, VBUS, &d));
        check_duty_range(d);

        foc_ab_t out = applied(d, VBUS);

        CHECK_NEAR(out.alpha, v.alpha, 1e-3);
        CHECK_NEAR(out.beta, v.beta, 1e-3);

        /*
		 * The linear circle touches the hexagon midway between active
		 * vectors (30, 90, ... deg); there the bus is fully used.
		 */
        if (i % 60 == 30) {
            float hi = fmaxf(d.a, fmaxf(d.b, d.c));
            float lo = fminf(d.a, fminf(d.b, d.c));

            CHECK_NEAR(hi - lo, 0.999, 1e-3);
        }
    }
}

static void test_small_vectors_are_centered(void) {
    /* Symmetric zero vectors: max and min duty are centered on 0.5. */
    foc_abc_t d;

    CHECK(!foc_svpwm((foc_ab_t){1.0f, -0.4f}, VBUS, &d));
    float hi = fmaxf(d.a, fmaxf(d.b, d.c));
    float lo = fminf(d.a, fminf(d.b, d.c));

    CHECK_NEAR(0.5f * (hi + lo), 0.5, TOL);
}

static void test_overmodulation_is_limited(void) {
    double mag = 2.0 * vmax_linear(VBUS);

    for (int i = 0; i < 12; i++) {
        double theta = 0.1 + 2.0 * FOC_TEST_PI * i / 12.0;
        foc_ab_t v = {(float)(mag * cos(theta)), (float)(mag * sin(theta))};
        foc_abc_t d;

        CHECK(foc_svpwm(v, VBUS, &d));
        check_duty_range(d);

        foc_ab_t out = applied(d, VBUS);

        /* Same angle, length clipped to the linear limit. */
        CHECK_NEAR(out.alpha, vmax_linear(VBUS) * cos(theta), 1e-3);
        CHECK_NEAR(out.beta, vmax_linear(VBUS) * sin(theta), 1e-3);
    }
}

static void test_invalid_bus_gives_zero_voltage(void) {
    const float bad[] = {0.0f, -5.0f, NAN};

    for (unsigned int i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        foc_abc_t d = {0};

        CHECK(foc_svpwm((foc_ab_t){1.0f, 1.0f}, bad[i], &d));
        CHECK_NEAR(d.a, 0.5, TOL);
        CHECK_NEAR(d.b, 0.5, TOL);
        CHECK_NEAR(d.c, 0.5, TOL);
    }
}

int main(void) {
    test_zero_vector();
    test_linear_range_sweep();
    test_small_vectors_are_centered();
    test_overmodulation_is_limited();
    test_invalid_bus_gives_zero_voltage();

    return foc_test_result("test_svpwm");
}
