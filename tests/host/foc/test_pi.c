/* SPDX-License-Identifier: Apache-2.0 */

#include "foc/foc_pi.h"
#include "foc_test.h"

#define TOL 1e-5
#define DT  50e-6f

static foc_pi_t make_pi(float kp, float ki) {
    foc_pi_t pi = {.kp = kp, .ki = ki, .out_min = -10.0f, .out_max = 10.0f};

    foc_pi_reset(&pi);
    return pi;
}

static void test_zero_error_gives_zero(void) {
    foc_pi_t pi = make_pi(2.0f, 1000.0f);

    for (int i = 0; i < 100; i++) {
        CHECK_NEAR(foc_pi_step(&pi, 0.0f, DT), 0.0, TOL);
    }
}

static void test_proportional_only(void) {
    foc_pi_t pi = make_pi(2.0f, 0.0f);

    CHECK_NEAR(foc_pi_step(&pi, 1.5f, DT), 3.0, TOL);
    CHECK_NEAR(foc_pi_step(&pi, -0.5f, DT), -1.0, TOL);
}

static void test_integral_accumulates(void) {
    /* ki * error * time = 1000 * 0.5 * (200 * 50 us) = 5.0 */
    foc_pi_t pi = make_pi(0.0f, 1000.0f);
    float out = 0.0f;

    for (int i = 0; i < 200; i++) {
        out = foc_pi_step(&pi, 0.5f, DT);
    }
    CHECK_NEAR(out, 5.0, 1e-3);
    CHECK_NEAR(pi.integral, 5.0, 1e-3);
}

static void test_output_is_saturated(void) {
    foc_pi_t pi = make_pi(100.0f, 1000.0f);

    for (int i = 0; i < 100; i++) {
        float out = foc_pi_step(&pi, 1.0f, DT);

        CHECK(out <= pi.out_max);
    }
    for (int i = 0; i < 100; i++) {
        float out = foc_pi_step(&pi, -1.0f, DT);

        CHECK(out >= pi.out_min);
    }
}

static void test_anti_windup_recovers_immediately(void) {
    foc_pi_t pi = make_pi(1.0f, 1000.0f);

    float out = 0.0f;

    /* Without anti-windup the integral would reach 1000 * 1 * 1 s = 1000. */
    for (int i = 0; i < 20000; i++) {
        out = foc_pi_step(&pi, 1.0f, DT);
    }
    CHECK_NEAR(out, 10.0, TOL);
    CHECK(foc_pi_step(&pi, -0.1f, DT) < pi.out_max);
    CHECK(pi.integral <= pi.out_max);
}

static void test_limits_can_change(void) {
    foc_pi_t pi = make_pi(0.0f, 1000.0f);

    for (int i = 0; i < 400; i++) {
        (void)foc_pi_step(&pi, 1.0f, DT);
    }
    /* Bus voltage dropped: the new limit applies at once. */
    pi.out_max = 3.0f;
    CHECK(foc_pi_step(&pi, 1.0f, DT) <= 3.0f);
}

static void test_reset_clears_state_only(void) {
    foc_pi_t pi = make_pi(2.0f, 1000.0f);

    for (int i = 0; i < 50; i++) {
        (void)foc_pi_step(&pi, 1.0f, DT);
    }
    foc_pi_reset(&pi);
    CHECK_NEAR(pi.integral, 0.0, TOL);
    CHECK_NEAR(pi.kp, 2.0, TOL);
    CHECK_NEAR(pi.ki, 1000.0, TOL);
    CHECK_NEAR(foc_pi_step(&pi, 0.0f, DT), 0.0, TOL);
}

int main(void) {
    test_zero_error_gives_zero();
    test_proportional_only();
    test_integral_accumulates();
    test_output_is_saturated();
    test_anti_windup_recovers_immediately();
    test_limits_can_change();
    test_reset_clears_state_only();

    return foc_test_result("test_pi");
}
