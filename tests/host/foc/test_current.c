/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc_test.h"

#define DT 50e-6f

static foc_current_t make_cc(float kp, float ki) {
    foc_current_t cc = {
        .pi_d = {.kp = kp, .ki = ki},
        .pi_q = {.kp = kp, .ki = ki},
    };

    foc_pi_reset(&cc.pi_d);
    foc_pi_reset(&cc.pi_q);
    return cc;
}

static foc_sincos_t sc_of(float theta) {
    foc_sincos_t sc = {.sin_theta = sinf(theta), .cos_theta = cosf(theta)};

    return sc;
}

/* Bit-for-bit equal (also tells -0.0f from 0.0f). */
static int same_bits(float a, float b) {
    uint32_t ua;
    uint32_t ub;

    (void)memcpy(&ua, &a, sizeof(ua));
    (void)memcpy(&ub, &b, sizeof(ub));
    return ua == ub;
}

/* Bit-identical to the hand-written chain, on random inputs. */
static void test_matches_manual_chain(void) {
    unsigned int seed = 12345u;

    for (int n = 0; n < 2000; n++) {
        float ia = foc_test_rand(&seed, -10.0f, 10.0f);
        float ic = foc_test_rand(&seed, -10.0f, 10.0f);
        foc_abc_t i_abc = {.a = ia, .b = -(ia + ic), .c = ic};
        foc_sincos_t sc = sc_of(foc_test_rand(&seed, 0.0f, 6.2831853f));
        foc_dq_t i_ref = {.d = foc_test_rand(&seed, -5.0f, 5.0f), .q = foc_test_rand(&seed, -10.0f, 10.0f)};
        float vbus = foc_test_rand(&seed, 5.0f, 30.0f);
        float v_lim = vbus * 0.57735027f;
        foc_current_t cc = make_cc(1.5f, 3000.0f);
        foc_pi_t pd = {.kp = 1.5f, .ki = 3000.0f, .out_min = -v_lim, .out_max = v_lim};
        foc_pi_t pq = pd;
        foc_current_out_t out;
        foc_abc_t duty;

        cc.pi_d.integral = foc_test_rand(&seed, -5.0f, 5.0f);
        cc.pi_q.integral = foc_test_rand(&seed, -5.0f, 5.0f);
        pd.integral = cc.pi_d.integral;
        pq.integral = cc.pi_q.integral;

        foc_current_step(&cc, i_abc, sc, i_ref, vbus, DT, &out);

        foc_dq_t i_dq = foc_park(foc_clarke(i_abc), sc);
        foc_dq_t v_dq = {.d = foc_pi_step(&pd, i_ref.d - i_dq.d, DT), .q = foc_pi_step(&pq, i_ref.q - i_dq.q, DT)};
        bool limited = foc_svpwm(foc_inv_park(v_dq, sc), vbus, &duty);

        CHECK(same_bits(out.i_dq.d, i_dq.d) && same_bits(out.i_dq.q, i_dq.q));
        CHECK(same_bits(out.v_dq.d, v_dq.d) && same_bits(out.v_dq.q, v_dq.q));
        CHECK(same_bits(cc.pi_d.integral, pd.integral) && same_bits(cc.pi_q.integral, pq.integral));
        CHECK(same_bits(out.duty.a, duty.a) && same_bits(out.duty.b, duty.b) && same_bits(out.duty.c, duty.c));
        CHECK(out.limited == limited);
    }
}

/* Zero error with zero integrals: zero voltage, duties 0.5. */
static void test_zero_error(void) {
    foc_current_t cc = make_cc(2.0f, 1000.0f);
    foc_abc_t i_abc = {.a = 1.0f, .b = -0.5f, .c = -0.5f}; /* 1 A on d at theta 0 */
    foc_dq_t i_ref = {.d = 1.0f, .q = 0.0f};
    foc_current_out_t out;

    foc_current_step(&cc, i_abc, sc_of(0.0f), i_ref, 24.0f, DT, &out);
    CHECK_NEAR(out.i_dq.d, 1.0, 1e-6);
    CHECK_NEAR(out.v_dq.d, 0.0, 1e-6);
    CHECK_NEAR(out.v_dq.q, 0.0, 1e-6);
    CHECK_NEAR(out.duty.a, 0.5, 1e-6);
    CHECK_NEAR(out.duty.b, 0.5, 1e-6);
    CHECK_NEAR(out.duty.c, 0.5, 1e-6);
    CHECK(!out.limited);
}

/* A constant error drives both PIs to +-vbus/sqrt(3) and keeps them there. */
static void test_saturates_at_bus_limit(void) {
    foc_current_t cc = make_cc(0.5f, 1000.0f);
    foc_abc_t zero = {0.0f, 0.0f, 0.0f};
    foc_dq_t i_ref = {.d = -2.0f, .q = 3.0f};
    foc_current_out_t out = {0};
    double v_lim = 12.0 / sqrt(3.0);

    for (int n = 0; n < 1000; n++) {
        foc_current_step(&cc, zero, sc_of(1.0f), i_ref, 12.0f, DT, &out);
    }
    CHECK_NEAR(out.v_dq.d, -v_lim, 1e-5);
    CHECK_NEAR(out.v_dq.q, v_lim, 1e-5);
    CHECK_NEAR(cc.pi_d.integral, -v_lim, 1e-5);
    CHECK_NEAR(cc.pi_q.integral, v_lim, 1e-5);
    CHECK(out.limited); /* |(v_d, v_q)| = sqrt(2) * v_lim is outside the circle */
    CHECK(out.duty.a >= 0.0f && out.duty.a <= 1.0f);
}

/* The limits follow vbus on every call, whatever the caller left in them. */
static void test_limits_follow_vbus(void) {
    foc_current_t cc = make_cc(100.0f, 0.0f);
    foc_abc_t zero = {0.0f, 0.0f, 0.0f};
    foc_dq_t i_ref = {.d = 0.0f, .q = 1.0f};
    foc_current_out_t out;

    cc.pi_q.out_min = -1000.0f;
    cc.pi_q.out_max = 1000.0f;
    foc_current_step(&cc, zero, sc_of(0.0f), i_ref, 24.0f, DT, &out);
    CHECK_NEAR(out.v_dq.q, 24.0 / sqrt(3.0), 1e-5);
    foc_current_step(&cc, zero, sc_of(0.0f), i_ref, 6.0f, DT, &out);
    CHECK_NEAR(out.v_dq.q, 6.0 / sqrt(3.0), 1e-5);
}

static void test_no_bus(void) {
    const float bad[] = {0.0f, -12.0f, NAN};

    for (size_t k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
        foc_current_t cc = make_cc(1.0f, 1000.0f);
        foc_abc_t i_abc = {.a = 0.0f, .b = 0.8660254f, .c = -0.8660254f}; /* 1 A on beta */
        foc_dq_t i_ref = {.d = 0.0f, .q = 5.0f};
        foc_current_out_t out;

        cc.pi_d.integral = 0.25f;
        cc.pi_q.integral = -0.5f;
        foc_current_step(&cc, i_abc, sc_of(0.0f), i_ref, bad[k], DT, &out);
        CHECK_NEAR(out.i_dq.q, 1.0, 1e-5);
        CHECK(out.v_dq.d == 0.0f && out.v_dq.q == 0.0f);
        CHECK(out.duty.a == 0.5f && out.duty.b == 0.5f && out.duty.c == 0.5f);
        CHECK(out.limited);
        CHECK(cc.pi_d.integral == 0.25f && cc.pi_q.integral == -0.5f);
    }
}

int main(void) {
    test_matches_manual_chain();
    test_zero_error();
    test_saturates_at_bus_limit();
    test_limits_follow_vbus();
    test_no_bus();
    return foc_test_result("test_current");
}
