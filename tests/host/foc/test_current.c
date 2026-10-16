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

/** Compare the capped loop with an independent PI/transform chain and both voltage limits. */
static void test_limited_chain(void) {
    unsigned int seed = 4321U;

    for (uint32_t k = 0U; k < 2000U; k++) {
        float bus = foc_test_rand(&seed, 0.1f, 20.0f);
        float cap = foc_test_rand(&seed, 0.01f, 2.0f);
        float axis = fminf(cap, bus * 0.57735027f);
        foc_current_t cc = make_cc(0.44f, 1260.0f);
        foc_current_t manual = cc;
        foc_abc_t currents = {.a = foc_test_rand(&seed, -2.0f, 2.0f), .b = 0.0f, .c = 0.0f};
        foc_dq_t ref = {.d = foc_test_rand(&seed, -5.0f, 5.0f), .q = foc_test_rand(&seed, -5.0f, 5.0f)};
        foc_sincos_t sc = sc_of(foc_test_rand(&seed, -3.14f, 3.14f));
        foc_current_out_t out;
        foc_abc_t duty;

        manual.pi_d.out_min = manual.pi_q.out_min = -axis;
        manual.pi_d.out_max = manual.pi_q.out_max = axis;
        foc_dq_t measured = foc_park(foc_clarke(currents), sc);
        foc_dq_t volts = {.d = foc_pi_step(&manual.pi_d, ref.d - measured.d, DT),
                          .q = foc_pi_step(&manual.pi_q, ref.q - measured.q, DT)};
        float length = hypotf(volts.d, volts.q);

        if (length > cap) {
            volts.d *= cap / length;
            volts.q *= cap / length;
        }
        (void)foc_svpwm(foc_inv_park(volts, sc), bus, &duty);
        foc_current_step_lim(&cc, currents, sc, ref, bus, DT, cap, &out);
        CHECK_NEAR(out.i_dq.d, measured.d, 1e-6);
        CHECK_NEAR(out.i_dq.q, measured.q, 1e-6);
        CHECK_NEAR(out.v_dq.d, volts.d, 1e-6);
        CHECK_NEAR(out.v_dq.q, volts.q, 1e-6);
        CHECK_NEAR(out.duty.a, duty.a, 1e-6);
        CHECK_NEAR(out.duty.b, duty.b, 1e-6);
        CHECK_NEAR(out.duty.c, duty.c, 1e-6);
        CHECK(fabsf(out.v_dq.d) <= axis + 1e-6f && fabsf(out.v_dq.q) <= axis + 1e-6f);
        CHECK(hypotf(out.v_dq.d, out.v_dq.q) <= cap + 1e-6f);
        CHECK(fabsf(cc.pi_d.integral) <= axis && fabsf(cc.pi_q.integral) <= axis);
    }
}

/** Saturation must keep integrals within the applied cap and recover promptly on reversal. */
static void test_limited_recovery(void) {
    foc_current_t cc = make_cc(0.44f, 1260.0f);
    foc_abc_t zero = {0};
    foc_dq_t ref = {.d = 10.0f, .q = -10.0f};
    foc_current_out_t out;

    for (uint32_t k = 0U; k < 1000U; k++) {
        foc_current_step_lim(&cc, zero, sc_of(0.0f), ref, 15.0f, DT, 1.0f, &out);
    }
    CHECK(out.limited);
    CHECK_NEAR(hypotf(out.v_dq.d, out.v_dq.q), 1.0, 1e-6);
    CHECK(cc.pi_d.integral == 1.0f && cc.pi_q.integral == -1.0f);
    ref.d = -0.3f;
    ref.q = 0.3f;
    for (uint32_t k = 0U; k < 60U; k++) {
        foc_current_step_lim(&cc, zero, sc_of(0.0f), ref, 15.0f, DT, 1.0f, &out);
    }
    CHECK(out.v_dq.d < 0.0f && out.v_dq.q > 0.0f && !out.limited);
    const float invalid[] = {0.0f, -1.0f, NAN};

    for (size_t k = 0U; k < sizeof(invalid) / sizeof(invalid[0]); k++) {
        float integral = cc.pi_d.integral;

        foc_current_step_lim(&cc, zero, sc_of(0.0f), ref, invalid[k], DT, 1.0f, &out);
        CHECK(out.v_dq.d == 0.0f && out.v_dq.q == 0.0f && out.limited);
        CHECK(out.duty.a == 0.5f && out.duty.b == 0.5f && out.duty.c == 0.5f);
        CHECK(cc.pi_d.integral == integral);
        foc_current_step_lim(&cc, zero, sc_of(0.0f), ref, 15.0f, DT, invalid[k], &out);
        CHECK(out.v_dq.d == 0.0f && out.v_dq.q == 0.0f && out.limited);
        CHECK(cc.pi_d.integral == integral);
    }
}

/** Simulate a locked-rotor RL axis with one full PWM period of command delay. */
static void test_locked_rotor(bool d_axis) {
    const double r = 0.2;
    const double l = 70e-6;
    const double decay = exp(-r * (double)DT / l);
    foc_current_t cc = make_cc((float)(l * 6283.18530718), (float)(r * 6283.18530718));
    double current = 0.0;
    double pending = 0.0;
    double peak = 0.0;
    uint32_t rise = 0U;

    for (uint32_t k = 0U; k < 200U; k++) {
        foc_dq_t ref = {.d = d_axis ? 0.3f : 0.0f, .q = d_axis ? 0.0f : 0.3f};
        foc_dq_t measured = {.d = d_axis ? (float)current : 0.0f, .q = d_axis ? 0.0f : (float)current};
        foc_abc_t abc = foc_inv_clarke(foc_inv_park(measured, sc_of(0.0f)));
        foc_current_out_t out;

        foc_current_step_lim(&cc, abc, sc_of(0.0f), ref, 15.0f, DT, 1.0f, &out);
        current = current * decay + pending / r * (1.0 - decay);
        pending = d_axis ? out.v_dq.d : out.v_dq.q;
        if (current >= 0.27 && rise == 0U) {
            rise = k + 1U;
        }
        peak = fmax(peak, current);
    }
    double rise_ms = (double)rise * (double)DT * 1000.0;
    double overshoot = (peak / 0.3 - 1.0) * 100.0;

    printf("locked rotor %s: rise90 %.3f ms, overshoot %.3f %%\n", d_axis ? "id" : "iq", rise_ms, overshoot);
    CHECK(rise > 0U && rise_ms <= 0.6);
    CHECK(overshoot < 20.0);
    CHECK_NEAR(current, 0.3, 1e-5);
}

int main(void) {
    test_limited_chain();
    test_limited_recovery();
    test_locked_rotor(false);
    test_locked_rotor(true);
    test_matches_manual_chain();
    test_zero_error();
    test_saturates_at_bus_limit();
    test_limits_follow_vbus();
    test_no_bus();
    return foc_test_result("test_current");
}
