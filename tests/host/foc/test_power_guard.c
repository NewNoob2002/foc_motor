/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>

#include "foc_test.h"
#include "power_guard.h"

/* Calibrated, key set, idle, 15 V, no current, stage off. */
static power_guard_in_t good_in(void) {
    power_guard_in_t in = {
        .ia = 0.0f, .ic = 0.0f, .vbus = 15.0f, .calibrated = true, .arm_key = true, .idle = true, .power_on = false};

    return in;
}

/* Runs n steps with the same input; returns the last output. */
static power_guard_out_t run(power_guard_t* g, const power_guard_in_t* in, uint32_t n) {
    power_guard_out_t out = {0};

    for (uint32_t i = 0; i < n; i++) {
        out = power_guard_step(g, in);
    }
    return out;
}

/* Brings a guard to the armed state and returns the input that did it, with power_on set as the BSP would. */
static power_guard_in_t armed(power_guard_t* g) {
    power_guard_in_t in = good_in();

    power_guard_reset(g);
    CHECK(run(g, &in, POWER_GUARD_VBUS_ARM_PERIODS).power_on);
    in.power_on = true;
    return in;
}

static void test_arms_after_vbus_settles(void) {
    power_guard_t g;
    power_guard_in_t in = good_in();

    power_guard_reset(&g);
    CHECK(!run(&g, &in, POWER_GUARD_VBUS_ARM_PERIODS - 1U).power_on);
    power_guard_out_t out = power_guard_step(&g, &in);
    CHECK(out.power_on);
    CHECK(out.trip == 0U);
}

/* Require 20 idle steps even when VBUS, calibration and the key are ready. */
static void test_arming_waits_for_idle(void) {
    power_guard_t g;
    power_guard_in_t in = good_in();

    power_guard_reset(&g);
    in.idle = false;
    CHECK(!run(&g, &in, POWER_GUARD_VBUS_ARM_PERIODS).power_on);
    CHECK(g.vbus_ok_n == POWER_GUARD_VBUS_ARM_PERIODS);
    CHECK(g.idle_n == 0U);
    in.idle = true;
    CHECK(!run(&g, &in, POWER_GUARD_IDLE_PERIODS - 1U).power_on);
    CHECK(g.idle_n == POWER_GUARD_IDLE_PERIODS - 1U);
    CHECK(power_guard_step(&g, &in).power_on);
    CHECK(run(&g, &in, 100U).power_on);
    CHECK(g.idle_n == POWER_GUARD_IDLE_PERIODS);
}

/* A single non-idle period discards all preceding idle time. */
static void test_non_idle_restarts_idle_count(void) {
    power_guard_t g;
    power_guard_in_t in = good_in();

    power_guard_reset(&g);
    in.arm_key = false;
    CHECK(!run(&g, &in, POWER_GUARD_VBUS_ARM_PERIODS).power_on);
    CHECK(g.idle_n == POWER_GUARD_IDLE_PERIODS);
    in.idle = false;
    in.arm_key = true;
    CHECK(!power_guard_step(&g, &in).power_on);
    CHECK(g.idle_n == 0U);
    in.idle = true;
    CHECK(!run(&g, &in, POWER_GUARD_IDLE_PERIODS - 1U).power_on);
    CHECK(power_guard_step(&g, &in).power_on);
}

static void test_each_condition_blocks_arming(void) {
    power_guard_t g;
    power_guard_in_t in;

    in = good_in();
    in.arm_key = false;
    power_guard_reset(&g);
    CHECK(!run(&g, &in, 3U * POWER_GUARD_VBUS_ARM_PERIODS).power_on);

    in = good_in();
    in.calibrated = false;
    power_guard_reset(&g);
    CHECK(!run(&g, &in, 3U * POWER_GUARD_VBUS_ARM_PERIODS).power_on);

    in = good_in();
    in.idle = false;
    power_guard_reset(&g);
    CHECK(!run(&g, &in, 3U * POWER_GUARD_VBUS_ARM_PERIODS).power_on);

    const float bad_vbus[] = {0.0f, 13.9f, 16.6f, 24.0f, NAN};

    for (size_t k = 0; k < sizeof(bad_vbus) / sizeof(bad_vbus[0]); k++) {
        in = good_in();
        in.vbus = bad_vbus[k];
        power_guard_reset(&g);
        CHECK(!run(&g, &in, 3U * POWER_GUARD_VBUS_ARM_PERIODS).power_on);
    }
}

/* One sample outside the arm window restarts the 100 ms count. */
static void test_vbus_dip_restarts_count(void) {
    power_guard_t g;
    power_guard_in_t in = good_in();

    power_guard_reset(&g);
    (void)run(&g, &in, POWER_GUARD_VBUS_ARM_PERIODS - 1U);
    in.vbus = 13.0f;
    CHECK(!power_guard_step(&g, &in).power_on);
    in.vbus = 15.0f;
    CHECK(!run(&g, &in, POWER_GUARD_VBUS_ARM_PERIODS - 1U).power_on);
    CHECK(power_guard_step(&g, &in).power_on);
}

static void test_key_cleared_disarms_and_rearm_needs_conditions(void) {
    power_guard_t g;
    power_guard_in_t in = armed(&g);

    in.arm_key = false;
    CHECK(!power_guard_step(&g, &in).power_on);
    in.power_on = false;
    in.arm_key = true;
    in.idle = false;
    CHECK(!run(&g, &in, 10U).power_on); /* not idle: stays off */
    in.idle = true;
    CHECK(!run(&g, &in, POWER_GUARD_IDLE_PERIODS - 1U).power_on);
    CHECK(power_guard_step(&g, &in).power_on); /* VBUS stayed good; idle has now settled too. */
}

/* Once on, leaving idle (a test vector) keeps the stage on. */
static void test_stays_on_when_not_idle(void) {
    power_guard_t g;
    power_guard_in_t in = armed(&g);

    in.idle = false;
    CHECK(run(&g, &in, 100U).power_on);
}

static void test_overcurrent_trips_each_phase(void) {
    const float cases[][2] = {
        {2.01f, 0.0f},  /* a */
        {-2.01f, 0.0f}, /* a negative */
        {0.0f, 2.01f},  /* c */
        {1.5f, 1.5f},   /* b = -3 A while a and c are under the limit */
        {NAN, 0.0f},
    };

    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
        power_guard_t g;
        power_guard_in_t in = armed(&g);

        in.ia = cases[k][0];
        in.ic = cases[k][1];
        power_guard_out_t out = power_guard_step(&g, &in);

        CHECK(out.trip == POWER_GUARD_TRIP_OVERCURRENT);
        CHECK(!out.power_on);
    }
}

static void test_under_limit_does_not_trip(void) {
    power_guard_t g;
    power_guard_in_t in = armed(&g);

    in.ia = 2.0f;
    in.ic = -1.9f; /* b = -0.1 */
    power_guard_out_t out = power_guard_step(&g, &in);

    CHECK(out.trip == 0U);
    CHECK(out.power_on);
}

/* Uncalibrated currents are not compared: the offsets are not known yet. */
static void test_no_current_trip_before_calibration(void) {
    power_guard_t g;
    power_guard_in_t in = good_in();

    power_guard_reset(&g);
    in.calibrated = false;
    in.ia = 50.0f;
    CHECK(power_guard_step(&g, &in).trip == 0U);
}

static void test_vbus_trip_needs_three_in_a_row_while_on(void) {
    power_guard_t g;
    power_guard_in_t in = armed(&g);

    in.vbus = 11.9f;
    CHECK(power_guard_step(&g, &in).trip == 0U);
    CHECK(power_guard_step(&g, &in).trip == 0U);
    in.vbus = 15.0f;
    CHECK(power_guard_step(&g, &in).trip == 0U); /* a good sample restarts the count */
    in.vbus = 17.1f;
    CHECK(power_guard_step(&g, &in).trip == 0U);
    CHECK(power_guard_step(&g, &in).trip == 0U);
    power_guard_out_t out = power_guard_step(&g, &in);

    CHECK(out.trip == POWER_GUARD_TRIP_VBUS);
    CHECK(!out.power_on);

    /* While off, VBUS anywhere does not trip (no DCBUS is the normal state). */
    in = good_in();
    in.vbus = 0.0f;
    power_guard_reset(&g);
    CHECK(run(&g, &in, 10U).trip == 0U);
}

int main(void) {
    test_arms_after_vbus_settles();
    test_arming_waits_for_idle();
    test_non_idle_restarts_idle_count();
    test_each_condition_blocks_arming();
    test_vbus_dip_restarts_count();
    test_key_cleared_disarms_and_rearm_needs_conditions();
    test_stays_on_when_not_idle();
    test_overcurrent_trips_each_phase();
    test_under_limit_does_not_trip();
    test_no_current_trip_before_calibration();
    test_vbus_trip_needs_three_in_a_row_while_on();
    return foc_test_result("test_power_guard");
}
