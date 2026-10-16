/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>

#include "foc_speed.h"
#include "foc_test.h"

/** Check exact quantized encoder slopes, both wraps and 1 kHz publication. */
static void test_estimator(void) {
    const int speeds[] = {60, -60, 150, -150, 200, -200};
    for (size_t k = 0U; k < sizeof(speeds) / sizeof(speeds[0]); k++) {
        foc_ctrl_speed_t speed;
        foc_ctrl_speed_reset(&speed, 4093U);
        int64_t old_position = 4093;
        int32_t expected[10] = {0};
        int32_t sum = 0;
        uint32_t bucket = 0U;
        int64_t bucket_start = old_position;
        for (uint32_t n = 1U; n <= 40000U; n++) {
            int64_t position = 4093 + (int64_t)floor((double)speeds[k] * 4096.0 * n / 1200000.0);
            int32_t before = speed.counts;
            bool update = foc_ctrl_speed_step(&speed, (uint32_t)position & 4095U);
            CHECK(update == (n % 20U == 0U));
            if (update) {
                sum -= expected[bucket];
                expected[bucket] = (int32_t)(position - bucket_start);
                sum += expected[bucket];
                bucket_start = position;
                bucket = (bucket + 1U) % 10U;
                CHECK(speed.counts == sum);
                if (n >= 200U) {
                    CHECK(fabs((double)speed.counts - speeds[k] * 4096.0 * 0.01 / 60.0) <= 1.0);
                }
            } else {
                CHECK(speed.counts == before);
            }
            old_position = position;
        }
        foc_ctrl_speed_reset(&speed, (uint32_t)old_position);
        CHECK(speed.counts == 0 && speed.trip_counts == 0);
        for (uint32_t n = 1U; n <= 200U; n++) {
            (void)foc_ctrl_speed_step(&speed, (uint32_t)old_position);
        }
        CHECK(speed.counts == 0);
    }
}

/** Verify both output limits, bounded windup, recovery and exact ramp increments. */
static void test_pi(void) {
    foc_speed_loop_t loop;
    foc_speed_loop_reset(&loop);
    CHECK_NEAR(loop.pi.kp, 0.0045482048, 1e-8);
    CHECK_NEAR(loop.pi.ki, 0.071443034, 1e-7);
    for (uint32_t n = 0U; n < 10000U; n++) {
        foc_speed_loop_update(&loop, 200.0f, 0.0f, true);
        CHECK(loop.iq <= 0.5f && loop.pi.integral <= 0.5f);
    }
    CHECK(loop.iq == 0.5f && loop.pi.integral == 0.5f);
    for (uint32_t n = 0U; n < 200U; n++) {
        foc_speed_loop_update(&loop, -200.0f, 0.0f, true);
    }
    CHECK(loop.iq == -0.5f && loop.pi.integral == -0.5f);
    for (uint32_t n = 0U; n < 100U; n++) {
        foc_speed_loop_update(&loop, 50.0f, 0.0f, true);
    }
    CHECK(loop.iq > 0.0f && loop.pi.integral > -0.5f);
    foc_speed_loop_reset(&loop);
    for (uint32_t n = 1U; n <= 250U; n++) {
        foc_speed_loop_update(&loop, 100.0f, 0.0f, false);
        CHECK_NEAR(loop.ref, (float)n * 0.4f, 0.0003);
    }
    foc_speed_loop_update(&loop, 100.0f, 0.0f, false);
    CHECK(loop.ref == 100.0f);
}

/** Run standalone pure-C speed estimator and PI regressions. */
int main(void) {
    test_estimator();
    test_pi();
    return foc_test_result("test_speed");
}
