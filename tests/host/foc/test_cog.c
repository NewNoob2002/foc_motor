/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "cog_map.h"
#include "cog_proc.h"
#include "foc_test.h"

#define TAU 6.283185307179586

/** Generate a noiseless gain sweep with prescribed first/second harmonics. */
static void fill_gain(cog_raw_t* raw) {
    for (uint32_t k = 0U; k < COG_GAIN_POINTS; k++) {
        double theta = (k * 4096U / COG_GAIN_POINTS) * TAU / 4096.0;
        double mag = 1000.0 * (1.0 + 0.01 * cos(theta - 0.7) + 0.03 * cos(2.0 * theta - 1.1));
        raw->gain[k].alpha_ma = (float)(mag * cos(theta));
        raw->gain[k].beta_ma = (float)(mag * sin(theta));
    }
}

/** Recover independent synthetic cogging, friction, noise and gain errors. */
static void test_processing(void) {
    cog_raw_t raw = {.phase = COG_DONE};
    cog_processed_t out;
    fill_gain(&raw);
    for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
        double theta = (b + 0.5) * TAU / FOC_CTRL_MAP_BINS;
        double cog = 140.0 * cos(14.0 * theta - 0.3) + 80.0 * cos(84.0 * theta - 1.7);
        double noise = 0.2 * cos(37.0 * theta);
        raw.count[0][b] = 1000U;
        raw.count[1][b] = 1000U;
        raw.sum[0][b] = (int32_t)lround((cog + 30.0 + 15.0 + noise) * 1000.0);
        raw.sum[1][b] = (int32_t)lround((cog - 30.0 + 15.0 - noise) * 1000.0);
    }
    CHECK(cog_process(&raw, &out) && out.valid);
    CHECK_NEAR(out.friction_ma, 30.0, 0.001);
    CHECK_NEAR(out.harmonic[3].amplitude_ma, 140.0, 0.01);
    CHECK_NEAR(out.harmonic[6].amplitude_ma, 80.0, 0.01);
    CHECK_NEAR(out.harmonic[3].phase_counts, 0.3 * 4096.0 / TAU, 0.1);
    CHECK_NEAR(out.harmonic[6].phase_counts, 1.7 * 4096.0 / TAU, 0.1);
    CHECK_NEAR(out.i0_ma, 1000.0, 0.01);
    CHECK_NEAR(out.e1_permille, 10.0, 0.01);
    CHECK_NEAR(out.e2_permille, 30.0, 0.01);
    CHECK_NEAR(out.p1_counts, 0.7 * 4096.0 / TAU, 0.1);
    CHECK_NEAR(out.p2_counts, 1.1 * 4096.0 / TAU, 0.1);
    for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
        double theta = (b + 0.5) * TAU / FOC_CTRL_MAP_BINS;
        CHECK_NEAR(out.ff[b], 140.0 * cos(14.0 * theta - 0.3) + 80.0 * cos(84.0 * theta - 1.7), 0.501);
    }
    raw.count[0][255] = COG_MIN_SAMPLES - 1U;
    CHECK(!cog_process(&raw, &out) && !out.valid);
    raw.count[0][255] = 1000U;
    raw.gain[2].alpha_ma = NAN;
    CHECK(!cog_process(&raw, &out));
    raw.phase = COG_FAILED;
    CHECK(!cog_process(&raw, &out));
}

/** Check centre-aligned interpolation, wrap, conditional addition and symmetric clamp. */
static void test_feedforward(void) {
    int16_t table[FOC_CTRL_MAP_BINS] = {0};
    table[0] = 100;
    table[1] = 300;
    table[255] = -100;
    CHECK_NEAR(cog_ff_lookup(table, 8U), 0.1, 1e-7);
    CHECK_NEAR(cog_ff_lookup(table, 16U), 0.2, 1e-7);
    CHECK_NEAR(cog_ff_lookup(table, 24U), 0.3, 1e-7);
    CHECK_NEAR(cog_ff_lookup(table, 0U), 0.0, 1e-7);
    CHECK_NEAR(cog_ff_lookup(table, 4095U), -0.0125, 1e-7);
    CHECK_NEAR(cog_ff_lookup(table, 4104U), 0.1, 1e-7);
    CHECK_NEAR(cog_ff_apply(0.4f, 0.5f, true, true, table, 24U), 0.5, 1e-7);
    CHECK_NEAR(cog_ff_apply(-0.49f, 0.5f, true, true, table, 4088U), -0.5, 1e-7);
    CHECK_NEAR(cog_ff_apply(0.4f, 0.5f, true, false, table, 24U), 0.4, 1e-7);
    CHECK_NEAR(cog_ff_apply(0.4f, 0.5f, false, true, table, 24U), 0.4, 1e-7);
}

/** Exercise every gain phase at its exact period boundary, followed by signed wrap accumulation. */
static void test_sequence(void) {
    cog_map_t m = {0};
    cog_map_in_t in = {.power = true, .armed = true, .alpha = 1.0f, .beta = 0.5f};
    CHECK(cog_map_start(&m, true, true, 0.1f, -1, 0U));
    for (uint32_t n = 0U; n < FOC_CTRL_MAP_BINS; n++) {
        CHECK(m.raw.phase == COG_CLEAR);
        cog_map_step(&m, &in);
    }
    for (uint32_t k = 0U; k < COG_GAIN_POINTS; k++) {
        CHECK(cog_map_angle(&m) == k * 4096U / COG_GAIN_POINTS);
        uint32_t n = 0U;
        while (m.raw.phase == COG_GAIN_RAMP && n++ < 3000U) {
            cog_map_step(&m, &in);
        }
        CHECK(n >= 1999U && n <= 2002U);
        for (n = 0U; n < COG_SETTLE_N; n++) {
            CHECK(m.raw.phase == COG_GAIN_SETTLE);
            cog_map_step(&m, &in);
        }
        for (n = 0U; n < COG_AVERAGE_N; n++) {
            CHECK(m.raw.phase == COG_GAIN_AVERAGE);
            cog_map_step(&m, &in);
        }
        CHECK_NEAR(m.raw.gain[k].alpha_ma, 1000.0, 0.001);
        CHECK_NEAR(m.raw.gain[k].beta_ma, 500.0, 0.001);
        n = 0U;
        while (m.raw.phase == COG_GAIN_DECAY && n++ < 3000U) {
            cog_map_step(&m, &in);
        }
        CHECK(n >= 2399U && n <= 2403U);
    }
    CHECK(m.raw.phase == COG_FORWARD_SETTLE && cog_map_current(&m));
    /* An ideal rotor that follows the target: direction is -1, so the calibrated
     * position rises while the raw count falls. Position and target start at 0
     * with the raw count 0 (it stayed 0 through the gain check).
     */
    in.alpha = 0.0f;
    in.beta = 0.0f;
    for (uint32_t d = 0U; d < 2U; d++) {
        in.rpm = 0.0f;
        for (uint32_t n = 0U; n < COG_POS_HOLD_N; n++) {
            CHECK(m.raw.phase == (d == 0U ? COG_FORWARD_SETTLE : COG_REVERSE_SETTLE));
            cog_map_step(&m, &in);
        }
        CHECK(m.raw.phase == (d == 0U ? COG_FORWARD : COG_REVERSE));
        in.rpm = d == 0U ? 6.0f : -6.0f;
        uint32_t n = 0U;
        while (m.raw.phase == (d == 0U ? COG_FORWARD : COG_REVERSE) && n++ < 1000000U) {
            cog_map_step(&m, &in);
            int32_t follow = (int32_t)lroundf((float)m.tgt_q16 * (1.0f / 65536.0f));
            in.enc = (uint32_t)(-follow) & 4095U;
            CHECK(fabsf((float)m.tgt_q16 * (1.0f / 65536.0f) - (float)m.pos) < 1.5f);
            /* The one-period follow lag (0.02 count) integrates to about 20 mA over the 20 s sweep. */
            CHECK(fabsf(m.iq_cmd) < 0.05f);
        }
        /* 2 turns at 410 counts/s: about 20 s. */
        CHECK(n > 390000U && n < 410000U);
    }
    const cog_raw_t* held = cog_map_get(&m);
    CHECK(held != NULL && held->phase == COG_DONE);
    /* About 780 periods per bin per turn at 410 counts/s; mean iq near 0 for an ideal follower. */
    for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
        for (uint32_t d = 0U; d < 2U; d++) {
            CHECK(m.raw.count[d][b] > 1400U && m.raw.count[d][b] < 1750U);
            CHECK(abs(m.raw.sum[d][b]) < 20 * (int32_t)m.raw.count[d][b]);
        }
    }
    cog_raw_t saved = m.raw;
    CHECK(!cog_map_start(&m, true, true, 0.1f, 1, 0U));
    in.power = false;
    cog_map_step(&m, &in);
    cog_map_abort(&m, COG_FAULT);
    CHECK(memcmp(&saved, &m.raw, sizeof(saved)) == 0);
    cog_map_release(&m);
    CHECK(cog_map_get(&m) == NULL);
    CHECK(cog_map_start(&m, true, true, 0.1f, 1, 0U));
}

/** Verify all early exits, strict slip timing, timeout and checked integer bounds. */
static void test_aborts(void) {
    cog_map_t m = {0};
    cog_map_in_t in = {.power = true, .armed = true};
    CHECK(cog_map_start(&m, false, true, 0.4f, 1, 0U));
    CHECK(cog_map_get(&m)->reason == COG_NOT_POWERED);
    cog_map_release(&m);
    CHECK(cog_map_start(&m, true, false, 0.4f, 1, 0U));
    CHECK(cog_map_get(&m)->reason == COG_CALIBRATION);
    for (cog_fail_t reason = COG_CANCELLED; reason <= COG_OVERFLOW; reason++) {
        cog_map_release(&m);
        CHECK(cog_map_start(&m, true, true, 0.4f, 1, 0U));
        in = (cog_map_in_t){.power = true, .armed = true};
        switch (reason) {
            case COG_CANCELLED:
                in.armed = false;
                break;
            case COG_FAULT:
                in.fault = true;
                break;
            case COG_NUMERIC:
                in.alpha = NAN;
                break;
            case COG_SLIP:
                /* 399 counts behind still runs; 401 aborts. */
                m.raw.phase = COG_FORWARD;
                m.tgt_q16 = 399 * 65536;
                cog_map_step(&m, &in);
                CHECK(cog_map_get(&m) == NULL);
                m.tgt_q16 = 401 * 65536;
                break;
            case COG_TIMEOUT:
                m.ticks = COG_TIMEOUT_N;
                break;
            case COG_OVERFLOW:
                m.raw.phase = COG_FORWARD;
                m.raw.count[0][0] = UINT16_MAX;
                break;
            default:
                break;
        }
        cog_map_step(&m, &in);
        CHECK(cog_map_get(&m) != NULL && m.raw.reason == reason && m.voltage == 0.0f);
    }
    CHECK((int64_t)COG_TIMEOUT_N * 500 < INT32_MAX);
}

/** Run numerical recovery and production sequencer regression checks. */
int main(void) {
    test_processing();
    test_feedforward();
    test_sequence();
    test_aborts();
    return foc_test_result("test_cog");
}
