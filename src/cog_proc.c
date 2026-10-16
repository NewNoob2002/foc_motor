/* SPDX-License-Identifier: Apache-2.0 */
#include "cog_proc.h"
#include <math.h>

#define TAU 6.28318530718f
const uint32_t cog_orders[COG_HARMONICS] = {1U, 2U, 7U, 14U, 28U, 42U, 84U};

/** Convert a cosine/sine coefficient pair to a wrapped count phase. */
static float phase_counts(float cosine, float sine) {
    float phase = atan2f(sine, cosine) * (4096.0f / TAU);
    return phase < 0.0f ? phase + 4096.0f : phase;
}

/** Divide only in the report thread, after checking the count. */
float cog_bin_mean(const cog_raw_t* raw, uint32_t direction, uint32_t bin) {
    return (float)raw->sum[direction][bin] / (float)raw->count[direction][bin];
}

/** Fit the five linear coefficients at the actual integer commanded angles. */
static bool gain_fit(const cog_raw_t* raw, cog_processed_t* out) {
    float normal[5][6] = {0};
    for (uint32_t k = 0U; k < COG_GAIN_POINTS; k++) {
        float a = raw->gain[k].alpha_ma;
        float b = raw->gain[k].beta_ma;
        float magnitude = hypotf(a, b);
        if (!isfinite(magnitude) || magnitude < 50.0f || magnitude > 2000.0f) {
            return false;
        }
        out->magnitude_ma[k] = magnitude;
        uint32_t command = k * 4096U / COG_GAIN_POINTS;
        float counts = (float)command;
        float theta = counts * (TAU / 4096.0f);
        float row[5] = {1.0f, cosf(theta), sinf(theta), cosf(2.0f * theta), sinf(2.0f * theta)};
        for (uint32_t i = 0U; i < 5U; i++) {
            for (uint32_t j = 0U; j < 5U; j++) {
                normal[i][j] += row[i] * row[j];
            }
            normal[i][5] += row[i] * magnitude;
        }
        float error = atan2f(b, a) * (4096.0f / TAU) - counts;
        if (error < -2048.0f) {
            error += 4096.0f;
        }
        if (error > 2048.0f) {
            error -= 4096.0f;
        }
        out->angle_error[k] = error;
    }
    /* This fixed-angle design is well conditioned (diagonal ~12,6,6,6,6).
     * Gaussian elimination avoids treating quantized angles as equally spaced.
     */
    for (uint32_t i = 0U; i < 5U; i++) {
        if (!(normal[i][i] > 1.0f)) {
            return false;
        }
        float reciprocal = 1.0f / normal[i][i];
        for (uint32_t j = i + 1U; j < 6U; j++) {
            normal[i][j] *= reciprocal;
        }
        for (uint32_t r = 0U; r < 5U; r++) {
            if (r != i) {
                float multiplier = normal[r][i];
                for (uint32_t j = i + 1U; j < 6U; j++) {
                    normal[r][j] -= multiplier * normal[i][j];
                }
            }
        }
    }
    out->i0_ma = normal[0][5];
    out->e1_permille = 1000.0f * hypotf(normal[1][5], normal[2][5]) / out->i0_ma;
    out->e2_permille = 1000.0f * hypotf(normal[3][5], normal[4][5]) / out->i0_ma;
    out->p1_counts = phase_counts(normal[1][5], normal[2][5]);
    out->p2_counts = phase_counts(normal[3][5], normal[4][5]);
    return true;
}

/** Fit gain harmonics, remove DC torque, analyze cogging and quantize the table. */
bool cog_process(const cog_raw_t* raw, cog_processed_t* out) {
    *out = (cog_processed_t){0};
    /* Keep completed phase-A diagnostics even if phase B subsequently failed. */
    bool gain_ok = gain_fit(raw, out);
    if (!gain_ok || raw->phase != COG_DONE || raw->reason != COG_OK) {
        return false;
    }
    float mean = 0.0f;
    for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
        if (raw->count[0][b] < COG_MIN_SAMPLES || raw->count[1][b] < COG_MIN_SAMPLES) {
            return false;
        }
        float f = cog_bin_mean(raw, 0U, b);
        float r = cog_bin_mean(raw, 1U, b);
        mean += (f + r) * (0.5f / FOC_CTRL_MAP_BINS);
        out->friction_ma += (f - r) * (0.5f / FOC_CTRL_MAP_BINS);
    }
    float cosine[COG_HARMONICS] = {0};
    float sine[COG_HARMONICS] = {0};
    for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
        float cog = 0.5f * (cog_bin_mean(raw, 0U, b) + cog_bin_mean(raw, 1U, b)) - mean;
        if (!isfinite(cog) || fabsf(cog) > 500.0f) {
            return false;
        }
        out->ff[b] = (int16_t)lroundf(cog);
        float angle = ((float)b + 0.5f) * (TAU / FOC_CTRL_MAP_BINS);
        for (uint32_t h = 0U; h < COG_HARMONICS; h++) {
            float theta = (float)cog_orders[h] * angle;
            cosine[h] += cog * cosf(theta) * (2.0f / FOC_CTRL_MAP_BINS);
            sine[h] += cog * sinf(theta) * (2.0f / FOC_CTRL_MAP_BINS);
        }
    }
    for (uint32_t h = 0U; h < COG_HARMONICS; h++) {
        out->harmonic[h].amplitude_ma = hypotf(cosine[h], sine[h]);
        out->harmonic[h].phase_counts = phase_counts(cosine[h], sine[h]);
    }
    out->valid = true;
    return true;
}
