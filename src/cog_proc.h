/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_COG_PROC_H
#define FOC_MOTOR_COG_PROC_H
#include "cog_map.h"

#define COG_HARMONICS 7U

typedef struct {
    float amplitude_ma;
    float phase_counts;
} cog_harmonic_t;
typedef struct {
    int16_t ff[FOC_CTRL_MAP_BINS];
    float friction_ma;
    cog_harmonic_t harmonic[COG_HARMONICS];
    float magnitude_ma[COG_GAIN_POINTS];
    float angle_error[COG_GAIN_POINTS];
    float i0_ma;
    float e1_permille;
    float p1_counts;
    float e2_permille;
    float p2_counts;
    bool valid;
} cog_processed_t;
extern const uint32_t cog_orders[COG_HARMONICS];

/** Process an immutable map in thread context; no hardware or RTOS dependencies.
 * Harmonic phases use A*cos(order*theta-phase), with theta at bin centres.
 * Order 84 is near the 256-bin resolution limit and is attenuated by binning
 * and interpolation. Gain e2 also includes phase-resistance imbalance;
 * e1 indicates residual offset and is not a sensor-gain correction.
 */
bool cog_process(const cog_raw_t* raw, cog_processed_t* out);
/** Return one directional mean in mA; requires a validated nonempty bin. */
float cog_bin_mean(const cog_raw_t* raw, uint32_t direction, uint32_t bin);

#endif
