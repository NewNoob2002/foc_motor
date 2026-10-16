/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_COG_MAP_H
#define FOC_MOTOR_COG_MAP_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#define FOC_CTRL_MAP_BINS  256U
#define FOC_CTRL_MAP_RPM   10
#define FOC_CTRL_MAP_REVS  3U
#define COG_GAIN_POINTS    12U
#define COG_SETTLE_N       6000U
#define COG_AVERAGE_N      2000U
#define COG_SPEED_SETTLE_N 20000U
#define COG_TIMEOUT_N      1100000U /* 55 s, including voltage decay/ramp. */
#define COG_MIN_SAMPLES    32U

typedef enum {
    COG_IDLE,
    COG_CLEAR,
    COG_GAIN_RAMP,
    COG_GAIN_SETTLE,
    COG_GAIN_AVERAGE,
    COG_GAIN_DECAY,
    COG_FORWARD_SETTLE,
    COG_FORWARD,
    COG_REVERSE_SETTLE,
    COG_REVERSE,
    COG_DONE,
    COG_FAILED
} cog_phase_t;
typedef enum {
    COG_OK,
    COG_NOT_POWERED,
    COG_CALIBRATION,
    COG_CANCELLED,
    COG_FAULT,
    COG_NUMERIC,
    COG_SLIP,
    COG_TIMEOUT,
    COG_OVERFLOW
} cog_fail_t;
typedef struct {
    float alpha_ma;
    float beta_ma;
    uint32_t enc;
} cog_gain_t;
typedef struct {
    int32_t sum[2][FOC_CTRL_MAP_BINS]; /* mA; forward then reverse, calibrated torque sign. */
    uint16_t count[2][FOC_CTRL_MAP_BINS];
    cog_gain_t gain[COG_GAIN_POINTS];
    cog_phase_t phase;
    cog_fail_t reason;
} cog_raw_t;

typedef struct {
    uint32_t enc;
    float alpha;
    float beta;
    float rpm;
    float iq;
    bool power;
    bool armed;
    bool fault;
} cog_map_in_t;

/** ISR-owned sequencer; ready transfers the entire raw buffer to the thread.
 * A start is refused while ready=1. The thread must finish processing, table
 * publication and reporting before release; no raw writes occur in that time.
 */
typedef struct {
    cog_raw_t raw;
    atomic_uint ready;
    uint32_t ticks;
    uint32_t clear_bin;
    uint32_t dwell;
    uint32_t slip;
    uint32_t point;
    uint32_t previous;
    int32_t travel;
    int32_t direction;
    float voltage;
    float target_v;
    float alpha_sum;
    float beta_sum;
} cog_map_t;

/** Begin a run only if the thread has released the preceding result. */
bool cog_map_start(cog_map_t* m, bool power, bool calibration, float voltage, int32_t direction, uint32_t enc);
/** Advance one 50 us period and publish a terminal buffer with release ordering. */
void cog_map_step(cog_map_t* m, const cog_map_in_t* in);
/** Abort an active run and publish its immutable partial result. */
void cog_map_abort(cog_map_t* m, cog_fail_t reason);
/** Acquire a terminal raw buffer, or NULL while the ISR owns it. */
const cog_raw_t* cog_map_get(const cog_map_t* m);
/** Return raw-buffer ownership to the ISR after processing and reporting. */
void cog_map_release(cog_map_t* m);
/** Return the calibrated electrical command angle for the gain check. */
uint32_t cog_map_angle(const cog_map_t* m);
/** Return the requested speed, or zero outside phase B. */
float cog_map_rpm(const cog_map_t* m);
/** Interpolate mA samples at mechanical bin centres, including the wrap. */
float cog_ff_lookup(const int16_t table[FOC_CTRL_MAP_BINS], uint32_t enc);
/** Add an accepted feed-forward request and clamp to the current reference limit. */
float cog_ff_apply(float iq, float limit, bool enabled, bool valid, const int16_t* table, uint32_t enc);

#endif
