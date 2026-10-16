/* SPDX-License-Identifier: Apache-2.0 */
#include "cog_map.h"

#include <math.h>
#include <stddef.h>

_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "ISR publication requires lock-free atomics");
/* |iq| <= 0.5 A, quantized to mA: even every period in one bin is
 * <= 1100000 * 500 = 550000000 < INT32_MAX. Counts are checked before
 * incrementing; uint16 saturation aborts rather than wrapping.
 */
_Static_assert(COG_TIMEOUT_N <= INT32_MAX / 500U, "map sums must fit int32");

/** Publish once; subsequent calls cannot alter the thread-owned buffer. */
static void finish(cog_map_t* m, cog_fail_t reason) {
    m->raw.reason = reason;
    m->raw.phase = reason == COG_OK ? COG_DONE : COG_FAILED;
    m->voltage = 0.0f;
    atomic_store_explicit(&m->ready, 1U, memory_order_release);
}

/** Start with bounded word stores; never clear a thread-owned result. */
bool cog_map_start(cog_map_t* m, bool power, bool calibration, float voltage, int32_t direction, uint32_t enc) {
    if (atomic_load_explicit(&m->ready, memory_order_acquire) != 0U) {
        return false;
    }
    /* Clear one bin per period before excitation, bounding mode-entry WCET. */
    volatile cog_raw_t* r = &m->raw;
    for (uint32_t k = 0U; k < COG_GAIN_POINTS; k++) {
        r->gain[k].alpha_ma = 0.0f;
        r->gain[k].beta_ma = 0.0f;
        r->gain[k].enc = 0U;
    }
    m->raw.phase = COG_CLEAR;
    m->clear_bin = 0U;
    m->raw.reason = COG_OK;
    m->ticks = 0U;
    m->dwell = 0U;
    m->slip = 0U;
    m->point = 0U;
    m->previous = enc;
    m->travel = 0;
    m->direction = direction;
    m->voltage = 0.0f;
    m->target_v = voltage;
    m->alpha_sum = 0.0f;
    m->beta_sum = 0.0f;
    if (!power) {
        finish(m, COG_NOT_POWERED);
    } else if (!calibration || !(voltage > 0.05f && voltage < 0.9f) || (direction != 1 && direction != -1)) {
        finish(m, COG_CALIBRATION);
    }
    return true;
}

/** Preserve partial measurements when cancellation or protection ends a run. */
void cog_map_abort(cog_map_t* m, cog_fail_t reason) {
    if (m->raw.phase > COG_IDLE && m->raw.phase < COG_DONE) {
        finish(m, reason);
    }
}

/** Advance the open-loop voltage sweep and bidirectional speed-map sequencer. */
void cog_map_step(cog_map_t* m, const cog_map_in_t* in) {
    if (m->raw.phase == COG_IDLE || m->raw.phase >= COG_DONE) {
        return;
    }
    if (in->fault) {
        cog_map_abort(m, COG_FAULT);
    } else if (!in->armed) {
        cog_map_abort(m, COG_CANCELLED);
    } else if (!in->power) {
        cog_map_abort(m, COG_NOT_POWERED);
    } else if (!isfinite(in->alpha) || !isfinite(in->beta) || !isfinite(in->rpm) || !isfinite(in->iq)) {
        cog_map_abort(m, COG_NUMERIC);
    } else if (++m->ticks > COG_TIMEOUT_N) {
        cog_map_abort(m, COG_TIMEOUT);
    }
    if (m->raw.phase >= COG_DONE) {
        return;
    }
    if (m->raw.phase != COG_GAIN_DECAY) {
        m->dwell++;
    }
    switch (m->raw.phase) {
        case COG_CLEAR:
            m->raw.sum[0][m->clear_bin] = 0;
            m->raw.sum[1][m->clear_bin] = 0;
            m->raw.count[0][m->clear_bin] = 0U;
            m->raw.count[1][m->clear_bin] = 0U;
            if (++m->clear_bin == FOC_CTRL_MAP_BINS) {
                m->raw.phase = COG_GAIN_RAMP;
                m->dwell = 0U;
            }
            break;
        case COG_GAIN_RAMP:
            m->voltage += 50e-6f; /* 1 V/s, including the first angle. */
            if (m->voltage >= m->target_v) {
                m->voltage = m->target_v;
                m->raw.phase = COG_GAIN_SETTLE;
                m->dwell = 0U;
            }
            break;
        case COG_GAIN_SETTLE:
            if (m->dwell == COG_SETTLE_N) {
                m->raw.phase = COG_GAIN_AVERAGE;
                m->dwell = 0U;
            }
            break;
        case COG_GAIN_AVERAGE:
            m->alpha_sum += in->alpha;
            m->beta_sum += in->beta;
            if (m->dwell == COG_AVERAGE_N) {
                m->raw.gain[m->point].alpha_ma = m->alpha_sum * (1000.0f / COG_AVERAGE_N);
                m->raw.gain[m->point].beta_ma = m->beta_sum * (1000.0f / COG_AVERAGE_N);
                m->raw.gain[m->point].enc = in->enc;
                m->alpha_sum = 0.0f;
                m->beta_sum = 0.0f;
                m->raw.phase = COG_GAIN_DECAY;
                m->dwell = 0U;
            }
            break;
        case COG_GAIN_DECAY:
            m->voltage -= 50e-6f;
            if (m->voltage <= 0.0f) {
                m->voltage = 0.0f;
                /* Wait 20 ms at zero voltage before stepping: no energized
                 * angle discontinuity. The rotor follows each 1 V/s ramp.
                 */
                if (m->dwell++ >= 400U) {
                    m->point++;
                    m->dwell = 0U;
                    m->raw.phase = m->point == COG_GAIN_POINTS ? COG_FORWARD_SETTLE : COG_GAIN_RAMP;
                }
            } else {
                m->dwell = 0U;
            }
            break;
        default: {
            float target = cog_map_rpm(m);
            m->slip = fabsf(in->rpm - target) > 5.0f ? m->slip + 1U : 0U;
            if (m->slip > 20000U) {
                cog_map_abort(m, COG_SLIP);
                break;
            }
            int32_t delta = (int32_t)((in->enc - m->previous + 2048U) & 4095U) - 2048;
            m->previous = in->enc;
            if (m->raw.phase == COG_FORWARD_SETTLE || m->raw.phase == COG_REVERSE_SETTLE) {
                if (m->dwell == COG_SPEED_SETTLE_N) {
                    m->raw.phase = m->raw.phase == COG_FORWARD_SETTLE ? COG_FORWARD : COG_REVERSE;
                    m->dwell = 0U;
                    m->travel = 0;
                }
                break;
            }
            uint32_t d = m->raw.phase == COG_FORWARD ? 0U : 1U;
            uint32_t b = (in->enc & 4095U) >> 4U;
            if (fabsf(in->iq) > 0.50001f || m->raw.count[d][b] == UINT16_MAX) {
                cog_map_abort(m, COG_OVERFLOW);
                break;
            }
            m->raw.sum[d][b] += (int32_t)(in->iq * 1000.0f);
            m->raw.count[d][b]++;
            m->travel += delta * m->direction * (d == 0U ? 1 : -1);
            if (m->travel >= (int32_t)(FOC_CTRL_MAP_REVS * 4096U)) {
                if (d == 0U) {
                    m->raw.phase = COG_REVERSE_SETTLE;
                    m->dwell = 0U;
                    m->slip = 0U;
                } else {
                    finish(m, COG_OK);
                }
            }
            break;
        }
    }
}

/** Acquire all raw stores preceding terminal publication. */
const cog_raw_t* cog_map_get(const cog_map_t* m) {
    return atomic_load_explicit(&m->ready, memory_order_acquire) != 0U ? &m->raw : NULL;
}

/** Release only after the thread no longer accesses raw data. */
void cog_map_release(cog_map_t* m) {
    atomic_store_explicit(&m->ready, 0U, memory_order_release);
}

/** Use integer constant division only on angle changes, outside phase B. */
uint32_t cog_map_angle(const cog_map_t* m) {
    return m->point < COG_GAIN_POINTS ? m->point * 4096U / COG_GAIN_POINTS : 0U;
}

/** Select the speed loop only while collecting phase B. */
float cog_map_rpm(const cog_map_t* m) {
    if (m->raw.phase == COG_FORWARD_SETTLE || m->raw.phase == COG_FORWARD) {
        return (float)FOC_CTRL_MAP_RPM;
    }
    if (m->raw.phase == COG_REVERSE_SETTLE || m->raw.phase == COG_REVERSE) {
        return -(float)FOC_CTRL_MAP_RPM;
    }
    return 0.0f;
}

/** Bin b represents count 16*b+8; shift by half a bin before interpolation. */
float cog_ff_lookup(const int16_t table[FOC_CTRL_MAP_BINS], uint32_t enc) {
    uint32_t position = (enc - 8U) & 4095U;
    uint32_t b = position >> 4U;
    float fraction = (float)(position & 15U) * (1.0f / 16.0f);
    float left = (float)table[b];
    return (left + ((float)table[(b + 1U) & 255U] - left) * fraction) * 0.001f;
}

/** Keep the existing torque-current limits after adding position feed-forward. */
float cog_ff_apply(float iq, float limit, bool enabled, bool valid, const int16_t* table, uint32_t enc) {
    if (enabled && valid) {
        iq += cog_ff_lookup(table, enc);
    }
    if (iq > limit) {
        return limit;
    }
    return iq < -limit ? -limit : iq;
}
