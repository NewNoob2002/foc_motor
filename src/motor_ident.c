/* SPDX-License-Identifier: Apache-2.0 */
#include "motor_ident.h"

#include <math.h>

/** Convert a duration to the closest whole number of PWM periods. */
static uint32_t periods(float seconds) {
    return (uint32_t)(seconds / IDENT_PERIOD_S + 0.5f);
}

/** Reset only per-target timers, retaining the filter and servo voltage. */
static void next_target(motor_ident_t* m, motor_ident_phase_t phase) {
    m->result.state = phase;
    m->target_n = 0U;
    m->reached_n = 0U;
    m->hold_n = 0U;
    m->reached = false;
}

/** Clear fixed storage and begin the first alignment target. */
void motor_ident_start(motor_ident_t* m) {
    *m = (motor_ident_t){0};
    m->theta_e_count = IDENT_ALIGN_90;
    next_target(m, IDENT_ALIGN_FIRST);
}

/** Latch an abort without discarding measurements already collected. */
void motor_ident_abort(motor_ident_t* m, motor_ident_fail_t reason) {
    if (m->result.state != IDENT_IDLE && m->result.state != IDENT_DONE && m->result.state != IDENT_FAILED) {
        m->result.state = IDENT_FAILED;
        m->result.fail_reason = reason;
        m->servo_v = 0.0f;
    }
}

/** Regulate current and require a continuous filtered-current dwell before declaring arrival. */
static void servo(motor_ident_t* m, float i, float ref) {
    /*
     * For an RL plant the continuous characteristic is L*s*s + R*s + Ki:
     * positive R,L,Ki give negative-real-part poles. Ki*T = 0.00025 V/A
     * keeps the sampled loop slow relative to the plant for the stated
     * bench range (R=0.02..5 ohm, L<5 mH), including the PWM preload delay.
     * This is a model argument, not a measured target WCET/stability result.
     */
    m->servo_v += IDENT_SERVO_KI * (ref - i) * IDENT_PERIOD_S;
    m->servo_v = fminf(IDENT_V_MAX, fmaxf(0.0f, m->servo_v));
    if (!m->reached) {
        float band = ref > 0.0f ? IDENT_REACHED_FRAC * ref : IDENT_ZERO_BAND_A;

        m->target_n++;
        m->reached_n = fabsf(m->filtered_i - ref) < band ? m->reached_n + 1U : 0U;
        if (m->reached_n >= periods(IDENT_REACHED_S)) {
            m->reached = true;
        } else if (m->target_n >= periods(IDENT_SERVO_TIMEOUT_S)) {
            motor_ident_abort(m, IDENT_FAIL_SERVO_TIMEOUT);
        }
    }
}

/** Validate the short mechanical move and compute an offset after direction correction. */
static void align_result(motor_ident_t* m, uint32_t enc) {
    motor_ident_result_t* r = &m->result;

    r->enc_0 = enc;
    r->delta_mech = (int32_t)(r->enc_0 % IDENT_COUNTS) - (int32_t)(r->enc_90 % IDENT_COUNTS);
    if (r->delta_mech > (int32_t)IDENT_COUNTS / 2) {
        r->delta_mech -= (int32_t)IDENT_COUNTS;
    } else if (r->delta_mech < -(int32_t)IDENT_COUNTS / 2) {
        r->delta_mech += (int32_t)IDENT_COUNTS;
    }
    float electrical_move = (float)IDENT_POLE_PAIRS * fabsf((float)r->delta_mech);

    if (r->delta_mech == 0 || fabsf(electrical_move - (float)IDENT_ALIGN_90) > (float)IDENT_ALIGN_TOL_E) {
        motor_ident_abort(m, IDENT_FAIL_ALIGNMENT);
        return;
    }
    r->direction = r->delta_mech < 0 ? 1 : -1;
    uint32_t raw_offset = ((r->enc_0 % IDENT_COUNTS) * IDENT_POLE_PAIRS) % IDENT_COUNTS;

    r->offset_e = r->direction > 0 ? raw_offset : (IDENT_COUNTS - raw_offset) % IDENT_COUNTS;
    next_target(m, IDENT_RESISTANCE);
}

/** Fit the four upper current targets once; retain every measured point for inspection. */
static void fit_resistance(motor_ident_t* m) {
    motor_ident_result_t* r = &m->result;
    float mean_i = 0.0f;
    float mean_v = 0.0f;

    /* Select by nominal target: noise can put the measured 0.75 A just below 0.75. */
    for (uint32_t k = IDENT_R_FIT_FIRST; k < IDENT_R_POINTS; k++) {
        mean_i += r->points[k].i;
        mean_v += r->points[k].v;
    }
    mean_i /= (float)(IDENT_R_POINTS - IDENT_R_FIT_FIRST);
    mean_v /= (float)(IDENT_R_POINTS - IDENT_R_FIT_FIRST);
    float variance = 0.0f;
    float covariance = 0.0f;

    for (uint32_t k = IDENT_R_FIT_FIRST; k < IDENT_R_POINTS; k++) {
        float di = r->points[k].i - mean_i;

        variance += di * di;
        covariance += di * (r->points[k].v - mean_v);
    }
    if (!(variance > 0.0f && covariance > 0.0f)) {
        motor_ident_abort(m, IDENT_FAIL_RESISTANCE);
        return;
    }
    r->r = covariance / variance;
    r->v0 = mean_v - r->r * mean_i;
    for (uint32_t k = IDENT_R_FIT_FIRST; k < IDENT_R_POINTS; k++) {
        float residual = fabsf(r->points[k].v - (r->r * r->points[k].i + r->v0));

        r->max_residual = fmaxf(r->max_residual, residual);
    }
    float last_di = r->points[IDENT_R_POINTS - 1U].i - r->points[IDENT_R_POINTS - 2U].i;

    if (!(last_di > 0.0f) || !isfinite(r->r) || !isfinite(r->v0)) {
        motor_ident_abort(m, IDENT_FAIL_RESISTANCE);
        return;
    }
    float last_slope = (r->points[IDENT_R_POINTS - 1U].v - r->points[IDENT_R_POINTS - 2U].v) / last_di;

    r->r_nonlinear = r->max_residual > IDENT_R_RESIDUAL_FRAC * r->r * IDENT_R_TOP_A
                     || fabsf(last_slope - r->r) > IDENT_R_SLOPE_FRAC * r->r;
    next_target(m, IDENT_L_BIAS);
}

/** Record one settled operating point, then advance to the next target or the L injection. */
static void resistance_step(motor_ident_t* m, float i) {
    static const float targets[IDENT_R_POINTS] = IDENT_R_TARGETS_A;

    servo(m, i, targets[m->point]);
    if (m->result.state == IDENT_FAILED || !m->reached) {
        return;
    }
    if (m->hold_n < periods(IDENT_R_SETTLE_S)) {
        m->hold_n++;
        return;
    }
    m->sum_i += i;
    m->sum_v += m->servo_v;
    m->avg_n++;
    if (m->avg_n == IDENT_R_AVG_N) {
        m->result.points[m->point].i = m->sum_i / (float)IDENT_R_AVG_N;
        m->result.points[m->point].v = m->sum_v / (float)IDENT_R_AVG_N;
        m->point++;
        m->avg_n = 0U;
        m->sum_i = 0.0f;
        m->sum_v = 0.0f;
        if (m->point == IDENT_R_POINTS) {
            fit_resistance(m);
        } else {
            next_target(m, IDENT_RESISTANCE);
        }
    }
}

/** Return the sign of a square-wave command indexed by its output period. */
static float injection_sign(uint32_t n) {
    return ((n / IDENT_L_HALF) & 1U) == 0U ? 1.0f : -1.0f;
}

/** Accumulate current increments against the voltage that actually caused them. */
static float inductance_step(motor_ident_t* m, float i) {
    const uint32_t total = IDENT_L_CYCLES * 2U * IDENT_L_HALF;
    float bias = m->result.points[IDENT_R_BIAS_POINT].v;

    /* At call n, i[n]-i[n-1] is caused by output n-2 (CCR preload).
     * Flush two calls after the last output so the final increment is counted.
     */
    if (m->l_n >= 2U && m->l_n - 2U >= IDENT_L_DISCARD * 2U * IDENT_L_HALF) {
        m->sum_delta += injection_sign(m->l_n - 2U) * (i - m->prev_i);
        m->increments++;
    }
    m->prev_i = i;
    if (m->l_n == total + 1U) {
        float mean = m->sum_delta / (float)m->increments;

        if (!(mean > 0.0f) || !isfinite(mean)) {
            motor_ident_abort(m, IDENT_FAIL_INDUCTANCE);
            return 0.0f;
        }
        m->result.l = IDENT_L_VH * IDENT_PERIOD_S / mean;
        m->result.ipp = IDENT_L_VH * (float)IDENT_L_HALF * IDENT_PERIOD_S / m->result.l;
        if (!isfinite(m->result.l) || m->result.ipp > IDENT_L_IPP_MAX_A) {
            motor_ident_abort(m, IDENT_FAIL_RIPPLE);
        } else {
            m->result.state = IDENT_DONE;
        }
        return 0.0f;
    }
    float voltage = m->l_n < total ? bias + IDENT_L_VH * injection_sign(m->l_n) : bias;

    m->l_n++;
    return voltage;
}

/** Apply safety checks, advance one bounded phase, and emit the next PWM command. */
void motor_ident_step(motor_ident_t* m, const motor_ident_in_t* in, motor_ident_out_t* out) {
    float voltage = 0.0f;

    if (!isfinite(in->i_d)) {
        motor_ident_abort(m, IDENT_FAIL_NUMERIC);
    } else if (fabsf(in->i_d) > IDENT_I_ABORT_A) {
        motor_ident_abort(m, IDENT_FAIL_OVERCURRENT);
    } else if (!in->power_on) {
        motor_ident_abort(m, IDENT_FAIL_NOT_POWERED);
    }
    if (m->result.state > IDENT_IDLE && m->result.state < IDENT_DONE) {
        m->run_n++;
        m->filtered_i += IDENT_FILTER_K * (in->i_d - m->filtered_i);
        if (m->run_n > periods(IDENT_RUN_TIMEOUT_S)) {
            motor_ident_abort(m, IDENT_FAIL_RUN_TIMEOUT);
        }
    }
    switch (m->result.state) {
        case IDENT_ALIGN_FIRST:
        case IDENT_ALIGN_ZERO:
            servo(m, in->i_d, IDENT_ALIGN_A);
            if (m->reached && ++m->hold_n >= periods(IDENT_ALIGN_HOLD_S)) {
                if (m->result.state == IDENT_ALIGN_FIRST) {
                    m->result.enc_90 = in->enc_count;
                    next_target(m, IDENT_ALIGN_DECAY);
                    m->servo_v = 0.0f;
                } else {
                    align_result(m, in->enc_count);
                }
            }
            voltage = m->servo_v;
            break;
        case IDENT_ALIGN_DECAY:
            /* Remove excitation and verify decay before rotating the vector 90 degrees.
             * This avoids an abrupt full-current q-axis torque impulse at the switch.
             */
            servo(m, in->i_d, 0.0f);
            m->servo_v = 0.0f;
            if (m->reached) {
                m->theta_e_count = 0U;
                next_target(m, IDENT_ALIGN_ZERO);
            }
            break;
        case IDENT_RESISTANCE:
            resistance_step(m, in->i_d);
            voltage = m->servo_v;
            break;
        case IDENT_L_BIAS:
            /* The final R point is 1.5 A. Restore 1 A before adding positive
             * injection, otherwise the first pulse can cross the abort limit.
             * Freeze at the recorded R voltage only after the bias has settled.
             */
            servo(m, in->i_d, IDENT_ALIGN_A);
            if (m->reached && ++m->hold_n >= periods(IDENT_R_SETTLE_S)) {
                next_target(m, IDENT_INDUCTANCE);
            }
            voltage = m->servo_v;
            break;
        case IDENT_INDUCTANCE:
            voltage = inductance_step(m, in->i_d);
            break;
        default:
            break;
    }
    out->done = m->result.state == IDENT_DONE;
    out->failed = m->result.state == IDENT_FAILED;
    out->v_d = out->done || out->failed ? 0.0f : voltage;
    out->theta_e_count = m->theta_e_count;
}
