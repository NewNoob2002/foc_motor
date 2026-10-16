/* SPDX-License-Identifier: Apache-2.0 */

#include "cog_proc.h"
#include "encoder.h"
#include "foc_bench.h"
#include "foc_ctrl.h"
#include "pwm_adc.h"
#include "sincos.h"

#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(foc_motor, LOG_LEVEL_INF);

#define ASSERT_SHUTDOWN_LOW(node_id, port_id, pin, name)                                                               \
    BUILD_ASSERT(DT_NODE_HAS_PROP(node_id, gpio_hog), name " must be a GPIO hog");                                     \
    BUILD_ASSERT(DT_NODE_HAS_PROP(node_id, output_low), name " must start low");                                       \
    BUILD_ASSERT(DT_SAME_NODE(DT_PARENT(node_id), port_id), name " wrong port");                                       \
    BUILD_ASSERT(DT_GPIO_HOG_PIN_BY_IDX(node_id, 0) == (pin), name " wrong pin")

ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd1_hog), DT_NODELABEL(gpiob), 0, "SD1");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd2_hog), DT_NODELABEL(gpioa), 1, "SD2");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd3_hog), DT_NODELABEL(gpioa), 2, "SD3");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd4_hog), DT_NODELABEL(gpioa), 0, "SD4");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(timers1), okay),
             "TIM1 node must stay disabled: no Zephyr PWM driver or pins (src/pwm_adc.c)");
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_console), DT_NODELABEL(usart3))
                 && DT_SAME_NODE(DT_CHOSEN(zephyr_shell_uart), DT_NODELABEL(usart3)),
             "Console and shell UART must use the USART3 header");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay), "CH340 USART1 must remain disabled");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(usart3), current_speed) == 921600, "USART3 must run at 921600 baud");
BUILD_ASSERT(DT_DMAS_HAS_NAME(DT_NODELABEL(usart3), tx), "USART3 must have a TX DMA channel");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, channel) == 2,
             "USART3 TX must use DMA selector 2 (DMA1 channel 3) during M1");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, slot) == 29, "USART3 TX must use DMAMUX request 29");

/* Telemetry: a "rot" line every ROT_PERIOD_MS, the full report every REPORT_EVERY lines. */
#define ROT_PERIOD_MS     100
#define REPORT_EVERY      10U

/* TIM1 counts at HCLK: 170 ticks per microsecond. */
#define TIM1_TICKS_PER_US 170U

/*
 * The report path (log package + log message allocation) needs more than the
 * 512-byte main stack that the M1 budget pins, so it runs in its own
 * statically allocated thread, which also runs the Stage 5 FOC bench first.
 * Lower priority than the log thread (6).
 */
#define REPORT_STACK_SIZE 1024
#define REPORT_PRIORITY   7

K_THREAD_STACK_DEFINE(report_stack, REPORT_STACK_SIZE);
static struct k_thread report_thread;

/* One capture line is about 60 characters, 0.7 ms at 921600 baud; the pause keeps the 1 KiB log buffer from dropping. */
#define CAPTURE_LINE_PAUSE_MS 2

/* Supervise FOC progress in system-timer ISR context, independent of report-thread delays. */
static void supervise_expiry(struct k_timer* timer) {
    ARG_UNUSED(timer);
    pwm_adc_supervise();
}

K_TIMER_DEFINE(supervise_timer, supervise_expiry, NULL);

static const char* const mode_names[] = {
    [FOC_MODE_IDLE] = "idle (zero vector)",
    [FOC_MODE_DC] = "dc (fixed vector, ramped)",
    [FOC_MODE_STEP] = "step (capture)",
    [FOC_MODE_VOLTAGE] = "voltage (stage 6, 1 V at the encoder angle)",
    [FOC_MODE_CURRENT] = "current loop (calibrated dq)",
    [FOC_MODE_IDENT] = "ident",
    [FOC_MODE_OPENLOOP] = "openloop (rotating vector, ramped)",
    [FOC_MODE_MAP] = "cogging map",
    [FOC_MODE_SPEED] = "speed loop (calibrated dq)",
};

/* Thousandths, truncated toward zero; the inputs are a few units at most. */
static int32_t milli(float x) {
    return (int32_t)(x * 1000.0f);
}

/* Duty in 1/10000. */
static uint32_t duty_e4(float d) {
    return (uint32_t)(d * 10000.0f + 0.5f);
}

/* Kept out of line so its log packages live on this frame, not report_loop's. */
static __noinline void report(uint32_t n, const pwm_adc_stats_t* st, bool timing_valid, const foc_ctrl_snapshot_t* fc) {
    LOG_INF("  calib_valid %u, direction %d, offset_e %u electrical counts", fc->calib_valid ? 1U : 0U,
            fc->calib_direction, fc->calib_offset_e);
    if (st->isr_count == 0U) {
        LOG_WRN("report %u: no ISR in this window; fault 0x%x, DMA errors %u, ADC overruns %u", n, st->fault,
                st->dma_errors, st->adc_overruns);
        return;
    }

    /* Counting down, CNT = ARR - ticks since the top, where the ADC was triggered. */
    uint32_t after_top_min = (PWM_ARR - st->cnt_max) * 100U / TIM1_TICKS_PER_US;
    uint32_t after_top_max = (PWM_ARR - st->cnt_min) * 100U / TIM1_TICKS_PER_US;

    if (timing_valid) {
        LOG_INF("report %u: %u ISR/s, period %u..%u cyc, exec max %u cyc (budget %u)", n, st->isr_count, st->period_min,
                st->period_max, st->exec_max, PWM_ADC_EXEC_BUDGET);
        LOG_INF("  segments pre/step/post max %u/%u/%u cyc", st->pre_max, st->step_max, st->post_max);
    } else {
        LOG_WRN("report %u: %u ISR/s, DWT was stopped (debugger?) and is restarted; cycle figures invalid", n,
                st->isr_count);
    }
    LOG_INF("  TIM1 CNT at ISR %u..%u (%u.%02u..%u.%02u us after top), counting up in %u", st->cnt_min, st->cnt_max,
            after_top_min / 100U, after_top_min % 100U, after_top_max / 100U, after_top_max % 100U, st->counting_up);
    LOG_INF("  overruns %u, missed %u, fault 0x%x, DMA errors %u, ADC overruns %u; CCR %u %u %u", st->overruns,
            st->missed, st->fault, st->dma_errors, st->adc_overruns, st->ccr[0], st->ccr[1], st->ccr[2]);
    if (fc->cal_failed) {
        LOG_ERR("  current calibration FAILED: offsets IA %d.%03d IC %d.%03d counts; spreads IA %u IC %u counts; "
                "power remains disabled until reset",
                milli(fc->ia_offset) / 1000, milli(fc->ia_offset) % 1000, milli(fc->ic_offset) / 1000,
                milli(fc->ic_offset) % 1000, fc->ia_spread, fc->ic_spread);
        return;
    }
    if (!fc->calibrated) {
        LOG_INF("  calibrating current offsets");
        return;
    }
    LOG_INF("  mode %s, step %u; offsets IA %d.%03d IC %d.%03d counts", mode_names[fc->mode], fc->steps,
            milli(fc->ia_offset) / 1000, milli(fc->ia_offset) % 1000, milli(fc->ic_offset) / 1000,
            milli(fc->ic_offset) % 1000);
    LOG_INF("  dtc %s%s", fc->dtc ? "on" : "off", fc->dtc_refused ? " (refused: V0 outside 50..600 mV)" : "");
    LOG_INF("  cff %s%s, ff_valid %u; map phase %u reason %u", fc->cff ? "on" : "off",
            fc->cff_refused ? " (refused: no valid map)" : "", fc->ff_valid ? 1U : 0U, (unsigned int)fc->map_phase,
            (unsigned int)fc->map_reason);
    LOG_INF("  ident phase %u", (unsigned int)fc->ident_phase);
    LOG_INF("  i_d %d mA, i_q %d mA, v_d %d mV, v_q %d mV, limited %d", milli(fc->i_d), milli(fc->i_q), milli(fc->v_d),
            milli(fc->v_q), fc->limited ? 1 : 0);
    if (fc->mode == FOC_MODE_SPEED) {
        LOG_INF("  speed kp %d uA/rpm, ki %d uA/(rpm s), ref %d milli-rpm, latched %d rpm",
                (int32_t)(FOC_CTRL_SPD_KP * 1e6f), (int32_t)(FOC_CTRL_SPD_KI * 1e6f), milli(fc->spd_ref_rpm),
                fc->spd_target_rpm);
    }
    if (fc->mode == FOC_MODE_CURRENT || fc->mode == FOC_MODE_SPEED) {
        LOG_INF("  cc refs id %d iq %d mA, kp %d mV/A, ki %d V/(A s), speed %d rpm", milli(fc->i_d_ref),
                milli(fc->i_q_ref), milli(fc->cc_kp), (int32_t)fc->cc_ki, fc->speed_counts * 6000 / 4096);
    }
}

/* Power-stage state and the averaged measurements for the R test. */
static __noinline void report_power(const pwm_adc_stats_t* st, const foc_ctrl_snapshot_t* fc) {
    LOG_INF("  power %s (request %d, on %u times), VBUS %d mV (avg %d mV); avg IA %d IB %d IC %d mA",
            st->power_on ? "ON" : "off", fc->power_req ? 1 : 0, st->power_ons, milli(fc->vbus), milli(fc->vbus_f),
            milli(fc->ia_f), milli(fc->ib_f), milli(fc->ic_f));
    LOG_INF("  cmd: arm %s, mode %u%s, v %u mV, theta_e %u; encoder mech %u",
            foc_ctrl_cmd.arm == FOC_CTRL_ARM_KEY ? "KEY" : "0", foc_ctrl_cmd.mode,
            fc->mode_refused ? " (refused, idle runs)" : "", foc_ctrl_cmd.v_mv, foc_ctrl_cmd.theta_e,
            fc->enc_count % ENCODER_COUNTS_PER_REV);
}

/** Emit speed telemetry at 100 ms intervals, including while a capture is being dumped. */
static __noinline void report_speed(foc_ctrl_snapshot_t* fc) {
    static int64_t next_ms;
    int64_t now = k_uptime_get();

    if (now < next_ms) {
        return;
    }
    next_ms = (now / ROT_PERIOD_MS + 1) * ROT_PERIOD_MS;
    foc_ctrl_snapshot(fc);
    if (fc->mode == FOC_MODE_SPEED) {
        LOG_INF("spd %u %d %d %d %d", fc->steps, (int32_t)fc->spd_ref_rpm, (int32_t)(fc->speed_rpm * 10.0f),
                milli(fc->i_q_ref), milli(fc->i_q));
    }
}

/* Prints a finished step capture, then hands the buffer back to the ISR. */
static __noinline void dump_capture(const foc_ctrl_capture_t* c, foc_ctrl_snapshot_t* fc) {
    if (c->kind == 2U) {
        LOG_INF("cap_begin seq %u, kind 2, refs %d -> %d rpm, VBUS %d mV, power %s, %u samples at 1 kHz, "
                "step at %u, dtc %s%s, cff %s%s",
                c->seq, c->old_spd_rpm, c->new_spd_rpm, milli(c->vbus), c->power_on ? "on" : "OFF",
                FOC_CTRL_CAPTURE_LEN, FOC_CTRL_CAPTURE_PRE, c->dtc ? "on" : "off",
                c->dtc_refused ? " (refused: implausible V0)" : "", c->cff ? "on" : "off",
                c->cff_refused ? " (refused: no valid map)" : "");
    } else if (c->kind == 1U) {
        LOG_INF("cap_begin seq %u, kind 1, refs id %d -> %d iq %d -> %d mA, VBUS %d mV, power %s, %u samples, "
                "step at %u, dtc %s%s, cff %s%s",
                c->seq, c->old_id_ma, c->new_id_ma, c->old_iq_ma, c->new_iq_ma, milli(c->vbus),
                c->power_on ? "on" : "OFF", FOC_CTRL_CAPTURE_LEN, FOC_CTRL_CAPTURE_PRE, c->dtc ? "on" : "off",
                c->dtc_refused ? " (refused: implausible V0)" : "", c->cff ? "on" : "off",
                c->cff_refused ? " (refused: no valid map)" : "");
    } else {
        LOG_INF("cap_begin seq %u, step %u mV at theta_e %u, VBUS %d mV, power %s, %u samples (%u zero, %u on)", c->seq,
                c->v_mv, c->theta_e, milli(c->vbus), c->power_on ? "on" : "OFF", FOC_CTRL_CAPTURE_LEN,
                FOC_CTRL_CAPTURE_PRE, FOC_CTRL_CAPTURE_ON);
    }
    for (uint32_t k = 0; k < FOC_CTRL_CAPTURE_LEN; k++) {
        if (c->kind == 2U) {
            LOG_INF("capspd %u %u %d %d", c->seq, k, c->ia_ma[k], c->ic_ma[k]);
        } else if (c->kind == 1U) {
            LOG_INF("capdq %u %u %d %d", c->seq, k, c->ia_ma[k], c->ic_ma[k]);
        } else {
            LOG_INF("cap %u %u %d %d", c->seq, k, c->ia_ma[k], c->ic_ma[k]);
        }
        k_msleep(CAPTURE_LINE_PAUSE_MS);
        report_speed(fc);
    }
    LOG_INF("cap_end seq %u", c->seq);
    foc_ctrl_capture_release();
}

/** Scale identification diagnostics without undefined integer conversion for an ill-conditioned fit. */
static int32_t ident_units(float value, float scale) {
    float scaled = value * scale;

    if (!(scaled > -2147483648.0f)) {
        return INT32_MIN; /* Includes NaN; terminal diagnostics must remain printable. */
    }
    if (scaled >= 2147483648.0f) {
        return INT32_MAX;
    }
    return (int32_t)scaled;
}

/** Print one terminal identification with explicit units, then release its ISR buffer. */
static __noinline void dump_ident(const foc_ctrl_ident_t* r) {
    LOG_INF("ident state %u, fail %u; enc120 %u enc240 %u enc360 %u raw mechanical counts", (unsigned int)r->state,
            (unsigned int)r->fail_reason, r->enc_120, r->enc_240, r->enc_360);
    LOG_INF("ident delta_a %d delta_b %d mechanical counts, direction %d, offset %u electrical counts", r->delta_a,
            r->delta_b, r->direction, r->offset_e);
    LOG_INF("ident park A %u B %u, return120 %u raw counts; friction half %d electrical counts", r->enc_360,
            r->enc_zero_back, r->enc_return_120, r->friction_half_e);
    LOG_INF("ident rotation c %d, load %d, offset_e_new %u electrical counts", r->offset_c, r->offset_load,
            r->offset_e_new);
    k_msleep(CAPTURE_LINE_PAUSE_MS);
    for (uint32_t k = 0U; k < IDENT_R_POINTS; k++) {
        LOG_INF("ident R point %u: %d mV, %d mA", k, milli(r->points[k].v), milli(r->points[k].i));
        k_msleep(CAPTURE_LINE_PAUSE_MS);
    }
    LOG_INF("ident R %d mOhm, V0 %d mV, residual %d mV, nonlinear %u; L %d uH, ipp %d mA", ident_units(r->r, 1000.0f),
            ident_units(r->v0, 1000.0f), ident_units(r->max_residual, 1000.0f), r->r_nonlinear ? 1U : 0U,
            ident_units(r->l, 1e6f), ident_units(r->ipp, 1000.0f));
    foc_ctrl_ident_release();
}

/** Process and report a terminal map before returning ownership to the ISR. */
static __noinline void dump_map(const cog_raw_t* raw, foc_ctrl_snapshot_t* fc) {
    static cog_processed_t result;
    bool ok = cog_process(raw, &result);
    LOG_INF("cogmap_result phase %u reason %u", (unsigned int)raw->phase, (unsigned int)raw->reason);
    for (uint32_t k = 0U; k < COG_GAIN_POINTS; k++) {
        float a = raw->gain[k].alpha_ma;
        float beta = raw->gain[k].beta_ma;
        LOG_INF("gainchk %u %u %d %d %d %d %d", k, k * 4096U / COG_GAIN_POINTS, ident_units(a, 1.0f),
                ident_units((-a + 1.7320508f * beta) * 0.5f, 1.0f), ident_units((-a - 1.7320508f * beta) * 0.5f, 1.0f),
                ident_units(result.magnitude_ma[k], 1.0f), ident_units(result.angle_error[k], 1.0f));
        k_msleep(CAPTURE_LINE_PAUSE_MS);
        report_speed(fc);
    }
    LOG_INF("gainfit I0 %d e1 %d p1 %d e2 %d p2 %d", ident_units(result.i0_ma, 1.0f),
            ident_units(result.e1_permille, 1.0f), ident_units(result.p1_counts, 1.0f),
            ident_units(result.e2_permille, 1.0f), ident_units(result.p2_counts, 1.0f));
    if (ok) {
        for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
            LOG_INF("cogmap %u %d %d %d", b, (int32_t)cog_bin_mean(raw, 0U, b), (int32_t)cog_bin_mean(raw, 1U, b),
                    result.ff[b]);
            k_msleep(CAPTURE_LINE_PAUSE_MS);
            report_speed(fc);
        }
        for (uint32_t h = 0U; h < COG_HARMONICS; h++) {
            LOG_INF("cogharm %u %d %d", cog_orders[h], (int32_t)result.harmonic[h].amplitude_ma,
                    (int32_t)result.harmonic[h].phase_counts);
            k_msleep(CAPTURE_LINE_PAUSE_MS);
        }
        LOG_INF("friction %d", (int32_t)result.friction_ma);
        LOG_INF("gain e2 includes phase resistance imbalance; order 84 is near the 256-bin resolution limit");
    }
    LOG_INF("cogmap_ok %u", ok ? 1U : 0U);
    foc_ctrl_map_release(ok ? result.ff : NULL);
}

/* Mechanical count, electrical angle (0..4095) and the three duties of one ISR step. */
static __noinline void report_rot(const foc_ctrl_snapshot_t* fc) {
    LOG_INF("rot %u %u %u %u %u %u", fc->steps, fc->enc_count % ENCODER_COUNTS_PER_REV, fc->theta_e_count,
            duty_e4(fc->duty_a), duty_e4(fc->duty_b), duty_e4(fc->duty_c));
}

/** Print commanded and calibrated rotor angles at the rotation telemetry cadence. */
static __noinline void report_openloop(const foc_ctrl_snapshot_t* fc) {
    if (fc->calib_valid) {
        int32_t err = (int32_t)((fc->theta_e_count + 2048U - fc->theta_meas_count) & 4095U) - 2048;

        LOG_INF("ol %u %u %u %d", fc->steps, fc->theta_e_count, fc->theta_meas_count, err);
    } else {
        LOG_INF("ol %u %u invalid", fc->steps, fc->theta_e_count);
    }
}

/* Stage 5 bench first, while no ISR can disturb the timing; then the FOC ISR and its telemetry. */
static void report_loop(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    pwm_adc_stats_t st;
    foc_ctrl_snapshot_t fc;

    sincos_init();
    foc_bench_run();

    foc_ctrl_init();
    int ret = pwm_adc_init(foc_ctrl_step, foc_ctrl_fault);

    if (ret < 0) {
        LOG_ERR("pwm_adc_init failed: %d", ret);
        return;
    }
    k_timer_start(&supervise_timer, K_MSEC(10), K_MSEC(10));
    LOG_INF("FOC ISR on: TIM1 ARR %u (20 kHz center-aligned, RCR 1), ADC2 [IA, IC, VBUS] on TRGO2 at the top; "
            "offset calibration %u periods; power stage off",
            PWM_ARR, FOC_CTRL_CAL_SAMPLES);
    LOG_INF("command block foc_ctrl_cmd at %p: arm +0, mode +4, v_mv +8, theta_e +12, step_seq +16, rate_e +20; "
            "id_ma +24, iq_ma +28, dtc +32, spd_rpm +36, spd_step +40, cff +44; arm key 0x%08x",
            (void*)&foc_ctrl_cmd, FOC_CTRL_ARM_KEY);

    /* Drop the start-up window. */
    (void)pwm_adc_stats_take(&st);
    bool supervisor_reported = false;

    for (uint32_t n = 1;; n++) {
        k_msleep((int32_t)(ROT_PERIOD_MS - k_uptime_get() % ROT_PERIOD_MS));

        if (!supervisor_reported && pwm_adc_supervisor_tripped()) {
            LOG_ERR("FOC ISR progress below 150 per 10 ms while powered; power stage off, timer stopped until reset");
            supervisor_reported = true;
        }
        if (!pwm_adc_outputs_safe()) {
            pwm_adc_stop();
            LOG_ERR("pin invariant broken (PC0-PC3, SD1-SD4 or CCER); power stage off, outputs off, timer stopped");
            return;
        }
        foc_ctrl_snapshot(&fc);
        if (fc.calibrated) {
            report_rot(&fc);
            report_speed(&fc);
            if (fc.mode == FOC_MODE_OPENLOOP) {
                report_openloop(&fc);
            }
        }
        if (n % REPORT_EVERY == 0U) {
            bool timing_valid = pwm_adc_stats_take(&st);

            report(n / REPORT_EVERY, &st, timing_valid, &fc);
            if (fc.calibrated) {
                report_power(&st, &fc);
            }
        }

        const foc_ctrl_capture_t* c = foc_ctrl_capture_get();

        if (c != NULL) {
            dump_capture(c, &fc);
        }
        const foc_ctrl_ident_t* ident = foc_ctrl_ident_get();

        if (ident != NULL) {
            dump_ident(ident);
        }
        const cog_raw_t* raw = foc_ctrl_map_get();
        if (raw != NULL) {
            dump_map(raw, &fc);
        }
    }
}

int main(void) {
    int ret = encoder_init();

    LOG_INF("Stage 8 build: FOC bench, then the FOC ISR; power stage off until armed (VBUS 14.0-16.5 V, arm key); "
            "TIM1 CH1-3 also on PA8-PA10, CH3N on PB9 (U7-4)");
    if (ret < 0) {
        LOG_ERR("encoder_init failed: %d", ret);
        return 0;
    }

    (void)k_thread_create(&report_thread, report_stack, K_THREAD_STACK_SIZEOF(report_stack), report_loop, NULL, NULL,
                          NULL, REPORT_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
