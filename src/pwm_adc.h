/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: TIM1 center-aligned PWM timebase, ADC2 [IA, IC] triggered from TIM1,
 * DMA2 channel 1 moving both results, and the DMA transfer-complete ISR
 * that is the FOC ISR (docs/design/zephyr_architecture.md 2.3, 4.1).
 *
 * Power stage (Stage 7): off at boot. SD1-SD4 are low (devicetree hogs)
 * and PC0-PC3 (the EG2104 IN pins) are analog, since with no DCBUS the
 * EG2104 VCC is 0 V and IN must stay below VCC + 0.3 V (HW-PWM-009, R-19).
 * The control step switches it on and off (pwm_adc_cmd_t.power_on); the
 * ISR does the pin work: on = PC0-PC2 to TIM1 CH1-3 (AF2), then SD1-SD3
 * high; off = SD1-SD3 low first, then PC0-PC2 back to analog. SD4 and PC3
 * (the D leg) are never touched. The policy (VBUS window, current trip)
 * lives in src/power_guard.c.
 *
 * Scope route: TIM1 CH1-3 also come out on the unconnected PA8/PA9/PA10,
 * and CH3N (CH3 inverted) on PB9 at the U7-4 header (user-approved
 * 2026-10-14). pwm_adc_outputs_safe() checks the pin invariant.
 *
 * Timing: ADC2 converts at the counter top (TRGO2), the ISR computes the
 * duties and writes CCR1-3 into the preload registers; RCR = 1 makes the
 * only update event the next top, so a sample taken at top k drives the
 * period that starts at top k + 1 (one period, 50 us, of delay).
 */

#ifndef FOC_MOTOR_PWM_ADC_H
#define FOC_MOTOR_PWM_ADC_H

#include <stdbool.h>
#include <stdint.h>

/* 170 MHz / (2 * 4250) = 20 kHz, center-aligned. */
#define PWM_ARR                     4250U

/* AGENTS.md: FOC ISR WCET at most 20 us. A longer run is an overrun. */
#define PWM_ADC_EXEC_BUDGET         3400U

/* Overruns in a row that latch the fault. */
#define PWM_ADC_OVERRUN_FAULT_COUNT 3U

/* A powered ISR gap above two 8500-cycle PWM periods is a fault. */
#define PWM_ADC_GAP_FAULT_CYCLES    17000U

/* Minimum progress per 10 ms timer tick (200 ISR entries expected). */
#define PWM_ADC_SUPERVISE_MIN_ISRS  150U

/*
 * Sticky fault reasons (pwm_adc_stats_t.fault). A fault switches the power
 * stage off, sets MOE = 0 and 50 % duty, and stops the control steps until
 * reset.
 */
#define PWM_ADC_FAULT_OVERRUN       0x01U /* PWM_ADC_OVERRUN_FAULT_COUNT overruns in a row */
#define PWM_ADC_FAULT_DMA_ERROR     0x02U
#define PWM_ADC_FAULT_OVERCURRENT   0x04U  /* from the control step (power_guard) */
#define PWM_ADC_FAULT_VBUS          0x08U  /* from the control step (power_guard) */
#define PWM_ADC_FAULT_PINS          0x10U  /* pin invariant broken when switching on */
#define PWM_ADC_FAULT_ADC_OVERRUN   0x20U  /* OVR at ISR entry, DMA samples may be stale or shifted */
#define PWM_ADC_FAULT_NUMERIC       0x40U  /* a duty is outside [0, 1], including NaN */
#define PWM_ADC_FAULT_ISR_GAP       0x80U  /* powered ISR-entry gap exceeds PWM_ADC_GAP_FAULT_CYCLES */
#define PWM_ADC_FAULT_OVERSPEED     0x100U /* powered current loop above the mechanical speed limit */

/*
 * High-side duty limits. The bootstrap capacitor only charges while the low
 * side is on, so every period keeps at least 5 % (2.5 us) of low-side time
 * (docs/design/zephyr_architecture.md 2.2; the value is not yet checked on
 * the gate waveform).
 */
#define PWM_ADC_DUTY_MIN            0.05f
#define PWM_ADC_DUTY_MAX            0.95f

/* What the ISR hands to the control step each period. */
typedef struct {
    uint16_t ia_code;   /* ADC2 IN3, sampled at the counter top */
    uint16_t ic_code;   /* ADC2 IN17, sampled right after IA */
    uint16_t vbus_code; /* ADC2 IN11 (PC5), sampled after IC */
    uint32_t enc_count; /* TIM3 count read in the ISR */
    bool power_on;      /* SD1-SD3 high and PC0-PC2 driving the gate drivers */
} pwm_adc_sample_t;

/* What the control step returns each period. */
typedef struct {
    float a; /* high-side duty per phase; must be in [0, 1], then clamped to the bootstrap limits */
    float b;
    float c;
    bool power_on; /* wanted power-stage state; the ISR switches when it differs */
    uint32_t trip; /* PWM_ADC_FAULT_* bits; non-zero latches the fault at once */
} pwm_adc_cmd_t;

/*
 * Runs inside the zero-latency ISR: no kernel, device or logging API, no
 * locks, no waiting (AGENTS.md).
 */
typedef void (*pwm_adc_step_fn)(const pwm_adc_sample_t* sample, pwm_adc_cmd_t* cmd);

/* ISR measurements over one window, written only by the ISR. */
typedef struct {
    uint32_t isr_count;
    uint32_t period_min; /* DWT cycles between ISR entries */
    uint32_t period_max;
    uint32_t exec_max; /* DWT cycles from ISR entry to exit */
    uint16_t cnt_min;  /* TIM1 CNT at ISR entry */
    uint16_t cnt_max;
    uint32_t counting_up; /* ISRs that saw TIM1 counting up */
    uint32_t ia_sum;
    uint32_t ic_sum;
    uint16_t ccr[3]; /* CCR1-3 written by the latest ISR */
    bool power_on;   /* power stage state at the latest ISR */
    /* Sticky across windows: */
    uint32_t power_ons; /* times the stage was switched on since boot */
    uint32_t overruns;  /* ISRs longer than PWM_ADC_EXEC_BUDGET */
    uint32_t missed;    /* ISRs that ended after the next ADC result had arrived */
    uint32_t fault;     /* PWM_ADC_FAULT_* bits; latched until reset */
    uint32_t dma_errors;
    uint32_t adc_overruns;
} pwm_adc_stats_t;

/*
 * Starts TIM1 (CH1-3 on PA8-PA10, CH3N on PB9, all at 50 %), ADC2 [IA, IC,
 * VBUS] and DMA, with the power stage off; from the first ISR on, step()
 * sets the duties and the power-stage state. Returns 0 or -errno.
 */
int pwm_adc_init(pwm_adc_step_fn step);

/*
 * True while: PC3 is analog and SD4 is a low output; only CH1-3 and CH3N
 * are enabled in CCER (CH2N could reach PB0 = SD1); and either the power
 * stage is off with PC0-PC2 analog and SD1-SD3 low, or it is on with
 * PC0-PC2 in alternate-function mode. If not, the caller must call
 * pwm_adc_stop(). Reads the power state before and after the registers;
 * retries a state change at most three times, then returns false.
 */
bool pwm_adc_outputs_safe(void);

/*
 * Disable the DMA IRQ first, switch power off, clear MOE and stop TIM1.
 * May run in thread or system-timer ISR context. Final until reset: do not
 * call pwm_adc_init() again after a stop.
 */
void pwm_adc_stop(void);

/*
 * Check FOC progress from the system-timer ISR every 10 ms without logging.
 * If powered and fewer than PWM_ADC_SUPERVISE_MIN_ISRS entries occurred,
 * stop permanently and latch the supervisor indication.
 */
void pwm_adc_supervise(void);

/* Return whether the timer supervisor tripped; sticky until reset. */
bool pwm_adc_supervisor_tripped(void);

/* True while the power stage is on. */
bool pwm_adc_power_is_on(void);

/* ISR entries since init (wraps); the independent 10 ms supervisor checks progress. */
uint32_t pwm_adc_isr_total(void);

/*
 * Copies the current window and asks the ISR to start a new one at its next
 * entry. The copy is consistent: it retries if the ISR ran during it.
 * Returns false if the DWT cycle counter had been stopped (a debugger
 * disconnect does this); it is restarted, and the window's cycle figures
 * (period, exec_max) are invalid.
 */
bool pwm_adc_stats_take(pwm_adc_stats_t* out);

#endif /* FOC_MOTOR_PWM_ADC_H */
