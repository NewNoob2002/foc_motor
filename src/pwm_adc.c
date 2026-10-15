/* SPDX-License-Identifier: Apache-2.0 */

#include "pwm_adc.h"

#include "cycles.h"
#include "encoder.h"

#include <errno.h>

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_adc.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_tim.h>

/*
 * The devicetree nodes stay disabled so no Zephyr driver or pinctrl claims
 * them; the BSP takes the addresses and bus clocks from them.
 */
#define TIM1_NODE DT_NODELABEL(timers1)
#define ADC2_NODE DT_NODELABEL(adc2)
#define DMA2_NODE DT_NODELABEL(dma2)

BUILD_ASSERT(!DT_NODE_HAS_STATUS(TIM1_NODE, okay), "TIM1 must have no Zephyr driver or pins");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(ADC2_NODE, okay), "ADC2 is owned by this BSP, not the Zephyr ADC driver");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DMA2_NODE, okay), "DMA2 is owned by this BSP, not the Zephyr DMA driver");
BUILD_ASSERT(DT_REG_ADDR(TIM1_NODE) == TIM1_BASE && DT_REG_ADDR(ADC2_NODE) == ADC2_BASE
                 && DT_REG_ADDR(DMA2_NODE) == DMA2_BASE,
             "unexpected peripheral addresses");

/* Fixed-rank sequence [IA, IC] (HW-ADC-007). */
#define ADC_CH_IA             LL_ADC_CHANNEL_3  /* PA6 */
#define ADC_CH_IC             LL_ADC_CHANNEL_17 /* PA4 */

/*
 * Zero-latency: kernel critical sections (BASEPRI) do not mask it, so its
 * entry time does not depend on what the threads are doing. It is the only
 * one (AGENTS.md), and it must not call any kernel API.
 */
#define DMA_IRQ_PRIORITY      0

/* ADC regular group start: OC4REF rises one tick before the counter top. */
#define TRIGGER_CCR4          (PWM_ARR - 1U)

/* 50 %: zero line-to-line voltage, used before the first step and after a fault. */
#define CCR_DUTY_50           (PWM_ARR / 2U)

/* PC0-PC3 (PWMA-D) must stay analog: MODER bits 0-7 all set. */
#define GPIOC_PWM_PINS_ANALOG 0xFFU

/*
 * Scope pins: TIM1 CH1-3 on PA8/PA9/PA10 (AF6, LQFP 41-43, not connected on
 * this board; checked floating 2026-10-14) and TIM1 CH3N on PB9 (AF12, the
 * U7-4 header pin, floating since the Z wire went to U7-3). CH3N's other
 * pins PB1, PB15 and PF0 are not in AF mode, so only PB9 carries it.
 */
#define PROBE_AF              LL_GPIO_AF_6
#define CH3N_PIN              LL_GPIO_PIN_9
#define CH3N_AF               LL_GPIO_AF_12

/* No CH4 pin and no CH1N/CH2N (CH2N could reach PB0 = SD1). */
#define CCER_ALLOWED          (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC3NE)

static volatile uint16_t adc_buf[2];

/*
 * Single writer (the ISR). seq is odd while the ISR updates stats, so a
 * reader retries if it saw an odd value or a change across its copy.
 */
static pwm_adc_stats_t stats;
static atomic_t seq;
static atomic_t reset_req;
static uint32_t last_entry;
static bool have_last_entry;

/* Set once by pwm_adc_init() before the ISR is enabled. */
static pwm_adc_step_fn step_fn;
static uint32_t overruns_in_a_row;

/*
 * Forced-overrun test (Stage 6): the debugger writes test_spin_cycles, then
 * test_spin_count; the next test_spin_count ISRs busy-wait until they have
 * run test_spin_cycles. Both stay 0 in normal operation.
 */
static volatile uint32_t test_spin_cycles;
static volatile uint32_t test_spin_count;

static void stats_reset_locked(void) {
    pwm_adc_stats_t keep = stats;

    stats = (pwm_adc_stats_t){
        .period_min = UINT32_MAX,
        .cnt_min = UINT16_MAX,
        .ccr = {keep.ccr[0], keep.ccr[1], keep.ccr[2]},
        .overruns = keep.overruns,
        .missed = keep.missed,
        .fault = keep.fault,
        .dma_errors = keep.dma_errors,
        .adc_overruns = keep.adc_overruns,
    };
}

static uint16_t duty_to_ccr(float duty) {
    /* PWM mode 1, center-aligned: the output is high while CNT < CCR, so duty = CCR / ARR. */
    if (!(duty > 0.0f)) {
        return 0U; /* also NaN */
    }
    if (duty >= 1.0f) {
        return (uint16_t)PWM_ARR;
    }
    return (uint16_t)(duty * (float)PWM_ARR + 0.5f);
}

static void set_ccr(uint16_t a, uint16_t b, uint16_t c) {
    /* Preload: the values wait for the next update event, the next counter top. */
    LL_TIM_OC_SetCompareCH1(TIM1, a);
    LL_TIM_OC_SetCompareCH2(TIM1, b);
    LL_TIM_OC_SetCompareCH3(TIM1, c);
    stats.ccr[0] = a;
    stats.ccr[1] = b;
    stats.ccr[2] = c;
}

/* Outputs off (MOE = 0) and 50 % duty; the counter keeps running so the ISR keeps its statistics. */
static void fault_latch(uint32_t reason) {
    LL_TIM_DisableAllOutputs(TIM1);
    set_ccr(CCR_DUTY_50, CCR_DUTY_50, CCR_DUTY_50);
    stats.fault |= reason;
}

ISR_DIRECT_DECLARE(dma_isr) {
    uint32_t entry = cycles_now();

    uint16_t cnt = (uint16_t)LL_TIM_GetCounter(TIM1);
    bool up = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
    bool dma_error = LL_DMA_IsActiveFlag_TE1(DMA2) != 0U;
    bool overrun = LL_ADC_IsActiveFlag_OVR(ADC2) != 0U;
    pwm_adc_sample_t sample = {.ia_code = adc_buf[0], .ic_code = adc_buf[1], .enc_count = encoder_count()};

    LL_DMA_ClearFlag_GI1(DMA2);
    if (overrun) {
        LL_ADC_ClearFlag_OVR(ADC2);
    }

    (void)atomic_inc(&seq);
    if (atomic_cas(&reset_req, 1, 0)) {
        stats_reset_locked();
    }
    if (dma_error) {
        stats.dma_errors++;
        fault_latch(PWM_ADC_FAULT_DMA_ERROR);
    }
    if (stats.fault == 0U) {
        pwm_adc_duty_t duty;

        step_fn(&sample, &duty);
        set_ccr(duty_to_ccr(duty.a), duty_to_ccr(duty.b), duty_to_ccr(duty.c));
    }

    if (have_last_entry) {
        uint32_t period = entry - last_entry;

        stats.period_min = MIN(stats.period_min, period);
        stats.period_max = MAX(stats.period_max, period);
    }
    last_entry = entry;
    have_last_entry = true;

    stats.isr_count++;
    stats.cnt_min = MIN(stats.cnt_min, cnt);
    stats.cnt_max = MAX(stats.cnt_max, cnt);
    stats.counting_up += up ? 1U : 0U;
    stats.ia_sum += sample.ia_code;
    stats.ic_sum += sample.ic_code;
    stats.adc_overruns += overrun ? 1U : 0U;

    if (test_spin_count > 0U) {
        test_spin_count = test_spin_count - 1U;
        while (cycles_now() - entry < test_spin_cycles) {}
    }

    uint32_t exec = cycles_now() - entry;

    stats.exec_max = MAX(stats.exec_max, exec);
    if (exec > PWM_ADC_EXEC_BUDGET) {
        stats.overruns++;
        overruns_in_a_row++;
        if (overruns_in_a_row >= PWM_ADC_OVERRUN_FAULT_COUNT) {
            fault_latch(PWM_ADC_FAULT_OVERRUN);
        }
    } else {
        overruns_in_a_row = 0U;
    }
    /* The next [IA, IC] already arrived: this ISR ran past the next counter top. */
    if (LL_DMA_IsActiveFlag_TC1(DMA2) != 0U) {
        stats.missed++;
    }
    (void)atomic_inc(&seq);
    return 0; /* no reschedule: required for a zero-latency ISR */
}

static int clocks_on(void) {
    const struct device* const rcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
    const struct stm32_pclken clocks[] = {
        STM32_CLOCK_INFO(0, TIM1_NODE),
        STM32_CLOCK_INFO(0, ADC2_NODE),
        STM32_CLOCK_INFO(0, DMA2_NODE),
        /* Also on for the USART3 DMA; enabling it again is harmless. */
        STM32_CLOCK_INFO(0, DT_NODELABEL(dmamux1)),
    };

    if (!device_is_ready(rcc)) {
        return -ENODEV;
    }
    for (size_t i = 0; i < ARRAY_SIZE(clocks); i++) {
        int ret = clock_control_on(rcc, (clock_control_subsys_t)&clocks[i]);

        if (ret < 0) {
            return ret;
        }
    }
    return 0;
}

/* PA8-PA10 and PB9 to TIM1. PC0-PC3 untouched. */
static void probe_pins_setup(void) {
    /* SetPinMode/Speed/Pull and SetAFPin act on one pin each (POSITION_VAL), not on a mask. */
    static const uint32_t probe_pins[] = {LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10};

    for (size_t i = 0; i < ARRAY_SIZE(probe_pins); i++) {
        LL_GPIO_SetPinOutputType(GPIOA, probe_pins[i], LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinSpeed(GPIOA, probe_pins[i], LL_GPIO_SPEED_FREQ_LOW);
        LL_GPIO_SetPinPull(GPIOA, probe_pins[i], LL_GPIO_PULL_NO);
        LL_GPIO_SetAFPin_8_15(GPIOA, probe_pins[i], PROBE_AF);
        LL_GPIO_SetPinMode(GPIOA, probe_pins[i], LL_GPIO_MODE_ALTERNATE);
    }

    LL_GPIO_SetPinOutputType(GPIOB, CH3N_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOB, CH3N_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOB, CH3N_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetAFPin_8_15(GPIOB, CH3N_PIN, CH3N_AF);
    LL_GPIO_SetPinMode(GPIOB, CH3N_PIN, LL_GPIO_MODE_ALTERNATE);
}

/* Counter, compares and TRGO2; outputs stay off until pwm_adc_init() checks the pins. */
static void tim1_setup(void) {
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_SetPrescaler(TIM1, 0U);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_UP);
    LL_TIM_SetAutoReload(TIM1, PWM_ARR);
    LL_TIM_EnableARRPreload(TIM1);
    /*
     * RCR 1, loaded by the UG below before the counter starts: one update
     * event per period, at the top, where all high sides are off (stage 4
     * scope check, mode 5). RCR 0 would add an update at the bottom that
     * splits a high-side pulse between the old and the new duty.
     */
    LL_TIM_SetRepetitionCounter(TIM1, 1U);

    static const uint32_t channels[] = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3};
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        LL_TIM_OC_SetMode(TIM1, channels[i], LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_EnablePreload(TIM1, channels[i]);
    }
    LL_TIM_OC_SetCompareCH1(TIM1, CCR_DUTY_50);
    LL_TIM_OC_SetCompareCH2(TIM1, CCR_DUTY_50);
    LL_TIM_OC_SetCompareCH3(TIM1, CCR_DUTY_50);

    /* CH4 is internal only: PWM2 makes OC4REF rise when CNT reaches CCR4 near the top. */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIM1, TRIGGER_CCR4);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);

    LL_TIM_WriteReg(TIM1, CCER, 0U);
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
}

static int adc2_setup(void) {
    LL_ADC_SetCommonClock(ADC12_COMMON, LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);
    k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    for (int i = 0; LL_ADC_IsCalibrationOnGoing(ADC2) != 0U; i++) {
        if (i > 1000) {
            return -ETIMEDOUT;
        }
        k_busy_wait(1);
    }
    /* 4 ADC clocks at 42.5 MHz between calibration end and ADEN. */
    k_busy_wait(1);

    LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC2, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
    LL_ADC_REG_SetTriggerEdge(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
    LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetOverrun(ADC2, LL_ADC_REG_OVR_DATA_PRESERVED);
    LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC_CH_IA);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, ADC_CH_IC);
    LL_ADC_SetChannelSingleDiff(ADC2, ADC_CH_IA, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSingleDiff(ADC2, ADC_CH_IC, LL_ADC_SINGLE_ENDED);
    LL_ADC_SetChannelSamplingTime(ADC2, ADC_CH_IA, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC2, ADC_CH_IC, LL_ADC_SAMPLINGTIME_47CYCLES_5);

    LL_ADC_ClearFlag_ADRDY(ADC2);
    LL_ADC_Enable(ADC2);
    for (int i = 0; LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0U; i++) {
        if (i > 1000) {
            return -ETIMEDOUT;
        }
        k_busy_wait(1);
    }
    return 0;
}

static void dma2_setup(void) {
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_1,
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT
                              | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD
                              | LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)adc_buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, ARRAY_SIZE(adc_buf));
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC2);
    LL_DMA_ClearFlag_GI1(DMA2);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
}

bool pwm_adc_outputs_safe(void) {
    return (LL_TIM_ReadReg(TIM1, CCER) & ~CCER_ALLOWED) == 0U
           && (LL_GPIO_ReadReg(GPIOC, MODER) & GPIOC_PWM_PINS_ANALOG) == GPIOC_PWM_PINS_ANALOG;
}

void pwm_adc_stop(void) {
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_DisableCounter(TIM1);
}

int pwm_adc_init(pwm_adc_step_fn step) {
    if (step == NULL) {
        return -EINVAL;
    }
    step_fn = step;

    int ret = clocks_on();

    if (ret < 0) {
        return ret;
    }
    (void)cycles_start();
    tim1_setup();
    if (!pwm_adc_outputs_safe()) {
        return -EPERM;
    }
    probe_pins_setup();
    ret = adc2_setup();
    if (ret < 0) {
        return ret;
    }
    dma2_setup();

    stats_reset_locked();
    stats.ccr[0] = CCR_DUTY_50;
    stats.ccr[1] = CCR_DUTY_50;
    stats.ccr[2] = CCR_DUTY_50;
    IRQ_DIRECT_CONNECT(DMA2_Channel1_IRQn, DMA_IRQ_PRIORITY, dma_isr, IRQ_ZERO_LATENCY);
    irq_enable(DMA2_Channel1_IRQn);

    /* The ADC waits for TRGO2; the first trigger comes once the counter runs. */
    LL_ADC_REG_StartConversion(ADC2);
    /* Dead time stays 0, so CH3N is exactly CH3 inverted. */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    if (!pwm_adc_outputs_safe()) {
        pwm_adc_stop();
        return -EPERM;
    }
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    return 0;
}

bool pwm_adc_stats_take(pwm_adc_stats_t* out) {
    atomic_val_t before;
    bool timing_valid = cycles_start();

    do {
        before = atomic_get(&seq);
        *out = stats;
    } while (((uint32_t)before & 1U) != 0U || atomic_get(&seq) != before);

    (void)atomic_set(&reset_req, 1);
    return timing_valid;
}
