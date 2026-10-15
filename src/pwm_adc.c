/* SPDX-License-Identifier: Apache-2.0 */

#include "pwm_adc.h"

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

/* DMA2 channel 1 is above every Zephyr driver IRQ (priority 5). */
#define DMA_IRQ_PRIORITY      1

/* ADC regular group start: OC4REF rises one tick before the counter top. */
#define TRIGGER_CCR4          (PWM_ARR - 1U)

/* Fixed Stage 4 duties 0.25 / 0.50 / 0.75 of the period. */
#define CCR_DUTY_25           (PWM_ARR / 4U)
#define CCR_DUTY_50           (PWM_ARR / 2U)
#define CCR_DUTY_75           (PWM_ARR * 3U / 4U)

/* PC0-PC3 (PWMA-D) must stay analog: MODER bits 0-7 all set. */
#define GPIOC_PWM_PINS_ANALOG 0xFFU

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

static void stats_reset_locked(void) {
    uint32_t dma_errors = stats.dma_errors;
    uint32_t adc_overruns = stats.adc_overruns;

    stats = (pwm_adc_stats_t){
        .period_min = UINT32_MAX,
        .cnt_min = UINT16_MAX,
        .dma_errors = dma_errors,
        .adc_overruns = adc_overruns,
    };
}

static void dma_isr(const void* arg) {
    ARG_UNUSED(arg);

    uint32_t entry = DWT->CYCCNT;
    uint16_t cnt = (uint16_t)LL_TIM_GetCounter(TIM1);
    bool up = LL_TIM_GetDirection(TIM1) == LL_TIM_COUNTERDIRECTION_UP;
    bool dma_error = LL_DMA_IsActiveFlag_TE1(DMA2) != 0U;
    bool overrun = LL_ADC_IsActiveFlag_OVR(ADC2) != 0U;

    LL_DMA_ClearFlag_GI1(DMA2);
    if (overrun) {
        LL_ADC_ClearFlag_OVR(ADC2);
    }

    (void)atomic_inc(&seq);
    if (atomic_cas(&reset_req, 1, 0)) {
        stats_reset_locked();
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
    stats.ia_last = adc_buf[0];
    stats.ic_last = adc_buf[1];
    stats.ia_sum += stats.ia_last;
    stats.ic_sum += stats.ic_last;
    stats.dma_errors += dma_error ? 1U : 0U;
    stats.adc_overruns += overrun ? 1U : 0U;
    stats.exec_max = MAX(stats.exec_max, DWT->CYCCNT - entry);
    (void)atomic_inc(&seq);
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

static void dwt_start(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Counter, compares and TRGO2 only; no output is enabled. */
static void tim1_setup(void) {
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_SetPrescaler(TIM1, 0U);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_UP);
    LL_TIM_SetAutoReload(TIM1, PWM_ARR);
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetRepetitionCounter(TIM1, 0U);

    static const uint32_t channels[] = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3};
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        LL_TIM_OC_SetMode(TIM1, channels[i], LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_EnablePreload(TIM1, channels[i]);
    }
    LL_TIM_OC_SetCompareCH1(TIM1, CCR_DUTY_25);
    LL_TIM_OC_SetCompareCH2(TIM1, CCR_DUTY_50);
    LL_TIM_OC_SetCompareCH3(TIM1, CCR_DUTY_75);

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

/* The Stage 4 safety condition, read back from the hardware. */
static bool outputs_disabled(void) {
    return LL_TIM_ReadReg(TIM1, CCER) == 0U && LL_TIM_IsEnabledAllOutputs(TIM1) == 0U
           && (LL_GPIO_ReadReg(GPIOC, MODER) & GPIOC_PWM_PINS_ANALOG) == GPIOC_PWM_PINS_ANALOG;
}

int pwm_adc_init(void) {
    int ret = clocks_on();

    if (ret < 0) {
        return ret;
    }
    dwt_start();
    tim1_setup();
    if (!outputs_disabled()) {
        return -EPERM;
    }
    ret = adc2_setup();
    if (ret < 0) {
        return ret;
    }
    dma2_setup();

    stats_reset_locked();
    IRQ_CONNECT(DMA2_Channel1_IRQn, DMA_IRQ_PRIORITY, dma_isr, NULL, 0);
    irq_enable(DMA2_Channel1_IRQn);

    /* The ADC waits for TRGO2; the first trigger comes once the counter runs. */
    LL_ADC_REG_StartConversion(ADC2);
    LL_TIM_EnableCounter(TIM1);

    if (!outputs_disabled()) {
        LL_TIM_DisableCounter(TIM1);
        return -EPERM;
    }
    return 0;
}

void pwm_adc_stats_take(pwm_adc_stats_t* out) {
    atomic_val_t before;

    do {
        before = atomic_get(&seq);
        *out = stats;
    } while (((uint32_t)before & 1U) != 0U || atomic_get(&seq) != before);

    (void)atomic_set(&reset_req, 1);
}
