/* SPDX-License-Identifier: Apache-2.0 */

#include "analog.h"
#include "encoder.h"
#include "pwm_adc.h"

#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/drivers/uart.h>
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

#define REPORT_PERIOD_MS  1000

/* Mid-scale code: where IA/IC sit with zero current if VREF1.65 is exactly VDDA / 2. */
#define MID_SCALE_X100    (ANALOG_FULL_SCALE * 100 / 2)

/* TIM1 counts at HCLK: 170 ticks per microsecond. */
#define TIM1_TICKS_PER_US 170U

/*
 * The report path (log package + log message allocation) needs more than the
 * 512-byte main stack that the M1 budget pins, so it runs in its own
 * statically allocated thread. Lower priority than the log thread (6).
 */
#define REPORT_STACK_SIZE 1024
#define REPORT_PRIORITY   7

K_THREAD_STACK_DEFINE(report_stack, REPORT_STACK_SIZE);
static struct k_thread report_thread;

static int32_t mean_x100(uint32_t sum, uint32_t n) {
    return (n == 0U) ? 0 : (int32_t)(((uint64_t)sum * 100U + n / 2U) / n);
}

/* Kept out of line so its log packages live on this frame, not report_loop's. */
static __noinline void report(uint32_t n, const pwm_adc_stats_t* st, bool timing_valid) {
    if (st->isr_count == 0U) {
        LOG_WRN("report %u: no ISR in this window; DMA errors %u, ADC overruns %u", n, st->dma_errors,
                st->adc_overruns);
        return;
    }

    /* Counting down, CNT = ARR - ticks since the top, where the ADC was triggered. */
    uint32_t after_top_min = (PWM_ARR - st->cnt_max) * 100U / TIM1_TICKS_PER_US;
    uint32_t after_top_max = (PWM_ARR - st->cnt_min) * 100U / TIM1_TICKS_PER_US;
    int32_t ia = mean_x100(st->ia_sum, st->isr_count);
    int32_t ic = mean_x100(st->ic_sum, st->isr_count);

    if (timing_valid) {
        LOG_INF("report %u: %u ISR/s, period %u..%u cyc, exec max %u cyc", n, st->isr_count, st->period_min,
                st->period_max, st->exec_max);
    } else {
        LOG_WRN("report %u: %u ISR/s, DWT was stopped (debugger?) and is restarted; cycle figures invalid", n,
                st->isr_count);
    }
    LOG_INF("  TIM1 CNT at ISR %u..%u (%u.%02u..%u.%02u us after top), counting up in %u", st->cnt_min, st->cnt_max,
            after_top_min / 100U, after_top_min % 100U, after_top_max / 100U, after_top_max % 100U, st->counting_up);
    LOG_INF("  IA mean %d.%02d (%+d mA vs mid-scale), IC mean %d.%02d (%+d mA)", ia / 100, ia % 100,
            analog_phase_current_ma(ia, MID_SCALE_X100), ic / 100, ic % 100,
            analog_phase_current_ma(ic, MID_SCALE_X100));
    LOG_INF("  DMA errors %u, ADC overruns %u, encoder mech %u", st->dma_errors, st->adc_overruns,
            encoder_count() % ENCODER_COUNTS_PER_REV);
}

static const char* const mode_names[] = {
    [PWM_MODE_FIXED] = "1: CH1-3 fixed 25/50/75 %, RCR 0",
    [PWM_MODE_ALT_RCR0] = "2: CH1 alternates 25/75 % per period, RCR 0",
    [PWM_MODE_ALT_RCR1] = "3: CH1 alternates 25/75 % per period, RCR 1",
};

/* A key '1'-'3' on the console UART selects the scope test mode. */
static void poll_mode_key(const struct device* uart) {
    unsigned char c;

    while (uart_poll_in(uart, &c) == 0) {
        if (c >= '1' && c < (unsigned char)('1' + ARRAY_SIZE(mode_names))) {
            pwm_mode_t m = (pwm_mode_t)(c - '1');

            pwm_adc_set_mode(m);
            LOG_INF("mode %s", mode_names[m]);
        }
    }
}

static void report_loop(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device* const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    pwm_adc_stats_t st;

    /* Drop the start-up window. */
    (void)pwm_adc_stats_take(&st);
    for (uint32_t n = 1;; n++) {
        k_msleep(REPORT_PERIOD_MS);
        if (!pwm_adc_outputs_safe()) {
            pwm_adc_stop();
            LOG_ERR("TIM1 output invariant broken (PC0-PC3 or CCER); outputs off, timer stopped");
            return;
        }
        poll_mode_key(uart);

        bool timing_valid = pwm_adc_stats_take(&st);

        report(n, &st, timing_valid);
    }
}

int main(void) {
    int ret = encoder_init();

    LOG_INF("Stage 4 scope build; TIM1 CH1-3 on PA8-PA10 only, PC0-PC3 analog, ISR marker PB9; SD1-SD4 low");
    if (ret < 0) {
        LOG_ERR("encoder_init failed: %d", ret);
        return 0;
    }
    ret = pwm_adc_init();
    if (ret < 0) {
        LOG_ERR("pwm_adc_init failed: %d", ret);
        return 0;
    }
    LOG_INF("TIM1 ARR %u (20 kHz center-aligned), ADC2 [IA, IC] on TRGO2 at the top; mode %s", PWM_ARR,
            mode_names[PWM_MODE_FIXED]);

    (void)k_thread_create(&report_thread, report_stack, K_THREAD_STACK_SIZEOF(report_stack), report_loop, NULL, NULL,
                          NULL, REPORT_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
