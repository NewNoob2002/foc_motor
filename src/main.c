/* SPDX-License-Identifier: Apache-2.0 */

#include "analog.h"
#include "encoder.h"

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
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(timers1), okay), "TIM1 must remain disabled during M1");
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_console), DT_NODELABEL(usart3))
                 && DT_SAME_NODE(DT_CHOSEN(zephyr_shell_uart), DT_NODELABEL(usart3)),
             "Console and shell UART must use the USART3 header");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay), "CH340 USART1 must remain disabled");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(usart3), current_speed) == 921600, "USART3 must run at 921600 baud");
BUILD_ASSERT(DT_DMAS_HAS_NAME(DT_NODELABEL(usart3), tx), "USART3 must have a TX DMA channel");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, channel) == 2,
             "USART3 TX must use DMA selector 2 (DMA1 channel 3) during M1");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, slot) == 29, "USART3 TX must use DMAMUX request 29");

#define SAMPLES_PER_CHANNEL 1000U
#define REPORT_PERIOD_MS    1000

/* Mid-scale code: where IA/IC sit with zero current if VREF1.65 is exactly VDDA / 2. */
#define MID_SCALE_X100      (ANALOG_FULL_SCALE * 100 / 2)

typedef struct {
    uint32_t n;
    uint32_t sum;
    uint64_t sum_sq;
    uint16_t min;
    uint16_t max;
} sample_stats_t;

/* Integer square root (floor), for the standard deviation without the FPU. */
static uint32_t isqrt64(uint64_t v) {
    uint64_t root = 0;
    uint64_t bit = 1ULL << 62U;

    while (bit > v) {
        bit >>= 2U;
    }
    while (bit != 0) {
        if (v >= root + bit) {
            v -= root + bit;
            root = (root >> 1U) + bit;
        } else {
            root >>= 1U;
        }
        bit >>= 2U;
    }
    return (uint32_t)root;
}

static int sample_channel(analog_channel_t ch, uint32_t n, sample_stats_t* st) {
    *st = (sample_stats_t){.min = UINT16_MAX};

    for (uint32_t i = 0; i < n; i++) {
        uint16_t code;
        int ret = analog_read(ch, &code);

        if (ret < 0) {
            return ret;
        }
        st->n++;
        st->sum += code;
        st->sum_sq += (uint64_t)code * code;
        st->min = MIN(st->min, code);
        st->max = MAX(st->max, code);
    }
    return 0;
}

static int32_t mean_x100(const sample_stats_t* st) {
    return (int32_t)(((uint64_t)st->sum * 100U + st->n / 2U) / st->n);
}

/* Sample standard deviation in hundredths of a count. */
static int32_t sd_x100(const sample_stats_t* st) {
    uint64_t n = st->n;

    if (n < 2U) {
        return 0;
    }
    /* n * sum_sq - sum^2 = n^2 * variance_population; exact in integers, <= 1.7e13 for n = 1000. */
    uint64_t spread = n * st->sum_sq - (uint64_t)st->sum * st->sum;

    return (int32_t)isqrt64(spread * 10000U / (n * (n - 1U)));
}

/*
 * Kept out of line and split into short log calls: deferred logging builds
 * each message package on the caller's stack, and main has only 512 bytes.
 */
static __noinline void report(analog_channel_t ch, const sample_stats_t* st) {
    int32_t mean = mean_x100(st);
    int32_t sd = sd_x100(st);

    LOG_INF("%-4s mean %4d.%02d sd %d.%02d min %4u max %4u", analog_name(ch), mean / 100, mean % 100, sd / 100,
            sd % 100, st->min, st->max);

    switch (ch) {
        case ANALOG_CH_IA:
        case ANALOG_CH_IC:
            /* Current vs. the ideal mid-scale offset; the noise on the same scale. */
            LOG_INF("     %4d mV, %+d mA vs mid-scale, sd %d mA", analog_code_x100_to_mv(mean),
                    analog_phase_current_ma(mean, MID_SCALE_X100), analog_phase_current_ma(sd, 0));
            break;
        case ANALOG_CH_VBUS:
            LOG_INF("     %4d mV, bus %d mV", analog_code_x100_to_mv(mean), analog_vbus_mv(mean));
            break;
        case ANALOG_CH_TEMP:
            LOG_INF("     %4d mV, NTC %d ohm", analog_code_x100_to_mv(mean), analog_ntc_ohm(mean));
            break;
        default:
            break;
    }
}

/*
 * The report path (log package + log message allocation + ADC driver) needs
 * more than the 512-byte main stack that the M1 budget pins, so it runs in
 * its own statically allocated thread. Lower priority than the log thread (6).
 */
#define REPORT_STACK_SIZE 1024
#define REPORT_PRIORITY   7

K_THREAD_STACK_DEFINE(report_stack, REPORT_STACK_SIZE);
static struct k_thread report_thread;

static void report_loop(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (uint32_t n = 1;; n++) {
        LOG_INF("report %u, encoder mech %u", n, encoder_count() % ENCODER_COUNTS_PER_REV);
        for (int ch = 0; ch < ANALOG_CH_COUNT; ch++) {
            sample_stats_t st;
            int ret = sample_channel((analog_channel_t)ch, SAMPLES_PER_CHANNEL, &st);

            if (ret < 0) {
                LOG_ERR("%s read failed: %d", analog_name((analog_channel_t)ch), ret);
                return;
            }
            report((analog_channel_t)ch, &st);
        }
        k_msleep(REPORT_PERIOD_MS);
    }
}

int main(void) {
    int ret = encoder_init();

    LOG_INF("Stage 3 ADC; SD1-SD4 low; TIM1 disabled; USART3 DMA TX 921600");
    if (ret < 0) {
        LOG_ERR("encoder_init failed: %d", ret);
        return 0;
    }
    ret = analog_init();
    if (ret < 0) {
        LOG_ERR("analog_init failed: %d", ret);
        return 0;
    }
    LOG_INF("%u samples per channel per report, VDDA %d mV assumed", SAMPLES_PER_CHANNEL,
            analog_code_x100_to_mv(ANALOG_FULL_SCALE * 100));

    (void)k_thread_create(&report_thread, report_stack, K_THREAD_STACK_SIZEOF(report_stack), report_loop, NULL, NULL,
                          NULL, REPORT_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
