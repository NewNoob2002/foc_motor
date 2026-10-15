/* SPDX-License-Identifier: Apache-2.0 */

#include "encoder.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_tim.h>

#define ENCODER_NODE DT_NODELABEL(encoder)
#define ENCODER_TIM  ((TIM_TypeDef*)DT_REG_ADDR(DT_PARENT(ENCODER_NODE)))

BUILD_ASSERT(DT_NODE_HAS_STATUS(ENCODER_NODE, okay), "encoder qdec node must be enabled");
BUILD_ASSERT(DT_REG_ADDR(DT_PARENT(ENCODER_NODE)) == TIM3_BASE, "encoder must use TIM3");
BUILD_ASSERT(ENCODER_COUNTS_PER_REV == 4096, "KTH7823 default ABZ resolution (HW-ENC-003)");

static const struct device* const qdec = DEVICE_DT_GET(ENCODER_NODE);
static const struct gpio_dt_spec z_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), encoder_z_gpios);
static struct gpio_callback z_cb;

/* Single writer (z_isr); readers retry if an edge lands mid-read. */
static atomic_t z_events;
static volatile uint32_t z_count;

static void z_isr(const struct device* port, struct gpio_callback* cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    z_count = LL_TIM_GetCounter(ENCODER_TIM);
    /* atomic_inc is a full barrier, so z_count is visible before the new event number. */
    (void)atomic_inc(&z_events);
}

int encoder_init(void) {
    int ret;

    if (!device_is_ready(qdec) || !gpio_is_ready_dt(&z_pin)) {
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&z_pin, GPIO_INPUT);
    if (ret < 0) {
        return ret;
    }

    gpio_init_callback(&z_cb, z_isr, BIT(z_pin.pin));
    ret = gpio_add_callback_dt(&z_pin, &z_cb);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&z_pin, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        (void)gpio_remove_callback_dt(&z_pin, &z_cb);
    }

    return ret;
}

uint32_t encoder_count(void) {
    return LL_TIM_GetCounter(ENCODER_TIM);
}

uint32_t encoder_period(void) {
    return LL_TIM_GetAutoReload(ENCODER_TIM) + 1U;
}

encoder_index_t encoder_index_get(void) {
    encoder_index_t idx;
    atomic_val_t before;

    do {
        before = atomic_get(&z_events);
        idx.count = z_count;
        idx.events = (uint32_t)atomic_get(&z_events);
    } while ((uint32_t)before != idx.events);

    return idx;
}
