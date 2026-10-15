/* SPDX-License-Identifier: Apache-2.0 */

#include "analog.h"

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

#define ANALOG_USER_NODE    DT_PATH(zephyr_user)

/* Board constants from DOC-MAIN-SCH. */
#define SHUNT_GAIN_MA_PER_V 20000 /* 1 / (5 mOhm * gain 10) = 20 A/V */
#define VBUS_DIV_TOP_OHM    100000
#define VBUS_DIV_BOT_OHM    4700
#define NTC_PULLUP_OHM      10000

static const struct adc_dt_spec channels[ANALOG_CH_COUNT] = {
    [ANALOG_CH_IA] = ADC_DT_SPEC_GET_BY_NAME(ANALOG_USER_NODE, ia),
    [ANALOG_CH_IC] = ADC_DT_SPEC_GET_BY_NAME(ANALOG_USER_NODE, ic),
    [ANALOG_CH_VBUS] = ADC_DT_SPEC_GET_BY_NAME(ANALOG_USER_NODE, vbus),
    [ANALOG_CH_TEMP] = ADC_DT_SPEC_GET_BY_NAME(ANALOG_USER_NODE, temp),
};

static const char* const names[ANALOG_CH_COUNT] = {"IA", "IC", "VBUS", "TEMP"};

/*
 * All channels use ADC_REF_INTERNAL = VREF+ = VDDA. adc_dt_spec.vref_mv is
 * only set when a channel has zephyr,vref-mv, so take it from the driver
 * (the ADC node's vref-mv, default 3300).
 */
static int32_t vdda_mv;

BUILD_ASSERT(DT_REG_ADDR(DT_IO_CHANNELS_CTLR_BY_NAME(ANALOG_USER_NODE, ia)) == ADC2_BASE, "IA must be on ADC2");

int analog_init(void) {
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        if (!adc_is_ready_dt(&channels[i])) {
            return -ENODEV;
        }
        int ret = adc_channel_setup_dt(&channels[i]);
        if (ret < 0) {
            return ret;
        }
    }
    vdda_mv = adc_ref_internal(channels[0].dev);
    return (vdda_mv > 0) ? 0 : -EINVAL;
}

int analog_read(analog_channel_t ch, uint16_t* code) {
    if ((unsigned int)ch >= ANALOG_CH_COUNT || code == NULL) {
        return -EINVAL;
    }

    uint16_t sample = 0;
    struct adc_sequence seq = {
        .buffer = &sample,
        .buffer_size = sizeof(sample),
    };
    int ret = adc_sequence_init_dt(&channels[ch], &seq);
    if (ret < 0) {
        return ret;
    }
    ret = adc_read_dt(&channels[ch], &seq);
    if (ret < 0) {
        return ret;
    }
    *code = sample;
    return 0;
}

const char* analog_name(analog_channel_t ch) {
    return ((unsigned int)ch < ANALOG_CH_COUNT) ? names[ch] : "?";
}

int32_t analog_code_x100_to_mv(int32_t code_x100) {
    int64_t num = (int64_t)code_x100 * vdda_mv;

    return (int32_t)(num / (100LL * ANALOG_FULL_SCALE));
}

int32_t analog_phase_current_ma(int32_t code_x100, int32_t offset_x100) {
    int64_t num = (int64_t)(code_x100 - offset_x100) * vdda_mv * SHUNT_GAIN_MA_PER_V;

    /* mV * (mA/V) / 1000 = mA */
    return (int32_t)(num / (100LL * ANALOG_FULL_SCALE * 1000));
}

int32_t analog_vbus_mv(int32_t code_x100) {
    int64_t pin_mv = analog_code_x100_to_mv(code_x100);

    return (int32_t)(pin_mv * (VBUS_DIV_TOP_OHM + VBUS_DIV_BOT_OHM) / VBUS_DIV_BOT_OHM);
}

int32_t analog_ntc_ohm(int32_t code_x100) {
    int64_t den = 100LL * ANALOG_FULL_SCALE - code_x100;

    if (den <= 0) {
        return -1;
    }
    return (int32_t)((int64_t)NTC_PULLUP_OHM * code_x100 / den);
}
