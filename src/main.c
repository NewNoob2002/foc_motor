/* SPDX-License-Identifier: Apache-2.0 */

#include "encoder.h"

#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(foc_motor, LOG_LEVEL_INF);

#define ASSERT_SHUTDOWN_LOW(node_id, port_id, pin, name)                         \
	BUILD_ASSERT(DT_NODE_HAS_PROP(node_id, gpio_hog), name " must be a GPIO hog"); \
	BUILD_ASSERT(DT_NODE_HAS_PROP(node_id, output_low), name " must start low");   \
	BUILD_ASSERT(DT_SAME_NODE(DT_PARENT(node_id), port_id), name " wrong port");   \
	BUILD_ASSERT(DT_GPIO_HOG_PIN_BY_IDX(node_id, 0) == pin, name " wrong pin")

ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd1_hog), DT_NODELABEL(gpiob), 0, "SD1");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd2_hog), DT_NODELABEL(gpioa), 1, "SD2");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd3_hog), DT_NODELABEL(gpioa), 2, "SD3");
ASSERT_SHUTDOWN_LOW(DT_NODELABEL(sd4_hog), DT_NODELABEL(gpioa), 0, "SD4");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(timers1), okay),
	     "TIM1 must remain disabled during M1");
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_console), DT_NODELABEL(usart3)) &&
	     DT_SAME_NODE(DT_CHOSEN(zephyr_shell_uart), DT_NODELABEL(usart3)),
	     "Console and shell UART must use the USART3 header");
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay),
	     "CH340 USART1 must remain disabled");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(usart3), current_speed) == 921600,
	     "USART3 must run at 921600 baud");
BUILD_ASSERT(DT_DMAS_HAS_NAME(DT_NODELABEL(usart3), tx),
	     "USART3 must have a TX DMA channel");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, channel) == 2,
	     "USART3 TX must use DMA selector 2 (DMA1 channel 3) during M1");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart3), tx, slot) == 29,
	     "USART3 TX must use DMAMUX request 29");

/* hardware_reference.md section 10: 14 poles. */
#define MOTOR_POLE_PAIRS 7U
#define PRINT_PERIOD_MS 100

/* Signed shortest step from `from` to `to` on a counter that wraps at `period`. */
static int32_t wrap_delta(uint32_t from, uint32_t to, uint32_t period)
{
	uint32_t d = (to + period - from) % period;

	return (d >= period / 2U) ? (int32_t)d - (int32_t)period : (int32_t)d;
}

int main(void)
{
	int ret = encoder_init();

	LOG_INF("Stage 2 encoder; SD1-SD4 low; TIM1 disabled; USART3 DMA TX 921600");
	if (ret < 0) {
		LOG_ERR("encoder_init failed: %d", ret);
		return 0;
	}

	/*
	 * Turning by hand moves far less than period / 2 counts (7.5 turns) per
	 * print period, so wrap_delta() never picks the wrong direction.
	 */
	const uint32_t period = encoder_period();
	uint32_t prev = encoder_count();
	int32_t pos = 0; /* counts since boot; + = TIM3 counting up */
	encoder_index_t z_last = encoder_index_get();
	int32_t pos_at_last_z = 0;
	bool have_z = false;

	LOG_INF("TIM3 period %u counts, %u counts/turn, %u pole pairs", period,
		ENCODER_COUNTS_PER_REV, MOTOR_POLE_PAIRS);

	while (true) {
		k_msleep(PRINT_PERIOD_MS);

		uint32_t cnt = encoder_count();
		encoder_index_t z = encoder_index_get();

		pos += wrap_delta(prev, cnt, period);
		prev = cnt;

		/*
		 * theta_e from the count within the turn: 4096 / 7 is not an
		 * integer, so reduce in integers first, then scale. No offset
		 * yet; the d-axis alignment is measured in Stage 7.
		 */
		uint32_t mech = cnt % ENCODER_COUNTS_PER_REV;
		uint32_t elec = (mech * MOTOR_POLE_PAIRS) % ENCODER_COUNTS_PER_REV;
		uint32_t theta_e_x10 = (elec * 3600U) / ENCODER_COUNTS_PER_REV;

		/* Floor division, so -1 count is turn -1, not turn 0. */
		const int32_t cpr = (int32_t)ENCODER_COUNTS_PER_REV;
		int32_t turn = (pos >= 0) ? pos / cpr : -((-pos + cpr - 1) / cpr);

		LOG_INF("cnt %5u pos %+7d turn %+4d mech %4u theta_e %3u.%u deg", cnt, pos, turn,
			mech, theta_e_x10 / 10U, theta_e_x10 % 10U);

		if (z.events != z_last.events) {
			int32_t pos_at_z = pos - wrap_delta(z.count, cnt, period);

			/* A healthy encoder gives the same mech count at every Z and
			 * multiples of 4096 between Zs in the same direction.
			 */
			LOG_INF("Z #%u (+%u) mech %4u pos %+7d, %+d since last Z", z.events,
				z.events - z_last.events, z.count % ENCODER_COUNTS_PER_REV,
				pos_at_z, have_z ? pos_at_z - pos_at_last_z : 0);
			z_last = z;
			pos_at_last_z = pos_at_z;
			have_z = true;
		}
	}

	return 0;
}
