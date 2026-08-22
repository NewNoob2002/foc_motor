/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/devicetree/gpio.h>
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
BUILD_ASSERT(DT_PROP(DT_NODELABEL(usart1), current_speed) == 921600,
	     "USART1 must run at 921600 baud");
BUILD_ASSERT(DT_DMAS_HAS_NAME(DT_NODELABEL(usart1), tx),
	     "USART1 must have a TX DMA channel");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart1), tx, channel) == 2,
	     "USART1 TX must use DMA selector 2 (DMA1 channel 3) during M1");
BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_NODELABEL(usart1), tx, slot) == 25,
	     "USART1 TX must use DMAMUX request 25");

int main(void)
{
	LOG_INF("M1 safe boot; SD1-SD4 low; TIM1 disabled; UART DMA TX 921600");

	for (uint32_t sequence = 0; sequence < 16U; sequence++) {
		LOG_INF("uart_dma_tx_test seq=%u", sequence);
	}

	return 0;
}
