#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const cdc = DEVICE_DT_GET(DT_NODELABEL(board_cdc_acm_uart));

int main(void)
{
	if (!gpio_is_ready_dt(&led) || !device_is_ready(cdc)) {
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return ret;
	}

	int64_t next_blink = k_uptime_get() + 500;

	while (true) {
		if (k_uptime_get() >= next_blink) {
			ret = gpio_pin_toggle_dt(&led);
			if (ret < 0) {
				return ret;
			}
			next_blink = k_uptime_get() + 500;
		}

		uint32_t dtr = 0;

		ret = uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr);
		if (ret < 0) {
			return ret;
		}
		if (dtr != 0) {
			/* Small polling echo: raw bytes, no console/banner mixed in. */
			for (int i = 0; i < 64; ++i) {
				unsigned char byte;

				ret = uart_poll_in(cdc, &byte);
				if (ret == -1) {
					break;
				}
				if (ret < 0) {
					return ret;
				}
				uart_poll_out(cdc, byte);
			}
		}
		k_msleep(1);
	}
}
