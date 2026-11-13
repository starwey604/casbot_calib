#include "usb_transport.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(board_cdc_acm_uart));
RING_BUF_DECLARE(tx_bytes, 16384);
K_MSGQ_DEFINE(rx_lines, USB_COMMAND_MAX, 8, 1);
static struct k_spinlock tx_lock, rx_lock;
static atomic_t tx_drops, rx_drops;
static char input[USB_COMMAND_MAX];
static size_t input_length;
static bool discard_line;

static void receive_byte(uint8_t byte)
{
	if (byte == '\r') { return; }
	if (byte == '\n') {
		if (!discard_line && input_length != 0) {
			input[input_length] = '\0';
			if (k_msgq_put(&rx_lines, input, K_NO_WAIT) != 0) { atomic_inc(&rx_drops); }
		}
		input_length = 0; discard_line = false;
	} else if (!discard_line) {
		if (byte < 32 || byte > 126 || input_length == sizeof(input) - 1) {
			/* Drop the WHOLE command; never execute its truncated suffix. */
			atomic_inc(&rx_drops); discard_line = true; input_length = 0;
		} else { input[input_length++] = (char)byte; }
	}
}

static void interrupt(const struct device *dev, void *user)
{
	ARG_UNUSED(user);
	for (;;) {
		uart_irq_update(dev);
		if (uart_irq_is_pending(dev) <= 0) { break; }
		if (uart_irq_rx_ready(dev)) {
			uint8_t bytes[64];
			int n = uart_fifo_read(dev, bytes, sizeof(bytes));
			uint32_t dtr = 0;
			(void)uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
			k_spinlock_key_t key = k_spin_lock(&rx_lock);
			if (dtr) {
				for (int i = 0; i < n; i++) { receive_byte(bytes[i]); }
			} else { input_length = 0; discard_line = false; }
			k_spin_unlock(&rx_lock, key);
		}
		if (uart_irq_tx_ready(dev)) {
			k_spinlock_key_t key = k_spin_lock(&tx_lock);
			uint8_t *data;
			uint32_t length = MIN(ring_buf_get_ptr(&tx_bytes, &data, 0), 64);
			if (length == 0) { uart_irq_tx_disable(dev); }
			else {
				int sent = uart_fifo_fill(dev, data, length);
				ring_buf_consume(&tx_bytes, sent > 0 ? (uint32_t)sent : 0);
			}
			k_spin_unlock(&tx_lock, key);
		}
	}
}

int bridge_usb_init(void)
{
	if (!device_is_ready(uart)) { return -ENODEV; }
	int err = uart_irq_callback_user_data_set(uart, interrupt, NULL);
	if (err == 0) { uart_irq_rx_enable(uart); }
	return err;
}

bool bridge_usb_connected(void)
{
	uint32_t dtr = 0;
	return uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr != 0;
}

bool bridge_usb_line(char *line) { return k_msgq_get(&rx_lines, line, K_NO_WAIT) == 0; }

int bridge_usb_json(const char *format, ...)
{
	char line[USB_JSON_MAX];
	va_list args;
	va_start(args, format);
	int n = vsnprintf(line, sizeof(line) - 1, format, args);
	va_end(args);
	if (n < 0 || n >= (int)sizeof(line) - 1) { atomic_inc(&tx_drops); return -EMSGSIZE; }
	if (!bridge_usb_connected()) { atomic_inc(&tx_drops); return -ENOTCONN; }
	line[n++] = '\n';
	k_spinlock_key_t key = k_spin_lock(&tx_lock);
	int err = 0;
	if (ring_buf_space_get(&tx_bytes) < (uint32_t)n) { atomic_inc(&tx_drops); err = -ENOBUFS; }
	else {
		(void)ring_buf_put(&tx_bytes, (const uint8_t *)line, n);
		uart_irq_tx_enable(uart);
	}
	k_spin_unlock(&tx_lock, key);
	return err;
}

void bridge_usb_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&tx_lock);
	uart_irq_tx_disable(uart); ring_buf_reset(&tx_bytes);
	k_spin_unlock(&tx_lock, key);
	key = k_spin_lock(&rx_lock);
	input_length = 0; discard_line = false; k_msgq_purge(&rx_lines);
	k_spin_unlock(&rx_lock, key);
}

uint32_t bridge_usb_tx_drops(void) { return (uint32_t)atomic_get(&tx_drops); }
uint32_t bridge_usb_rx_drops(void) { return (uint32_t)atomic_get(&rx_drops); }
