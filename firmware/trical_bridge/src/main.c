#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trical_bridge, LOG_LEVEL_INF);

static const struct device *const usb_data =
	DEVICE_DT_GET(DT_NODELABEL(board_cdc_acm_uart));

int main(void)
{
	int err;

	if (!device_is_ready(usb_data)) {
		LOG_ERR("USB CDC ACM device is not ready");
		return -ENODEV;
	}

	err = bt_enable(NULL);
	if (err != 0) {
		LOG_ERR("Bluetooth initialization failed (%d)", err);
		return err;
	}

	LOG_INF("Build foundation ready: USB CDC ACM, BLE central, %d connection slots",
		CONFIG_BT_MAX_CONN);
	LOG_INF("Instrument scanning, connection and QAS commands are not enabled");

	/* USB is initialized by Zephyr at boot. Instrument acquisition comes next. */
	for (;;) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
