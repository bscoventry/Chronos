/*
 * Session-gated hardware: UART NUS bridge and AD5933 probe run only when BLE opens.
 */

#include "chronos_hw.h"
#include "BLE.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#if defined(CONFIG_CHRONOS_ZMEAS)
#include "ad5933.h"
#endif

LOG_MODULE_REGISTER(chronos_hw, LOG_LEVEL_INF);

static bool hw_inited;
static bool hw_suspended;

#if IS_ENABLED(CONFIG_PM_DEVICE)

static void pm_suspend_dev(const struct device *dev)
{
	if (dev == NULL || !device_is_ready(dev)) {
		return;
	}

	(void)pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
}

static void pm_resume_dev(const struct device *dev)
{
	if (dev == NULL || !device_is_ready(dev)) {
		return;
	}

	(void)pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
}

#endif /* CONFIG_PM_DEVICE */

static void session_hw_suspend_devices(void)
{
#if IS_ENABLED(CONFIG_PM_DEVICE)
#if DT_HAS_CHOSEN(nordic_nus_uart)
	pm_suspend_dev(DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart)));
#endif
#if defined(CONFIG_CHRONOS_ZMEAS) && DT_NODE_HAS_STATUS(DT_NODELABEL(i2c22), okay)
	pm_suspend_dev(DEVICE_DT_GET(DT_NODELABEL(i2c22)));
#endif
#endif
}

static void session_hw_resume_devices(void)
{
#if IS_ENABLED(CONFIG_PM_DEVICE)
#if DT_HAS_CHOSEN(nordic_nus_uart)
	pm_resume_dev(DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart)));
#endif
#if defined(CONFIG_CHRONOS_ZMEAS) && DT_NODE_HAS_STATUS(DT_NODELABEL(i2c22), okay)
	pm_resume_dev(DEVICE_DT_GET(DT_NODELABEL(i2c22)));
#endif
#endif
}

int chronos_session_hw_init(void)
{
	int err = 0;

	if (hw_suspended) {
		session_hw_resume_devices();
		hw_suspended = false;
		return 0;
	}

	if (hw_inited) {
		return 0;
	}

#if defined(CONFIG_BT)
	err = uart_init();
	if (err != 0) {
		LOG_WRN("UART init failed: %d (BLE NUS over-air still works)", err);
	}
#endif

#if defined(CONFIG_CHRONOS_ZMEAS)
	{
		int zerr = ad5933_phase_a_probe(false);

		if (zerr != 0) {
			LOG_WRN("AD5933 probe failed: %d", zerr);
		}
	}
#endif

	hw_inited = true;
	return err;
}

void chronos_session_hw_suspend(void)
{
	if (!hw_inited || hw_suspended) {
		return;
	}

	session_hw_suspend_devices();
	hw_suspended = true;
}

bool chronos_session_hw_is_ready(void)
{
	return hw_inited && !hw_suspended;
}
