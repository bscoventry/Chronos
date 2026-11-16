/*
 * System OFF with P1.15 reed (GPIO level-high) wake.
 * Enter when BLE session is off and stimulation is not running.
 * Built only when CONFIG_CHRONOS_SYSTEM_OFF=y (see CMakeLists.txt).
 */

#include "power_off.h"
#include "ble_session.h"
#include "chronos_reed.h"
#include "timer.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SOC_SERIES_NRF54LX)
#include <nrfx.h>
#include <helpers/nrfx_reset_reason.h>
#endif

#include "chronos_hw.h"

#if !defined(CONFIG_SPI)
#include "spi.h"
extern int spi_ok;
#endif

LOG_MODULE_REGISTER(chronos_power, LOG_LEVEL_INF);

/*
 * nRF54L15 anomaly 48: when a wakeup source is active during a reset, RESETREAS
 * can report that wakeup alongside RESETPIN. A pin reset misread as a reed wake
 * would skip System OFF and open a BLE session, so RESETPIN wins and the stale
 * wakeup bits are cleared. Latched once because enter_system_off() clears
 * RESETREAS before sys_poweroff().
 */
static uint32_t boot_reset_cause;
static bool boot_reset_cause_valid;

static void boot_reset_cause_latch(void)
{
	uint32_t cause = 0;

	if (boot_reset_cause_valid) {
		return;
	}

	boot_reset_cause_valid = true;

	if (hwinfo_get_reset_cause(&cause) != 0) {
		return;
	}

	if ((cause & RESET_PIN) != 0U) {
#if defined(CONFIG_SOC_SERIES_NRF54LX)
		if (NRF_ERRATA_DYNAMIC_CHECK(54L, 48)) {
			nrfx_reset_reason_clear(~NRFX_RESET_REASON_RESETPIN_MASK);
		}
#endif
		cause = RESET_PIN;
	}

	boot_reset_cause = cause;
}

uint32_t chronos_boot_reset_cause(void)
{
	boot_reset_cause_latch();

	return boot_reset_cause;
}

bool chronos_boot_was_reed_wake(void)
{
	return (chronos_boot_reset_cause() & RESET_LOW_POWER_WAKE) != 0U;
}

static void suspend_console_if_any(void)
{
#if IS_ENABLED(CONFIG_PM_DEVICE)
	const struct device *cons = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console));

	if (cons != NULL && device_is_ready(cons)) {
		(void)pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	}
#endif
}

static void suspend_session_devices(void)
{
#if IS_ENABLED(CONFIG_PM_DEVICE)
#if DT_HAS_CHOSEN(nordic_nus_uart)
	const struct device *uart_dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(nordic_nus_uart));

	if (uart_dev != NULL && device_is_ready(uart_dev)) {
		(void)pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);
	}
#endif
#if defined(CONFIG_CHRONOS_ZMEAS) && DT_NODE_HAS_STATUS(DT_NODELABEL(i2c22), okay)
	const struct device *i2c = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c22));

	if (i2c != NULL && device_is_ready(i2c)) {
		(void)pm_device_action_run(i2c, PM_DEVICE_ACTION_SUSPEND);
	}
#endif
#endif
}

static void enter_system_off(void)
{
	if (!chronos_reed_is_low()) {
		LOG_WRN("P1.15 not low; skip System OFF (would wake immediately)");
		return;
	}

	LOG_INF("Entering System OFF (wake: reed P1.15 high)");

	timer_stim_hfclk_release();
#if !defined(CONFIG_SPI)
	if (spi_ok) {
		spi_prepare_for_power_off();
	}
#endif
	chronos_session_hw_suspend();
	ble_session_arm_reed_wake_for_system_off();
	suspend_console_if_any();
	suspend_session_devices();

	/* Allow RTT/log backends to drain before clocks stop. */
	k_msleep(50);

	(void)hwinfo_clear_reset_cause();
	sys_poweroff();
}

void chronos_power_off_try_enter(void)
{
	if (timer_stimulation_is_enabled()) {
		return;
	}

	if (ble_session_is_active()) {
		return;
	}

	enter_system_off();
}

