/*
 * Reed switch on P1.15 — shared by ble_session and System OFF entry.
 */

#include "chronos_reed.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(chronos_reed, LOG_LEVEL_INF);

#define REED_GPIO_NODE DT_NODELABEL(gpio1)
#define REED_PIN       15

static const struct device *reed_port;
static bool reed_gpio_ready;

int chronos_reed_gpio_init(void)
{
	int err;

	if (reed_gpio_ready) {
		return 0;
	}

	reed_port = DEVICE_DT_GET(REED_GPIO_NODE);
	if (!device_is_ready(reed_port)) {
		LOG_ERR("GPIO1 not ready for reed (P1.15)");
		return -ENODEV;
	}

	err = gpio_pin_configure(reed_port, REED_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (err != 0) {
		LOG_ERR("Reed pin cfg err %d", err);
		return err;
	}

	reed_gpio_ready = true;
	return 0;
}

bool chronos_reed_is_low(void)
{
	int level;

	if (!reed_gpio_ready || !device_is_ready(reed_port)) {
		return false;
	}

	level = gpio_pin_get(reed_port, REED_PIN);
	if (level < 0) {
		return false;
	}

	return level == 0;
}

void chronos_reed_arm_system_off_wake(void)
{
	if (!reed_gpio_ready || !device_is_ready(reed_port)) {
		return;
	}

	(void)gpio_pin_interrupt_configure(reed_port, REED_PIN, GPIO_INT_DISABLE);
	(void)gpio_pin_configure(reed_port, REED_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	(void)gpio_pin_interrupt_configure(reed_port, REED_PIN, GPIO_INT_LEVEL_HIGH);
}

const struct device *chronos_reed_gpio_device(void)
{
	if (!reed_gpio_ready) {
		return NULL;
	}

	return reed_port;
}
