#ifndef POWER_OFF_H
#define POWER_OFF_H

#include <stdbool.h>
#include <stdint.h>

#if defined(CONFIG_BT) && defined(CONFIG_CHRONOS_SYSTEM_OFF)

/** Boot reset cause (Zephyr RESET_* flags), latched before RESETREAS is cleared. */
uint32_t chronos_boot_reset_cause(void);

/** True when this boot followed GPIO (reed) wake from System OFF. */
bool chronos_boot_was_reed_wake(void);

/** Enter System OFF when BLE is off and stimulation is not running. */
void chronos_power_off_try_enter(void);

#else

static inline uint32_t chronos_boot_reset_cause(void)
{
	return 0U;
}

static inline bool chronos_boot_was_reed_wake(void)
{
	return false;
}

static inline void chronos_power_off_try_enter(void) {}

#endif

#endif /* POWER_OFF_H */
