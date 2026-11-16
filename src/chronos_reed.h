#ifndef CHRONOS_REED_H
#define CHRONOS_REED_H

#include <stdbool.h>

/** Configure P1.15 input pull-down (idempotent). */
int chronos_reed_gpio_init(void);

/** True when reed input reads low (open / not swiped). */
bool chronos_reed_is_low(void);

/** Level-high GPIO sense for System OFF wake; removes runtime reed IRQ. */
void chronos_reed_arm_system_off_wake(void);

/** GPIO port after chronos_reed_gpio_init(); NULL if not ready. */
const struct device *chronos_reed_gpio_device(void);

#endif /* CHRONOS_REED_H */
