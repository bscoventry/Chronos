#ifndef CHRONOS_BOOT_H
#define CHRONOS_BOOT_H

#if defined(CONFIG_BT)

/** SPI, timers, stim defaults, optional session HW (full app init). */
void chronos_app_init_full(void);

#endif

#endif /* CHRONOS_BOOT_H */
