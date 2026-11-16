#ifndef CHRONOS_HW_H
#define CHRONOS_HW_H

#include <stdbool.h>

/** UART (NUS bridge) and optional ZMEAS — deferred until BLE session opens. */
int chronos_session_hw_init(void);

/** Suspend session peripherals when BLE closes (PM device suspend). */
void chronos_session_hw_suspend(void);

bool chronos_session_hw_is_ready(void);

#endif /* CHRONOS_HW_H */
