#ifndef BLE_SESSION_H
#define BLE_SESSION_H

#include <stdbool.h>

#if defined(CONFIG_BT)

/** Reed P1.15 (NO, swipe to VCC): each rising edge toggles BLE after lockout (ble_session.c). */
void ble_session_init(void);

/** Reset 5-minute inactivity timeout (call on each NUS RX). */
void ble_session_feed_activity(void);

/** True while BLE session is advertising / stack enabled. */
bool ble_session_is_active(void);

/** Level-high sense on P1.15 for System OFF wake (CONFIG_CHRONOS_SYSTEM_OFF). */
void ble_session_arm_reed_wake_for_system_off(void);

#else

static inline void ble_session_init(void) {}
static inline void ble_session_feed_activity(void) {}
static inline bool ble_session_is_active(void) { return false; }
static inline void ble_session_arm_reed_wake_for_system_off(void) {}

#endif

#endif /* BLE_SESSION_H */
