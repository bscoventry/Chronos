#ifndef CHRONOS_HFCLK_H
#define CHRONOS_HFCLK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * HFXO ownership for the stimulation path.
 *
 * HFXO is referenced once for a whole stimulation session instead of being started
 * and stopped around each burst. Startup is 1650 us (the hfxo node's
 * startup-time-us, which MPSL also budgets for) against a 7.7 ms period at 130 Hz,
 * so per-burst cycling spent roughly a fifth of every period blocked inside the
 * period ISR before the first phase could start, and the DAC write path paid the
 * same cost again on every transfer.
 *
 * Requests go through the clock control onoff manager rather than CLOCK
 * TASKS_XOSTART/XOSTOP. The manager is reference counted and shared with MPSL, so
 * releasing here cannot stop the crystal while the radio still needs it. It also
 * routes through nrfx_clock, which applies the nRF54L15 anomaly 39 ordering
 * (PLLSTART before XOSTART, PLLSTOP after XOSTOP) on the revisions that need it.
 */

/**
 * Reference HFXO for a stimulation session. Idempotent.
 *
 * Blocks until the crystal is stable, so thread context only.
 *
 * @retval 0 on success or when already held, negative errno otherwise.
 */
int chronos_hfclk_acquire(void);

/**
 * Drop the session reference. Idempotent, and safe to call from an ISR.
 *
 * MPSL keeps HFXO running if the radio still needs it.
 */
void chronos_hfclk_release(void);

/** True when HFXO is running as the high-frequency source. Non-blocking, ISR safe. */
bool chronos_hfclk_is_ready(void);

/**
 * Non-blocking readiness check that records failures. Call once per burst.
 *
 * Never waits: a burst that stalls waiting for the crystal delays every phase in
 * the pulse and blocks higher-priority radio work.
 */
bool chronos_hfclk_check_ready(void);

/** Bursts that started without HFXO stable; those ran SPI/DAC timing off HFINT. */
extern uint32_t chronos_hfclk_not_ready_count;

#endif /* CHRONOS_HFCLK_H */
