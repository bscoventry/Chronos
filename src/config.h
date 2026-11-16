#ifndef CONFIG_H
#define CONFIG_H

#define MEASURE_TIMER 0 // 1: testing timer accuracy with built-in timer
                        // 0: disable measurement timer
#define SPI_VERBOSE 0   // 1: enable verbose SPI logging

/* CS hold after END before cs_deselect (us). DAC8832 latches on the CS rising edge, so
 * this delay sets when the stimulus phase actually starts. Increase if CS rises before
 * SCLK finishes. */
#define SPI_CS_HOLD_US  2u

/*
 * SPIM bit rate for DAC8832.
 *
 * SPIM21's peripheral core is 16 MHz with a prescaler range of 2..126, so 8 MHz is
 * prescaler 2. nRF54L15 anomaly 8 corrupts MOSI data when CPHA=0, prescaler > 2, and
 * the first transmitted bit is 1 — which at 2 MHz (prescaler 8) silently corrupted every
 * word with a set MSB (0xFFAA phase-2 amplitude and the 0x8000 neutral code) while
 * leaving 0x0056 intact, breaking biphasic charge balance. Prescaler 2 leaves the
 * anomaly's condition unsatisfied. 8 Mbps is also the max for non-high-speed SPIM.
 */
#define CONFIG_SPI_FREQUENCY_HZ      8000000u

/* High drive on SCK/MOSI. nrfx only forces this at 32 MHz, but the datasheet notes high
 * bit rates may require it. Cost here is negligible: ~2 us of switching per 40 ms. */
#define SPI_HIGH_DRIVE  1

/*
 * Drive strength for the analog-switch control pins (P1.16 / P1.02 / P1.03).
 *
 * Standard drive (S0S1) is intentional: faster edges (H0H1 / E0E1) increase charge
 * injection through the switch into the electrode path. P1 has no separate slew-rate
 * control (that exists only on P2), so DRIVE0/DRIVE1 is the only lever. Leave at 0
 * unless the switch fails to meet its logic thresholds into a heavy capacitive load.
 */
#define STIM_SWITCH_HIGH_DRIVE  0

/* P0.00: low while stimulation session is active (TIMER20 running); high when idle. */
#define STIM_SESSION_GPIO_PORT  0u
#define STIM_SESSION_GPIO_NUM   0u

/*
 * DAC code that delivers zero current. 0x8000 is the DAC8832's bipolar mid-code, but DAC
 * and amplifier offsets put the true zero a few LSB away.
 *
 * This code is commanded for the interphase gap and the inter-pulse interval. The series
 * switches are open across both, so the output stage is unloaded and this code does not by
 * itself keep the amplifier off a rail -- the make-before-break overlap at the start of the
 * next phase is what handles that. Where the value does matter is during those overlaps,
 * when the source is connected and driving the shunt. Trim on the bench with an in-line
 * current probe until the overlap reads zero.
 */
#define STIM_DAC_ZERO_CODE  0x8000u

/* Biphasic square-wave stimulation (compile-time; overrides BLE) */
#define CONFIG_STIM_AMPLITUDE        0xFFAA   /* DAC amplitude (16-bit). Phase 2 = opposite. */
#define CONFIG_PULSE_WIDTH_US        200u      /* Pulse duration per phase (us). Typically 200 us. */
#define CONFIG_INTER_PHASE_GAP_US    25u       /* Gap between phase 1 and phase 2 (us); timer.h SWITCH_PERIOD. */
#define CONFIG_STIM_FREQUENCY_HZ     25u       /* Biphasic pulse rate (Hz). Test default: 25 Hz (40 ms period). */

/*
 * Overlap between closing the series switches and opening the shunt at the start of each
 * phase (us). The amplifier recovers from rail and settles to the phase amplitude inside
 * this window while the shunt still absorbs its output.
 *
 * This now has to cover rail recovery rather than just a DAC step, so 10 us is very likely
 * short. It is also a hard constraint against the interphase gap: phase 2's pre-roll runs
 * inside CONFIG_INTER_PHASE_GAP_US, so this must stay comfortably below it. Scope the
 * recovery on the bench before trusting either value.
 */
#define CONFIG_DAC_SETTLE_US           10u

/*
 * Scale SAADC pin uV to signal uV at the electrode front-end (AIN0 / P1.04 path).
 * Default 1:1; tune on bench if the front-end divides or amplifies.
 */
#define CHRONOS_ADC_SIGNAL_SCALE_NUM   1000
#define CHRONOS_ADC_SIGNAL_SCALE_DEN   1000

/* On-chip A-beta closed loop: firmware ceilings (DAC magnitude above 0x8000). */
#define CL_HARD_MAX_AMP_MAG  0x7800u
#define CL_HARD_MAX_STEP_MAG 0x1000u

#endif /* CONFIG_H */
