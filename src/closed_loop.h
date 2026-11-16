#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

/*
 * On-chip A-beta closed loop. The GUI configures it and either runs one calibration block or
 * arms continuous control; the chip measures every block of pulses and sets the next amplitude
 * itself. Raw ADC samples keep streaming to the GUI throughout.
 *
 * Wire format (little-endian, packed) is mirrored in python/src/abeta_onchip.py.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/toolchain.h>

/* GUI -> chip CL_CTRL_CONFIG payload, after the control byte. */
struct cl_config_wire {
	float target_auc;            /* ADC counts x samples */
	uint16_t tol_permille;
	uint16_t rearm_tol_permille;
	uint16_t pre_samples;
	uint16_t post_samples;
	uint16_t baseline_samples;
	uint16_t win_start;
	uint16_t win_end;
	int16_t clip_lo;
	int16_t clip_hi;
	uint16_t amp_min;            /* DAC code magnitude above 0x8000 (cathode-leading) */
	uint16_t amp_max;
	uint16_t step_init;
	uint16_t step_min;
	uint8_t n_reps;
	uint8_t discard;             /* pulses skipped after every amplitude change */
	uint8_t in_band_required;
	uint8_t limit_blocks_required;
} __packed;

/* Chip -> GUI packet types. All start with a byte >= 0x80 so they never parse as text. */
#define CL_PKT_PULSE  0xC1u
#define CL_PKT_BLOCK  0xC2u
#define CL_PKT_STATE  0xC3u
#define CL_PKT_CONFIG 0xC4u

enum cl_state {
	CL_STATE_IDLE = 0,
	CL_STATE_BLOCK = 1,
	CL_STATE_ARMED = 2,
};

enum cl_pulse_status {
	CL_PULSE_USED = 0,
	CL_PULSE_CLIPPED = 1,
	CL_PULSE_DISCARD_SETTLE = 2,
	CL_PULSE_DISCARD_AMP = 3,    /* delivered at a different amplitude than the block */
	CL_PULSE_GAP = 4,            /* epoch samples missing from the ring */
	CL_PULSE_EXTRA = 5,          /* arrived after a calibration block finished */
};

enum cl_event {
	CL_EVT_NONE = 0,
	CL_EVT_BLOCK_STARTED = 1,
	CL_EVT_BLOCK_DONE = 2,
	CL_EVT_ARMED = 3,
	CL_EVT_DISARMED = 4,
	CL_EVT_DISCONNECTED = 5,
	CL_EVT_FAULT_UNREACHABLE_HIGH = 6,
	CL_EVT_FAULT_UNREACHABLE_LOW = 7,
	CL_EVT_FAULT_NO_ADC = 8,
	CL_EVT_FAULT_NO_PULSES = 9,
	CL_EVT_FAULT_NO_EPOCHS = 10,
	CL_EVT_REJECT_NO_CONFIG = 11,
	CL_EVT_REJECT_BAD_AMPLITUDE = 12,
	CL_EVT_REJECT_STIM_RUNNING = 13,
	CL_EVT_REJECT_EPOCH_TOO_LONG = 14,
	CL_EVT_REJECT_BUSY = 15,
	CL_EVT_STIMULATION_STARTED = 16,
};

struct cl_pulse_pkt {
	uint8_t type;
	uint32_t pulse_no;
	uint32_t onset_sample;       /* ADC sample index since the ADC session started */
	uint16_t amp_code;
	uint32_t block_no;
	uint8_t status;
} __packed;

struct cl_block_pkt {
	uint8_t type;
	uint32_t block_no;
	uint16_t amp_code;           /* amplitude the block was measured at */
	uint16_t next_amp_code;
	float auc;
	float auc_raw;
	float noise_auc;
	float auc_sem;
	float error_frac;
	float step;                  /* DAC code magnitude */
	uint8_t decision;            /* enum cl_decision */
	uint8_t n_reps;
	uint8_t converged;
} __packed;

struct cl_state_pkt {
	uint8_t type;
	uint8_t state;
	uint8_t event;
	uint16_t amp_code;
} __packed;

struct cl_config_pkt {
	uint8_t type;
	uint8_t accepted;
	struct cl_config_wire cfg;   /* as stored, after clamping to the firmware ceilings */
} __packed;

void closed_loop_init(void);
bool closed_loop_is_active(void);
bool closed_loop_blocks_manual_stim(void);

void closed_loop_configure(const uint8_t *payload, size_t len);
void closed_loop_run_block(void);
void closed_loop_arm(void);
void closed_loop_disarm(enum cl_event reason);

/** Process one filled SAADC buffer (ADC consumer thread). seq counts buffers since ADC start. */
void closed_loop_on_saadc_buffer(uint32_t seq, const int16_t *samples, size_t count);

/** Called from the stim period ISR immediately before phase 1 of every pulse. */
void closed_loop_on_stim_pulse(void);

#endif /* CLOSED_LOOP_H */
