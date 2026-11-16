#ifndef CL_ALGO_H
#define CL_ALGO_H

/*
 * A-beta block measurement and amplitude staircase. Plain C with no Zephyr dependencies so the
 * same code can be built for a host test. Mirrors python/src/abeta_analysis.py analyze_block()
 * and abeta_controller.py, with the controller changed to track indefinitely.
 *
 * Areas are in ADC counts x samples. The GUI converts to uV*ms with the ADC scale, front-end
 * gain and sample period, so the chip never needs either.
 */

#include <stdbool.h>
#include <stdint.h>

#define CL_MAX_EPOCH_SAMPLES 1024u
#define CL_MAX_REPS          32u

/* Epoch layout. Indices are into the epoch, where index pre_samples is the stimulus onset. */
struct cl_meas_cfg {
	uint16_t pre_samples;
	uint16_t post_samples;
	uint16_t baseline_samples; /* epoch[0 .. baseline_samples) is the baseline */
	uint16_t win_start;        /* A-beta window, epoch[win_start .. win_end) */
	uint16_t win_end;
	uint8_t n_reps;
	int16_t clip_lo;           /* a baseline or window sample at or beyond these is clipped */
	int16_t clip_hi;
};

struct cl_meas {
	struct cl_meas_cfg cfg;
	float sum[CL_MAX_EPOCH_SAMPLES];
	float rep_auc[CL_MAX_REPS];
	uint8_t n;
};

struct cl_block_result {
	float auc;      /* max(auc_raw - noise_auc, 0) */
	float auc_raw;
	float noise_auc;
	float auc_sem;  /* standard error of the per-rep areas; 0 when n < 2 */
	uint8_t n_reps;
};

enum cl_epoch_status {
	CL_EPOCH_USED = 0,
	CL_EPOCH_CLIPPED = 1,
};

bool cl_meas_cfg_valid(const struct cl_meas_cfg *cfg);
void cl_meas_init(struct cl_meas *m, const struct cl_meas_cfg *cfg);
void cl_meas_reset(struct cl_meas *m);
/* epoch holds pre_samples + post_samples samples. */
enum cl_epoch_status cl_meas_add_epoch(struct cl_meas *m, const int16_t *epoch);
bool cl_meas_complete(const struct cl_meas *m);
void cl_meas_result(const struct cl_meas *m, struct cl_block_result *r);

enum cl_decision {
	CL_DEC_MEASURED = 0,         /* single calibration block, no control */
	CL_DEC_HOLD = 1,             /* in band, not yet converged */
	CL_DEC_CONVERGED = 2,        /* in band for in_band_required blocks; keeps tracking */
	CL_DEC_STEP_UP = 3,
	CL_DEC_STEP_DOWN = 4,
	CL_DEC_HOLD_AT_LIMIT = 5,
	CL_DEC_UNREACHABLE_HIGH = 6, /* fault: at max amplitude and still below the band */
	CL_DEC_UNREACHABLE_LOW = 7,  /* fault: at min amplitude and still above the band */
};

/* Amplitudes and steps are DAC code magnitudes above zero current (code - 0x8000). */
struct cl_ctrl_cfg {
	float target_auc;
	float tol_frac;        /* band before convergence */
	float rearm_tol_frac;  /* band after convergence; >= tol_frac so it does not chatter */
	uint16_t amp_min;
	uint16_t amp_max;
	uint16_t step_init;
	uint16_t step_min;
	uint8_t in_band_required;
	uint8_t limit_blocks_required;
};

struct cl_ctrl {
	struct cl_ctrl_cfg cfg;
	uint16_t amp;
	float step;
	int8_t last_dir;
	uint8_t same_dir;
	uint8_t in_band;
	uint8_t at_limit;
	bool converged;
};

struct cl_ctrl_step {
	enum cl_decision decision;
	float error_frac;
	uint16_t measured_amp;
	uint16_t next_amp;
	float step;
};

bool cl_ctrl_cfg_valid(const struct cl_ctrl_cfg *cfg);
void cl_ctrl_init(struct cl_ctrl *c, const struct cl_ctrl_cfg *cfg, uint16_t amp_start);
void cl_ctrl_update(struct cl_ctrl *c, float auc, struct cl_ctrl_step *out);
bool cl_decision_is_fault(enum cl_decision d);

#endif /* CL_ALGO_H */
