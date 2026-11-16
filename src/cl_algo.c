#include "cl_algo.h"

#include <math.h>
#include <string.h>

/* Consecutive same-direction steps before the step doubles back toward step_init. */
#define CL_REGROW_STEPS 3u

bool cl_meas_cfg_valid(const struct cl_meas_cfg *cfg)
{
	const uint32_t len = (uint32_t)cfg->pre_samples + cfg->post_samples;

	if (len == 0U || len > CL_MAX_EPOCH_SAMPLES) {
		return false;
	}
	if (cfg->baseline_samples < 2U || cfg->baseline_samples > cfg->pre_samples) {
		return false;
	}
	if (cfg->win_start < cfg->pre_samples || cfg->win_start >= cfg->win_end ||
	    cfg->win_end > len) {
		return false;
	}
	if (cfg->n_reps < 1U || cfg->n_reps > CL_MAX_REPS) {
		return false;
	}
	return cfg->clip_lo < cfg->clip_hi;
}

void cl_meas_init(struct cl_meas *m, const struct cl_meas_cfg *cfg)
{
	m->cfg = *cfg;
	cl_meas_reset(m);
}

void cl_meas_reset(struct cl_meas *m)
{
	memset(m->sum, 0, sizeof(m->sum));
	memset(m->rep_auc, 0, sizeof(m->rep_auc));
	m->n = 0U;
}

static bool range_clipped(const int16_t *x, uint16_t from, uint16_t to, int16_t lo, int16_t hi)
{
	for (uint16_t i = from; i < to; i++) {
		if (x[i] <= lo || x[i] >= hi) {
			return true;
		}
	}
	return false;
}

enum cl_epoch_status cl_meas_add_epoch(struct cl_meas *m, const int16_t *epoch)
{
	const struct cl_meas_cfg *cfg = &m->cfg;
	const uint16_t len = cfg->pre_samples + cfg->post_samples;
	const uint16_t win_len = cfg->win_end - cfg->win_start;
	float base = 0.0f;
	float base_abs = 0.0f;
	float win_abs = 0.0f;

	if (range_clipped(epoch, 0U, cfg->baseline_samples, cfg->clip_lo, cfg->clip_hi) ||
	    range_clipped(epoch, cfg->win_start, cfg->win_end, cfg->clip_lo, cfg->clip_hi)) {
		return CL_EPOCH_CLIPPED;
	}

	for (uint16_t i = 0; i < cfg->baseline_samples; i++) {
		base += (float)epoch[i];
	}
	base /= (float)cfg->baseline_samples;

	for (uint16_t i = 0; i < len; i++) {
		const float v = (float)epoch[i] - base;

		m->sum[i] += v;
		if (i < cfg->baseline_samples) {
			base_abs += fabsf(v);
		} else if (i >= cfg->win_start && i < cfg->win_end) {
			win_abs += fabsf(v);
		}
	}

	if (m->n < CL_MAX_REPS) {
		m->rep_auc[m->n] = win_abs - base_abs / (float)cfg->baseline_samples * (float)win_len;
	}
	m->n++;
	return CL_EPOCH_USED;
}

bool cl_meas_complete(const struct cl_meas *m)
{
	return m->n >= m->cfg.n_reps;
}

void cl_meas_result(const struct cl_meas *m, struct cl_block_result *r)
{
	const struct cl_meas_cfg *cfg = &m->cfg;
	const uint16_t win_len = cfg->win_end - cfg->win_start;
	const uint8_t n = m->n;
	float raw = 0.0f;
	float base_abs = 0.0f;

	memset(r, 0, sizeof(*r));
	r->n_reps = n;
	if (n == 0U) {
		return;
	}

	for (uint16_t i = 0; i < cfg->baseline_samples; i++) {
		base_abs += fabsf(m->sum[i] / (float)n);
	}
	for (uint16_t i = cfg->win_start; i < cfg->win_end; i++) {
		raw += fabsf(m->sum[i] / (float)n);
	}

	r->auc_raw = raw;
	r->noise_auc = base_abs / (float)cfg->baseline_samples * (float)win_len;
	r->auc = fmaxf(r->auc_raw - r->noise_auc, 0.0f);

	if (n > 1U) {
		float mean = 0.0f;
		float ss = 0.0f;

		for (uint8_t k = 0; k < n; k++) {
			mean += m->rep_auc[k];
		}
		mean /= (float)n;
		for (uint8_t k = 0; k < n; k++) {
			const float d = m->rep_auc[k] - mean;

			ss += d * d;
		}
		r->auc_sem = sqrtf(ss / (float)(n - 1U)) / sqrtf((float)n);
	}
}

bool cl_ctrl_cfg_valid(const struct cl_ctrl_cfg *cfg)
{
	if (!(cfg->target_auc > 0.0f)) {
		return false;
	}
	if (!(cfg->tol_frac > 0.0f && cfg->tol_frac < 1.0f)) {
		return false;
	}
	if (!(cfg->rearm_tol_frac >= cfg->tol_frac && cfg->rearm_tol_frac < 1.0f)) {
		return false;
	}
	if (cfg->amp_min >= cfg->amp_max) {
		return false;
	}
	if (cfg->step_min == 0U || cfg->step_min > cfg->step_init) {
		return false;
	}
	return cfg->in_band_required >= 1U && cfg->limit_blocks_required >= 1U;
}

void cl_ctrl_init(struct cl_ctrl *c, const struct cl_ctrl_cfg *cfg, uint16_t amp_start)
{
	c->cfg = *cfg;
	c->amp = amp_start;
	c->step = (float)cfg->step_init;
	c->last_dir = 0;
	c->same_dir = 0U;
	c->in_band = 0U;
	c->at_limit = 0U;
	c->converged = false;
}

static uint16_t clamp_amp(const struct cl_ctrl_cfg *cfg, int32_t amp)
{
	if (amp < (int32_t)cfg->amp_min) {
		return cfg->amp_min;
	}
	if (amp > (int32_t)cfg->amp_max) {
		return cfg->amp_max;
	}
	return (uint16_t)amp;
}

void cl_ctrl_update(struct cl_ctrl *c, float auc, struct cl_ctrl_step *out)
{
	const struct cl_ctrl_cfg *cfg = &c->cfg;
	const float err = (auc - cfg->target_auc) / cfg->target_auc;
	const float band = c->converged ? cfg->rearm_tol_frac : cfg->tol_frac;

	out->error_frac = err;
	out->measured_amp = c->amp;

	if (fabsf(err) <= band) {
		if (c->in_band < UINT8_MAX) {
			c->in_band++;
		}
		c->at_limit = 0U;
		if (c->in_band >= cfg->in_band_required) {
			c->converged = true;
		}
		out->decision = c->converged ? CL_DEC_CONVERGED : CL_DEC_HOLD;
	} else {
		const int8_t dir = (err < 0.0f) ? 1 : -1;
		int32_t delta;
		uint16_t proposed;

		c->in_band = 0U;
		c->converged = false;

		if (c->last_dir != 0 && dir != c->last_dir) {
			c->step = fmaxf(c->step * 0.5f, (float)cfg->step_min);
			c->same_dir = 1U;
		} else if (++c->same_dir >= CL_REGROW_STEPS) {
			c->step = fminf(c->step * 2.0f, (float)cfg->step_init);
			c->same_dir = 0U;
		}
		c->last_dir = dir;

		delta = (int32_t)lroundf(c->step);
		if (delta < 1) {
			delta = 1;
		}
		proposed = clamp_amp(cfg, (int32_t)c->amp + dir * delta);

		if (proposed == c->amp) {
			if (c->at_limit < UINT8_MAX) {
				c->at_limit++;
			}
			if (c->at_limit >= cfg->limit_blocks_required) {
				out->decision = (dir > 0) ? CL_DEC_UNREACHABLE_HIGH
							  : CL_DEC_UNREACHABLE_LOW;
			} else {
				out->decision = CL_DEC_HOLD_AT_LIMIT;
			}
		} else {
			c->at_limit = 0U;
			c->amp = proposed;
			out->decision = (dir > 0) ? CL_DEC_STEP_UP : CL_DEC_STEP_DOWN;
		}
	}

	out->next_amp = c->amp;
	out->step = c->step;
}

bool cl_decision_is_fault(enum cl_decision d)
{
	return d == CL_DEC_UNREACHABLE_HIGH || d == CL_DEC_UNREACHABLE_LOW;
}
