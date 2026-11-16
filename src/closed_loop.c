#include "closed_loop.h"
#include "cl_algo.h"
#include "config.h"
#include "data.h"
#include "ADC.h"
#include "BLE.h"
#include "spi.h"
#include "timer.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(chronos_closed_loop, LOG_LEVEL_INF);

#define CL_DAC_ZERO_MAG_CODE  0x8000u
#define CL_RING_SAMPLES       4096u
#define CL_MAX_PENDING        8u
/* ADC runs this long before the first pulse so the 3 Hz front-end high-pass settles. */
#define CL_ADC_SETTLE_MS      200
#define CL_WATCHDOG_MS        100
#define CL_NO_ADC_MS          250
#define CL_NO_PULSE_MARGIN_MS 250
#define CL_TX_DEPTH           24
#define CL_TX_MAX_LEN         40
#define CL_TX_STACK_SIZE      1536
#define CL_TX_PRIORITY        6

BUILD_ASSERT((CL_RING_SAMPLES & (CL_RING_SAMPLES - 1u)) == 0u, "ring must be a power of two");
BUILD_ASSERT(CL_RING_SAMPLES >= 2u * CL_MAX_EPOCH_SAMPLES, "ring must hold two epochs");
BUILD_ASSERT(sizeof(struct cl_config_wire) == 34U, "cl_config_wire size");
BUILD_ASSERT(sizeof(struct cl_config_pkt) <= CL_TX_MAX_LEN, "config packet too long");
BUILD_ASSERT(sizeof(struct cl_block_pkt) <= CL_TX_MAX_LEN, "block packet too long");

struct cl_pulse {
	uint32_t onset;
	uint32_t pulse_no;
	uint16_t amp_code;
};

struct cl_tx {
	uint8_t len;
	uint8_t data[CL_TX_MAX_LEN];
};

K_MSGQ_DEFINE(cl_pulse_msgq, sizeof(struct cl_pulse), 16, 4);
K_MSGQ_DEFINE(cl_tx_msgq, sizeof(struct cl_tx), CL_TX_DEPTH, 4);
K_MUTEX_DEFINE(cl_lock);
K_THREAD_STACK_DEFINE(cl_tx_stack, CL_TX_STACK_SIZE);

/* TX runs on its own queue so a notification blocked on BLE buffers never delays a stop. */
static struct k_work_q cl_tx_q;
static struct k_work cl_tx_work;
static struct k_work_delayable start_work;
static struct k_work_delayable watchdog_work;
static struct k_work stop_work;

static atomic_t state_val;
static atomic_t stop_reason;

/* Shared with the stim ISR. pending_amp is 0 or (0x10000 | code). */
static atomic_t pulses_enabled;
static atomic_t pending_amp;
static uint16_t isr_amp_code;
static uint32_t isr_pulse_no;

static bool cfg_valid;
static struct cl_config_wire cfg_wire;
static struct cl_meas_cfg meas_cfg;
static struct cl_ctrl_cfg ctrl_cfg;
static struct cl_meas meas;
static struct cl_ctrl ctrl;

static uint32_t block_no;
static uint16_t block_amp_code;
static uint8_t settle_left;
static uint32_t block_pulses;
static bool block_done;
static bool stim_started;
static uint32_t period_ms;

static int16_t ring[CL_RING_SAMPLES];
static uint32_t ring_end;
static uint32_t ring_valid_from;
static bool ring_started;
static struct cl_pulse pending[CL_MAX_PENDING];
static uint8_t n_pending;
static int16_t epoch_buf[CL_MAX_EPOCH_SAMPLES];

static int64_t last_buf_ms;
static int64_t last_pulse_ms;

static enum cl_state get_state(void)
{
	return (enum cl_state)atomic_get(&state_val);
}

static void tx_work_handler(struct k_work *work)
{
	struct cl_tx t;

	ARG_UNUSED(work);

	while (k_msgq_get(&cl_tx_msgq, &t, K_NO_WAIT) == 0) {
		(void)ble_send_raw(t.data, t.len);
	}
}

static void cl_send(const void *pkt, size_t len)
{
	struct cl_tx t;

	t.len = (uint8_t)len;
	memcpy(t.data, pkt, len);
	if (k_msgq_put(&cl_tx_msgq, &t, K_NO_WAIT) != 0) {
		LOG_WRN("CL TX queue full; packet 0x%02x dropped", t.data[0]);
	}
	(void)k_work_submit_to_queue(&cl_tx_q, &cl_tx_work);
}

static void send_state(enum cl_event event, uint16_t amp_code)
{
	const struct cl_state_pkt p = {
		.type = CL_PKT_STATE,
		.state = (uint8_t)get_state(),
		.event = (uint8_t)event,
		.amp_code = amp_code,
	};

	cl_send(&p, sizeof(p));
}

static void send_pulse(const struct cl_pulse *pl, enum cl_pulse_status status)
{
	const struct cl_pulse_pkt p = {
		.type = CL_PKT_PULSE,
		.pulse_no = pl->pulse_no,
		.onset_sample = pl->onset,
		.amp_code = pl->amp_code,
		.block_no = block_no,
		.status = (uint8_t)status,
	};

	cl_send(&p, sizeof(p));
}

static void send_config(bool accepted)
{
	struct cl_config_pkt p = {
		.type = CL_PKT_CONFIG,
		.accepted = accepted ? 1U : 0U,
	};

	p.cfg = cfg_wire;
	cl_send(&p, sizeof(p));
}

/* cl_lock held. Pulses already queued in the ISR are dropped by block_done or the amp check. */
static void request_stop(enum cl_event reason)
{
	block_done = true;
	atomic_set(&pulses_enabled, 0);
	atomic_set(&stop_reason, (atomic_val_t)reason);
	(void)k_work_submit(&stop_work);
}

/* cl_lock held. Returns false if already idle. */
static bool do_disarm(enum cl_event reason)
{
	if (get_state() == CL_STATE_IDLE) {
		return false;
	}

	atomic_set(&pulses_enabled, 0);
	atomic_set(&pending_amp, 0);
	atomic_set(&state_val, CL_STATE_IDLE);
	(void)k_work_cancel_delayable(&start_work);
	(void)k_work_cancel_delayable(&watchdog_work);
	chronos_stimulation_request_stop();
	stim_started = false;
	n_pending = 0U;
	send_state(reason, isr_amp_code);
	LOG_INF("Closed-loop stopped, event %d, last amplitude 0x%04x", (int)reason,
		isr_amp_code);
	return true;
}

static void start_block(uint16_t amp_code, bool amp_changed)
{
	block_amp_code = amp_code;
	settle_left = amp_changed ? cfg_wire.discard : 0U;
	block_pulses = 0U;
	cl_meas_reset(&meas);
}

static void finish_block(void)
{
	struct cl_block_result r;
	struct cl_block_pkt p = {
		.type = CL_PKT_BLOCK,
		.block_no = block_no,
		.amp_code = block_amp_code,
	};

	cl_meas_result(&meas, &r);
	p.auc = r.auc;
	p.auc_raw = r.auc_raw;
	p.noise_auc = r.noise_auc;
	p.auc_sem = r.auc_sem;
	p.n_reps = r.n_reps;

	if (get_state() == CL_STATE_BLOCK) {
		p.decision = CL_DEC_MEASURED;
		p.next_amp_code = block_amp_code;
		p.error_frac = (r.auc - ctrl_cfg.target_auc) / ctrl_cfg.target_auc;
		p.step = ctrl.step;
		cl_send(&p, sizeof(p));
		request_stop(CL_EVT_BLOCK_DONE);
		return;
	}

	struct cl_ctrl_step st;
	uint16_t next_code;
	bool changed;

	cl_ctrl_update(&ctrl, r.auc, &st);
	next_code = (uint16_t)(CL_DAC_ZERO_MAG_CODE + st.next_amp);
	p.decision = (uint8_t)st.decision;
	p.next_amp_code = next_code;
	p.error_frac = st.error_frac;
	p.step = st.step;
	p.converged = ctrl.converged ? 1U : 0U;
	cl_send(&p, sizeof(p));

	if (cl_decision_is_fault(st.decision)) {
		request_stop(st.decision == CL_DEC_UNREACHABLE_HIGH ? CL_EVT_FAULT_UNREACHABLE_HIGH
								     : CL_EVT_FAULT_UNREACHABLE_LOW);
		return;
	}

	changed = next_code != block_amp_code;
	if (changed) {
		atomic_set(&pending_amp, (atomic_val_t)(0x10000u | next_code));
	}
	block_no++;
	start_block(next_code, changed);
}

static void handle_new_pulse(const struct cl_pulse *pl)
{
	last_pulse_ms = k_uptime_get();

	if (block_done) {
		send_pulse(pl, CL_PULSE_EXTRA);
		return;
	}
	if (pl->amp_code != block_amp_code) {
		send_pulse(pl, CL_PULSE_DISCARD_AMP);
		return;
	}

	block_pulses++;
	if (block_pulses > (uint32_t)cfg_wire.discard + 2U * cfg_wire.n_reps + 2U) {
		send_pulse(pl, CL_PULSE_GAP);
		request_stop(CL_EVT_FAULT_NO_EPOCHS);
		return;
	}
	if (settle_left > 0U) {
		settle_left--;
		send_pulse(pl, CL_PULSE_DISCARD_SETTLE);
		return;
	}
	if (n_pending >= CL_MAX_PENDING) {
		send_pulse(pl, CL_PULSE_GAP);
		return;
	}
	pending[n_pending++] = *pl;
}

static void process_pending(void)
{
	const uint16_t pre = meas_cfg.pre_samples;
	const uint16_t len = meas_cfg.pre_samples + meas_cfg.post_samples;
	uint8_t i = 0U;

	while (i < n_pending) {
		const struct cl_pulse pl = pending[i];
		const uint32_t a = pl.onset - pre;
		const uint32_t b = a + len;

		if ((int32_t)(ring_end - b) < 0) {
			i++;
			continue;
		}

		n_pending--;
		memmove(&pending[i], &pending[i + 1U], (n_pending - i) * sizeof(pending[0]));

		if (block_done) {
			send_pulse(&pl, CL_PULSE_EXTRA);
			continue;
		}
		if (pl.amp_code != block_amp_code) {
			send_pulse(&pl, CL_PULSE_DISCARD_AMP);
			continue;
		}
		if ((int32_t)(a - ring_valid_from) < 0 || (ring_end - a) > CL_RING_SAMPLES) {
			send_pulse(&pl, CL_PULSE_GAP);
			continue;
		}

		for (uint16_t k = 0; k < len; k++) {
			epoch_buf[k] = ring[(a + k) & (CL_RING_SAMPLES - 1u)];
		}

		const enum cl_epoch_status st = cl_meas_add_epoch(&meas, epoch_buf);

		send_pulse(&pl, st == CL_EPOCH_USED ? CL_PULSE_USED : CL_PULSE_CLIPPED);
		if (cl_meas_complete(&meas)) {
			finish_block();
		}
	}
}

static void stop_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&cl_lock, K_FOREVER);
	(void)do_disarm((enum cl_event)atomic_get(&stop_reason));
	k_mutex_unlock(&cl_lock);
	adc_session_refresh();
}

static void start_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_mutex_lock(&cl_lock, K_FOREVER);
	if (get_state() != CL_STATE_IDLE && !stim_started) {
		atomic_set(&pulses_enabled, 1);
		chronos_stimulation_start(&pending_settings);
		stim_started = true;
		last_pulse_ms = k_uptime_get();
		send_state(CL_EVT_STIMULATION_STARTED, pending_settings.DAC_amplitude);
	}
	k_mutex_unlock(&cl_lock);
}

static void watchdog_work_handler(struct k_work *work)
{
	enum cl_event fault = CL_EVT_NONE;
	int64_t now;

	ARG_UNUSED(work);

	k_mutex_lock(&cl_lock, K_FOREVER);
	if (get_state() == CL_STATE_IDLE) {
		k_mutex_unlock(&cl_lock);
		return;
	}

	now = k_uptime_get();
	if (now - last_buf_ms > CL_NO_ADC_MS) {
		fault = CL_EVT_FAULT_NO_ADC;
	} else if (stim_started && now - last_pulse_ms > (int64_t)period_ms + CL_NO_PULSE_MARGIN_MS) {
		fault = CL_EVT_FAULT_NO_PULSES;
	}

	if (fault != CL_EVT_NONE) {
		(void)do_disarm(fault);
		k_mutex_unlock(&cl_lock);
		adc_session_refresh();
		return;
	}

	(void)k_work_reschedule(&watchdog_work, K_MSEC(CL_WATCHDOG_MS));
	k_mutex_unlock(&cl_lock);
}

/* cl_lock held. Returns CL_EVT_NONE on success, otherwise the rejection reason. */
static enum cl_event begin_session(enum cl_state target)
{
	const stim_setting *s = &pending_settings;
	uint32_t epoch_us;
	uint16_t mag;

	if (get_state() != CL_STATE_IDLE) {
		return CL_EVT_REJECT_BUSY;
	}
	if (!cfg_valid) {
		return CL_EVT_REJECT_NO_CONFIG;
	}
	if (timer_stimulation_is_enabled()) {
		return CL_EVT_REJECT_STIM_RUNNING;
	}
	if (s->DAC_amplitude < CL_DAC_ZERO_MAG_CODE || s->pulse_width == 0U) {
		return CL_EVT_REJECT_BAD_AMPLITUDE;
	}
	mag = (uint16_t)(s->DAC_amplitude - CL_DAC_ZERO_MAG_CODE);
	if (mag < cfg_wire.amp_min || mag > cfg_wire.amp_max) {
		return CL_EVT_REJECT_BAD_AMPLITUDE;
	}
	epoch_us = ((uint32_t)meas_cfg.pre_samples + meas_cfg.post_samples) *
		   SAADC_SAMPLE_INTERVAL_US;
	if (s->frequency == 0U || epoch_us >= 1000000u / s->frequency) {
		return CL_EVT_REJECT_EPOCH_TOO_LONG;
	}

	cl_meas_init(&meas, &meas_cfg);
	cl_ctrl_init(&ctrl, &ctrl_cfg, mag);
	block_no = 0U;
	block_done = false;
	stim_started = false;
	n_pending = 0U;
	ring_started = false;
	period_ms = (1000u + s->frequency - 1u) / s->frequency;
	k_msgq_purge(&cl_pulse_msgq);

	isr_amp_code = s->DAC_amplitude;
	isr_pulse_no = 0U;
	atomic_set(&pending_amp, 0);
	start_block(s->DAC_amplitude, true);

	last_buf_ms = k_uptime_get();
	last_pulse_ms = last_buf_ms;
	atomic_set(&state_val, (atomic_val_t)target);
	(void)k_work_reschedule(&start_work, K_MSEC(CL_ADC_SETTLE_MS));
	(void)k_work_reschedule(&watchdog_work, K_MSEC(CL_WATCHDOG_MS));
	return CL_EVT_NONE;
}

static void start_session(enum cl_state target, enum cl_event ok_event)
{
	enum cl_event rejected;

	k_mutex_lock(&cl_lock, K_FOREVER);
	rejected = begin_session(target);
	send_state(rejected == CL_EVT_NONE ? ok_event : rejected, pending_settings.DAC_amplitude);
	if (rejected == CL_EVT_NONE) {
		LOG_INF("Closed-loop %s at 0x%04x", target == CL_STATE_ARMED ? "armed" : "block",
			pending_settings.DAC_amplitude);
	} else {
		LOG_WRN("Closed-loop start rejected, event %d", (int)rejected);
	}
	k_mutex_unlock(&cl_lock);
}

void closed_loop_init(void)
{
	k_work_queue_init(&cl_tx_q);
	k_work_queue_start(&cl_tx_q, cl_tx_stack, K_THREAD_STACK_SIZEOF(cl_tx_stack),
			   CL_TX_PRIORITY, NULL);
	k_work_init(&cl_tx_work, tx_work_handler);
	k_work_init(&stop_work, stop_work_handler);
	k_work_init_delayable(&start_work, start_work_handler);
	k_work_init_delayable(&watchdog_work, watchdog_work_handler);

	atomic_set(&state_val, CL_STATE_IDLE);
	atomic_set(&pulses_enabled, 0);
	atomic_set(&pending_amp, 0);
	cfg_valid = false;
}

bool closed_loop_is_active(void)
{
	return get_state() != CL_STATE_IDLE;
}

bool closed_loop_blocks_manual_stim(void)
{
	return closed_loop_is_active();
}

void closed_loop_configure(const uint8_t *payload, size_t len)
{
	struct cl_config_wire w;

	k_mutex_lock(&cl_lock, K_FOREVER);

	if (get_state() != CL_STATE_IDLE) {
		send_state(CL_EVT_REJECT_BUSY, isr_amp_code);
		k_mutex_unlock(&cl_lock);
		return;
	}

	if (len != sizeof(w)) {
		LOG_WRN("CL config length %u (expected %u)", (unsigned int)len,
			(unsigned int)sizeof(w));
		cfg_valid = false;
		memset(&cfg_wire, 0, sizeof(cfg_wire));
		send_config(false);
		k_mutex_unlock(&cl_lock);
		return;
	}

	memcpy(&w, payload, sizeof(w));
	w.amp_max = MIN(w.amp_max, (uint16_t)CL_HARD_MAX_AMP_MAG);
	w.step_init = MIN(w.step_init, (uint16_t)CL_HARD_MAX_STEP_MAG);

	meas_cfg = (struct cl_meas_cfg){
		.pre_samples = w.pre_samples,
		.post_samples = w.post_samples,
		.baseline_samples = w.baseline_samples,
		.win_start = w.win_start,
		.win_end = w.win_end,
		.n_reps = w.n_reps,
		.clip_lo = w.clip_lo,
		.clip_hi = w.clip_hi,
	};
	ctrl_cfg = (struct cl_ctrl_cfg){
		.target_auc = w.target_auc,
		.tol_frac = (float)w.tol_permille / 1000.0f,
		.rearm_tol_frac = (float)w.rearm_tol_permille / 1000.0f,
		.amp_min = w.amp_min,
		.amp_max = w.amp_max,
		.step_init = w.step_init,
		.step_min = w.step_min,
		.in_band_required = w.in_band_required,
		.limit_blocks_required = w.limit_blocks_required,
	};

	cfg_wire = w;
	cfg_valid = isfinite(w.target_auc) && cl_meas_cfg_valid(&meas_cfg) &&
		    cl_ctrl_cfg_valid(&ctrl_cfg);
	send_config(cfg_valid);
	LOG_INF("CL config %s", cfg_valid ? "accepted" : "rejected");
	k_mutex_unlock(&cl_lock);
}

void closed_loop_run_block(void)
{
	start_session(CL_STATE_BLOCK, CL_EVT_BLOCK_STARTED);
}

void closed_loop_arm(void)
{
	start_session(CL_STATE_ARMED, CL_EVT_ARMED);
}

void closed_loop_disarm(enum cl_event reason)
{
	k_mutex_lock(&cl_lock, K_FOREVER);
	if (!do_disarm(reason) && reason != CL_EVT_DISCONNECTED) {
		send_state(reason, isr_amp_code);
	}
	k_mutex_unlock(&cl_lock);
}

void closed_loop_on_saadc_buffer(uint32_t seq, const int16_t *samples, size_t count)
{
	struct cl_pulse pl;
	uint32_t base;

	if (get_state() == CL_STATE_IDLE || count == 0U) {
		return;
	}

	k_mutex_lock(&cl_lock, K_FOREVER);
	if (get_state() == CL_STATE_IDLE) {
		k_mutex_unlock(&cl_lock);
		return;
	}

	base = seq * (uint32_t)count;
	if (!ring_started || base != ring_end) {
		ring_valid_from = base;
		ring_started = true;
	}
	for (size_t i = 0; i < count; i++) {
		ring[(base + i) & (CL_RING_SAMPLES - 1u)] = samples[i];
	}
	ring_end = base + (uint32_t)count;
	last_buf_ms = k_uptime_get();

	while (k_msgq_get(&cl_pulse_msgq, &pl, K_NO_WAIT) == 0) {
		handle_new_pulse(&pl);
	}
	process_pending();
	k_mutex_unlock(&cl_lock);
}

void closed_loop_on_stim_pulse(void)
{
	struct cl_pulse pl;
	atomic_val_t next;

	if (!atomic_get(&pulses_enabled)) {
		return;
	}

	next = atomic_clear(&pending_amp);
	if (next != 0) {
		isr_amp_code = (uint16_t)(next & 0xFFFF);
		stim_set_amplitude_code_quiet(isr_amp_code);
	}

	if (!adc_sample_count_get(&pl.onset)) {
		return;
	}
	pl.pulse_no = isr_pulse_no++;
	pl.amp_code = isr_amp_code;
	(void)k_msgq_put(&cl_pulse_msgq, &pl, K_NO_WAIT);
}
