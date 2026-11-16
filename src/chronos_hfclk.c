/*
 * HFXO ownership for the stimulation path. See chronos_hfclk.h for why the crystal
 * is held per session rather than per burst, and why requests go through the clock
 * control onoff manager instead of the CLOCK task registers.
 */

#include "chronos_hfclk.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>

#include <nrfx_clock.h>
#include <hal/nrf_clock.h>

/* Startup is 1650 us; allow margin for XOTUNE after a large temperature change. */
#define HFCLK_ACQUIRE_TIMEOUT K_MSEC(20)

uint32_t chronos_hfclk_not_ready_count;

static struct onoff_client hf_cli;
static bool hf_held;
static K_SEM_DEFINE(hf_started_sem, 0, 1);

static void hf_started(struct onoff_manager *mgr, struct onoff_client *cli,
		       uint32_t state, int res)
{
	ARG_UNUSED(mgr);
	ARG_UNUSED(cli);
	ARG_UNUSED(state);
	ARG_UNUSED(res);

	k_sem_give(&hf_started_sem);
}

int chronos_hfclk_acquire(void)
{
	struct onoff_manager *mgr;
	int err;

	if (hf_held) {
		return 0;
	}

	mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (mgr == NULL) {
		return -ENODEV;
	}

	k_sem_reset(&hf_started_sem);
	sys_notify_init_callback(&hf_cli.notify, hf_started);

	err = onoff_request(mgr, &hf_cli);
	if (err < 0) {
		return err;
	}

	/* Hold the reference even on timeout: the request is outstanding, and
	 * cancelling it would let MPSL stop the crystal mid-session.
	 */
	hf_held = true;

	if (k_sem_take(&hf_started_sem, HFCLK_ACQUIRE_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

void chronos_hfclk_release(void)
{
	struct onoff_manager *mgr;

	if (!hf_held) {
		return;
	}

	mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (mgr == NULL) {
		return;
	}

	hf_held = false;
	(void)onoff_cancel_or_release(mgr, &hf_cli);
}

bool chronos_hfclk_is_ready(void)
{
	nrf_clock_hfclk_t src;

	if (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &src)) {
		return false;
	}

	return src == NRF_CLOCK_HFCLK_HIGH_ACCURACY;
}

bool chronos_hfclk_check_ready(void)
{
	if (chronos_hfclk_is_ready()) {
		return true;
	}

	chronos_hfclk_not_ready_count++;

	return false;
}
