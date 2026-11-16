#include "ADC.h"
#include "BLE.h"
#include "closed_loop.h"

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>

LOG_MODULE_REGISTER(chronos_adc, LOG_LEVEL_DBG);

#define SAADC_BUF_POOL_DEPTH 8

struct adc_block {
	int16_t *samples;
	uint16_t seq;
};

static nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);
static nrfx_timer_t sample_cnt_timer = NRFX_TIMER_INSTANCE(ADC_SAMPLE_CNT_TIMER);
static nrfx_saadc_channel_t channel1 =
	NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN1, 0);

static int16_t saadc_sample_buffer[SAADC_BUF_POOL_DEPTH][SAADC_BUFFER_SIZE];
K_MSGQ_DEFINE(adc_sample_msgq, sizeof(struct adc_block), SAADC_BUF_POOL_DEPTH, 4);

static uint16_t saadc_block_seq;
static uint32_t saadc_next_buffer_idx;
static bool adc_hw_running;
static bool recording_stream;
static bool gppi_linked;
static bool sample_cnt_ready;

static nrfx_gppi_handle_t gppi_handle_sample;
static nrfx_gppi_handle_t gppi_handle_start;
static nrfx_gppi_handle_t gppi_handle_count;

static void configure_timer(void);
static void configure_sample_counter(void);
static void configure_saadc(void);
static void configure_ppi(void);
static void adc_hw_start(void);
static void adc_hw_stop(void);

static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	nrfx_err_t err;

	switch (p_event->type) {
	case NRFX_SAADC_EVT_READY:
		nrfx_timer_enable(&timer_instance);
		break;

	case NRFX_SAADC_EVT_BUF_REQ: {
		uint32_t next = saadc_next_buffer_idx % SAADC_BUF_POOL_DEPTH;

		err = nrfx_saadc_buffer_set(saadc_sample_buffer[next], SAADC_BUFFER_SIZE);
		if (err != 0) {
			LOG_ERR("nrfx_saadc_buffer_set err=%08x", err);
			return;
		}
		saadc_next_buffer_idx++;
		break;
	}

	case NRFX_SAADC_EVT_DONE: {
		struct adc_block blk = {
			.samples = p_event->data.done.p_buffer,
			.seq = saadc_block_seq++,
		};
		int ret = k_msgq_put(&adc_sample_msgq, &blk, K_NO_WAIT);

		if (ret != 0) {
			LOG_WRN("ADC sample queue full");
		}
		break;
	}

	default:
		break;
	}
}

static void adc_consumer_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct adc_block blk;

	k_sem_take(&ble_init_ok, K_FOREVER);

	while (1) {
		k_msgq_get(&adc_sample_msgq, &blk, K_FOREVER);
		closed_loop_on_saadc_buffer(blk.seq, blk.samples, SAADC_BUFFER_SIZE);
		if (recording_stream || closed_loop_is_active()) {
			(void)ble_send_samples(blk.seq, blk.samples, SAADC_BUFFER_SIZE);
		}
	}
}

K_THREAD_DEFINE(adc_consumer_tid, 2048, adc_consumer_thread, NULL, NULL, NULL, 5, 0, 0);

static void configure_timer(void)
{
	nrfx_err_t err;
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);

	err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
	if (err != 0) {
		LOG_ERR("nrfx_timer_init err=%08x", err);
		return;
	}

	{
		uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance,
							      SAADC_SAMPLE_INTERVAL_US);

		nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks,
					    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	}
}

static void configure_sample_counter(void)
{
	nrfx_err_t err;
	nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);

	cfg.mode = NRF_TIMER_MODE_COUNTER;
	err = nrfx_timer_init(&sample_cnt_timer, &cfg, NULL);
	if (err != 0) {
		LOG_ERR("sample counter init err=%08x", err);
		sample_cnt_ready = false;
		return;
	}
	nrfx_timer_clear(&sample_cnt_timer);
	nrfx_timer_enable(&sample_cnt_timer);
	sample_cnt_ready = true;
}

static void configure_saadc(void)
{
	nrfx_err_t err;

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)), DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
	if (err != 0) {
		LOG_ERR("nrfx_saadc_init err=%08x", err);
		return;
	}

	channel1.channel_config.gain = NRF_SAADC_GAIN1_4;
	channel1.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
	err = nrfx_saadc_channels_config(&channel1, 1);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_channels_config err=%08x", err);
		return;
	}

	{
		nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

		err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT,
						   &saadc_adv_config, saadc_event_handler);
		if (err != 0) {
			LOG_ERR("nrfx_saadc_advanced_mode_set err=%08x", err);
			return;
		}
	}

	err = nrfx_saadc_buffer_set(saadc_sample_buffer[0], SAADC_BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_buffer_set err=%08x", err);
		return;
	}
	err = nrfx_saadc_buffer_set(saadc_sample_buffer[1], SAADC_BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("nrfx_saadc_buffer_set err=%08x", err);
		return;
	}

	saadc_next_buffer_idx = 2U;

	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("nrfx_saadc_mode_trigger err=%08x", err);
	}
}

static void configure_ppi(void)
{
	nrfx_err_t err;

	if (gppi_linked) {
		return;
	}

	err = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
		&gppi_handle_sample);
	if (err != 0) {
		LOG_ERR("gppi sample alloc err=%08x", err);
		return;
	}

	err = nrfx_gppi_conn_alloc(
		nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START),
		&gppi_handle_start);
	if (err != 0) {
		LOG_ERR("gppi start alloc err=%08x", err);
		return;
	}

	if (sample_cnt_ready) {
		err = nrfx_gppi_conn_alloc(
			nrfx_timer_compare_event_address_get(&timer_instance,
							     NRF_TIMER_CC_CHANNEL0),
			nrfx_timer_task_address_get(&sample_cnt_timer, NRF_TIMER_TASK_COUNT),
			&gppi_handle_count);
		if (err != 0) {
			LOG_ERR("gppi count alloc err=%08x", err);
		}
	}

	nrfx_gppi_conn_enable(gppi_handle_sample);
	nrfx_gppi_conn_enable(gppi_handle_start);
	if (sample_cnt_ready) {
		nrfx_gppi_conn_enable(gppi_handle_count);
	}
	gppi_linked = true;
}

static void adc_hw_start(void)
{
	if (adc_hw_running) {
		return;
	}

	k_msgq_purge(&adc_sample_msgq);
	saadc_block_seq = 0U;
	configure_timer();
	configure_sample_counter();
	configure_saadc();
	configure_ppi();
	adc_hw_running = true;
	LOG_INF("SAADC session started");
}

static void adc_hw_stop(void)
{
	if (!adc_hw_running) {
		return;
	}

	if (gppi_linked && sample_cnt_ready) {
		nrfx_gppi_conn_disable(gppi_handle_count);
	}
	nrfx_timer_disable(&timer_instance);
	nrfx_timer_uninit(&timer_instance);
	if (sample_cnt_ready) {
		nrfx_timer_disable(&sample_cnt_timer);
		nrfx_timer_uninit(&sample_cnt_timer);
		sample_cnt_ready = false;
	}
	nrfx_saadc_uninit();
	saadc_next_buffer_idx = 2U;
	gppi_linked = false;
	adc_hw_running = false;
	LOG_INF("SAADC session stopped");
}

static bool adc_session_needed(void)
{
	return recording_stream || closed_loop_is_active();
}

void adc_init_module(void)
{
	recording_stream = false;
	adc_hw_running = false;
	sample_cnt_ready = false;
}

void adc_set_recording_stream(bool enable)
{
	recording_stream = enable;
}

bool adc_recording_stream_enabled(void)
{
	return recording_stream;
}

void adc_session_refresh(void)
{
	if (adc_session_needed()) {
		if (!adc_hw_running) {
			adc_hw_start();
		}
	} else {
		adc_hw_stop();
	}
}

bool adc_recording_is_active(void)
{
	return adc_hw_running;
}

bool adc_sample_count_get(uint32_t *out)
{
	if (!adc_hw_running || !sample_cnt_ready || out == NULL) {
		return false;
	}

	*out = nrfx_timer_capture(&sample_cnt_timer, NRF_TIMER_CC_CHANNEL0);
	return true;
}
