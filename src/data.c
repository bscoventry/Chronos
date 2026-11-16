#include <zephyr/types.h>
#include <string.h>
#include <stdio.h>

#include "data.h"
#include "config.h"
#include "spi.h"
#include "timer.h"
#if defined(CONFIG_BT)
#include "ADC.h"
#include "closed_loop.h"
#endif
#if defined(CONFIG_BT) && !defined(CONFIG_SPI) && defined(CONFIG_CHRONOS_STIM_NVM)
#include "stim_nvm.h"
#endif

stim_setting settings;
stim_setting pending_settings;
uint8_t ble_received_data[BLE_DATA_BUFFER_SIZE];
uint16_t ble_data_length;

void data_init_defaults(void)
{
	settings.DAC_amplitude = CONFIG_STIM_AMPLITUDE;
	settings.pulse_width = (uint16_t)CONFIG_PULSE_WIDTH_US;
	settings.frequency = (uint16_t)CONFIG_STIM_FREQUENCY_HZ;
	pending_settings = settings;
	update_dac1_amplitude(settings.DAC_amplitude);
	update_dac2_amplitude(settings.DAC_amplitude);
}

#if defined(CONFIG_BT)

void chronos_stimulation_start(const stim_setting *s)
{
	if (s->frequency == 0u || s->pulse_width == 0u) {
		printf("Stim start ignored: invalid frequency or pulse width\n");
		return;
	}

#if defined(CONFIG_CHRONOS_STIM_NVM)
	(void)stim_nvm_save(true, s);
#endif
	update_stim_frequency(s->frequency);
	update_pulse_width(s->pulse_width);
	update_dac1_amplitude(s->DAC_amplitude);
	update_dac2_amplitude(s->DAC_amplitude);
	timer_stimulation_enable();
}

void chronos_stimulation_update_params(const stim_setting *s)
{
#if defined(CONFIG_CHRONOS_STIM_NVM)
	(void)stim_nvm_save(true, s);
#endif
	update_stim_frequency(s->frequency);
	update_pulse_width(s->pulse_width);
	update_dac1_amplitude(s->DAC_amplitude);
	update_dac2_amplitude(s->DAC_amplitude);
}

void chronos_stimulation_request_stop(void)
{
	if (timer_stimulation_is_enabled()) {
#if defined(CONFIG_CHRONOS_STIM_NVM)
		(void)stim_nvm_save(false, &settings);
#endif
		stim_timer_request_stop_after_burst();
	}
}

static void process_stim_packet(stim_setting *active_settings, const uint8_t *data, uint16_t len)
{
	const uint8_t ctrl = data[0];
	stim_setting payload;
	const bool running = timer_stimulation_is_enabled();

	if (len != STIM_PACKET_LEN) {
		printf("BLE stim packet length %u (expected %u)\n", len, (unsigned int)STIM_PACKET_LEN);
		return;
	}

	memcpy(&payload, data + 1, sizeof(payload));

	if (ctrl == STIM_CTRL_START) {
		if (closed_loop_blocks_manual_stim()) {
			memcpy(&pending_settings, &payload, sizeof(stim_setting));
			chronos_stimulation_update_params(&payload);
			printf("Manual START ignored (closed-loop armed); params updated\n");
			return;
		}

		memcpy(active_settings, &payload, sizeof(stim_setting));
		memcpy(&pending_settings, &payload, sizeof(stim_setting));
		if (running) {
			chronos_stimulation_update_params(&payload);
			return;
		}
		chronos_stimulation_start(active_settings);
		return;
	}

	if (ctrl == STIM_CTRL_STOP) {
		if (running) {
			chronos_stimulation_request_stop();
		} else {
#if defined(CONFIG_CHRONOS_STIM_NVM)
			(void)stim_nvm_save(false, &payload);
#endif
			memcpy(&pending_settings, &payload, sizeof(stim_setting));
			update_dac1_amplitude(pending_settings.DAC_amplitude);
			update_dac2_amplitude(pending_settings.DAC_amplitude);
		}
		return;
	}

	printf("BLE unknown stim control byte 0x%02x\n", ctrl);
}

void process_received_data(stim_setting *active_settings, uint8_t *data, uint16_t len)
{
	if (len == 0U) {
		return;
	}

	switch (data[0]) {
	case STIM_CTRL_STOP:
	case STIM_CTRL_START:
		process_stim_packet(active_settings, data, len);
		return;

	case ADC_CTRL_START:
		adc_set_recording_stream(true);
		adc_session_refresh();
		printf("ADC recording stream enabled\n");
		return;

	case ADC_CTRL_STOP:
		adc_set_recording_stream(false);
		adc_session_refresh();
		printf("ADC recording stream disabled\n");
		return;

	case CL_CTRL_CONFIG:
		if (len < 1U + CL_CONFIG_WIRE_LEN) {
			printf("CL CONFIG length %u (expected %u)\n", len,
			       (unsigned int)(1U + CL_CONFIG_WIRE_LEN));
			return;
		}
		closed_loop_configure(data + 1, CL_CONFIG_WIRE_LEN);
		return;

	case CL_CTRL_RUN_BLOCK:
		closed_loop_run_block();
		adc_session_refresh();
		return;

	case CL_CTRL_ARM:
		closed_loop_arm();
		adc_session_refresh();
		return;

	case CL_CTRL_DISARM:
		closed_loop_disarm(CL_EVT_DISARMED);
		adc_session_refresh();
		return;

	default:
		printf("BLE unknown control byte 0x%02x len=%u\n", data[0], len);
		return;
	}
}

#else /* !CONFIG_BT */

void process_received_data(stim_setting *active_settings, uint8_t *data, uint16_t len)
{
	ARG_UNUSED(active_settings);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
}

#endif /* CONFIG_BT */
