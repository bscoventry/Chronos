/*
 * Full application peripheral init — deferred on cold boot until after System OFF attempt.
 */

#if defined(CONFIG_BT)

#include "chronos_boot.h"
#include "chronos_hw.h"
#include "data.h"
#include "spi.h"
#include "timer.h"
#include "config.h"
#include "ADC.h"
#include "closed_loop.h"

#if !defined(CONFIG_SPI) && defined(CONFIG_CHRONOS_STIM_NVM)
#include "stim_nvm.h"
#endif

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(chronos_boot, LOG_LEVEL_INF);

void chronos_app_init_full(void)
{
#if defined(CONFIG_SPI)
	k_msleep(10);
#else
	spi_init();
#endif

	timer_init();
	data_init_defaults();
	adc_init_module();
	closed_loop_init();

#if !defined(CONFIG_SPI)
#if defined(CONFIG_CHRONOS_STIM_NVM)
	{
		int nvm_err = stim_nvm_boot_load(&settings, &pending_settings);

		(void)nvm_err;
		update_pulse_width(settings.pulse_width);
		update_stim_frequency(settings.frequency);
		update_dac1_amplitude(settings.DAC_amplitude);
		update_dac2_amplitude(settings.DAC_amplitude);
		if (stim_nvm_wants_boot_resume()) {
			timer_stimulation_enable_from_boot();
		}
	}
#else
	update_pulse_width(settings.pulse_width);
	update_stim_frequency(settings.frequency);
	update_dac1_amplitude(settings.DAC_amplitude);
	update_dac2_amplitude(settings.DAC_amplitude);
#endif
#endif

#if MEASURE_TIMER == 1
	measurement_timer_init();
#endif

#if !defined(CONFIG_CHRONOS_DEFER_SESSION_HW_INIT)
	{
		int err = chronos_session_hw_init();

		if (err != 0) {
			LOG_WRN("Session HW init err %d", err);
		}
	}
#endif
}

#endif /* CONFIG_BT */
