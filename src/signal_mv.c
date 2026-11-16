#include "signal_mv.h"
#include "config.h"

/*
 * nRF SAADC: internal 0.6 V reference, gain 1/4, 12-bit result in int16.
 * Pin voltage (uV) before front-end scaling:
 *   V_pin_uV = counts * (Vref_uV / gain) / 4096
 * Signal uV applies CHRONOS_ADC_SIGNAL_SCALE_* for the analog front-end.
 */
int32_t signal_uv_from_saadc_mean(int32_t mean_counts)
{
	const int64_t pin_uv = (int64_t)mean_counts * 600000LL * 4LL / 4096LL;
	const int64_t signal = pin_uv * (int64_t)CHRONOS_ADC_SIGNAL_SCALE_NUM
			       / (int64_t)CHRONOS_ADC_SIGNAL_SCALE_DEN;

	return (int32_t)signal;
}
