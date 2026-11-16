#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nrfx_saadc.h>

/* 25 kHz: 2.5x the 10 kHz eCAP front-end bandwidth. */
#define SAADC_SAMPLE_INTERVAL_US 40
/*
 * One buffer is one NUS notification: 2-byte sequence + 120 x int16 = 242 bytes, which needs
 * a negotiated ATT MTU of at least 245. At 25 kHz a buffer completes every 4.8 ms, long enough
 * to ride out the ~1 ms stim burst ISR blocking the SAADC buffer refill.
 */
#define SAADC_BUFFER_SIZE        120
#define SAADC_INPUT_PIN1         NRFX_ANALOG_EXTERNAL_AIN0
#define TIMER_INSTANCE_NUMBER    NRF_TIMER22
#define ADC_SAMPLE_CNT_TIMER     NRF_TIMER21

void adc_init_module(void);
void adc_set_recording_stream(bool enable);
bool adc_recording_stream_enabled(void);
void adc_session_refresh(void);
bool adc_recording_is_active(void);

/** Monotonic sample index since the current ADC session started (PPI-counted). ISR-safe read. */
bool adc_sample_count_get(uint32_t *out);

#endif /* ADC_H */
