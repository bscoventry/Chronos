#ifndef DATA_H
#define DATA_H
#include <zephyr/types.h>

typedef struct {
	uint16_t DAC_amplitude;
	uint16_t pulse_width;
	uint16_t frequency;
} stim_setting;

/** Byte 0 of NUS payload: explicit run/stop (separate from DAC code). */
#define STIM_CTRL_STOP  0x00u
#define STIM_CTRL_START 0x01u
#define ADC_CTRL_START  0x02u
#define ADC_CTRL_STOP   0x03u
#define CL_CTRL_ARM     0x04u
#define CL_CTRL_DISARM  0x05u
#define CL_CTRL_CONFIG  0x06u
#define CL_CTRL_RUN_BLOCK 0x07u

#define STIM_PACKET_LEN (1u + sizeof(stim_setting))
#define CL_CONFIG_WIRE_LEN 34u

#define BLE_DATA_BUFFER_SIZE 64u

extern stim_setting settings;
extern stim_setting pending_settings;
extern uint8_t ble_received_data[];
extern uint16_t ble_data_length;

void data_init_defaults(void);
void process_received_data(stim_setting *settings, uint8_t *ble_received_data, uint16_t ble_data_length);

#if defined(CONFIG_BT)
void chronos_stimulation_start(const stim_setting *s);
void chronos_stimulation_request_stop(void);
void chronos_stimulation_update_params(const stim_setting *s);
#endif

#endif /* DATA_H */
