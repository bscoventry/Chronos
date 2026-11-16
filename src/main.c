/*
Author: Brandon S Coventry      Wisconsin Institute for Translational Neuroengineering
Date: 2026/02/12 Initialization
Purpose: Chronos firmware ported for the nRF54L15
Revision History: See Github history
Notes:
 - Includes the ability to turn bluetooth transmission on and off in config file. Set CONFIG_BT in config file to 0 to turn off BLE engine
 - Primary stimulation driver is still and interupt based routine. 
*/

/*Basic includes. Minimize to limit current draw from unused engines.*/
//Zephyr includes
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/irq.h> //Interrupt service routine handlers
#include <zephyr/device.h>  //Must import devicetree files
#include <zephyr/devicetree.h>
#include <soc.h>    //Standard System on chip imports
#include <zephyr/logging/log.h>   //Disable on compile
#include <inttypes.h>

#if defined(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#endif

//nRFX imports. Note we are primarily runnig
#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <zephyr/sys/atomic.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_clock.h>

#if defined(CONFIG_SPI)
#include <zephyr/drivers/spi.h>
#endif

//Chronos engine imports 
#include "BLE.h"  //BLE engine
#include "ble_session.h"
#include "power_off.h"
#include "chronos_hw.h"
#include "chronos_boot.h"
#include "chronos_reed.h"
#include "spi.h" //SPI to howland current source
#include "timer.h"  //Handles interrupt timing of stimulation
#include "rtc_stim.h"   //handles IRQ routines for stimulation
#include "data.h"
#include "config.h"   //Set compilation settings. Look here for CONFIG_BT
#if defined(CONFIG_BT) && !defined(CONFIG_SPI)
#if defined(CONFIG_CHRONOS_STIM_NVM)
#include "stim_nvm.h"
#endif
#endif

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

//Add UART Driver only if BLE is active. Note, UART is *virtual*, but still requires pin definitions. 
#if defined(CONFIG_BT)
#include <uart_async_adapter.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#endif

//Set up peripherial initialization
#if defined(CONFIG_BT)
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};
#endif /* CONFIG_BT */

static void init_pins(void) {
    /* SPI CS and analog-switch GPIO (nrfx SPIM21 + timer.c). */
#if !defined(CONFIG_SPI)
    nrf_gpio_cfg_output(DAC_CS_PIN);
    nrf_gpio_pin_set(DAC_CS_PIN);   /* high (inactive) */
    
#endif

    /*
     * All three analog switches open, matching timer_stim_switches_park(). Levels are driven
     * low before the pins become outputs so there is no glitch on the stimulus path out of
     * reset.
     *
     * Drive is set explicitly rather than via nrf_gpio_cfg_output(): standard (S0S1) slows
     * the edge into the analog switches and reduces charge injection on the electrode path.
     * P1.02 and P1.03 are the NFC pads, whose CPAD_NFC is 5 pF against 1 pF for P1.16, so
     * they inject the most and gain the most from the slower edge. See STIM_SWITCH_HIGH_DRIVE.
     */
    nrf_gpio_port_out_clear(STIM_GPIO_PORT,
                            BIT(SWITCH_PIN_FIRST) | BIT(SWITCH_PIN_INTER) |
                                    BIT(SWITCH_PIN_SECOND));
    {
#if (STIM_SWITCH_HIGH_DRIVE == 1)
	    const nrf_gpio_pin_drive_t drive = NRF_GPIO_PIN_H0H1;
#else
	    const nrf_gpio_pin_drive_t drive = NRF_GPIO_PIN_S0S1;
#endif
	    static const uint32_t switch_pins[] = {
		    NRF_GPIO_PIN_MAP(1, SWITCH_PIN_FIRST),
		    NRF_GPIO_PIN_MAP(1, SWITCH_PIN_INTER),
		    NRF_GPIO_PIN_MAP(1, SWITCH_PIN_SECOND),
	    };
        //nrf_gpio_pin_set(SWITCH_PIN_FIRST);    //*/Brandon remove this before production */
        //nrf_gpio_pin_set(SWITCH_PIN_SECOND);
	    for (size_t i = 0; i < ARRAY_SIZE(switch_pins); i++) {
		    nrf_gpio_cfg(switch_pins[i],
				 NRF_GPIO_PIN_DIR_OUTPUT,
				 NRF_GPIO_PIN_INPUT_DISCONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 drive,
				 NRF_GPIO_PIN_NOSENSE);
	    }
    }

    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(STIM_SESSION_GPIO_PORT, STIM_SESSION_GPIO_NUM));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(STIM_SESSION_GPIO_PORT, STIM_SESSION_GPIO_NUM));
}

int main(void)
{
    init_pins();

#if defined(CONFIG_BT)
#if defined(CONFIG_CHRONOS_SYSTEM_OFF) && defined(CONFIG_CHRONOS_MINIMAL_BOOT)
    /*
     * Cold boot: only safe GPIO + reed, then System OFF before SPI/timer/BLE
     * session setup. Reed GPIO wake runs full init below.
     */
    (void)chronos_reed_gpio_init();

    if (!chronos_boot_was_reed_wake()) {
        chronos_power_off_try_enter();
    }
#endif

    chronos_app_init_full();

#if MEASURE_TIMER == 1
    /* measurement_timer_init() is inside chronos_app_init_full when enabled */
#endif
    int err = 0;
    uint32_t experiment_counter = 0;

    if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err) {
            LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
            return 0;
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err) {
            LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
            return 0;
        }
    }

    ble_session_init();
    LOG_INF("Boot reset cause 0x%08x (RESET_* flags)", chronos_boot_reset_cause());
    LOG_INF("Reed P1.15: swipe to enable BLE; stimulation off until START byte");

#if defined(CONFIG_CHRONOS_SYSTEM_OFF)
    if (!ble_session_is_active()) {
        chronos_power_off_try_enter();
    }
#endif

    for (;;) {
        if (MEASURE_TIMER == 1) {
            k_msleep(10000);
            experiment_counter += 10;
            error_data my_error_data;
            get_error_data(&my_error_data);
            printf("Counter: %i Elapsed: %is\nEvent0 running error: %" PRIu32
                   " avg error: %" PRIu32 " max error: %" PRIu32 "\n"
                   "Events1-3 running error: %" PRIu32 " avg error: %" PRIu32
                   " max error: %" PRIu32 ", %" PRIu32 ", %" PRIu32 "\n",
                   my_error_data.mycounter,
                   experiment_counter,
                   my_error_data.event0_error,
                   (my_error_data.event0_error / my_error_data.mycounter),
                   my_error_data.event0_max,
                   my_error_data.myerror,
                   (my_error_data.myerror / my_error_data.mycounter),
                   my_error_data.event1_max,
                   my_error_data.event2_max,
                   my_error_data.event3_max);
        } else {
            k_sleep(K_FOREVER);
        }
    }
#else
    spi_init();
    timer_burst_init();
    update_pulse_width(CONFIG_PULSE_WIDTH_US);
    update_dac1_amplitude(CONFIG_STIM_AMPLITUDE);
    update_dac2_amplitude(CONFIG_STIM_AMPLITUDE);
    data_init_defaults();

    rtc_stim_start_lfclk();
    rtc_stim_init(CONFIG_STIM_FREQUENCY_HZ);
    LOG_INF("RTC-driven stimulation at %u Hz (no BLE)", CONFIG_STIM_FREQUENCY_HZ);
    for (;;) {
        k_sleep(K_FOREVER);
    }
#endif /* CONFIG_BT */
}

#if defined(CONFIG_BT)
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
        NULL, PRIORITY, 0, 0);
#endif
