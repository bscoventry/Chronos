cmake_minimum_required(VERSION 3.20.0)

list(APPEND CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf)

if(CHRONOS_RELEASE)
	list(APPEND CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/prj_release.conf)
endif()

if(CHRONOS_NO_BLE)
	list(APPEND CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/prj_no_ble.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Chronos_nRF54_2)

target_sources(app PRIVATE
  src/main.c
  src/BLE.c
  src/chronos_hfclk.c
  src/data.c
  src/rtc_stim.c
  src/spi.c
  src/stim_nvm.c
  src/timer.c
)

if(CONFIG_BT)
  target_sources(app PRIVATE
    src/ble_session.c
    src/chronos_hw.c
    src/chronos_boot.c
    src/chronos_reed.c
    src/ADC.c
    src/closed_loop.c
    src/cl_algo.c
    src/signal_mv.c
  )
endif()

if(CONFIG_CHRONOS_SYSTEM_OFF)
  target_sources(app PRIVATE src/power_off.c)
endif()

if(CONFIG_CHRONOS_ZMEAS)
  target_sources(app PRIVATE
    src/zmeas_gpio.c
    src/ad5933.c
  )
endif()
