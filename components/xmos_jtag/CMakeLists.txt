set(srcs
    "src/xmos_jtag.c"
    "src/xmos_xe.c"
    "src/jtag_gpio.c"
)

set(requires
    "driver"
    "esp_driver_gpio"
    "esp_timer"
    "esp_rom"
)

set(priv_requires "")

# PARLIO backend for ESP32-P4 (and any future chips with PARLIO)
if(CONFIG_XMOS_JTAG_BACKEND_PARLIO)
    list(APPEND srcs "src/jtag_parlio.c")
endif()

# Always link esp_driver_parlio on P4 so the dependency checker is satisfied
if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND priv_requires "esp_driver_parlio")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES ${requires}
    PRIV_REQUIRES ${priv_requires}
)
