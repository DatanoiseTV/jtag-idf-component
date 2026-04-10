set(srcs
    "src/xmos_jtag.c"
    "src/xmos_xe.c"
    "src/jtag_gpio.c"
)

# PARLIO backend only available on chips that support it (ESP32-P4)
if(CONFIG_XMOS_JTAG_BACKEND_PARLIO)
    list(APPEND srcs "src/jtag_parlio.c")
endif()

set(requires
    "driver"
    "esp_driver_gpio"
    "esp_timer"
    "esp_rom"
)

if(CONFIG_XMOS_JTAG_BACKEND_PARLIO)
    list(APPEND requires "esp_driver_parlio")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES ${requires}
)
