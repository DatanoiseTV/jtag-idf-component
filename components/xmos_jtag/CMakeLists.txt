set(srcs
    "src/xmos_jtag.c"
    "src/xmos_xe.c"
    "src/jtag_gpio.c"
    "src/svf_player.c"
    "src/ice40.c"
    "src/swd.c"
)

set(requires
    "driver"
    "esp_driver_gpio"
    "esp_timer"
    "esp_rom"
)

set(priv_requires "")

if(CONFIG_XMOS_JTAG_BACKEND_PARLIO)
    list(APPEND srcs "src/jtag_parlio.c")
endif()

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
