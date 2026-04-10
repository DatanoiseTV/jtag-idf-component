set(requires
    xmos_jtag
    esp_http_server
    esp_netif
    esp_event
    nvs_flash
)

if(NOT IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp_wifi)
else()
    list(APPEND requires esp_eth esp_hw_support)
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ${requires}
    EMBED_FILES "index.html"
)
