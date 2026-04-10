set(requires
    xmos_jtag
    esp_http_server
    esp_netif
    esp_event
    nvs_flash
)

# ESP32-P4 has no native WiFi; all other targets do
if(NOT IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp_wifi)
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ${requires}
    EMBED_FILES "index.html"
)
