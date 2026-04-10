idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES xmos_jtag esp_http_server esp_wifi nvs_flash esp_netif esp_event
    EMBED_FILES "index.html"
)
