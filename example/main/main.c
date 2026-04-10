/*
 * XMOS Web Flasher -- Example ESP-IDF application
 *
 * Provides a web UI for:
 *   - Identifying the connected XMOS device (IDCODE, family, tiles)
 *   - Uploading and inspecting .xe / .bin firmware files
 *   - Flashing firmware to XMOS RAM or SPI flash via JTAG
 *   - Monitoring flash progress in real time
 *
 * Connect to the ESP32's WiFi AP and open http://192.168.4.1
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "xmos_jtag.h"
#include "xmos_xe.h"

static const char *TAG = "xmos_web";

/* -------------------------------------------------------------------------
 * Configuration -- adjust pins for your board
 * ---------------------------------------------------------------------- */
#define WIFI_AP_SSID        "XMOS-Flasher"
#define WIFI_AP_PASS        "xmosjtag"
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    2

/*
 * JTAG pin assignment -- adjust for your board.
 *
 * ESP32-S3 (default target for this example):
 *   Any GPIO can be used. Pins below avoid strapping/PSRAM/USB pins.
 *
 * ESP32-P4-NANO (Waveshare) right header rows 7-10:
 *   TCK=47, TMS=48, TDI=46, TDO=45, TRST=53, SRST=54
 */
#define PIN_TCK             GPIO_NUM_12
#define PIN_TMS             GPIO_NUM_13
#define PIN_TDI             GPIO_NUM_14
#define PIN_TDO             GPIO_NUM_11
#define PIN_TRST            GPIO_NUM_NC
#define PIN_SRST            GPIO_NUM_NC

/* -------------------------------------------------------------------------
 * Globals
 * ---------------------------------------------------------------------- */
static xmos_jtag_handle_t s_jtag = NULL;
static xmos_chip_info_t s_chip_info = {0};
static bool s_identified = false;

/* Upload buffer -- holds the firmware file during upload */
#define MAX_FIRMWARE_SIZE   (8 * 1024 * 1024)  /* 8 MB max */
static uint8_t *s_fw_buf = NULL;
static size_t s_fw_len = 0;
static bool s_fw_ready = false;

/* Flash progress */
static volatile int s_flash_progress = -1;   /* -1 = idle, 0-100 = progress */
static volatile int s_flash_result = 0;      /* 0 = ok, <0 = error */
static char s_flash_status[128] = "Idle";

/* -------------------------------------------------------------------------
 * Embedded HTML
 * ---------------------------------------------------------------------- */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* -------------------------------------------------------------------------
 * WiFi AP setup
 * ---------------------------------------------------------------------- */
static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASS,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = WIFI_AP_MAX_CONN,
        },
    };
    if (strlen(WIFI_AP_PASS) == 0)
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s", WIFI_AP_SSID);
}

/* -------------------------------------------------------------------------
 * HTTP handlers
 * ---------------------------------------------------------------------- */

/* GET / -- serve the web UI */
static esp_err_t handler_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

/* GET /api/identify -- probe JTAG and return device info as JSON */
static esp_err_t handler_identify(httpd_req_t *req)
{
    if (!s_jtag) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JTAG not init");
        return ESP_FAIL;
    }

    esp_err_t err = xmos_jtag_identify(s_jtag, &s_chip_info);
    s_identified = (err == ESP_OK && s_chip_info.family != XMOS_FAMILY_UNKNOWN);

    const char *family = "Unknown";
    switch (s_chip_info.family) {
        case XMOS_FAMILY_XS1: family = "XS1"; break;
        case XMOS_FAMILY_XS2: family = "xCORE-200 (XS2)"; break;
        case XMOS_FAMILY_XS3: family = "xCORE.ai (XS3)"; break;
        default: break;
    }

    char json[256];
    snprintf(json, sizeof(json),
        "{\"ok\":%s,\"idcode\":\"0x%08lx\",\"family\":\"%s\","
        "\"tiles\":%d,\"revision\":%d}",
        s_identified ? "true" : "false",
        (unsigned long)s_chip_info.idcode,
        family, s_chip_info.num_tiles, s_chip_info.revision);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* POST /api/upload -- receive firmware file (XE or BIN) */
static esp_err_t handler_upload(httpd_req_t *req)
{
    if (req->content_len > MAX_FIRMWARE_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File too large");
        return ESP_FAIL;
    }

    if (!s_fw_buf) {
        s_fw_buf = heap_caps_malloc(MAX_FIRMWARE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fw_buf) {
            s_fw_buf = malloc(MAX_FIRMWARE_SIZE);
        }
        if (!s_fw_buf) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
            return ESP_FAIL;
        }
    }

    s_fw_len = 0;
    s_fw_ready = false;

    int remaining = req->content_len;
    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, (char *)s_fw_buf + s_fw_len,
                                      remaining < 4096 ? remaining : 4096);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) continue;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }
        s_fw_len += recv_len;
        remaining -= recv_len;
    }

    s_fw_ready = true;
    ESP_LOGI(TAG, "Firmware uploaded: %zu bytes", s_fw_len);

    /* Quick analysis: is it XE or raw binary? */
    const char *ftype = "unknown";
    int tiles = 0, segments = 0;
    uint32_t entry0 = 0, entry1 = 0;
    size_t code_size = 0;

    if (s_fw_len >= 4 && s_fw_buf[0] == 'X' && s_fw_buf[1] == 'M' &&
        s_fw_buf[2] == 'O' && s_fw_buf[3] == 'S') {
        ftype = "xe";
        /* Parse to get info */
        xe_parsed_t parsed;
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles;
            segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0];
            entry1 = parsed.entry_points[1];
            for (size_t i = 0; i < parsed.num_segments; i++)
                code_size += parsed.segments[i].filesz;
        }
    } else if (s_fw_len >= 4 && s_fw_buf[0] == 0x7F && s_fw_buf[1] == 'E' &&
               s_fw_buf[2] == 'L' && s_fw_buf[3] == 'F') {
        ftype = "elf";
        xe_parsed_t parsed;
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles;
            segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0];
            for (size_t i = 0; i < parsed.num_segments; i++)
                code_size += parsed.segments[i].filesz;
        }
    } else {
        ftype = "bin";
        code_size = s_fw_len;
    }

    char json[384];
    snprintf(json, sizeof(json),
        "{\"ok\":true,\"size\":%zu,\"type\":\"%s\","
        "\"tiles\":%d,\"segments\":%d,"
        "\"entry0\":\"0x%08lx\",\"entry1\":\"0x%08lx\","
        "\"code_size\":%zu}",
        s_fw_len, ftype, tiles, segments,
        (unsigned long)entry0, (unsigned long)entry1, code_size);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* Flash task -- runs in background */
static void flash_task(void *arg)
{
    int mode = (int)(intptr_t)arg;  /* 0=RAM, 1=SPI flash */
    s_flash_progress = 0;
    s_flash_result = 0;
    snprintf(s_flash_status, sizeof(s_flash_status), "Starting...");

    esp_err_t err;

    if (mode == 0) {
        /* Load to RAM via JTAG */
        snprintf(s_flash_status, sizeof(s_flash_status), "Loading to RAM via JTAG...");

        if (s_fw_len >= 4 && (s_fw_buf[0] == 'X' || s_fw_buf[0] == 0x7F)) {
            /* XE or ELF */
            s_flash_progress = 10;
            err = xmos_jtag_load_xe(s_jtag, s_fw_buf, s_fw_len, true);
        } else {
            /* Raw binary */
            s_flash_progress = 10;
            err = xmos_jtag_load_raw(s_jtag, 0, s_fw_buf, s_fw_len,
                                     0x00040000, 0x00080000);
        }

        if (err == ESP_OK) {
            s_flash_progress = 100;
            snprintf(s_flash_status, sizeof(s_flash_status), "Loaded to RAM OK");
        } else {
            s_flash_result = -1;
            snprintf(s_flash_status, sizeof(s_flash_status),
                     "RAM load failed: %s", esp_err_to_name(err));
        }
    } else {
        /* SPI flash programming */
        snprintf(s_flash_status, sizeof(s_flash_status),
                 "SPI flash not implemented yet (use direct SPI or stub)");
        s_flash_result = -1;
    }

    s_flash_progress = s_flash_result == 0 ? 100 : -1;
    vTaskDelete(NULL);
}

/* POST /api/flash?mode=ram|flash -- start flashing */
static esp_err_t handler_flash(httpd_req_t *req)
{
    if (!s_fw_ready || s_fw_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware uploaded");
        return ESP_FAIL;
    }
    if (!s_identified) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Device not identified");
        return ESP_FAIL;
    }
    if (s_flash_progress >= 0 && s_flash_progress < 100) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Flash in progress");
        return ESP_FAIL;
    }

    /* Parse mode from query string */
    char mode_str[16] = "ram";
    httpd_req_get_url_query_str(req, mode_str, sizeof(mode_str));

    int mode = 0;
    if (strstr(mode_str, "flash")) mode = 1;

    xTaskCreate(flash_task, "flash", 8192, (void *)(intptr_t)mode, 5, NULL);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"Flash started\"}");
    return ESP_OK;
}

/* GET /api/status -- poll flash progress */
static esp_err_t handler_status(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json),
        "{\"progress\":%d,\"result\":%d,\"status\":\"%s\"}",
        s_flash_progress, s_flash_result, s_flash_status);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * HTTP server setup
 * ---------------------------------------------------------------------- */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t uri_index    = { .uri = "/",              .method = HTTP_GET,  .handler = handler_index };
    httpd_uri_t uri_identify = { .uri = "/api/identify",  .method = HTTP_GET,  .handler = handler_identify };
    httpd_uri_t uri_upload   = { .uri = "/api/upload",    .method = HTTP_POST, .handler = handler_upload };
    httpd_uri_t uri_flash    = { .uri = "/api/flash",     .method = HTTP_POST, .handler = handler_flash };
    httpd_uri_t uri_status   = { .uri = "/api/status",    .method = HTTP_GET,  .handler = handler_status };

    httpd_register_uri_handler(server, &uri_index);
    httpd_register_uri_handler(server, &uri_identify);
    httpd_register_uri_handler(server, &uri_upload);
    httpd_register_uri_handler(server, &uri_flash);
    httpd_register_uri_handler(server, &uri_status);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Init JTAG */
    xmos_jtag_pins_t pins = {
        .tck    = PIN_TCK,
        .tms    = PIN_TMS,
        .tdi    = PIN_TDI,
        .tdo    = PIN_TDO,
        .trst_n = PIN_TRST,
        .srst_n = PIN_SRST,
    };
    ESP_ERROR_CHECK(xmos_jtag_init(&pins, &s_jtag));

    /* Start WiFi AP */
    wifi_init_ap();

    /* Start web server */
    start_webserver();

    ESP_LOGI(TAG, "Ready. Connect to WiFi '%s' and open http://192.168.4.1", WIFI_AP_SSID);
}
