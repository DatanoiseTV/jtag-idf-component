/*
 * XMOS Web Flasher -- Example ESP-IDF application
 *
 * Provides a web UI for:
 *   - Identifying the connected XMOS device
 *   - JTAG boundary scan (capture I/O pin states)
 *   - Uploading and inspecting .xe / .bin firmware files
 *   - Flashing firmware to XMOS RAM or SPI flash via JTAG
 *
 * On WiFi-capable chips (S3, C3, ...): creates AP, open http://192.168.4.1
 * On ESP32-P4: use Ethernet or USB networking (WiFi code is compiled out)
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "xmos_jtag.h"
#include "xmos_xe.h"

#if SOC_WIFI_SUPPORTED
#include "esp_wifi.h"
#endif

static const char *TAG = "xmos_web";

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */
#define WIFI_AP_SSID        "XMOS-Flasher"
#define WIFI_AP_PASS        "xmosjtag"
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    2

/*
 * JTAG pins -- adjust for your board.
 *
 * ESP32-S3 defaults (avoids strapping/PSRAM/USB pins):
 *   TCK=12, TMS=13, TDI=14, TDO=11
 *
 * Waveshare ESP32-P4-NANO right header rows 7-10:
 *   TCK=47, TMS=48, TDI=46, TDO=45, TRST=53, SRST=54
 */
#if CONFIG_IDF_TARGET_ESP32P4
#define PIN_TCK             GPIO_NUM_47
#define PIN_TMS             GPIO_NUM_48
#define PIN_TDI             GPIO_NUM_46
#define PIN_TDO             GPIO_NUM_45
#define PIN_TRST            GPIO_NUM_NC
#define PIN_SRST            GPIO_NUM_54
#else
#define PIN_TCK             GPIO_NUM_12
#define PIN_TMS             GPIO_NUM_13
#define PIN_TDI             GPIO_NUM_14
#define PIN_TDO             GPIO_NUM_11
#define PIN_TRST            GPIO_NUM_NC
#define PIN_SRST            GPIO_NUM_NC
#endif

/* -------------------------------------------------------------------------
 * Globals
 * ---------------------------------------------------------------------- */
static xmos_jtag_handle_t s_jtag = NULL;
static xmos_chip_info_t s_chip_info = {0};
static bool s_identified = false;

/* Boundary scan state */
static size_t s_bsr_len = 0;

/* Upload buffer */
#define MAX_FIRMWARE_SIZE   (8 * 1024 * 1024)
static uint8_t *s_fw_buf = NULL;
static size_t s_fw_len = 0;
static bool s_fw_ready = false;

/* Flash progress */
static volatile int s_flash_progress = -1;
static volatile int s_flash_result = 0;
static char s_flash_status[128] = "Idle";

/* -------------------------------------------------------------------------
 * Embedded HTML
 * ---------------------------------------------------------------------- */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* -------------------------------------------------------------------------
 * WiFi AP setup (only on WiFi-capable chips)
 * ---------------------------------------------------------------------- */
#if SOC_WIFI_SUPPORTED
static void wifi_init_ap(void)
{
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
#endif

/* -------------------------------------------------------------------------
 * HTTP handlers
 * ---------------------------------------------------------------------- */

static esp_err_t handler_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

/* GET /api/identify */
static esp_err_t handler_identify(httpd_req_t *req)
{
    esp_err_t err = xmos_jtag_identify(s_jtag, &s_chip_info);
    s_identified = (err == ESP_OK && s_chip_info.family != XMOS_FAMILY_UNKNOWN);

    const char *family = "Unknown";
    switch (s_chip_info.family) {
        case XMOS_FAMILY_XS1: family = "XS1"; break;
        case XMOS_FAMILY_XS2: family = "xCORE-200 (XS2)"; break;
        case XMOS_FAMILY_XS3: family = "xCORE.ai (XS3)"; break;
        default: break;
    }

    /* Auto-detect BSR length if identification succeeded */
    if (s_identified) {
        xmos_jtag_bscan_detect(s_jtag, &s_bsr_len);
    }

    char json[320];
    snprintf(json, sizeof(json),
        "{\"ok\":%s,\"idcode\":\"0x%08lx\",\"family\":\"%s\","
        "\"tiles\":%d,\"revision\":%d,\"bsr_len\":%zu}",
        s_identified ? "true" : "false",
        (unsigned long)s_chip_info.idcode,
        family, s_chip_info.num_tiles, s_chip_info.revision,
        s_bsr_len);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* GET /api/bscan -- capture boundary scan register */
static esp_err_t handler_bscan(httpd_req_t *req)
{
    if (!s_identified || s_bsr_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Device not identified or BSR not detected");
        return ESP_FAIL;
    }

    size_t bytes = (s_bsr_len + 7) / 8;
    uint8_t *bsr = calloc(1, bytes);
    if (!bsr) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    esp_err_t err = xmos_jtag_bscan_sample(s_jtag, bsr, s_bsr_len);
    if (err != ESP_OK) {
        free(bsr);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Scan failed");
        return ESP_FAIL;
    }

    /* Build JSON: { "bsr_len": N, "hex": "AABB...", "bits": "01100..." } */
    /* Hex string: 2 chars per byte */
    size_t hex_len = bytes * 2;
    /* Bit string: 1 char per bit */
    size_t json_size = 64 + hex_len + s_bsr_len + 4;
    char *json = malloc(json_size);
    if (!json) {
        free(bsr);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int pos = snprintf(json, json_size, "{\"bsr_len\":%zu,\"hex\":\"", s_bsr_len);

    /* Hex representation (MSB byte first for readability) */
    for (int i = (int)bytes - 1; i >= 0; i--)
        pos += snprintf(json + pos, json_size - pos, "%02X", bsr[i]);

    pos += snprintf(json + pos, json_size - pos, "\",\"bits\":\"");

    /* Bit string (MSB first) */
    for (int i = (int)s_bsr_len - 1; i >= 0; i--)
        json[pos++] = ((bsr[i / 8] >> (i % 8)) & 1) ? '1' : '0';

    pos += snprintf(json + pos, json_size - pos, "\"}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, pos);

    free(json);
    free(bsr);
    return ESP_OK;
}

/* POST /api/upload */
static esp_err_t handler_upload(httpd_req_t *req)
{
    if (req->content_len > MAX_FIRMWARE_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File too large");
        return ESP_FAIL;
    }

    if (!s_fw_buf) {
        s_fw_buf = heap_caps_malloc(MAX_FIRMWARE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fw_buf)
            s_fw_buf = malloc(MAX_FIRMWARE_SIZE);
        if (!s_fw_buf) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
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
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv error");
            return ESP_FAIL;
        }
        s_fw_len += recv_len;
        remaining -= recv_len;
    }

    s_fw_ready = true;
    ESP_LOGI(TAG, "Firmware uploaded: %zu bytes", s_fw_len);

    /* Analyse file type */
    const char *ftype = "bin";
    int tiles = 0, segments = 0;
    uint32_t entry0 = 0, entry1 = 0;
    size_t code_size = 0;

    xe_parsed_t parsed;
    if (s_fw_len >= 4 && s_fw_buf[0] == 'X' && s_fw_buf[1] == 'M' &&
        s_fw_buf[2] == 'O' && s_fw_buf[3] == 'S') {
        ftype = "xe";
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles;
            segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0];
            entry1 = parsed.entry_points[1];
            for (size_t i = 0; i < parsed.num_segments; i++)
                code_size += parsed.segments[i].filesz;
        }
    } else if (s_fw_len >= 4 && s_fw_buf[0] == 0x7F && s_fw_buf[1] == 'E') {
        ftype = "elf";
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles;
            segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0];
            for (size_t i = 0; i < parsed.num_segments; i++)
                code_size += parsed.segments[i].filesz;
        }
    } else {
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
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* Flash task */
static void flash_task(void *arg)
{
    int mode = (int)(intptr_t)arg;
    s_flash_progress = 0;
    s_flash_result = 0;
    snprintf(s_flash_status, sizeof(s_flash_status), "Starting...");

    esp_err_t err;

    if (mode == 0) {
        snprintf(s_flash_status, sizeof(s_flash_status), "Loading to RAM via JTAG...");
        s_flash_progress = 10;

        if (s_fw_len >= 4 && (s_fw_buf[0] == 'X' || s_fw_buf[0] == 0x7F))
            err = xmos_jtag_load_xe(s_jtag, s_fw_buf, s_fw_len, true);
        else
            err = xmos_jtag_load_raw(s_jtag, 0, s_fw_buf, s_fw_len,
                                     0x00040000, 0x00080000);

        if (err == ESP_OK) {
            s_flash_progress = 100;
            snprintf(s_flash_status, sizeof(s_flash_status), "Loaded to RAM OK");
        } else {
            s_flash_result = -1;
            snprintf(s_flash_status, sizeof(s_flash_status),
                     "Failed: %s", esp_err_to_name(err));
        }
    } else {
        snprintf(s_flash_status, sizeof(s_flash_status),
                 "SPI flash: provide a stub or use direct SPI mode");
        s_flash_result = -1;
    }

    s_flash_progress = s_flash_result == 0 ? 100 : -1;
    vTaskDelete(NULL);
}

/* POST /api/flash?mode=ram|flash */
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

    char mode_str[16] = "ram";
    httpd_req_get_url_query_str(req, mode_str, sizeof(mode_str));
    int mode = strstr(mode_str, "flash") ? 1 : 0;

    xTaskCreate(flash_task, "flash", 8192, (void *)(intptr_t)mode, 5, NULL);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"Flash started\"}");
    return ESP_OK;
}

/* GET /api/status */
static esp_err_t handler_status(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json),
        "{\"progress\":%d,\"result\":%d,\"status\":\"%s\"}",
        s_flash_progress, s_flash_result, s_flash_status);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * HTTP server
 * ---------------------------------------------------------------------- */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",              .method = HTTP_GET,  .handler = handler_index },
        { .uri = "/api/identify",  .method = HTTP_GET,  .handler = handler_identify },
        { .uri = "/api/bscan",     .method = HTTP_GET,  .handler = handler_bscan },
        { .uri = "/api/upload",    .method = HTTP_POST, .handler = handler_upload },
        { .uri = "/api/flash",     .method = HTTP_POST, .handler = handler_flash },
        { .uri = "/api/status",    .method = HTTP_GET,  .handler = handler_status },
    };
    for (int i = 0; i < sizeof(uris)/sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return server;
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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

#if SOC_WIFI_SUPPORTED
    wifi_init_ap();
    ESP_LOGI(TAG, "Connect to WiFi '%s' and open http://192.168.4.1", WIFI_AP_SSID);
#else
    ESP_LOGW(TAG, "No WiFi on this chip. Use Ethernet or USB networking.");
    ESP_LOGI(TAG, "HTTP server will start once a network interface is up.");
#endif

    start_webserver();
}
