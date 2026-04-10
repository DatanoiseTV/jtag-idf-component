/*
 * XMOS Web Flasher -- Example ESP-IDF application
 *
 * Provides a web UI for:
 *   - Identifying the connected XMOS device
 *   - JTAG boundary scan (capture I/O pin states)
 *   - Uploading and inspecting .xe / .bin firmware files
 *   - Flashing firmware to XMOS RAM or SPI flash via JTAG
 *
 * Networking:
 *   ESP32-S3/C3/...: WiFi AP, open http://192.168.4.1
 *   ESP32-P4-NANO:   Ethernet (IP101GRI via RMII), DHCP
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

#if CONFIG_IDF_TARGET_ESP32P4
#include "esp_ldo_regulator.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth_phy_ip101.h"
#include "driver/gpio.h"
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
 * ESP32-P4-NANO: GPIO39-44 are powered by internal LDO_VO4.
 *   We set LDO_VO4 to 1.8V to match XMOS JTAG I/O levels directly --
 *   no level shifters needed.  6 pins: TCK, TMS, TDI, TDO, TRST, SRST.
 *
 * ESP32-S3: 3.3V I/O -- use level shifters for 1.8V XMOS JTAG.
 */
#if CONFIG_IDF_TARGET_ESP32P4
#define PIN_TCK             GPIO_NUM_39
#define PIN_TMS             GPIO_NUM_40
#define PIN_TDI             GPIO_NUM_41
#define PIN_TDO             GPIO_NUM_42
#define PIN_TRST            GPIO_NUM_43
#define PIN_SRST            GPIO_NUM_44
#define JTAG_LDO_CHAN       4       /* LDO_VO4 powers GPIO39-44 */
#define JTAG_LDO_MV        1800    /* 1.8V matches XMOS JTAG levels */
#else
#define PIN_TCK             GPIO_NUM_12
#define PIN_TMS             GPIO_NUM_13
#define PIN_TDI             GPIO_NUM_14
#define PIN_TDO             GPIO_NUM_11
#define PIN_TRST            GPIO_NUM_NC
#define PIN_SRST            GPIO_NUM_NC
#endif

/* ESP32-P4-NANO Ethernet (IP101GRI via RMII) */
#if CONFIG_IDF_TARGET_ESP32P4
#define ETH_PHY_ADDR        1
#define ETH_PHY_RST_GPIO    GPIO_NUM_51
#define ETH_MDC_GPIO        GPIO_NUM_31
#define ETH_MDIO_GPIO       GPIO_NUM_52
#endif

/* -------------------------------------------------------------------------
 * Globals
 * ---------------------------------------------------------------------- */
static xmos_jtag_handle_t s_jtag = NULL;
static xmos_chip_info_t s_chip_info = {0};
static bool s_identified = false;
static size_t s_bsr_len = 0;

#define MAX_FIRMWARE_SIZE   (8 * 1024 * 1024)
static uint8_t *s_fw_buf = NULL;
static size_t s_fw_len = 0;
static bool s_fw_ready = false;

static volatile int s_flash_progress = -1;
static volatile int s_flash_result = 0;
static char s_flash_status[128] = "Idle";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* -------------------------------------------------------------------------
 * Network init
 * ---------------------------------------------------------------------- */

#if SOC_WIFI_SUPPORTED
static void net_init_wifi_ap(void)
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

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s, open http://192.168.4.1", WIFI_AP_SSID);
}
#endif

#if CONFIG_IDF_TARGET_ESP32P4
static void net_init_ethernet(void)
{
    /* Reset PHY */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << ETH_PHY_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(ETH_PHY_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ETH_PHY_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Create default netif for Ethernet */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    /* MAC config (internal EMAC with RMII) */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);

    /* PHY config (IP101GRI) */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = -1;  /* Already reset above */
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    /* Install Ethernet driver */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    /* Attach to netif */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(TAG, "Ethernet started (IP101GRI RMII). Waiting for DHCP...");
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

static esp_err_t handler_identify(httpd_req_t *req)
{
    esp_err_t err = xmos_jtag_identify(s_jtag, &s_chip_info);
    s_identified = (err == ESP_OK && s_chip_info.family != XMOS_FAMILY_UNKNOWN);

    const char *family;
    if (err == ESP_ERR_NOT_FOUND) {
        family = "No device connected";
    } else {
        switch (s_chip_info.family) {
            case XMOS_FAMILY_XS1: family = "XS1"; break;
            case XMOS_FAMILY_XS2: family = "xCORE-200 (XS2)"; break;
            case XMOS_FAMILY_XS3: family = "xCORE.ai (XS3)"; break;
            default: family = "Unknown device"; break;
        }
    }

    if (s_identified)
        xmos_jtag_bscan_detect(s_jtag, &s_bsr_len);

    char idstr[16] = "--";
    if (err != ESP_ERR_NOT_FOUND)
        snprintf(idstr, sizeof(idstr), "0x%08lx", (unsigned long)s_chip_info.idcode);

    char json[320];
    snprintf(json, sizeof(json),
        "{\"ok\":%s,\"idcode\":\"%s\",\"family\":\"%s\","
        "\"tiles\":%d,\"revision\":%d,\"bsr_len\":%zu}",
        s_identified ? "true" : "false",
        idstr, family, s_chip_info.num_tiles, s_chip_info.revision,
        s_bsr_len);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* GET /api/chain -- scan JTAG chain, return all devices */
static esp_err_t handler_chain(httpd_req_t *req)
{
    jtag_chain_t chain;
    esp_err_t err = xmos_jtag_scan_chain(s_jtag, &chain);
    if (err != ESP_OK) {
        char errmsg[128];
        snprintf(errmsg, sizeof(errmsg),
                 "{\"devices\":[],\"error\":\"%s\"}", esp_err_to_name(err));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, errmsg);
        return ESP_OK;
    }

    /* Build JSON array */
    char *json = malloc(2048);
    if (!json) return ESP_FAIL;
    int pos = snprintf(json, 2048, "{\"devices\":[");

    for (size_t i = 0; i < chain.num_devices; i++) {
        const jtag_chain_device_t *d = &chain.devices[i];
        if (i > 0) json[pos++] = ',';
        pos += snprintf(json + pos, 2048 - pos,
            "{\"idcode\":\"0x%08lx\",\"name\":\"%s\","
            "\"manufacturer\":\"0x%03x\",\"part\":\"0x%04x\","
            "\"version\":%d,\"ir_len\":%d}",
            (unsigned long)d->idcode, d->name,
            d->manufacturer, d->part_number,
            d->version, d->ir_len);
    }
    pos += snprintf(json + pos, 2048 - pos, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, pos);
    free(json);
    return ESP_OK;
}

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

    size_t hex_len = bytes * 2;
    size_t json_size = 64 + hex_len + s_bsr_len + 4;
    char *json = malloc(json_size);
    if (!json) { free(bsr); return ESP_FAIL; }

    int pos = snprintf(json, json_size, "{\"bsr_len\":%zu,\"hex\":\"", s_bsr_len);
    for (int i = (int)bytes - 1; i >= 0; i--)
        pos += snprintf(json + pos, json_size - pos, "%02X", bsr[i]);
    pos += snprintf(json + pos, json_size - pos, "\",\"bits\":\"");
    for (int i = (int)s_bsr_len - 1; i >= 0; i--)
        json[pos++] = ((bsr[i / 8] >> (i % 8)) & 1) ? '1' : '0';
    pos += snprintf(json + pos, json_size - pos, "\"}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, pos);
    free(json);
    free(bsr);
    return ESP_OK;
}

static esp_err_t handler_upload(httpd_req_t *req)
{
    if (req->content_len > MAX_FIRMWARE_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File too large");
        return ESP_FAIL;
    }

    if (!s_fw_buf) {
        s_fw_buf = heap_caps_malloc(MAX_FIRMWARE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fw_buf) s_fw_buf = malloc(MAX_FIRMWARE_SIZE);
        if (!s_fw_buf) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
            return ESP_FAIL;
        }
    }

    s_fw_len = 0;
    s_fw_ready = false;
    int remaining = req->content_len;
    while (remaining > 0) {
        int n = httpd_req_recv(req, (char *)s_fw_buf + s_fw_len,
                               remaining < 4096 ? remaining : 4096);
        if (n <= 0) { if (n == HTTPD_SOCK_ERR_TIMEOUT) continue; return ESP_FAIL; }
        s_fw_len += n;
        remaining -= n;
    }
    s_fw_ready = true;

    const char *ftype = "bin";
    int tiles = 0, segments = 0;
    uint32_t entry0 = 0, entry1 = 0;
    size_t code_size = 0;

    xe_parsed_t parsed;
    if (s_fw_len >= 4 && s_fw_buf[0] == 'X' && s_fw_buf[1] == 'M') {
        ftype = "xe";
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles; segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0]; entry1 = parsed.entry_points[1];
            for (size_t i = 0; i < parsed.num_segments; i++) code_size += parsed.segments[i].filesz;
        }
    } else if (s_fw_len >= 4 && s_fw_buf[0] == 0x7F && s_fw_buf[1] == 'E') {
        ftype = "elf";
        if (xe_parse(s_fw_buf, s_fw_len, &parsed) == ESP_OK) {
            tiles = parsed.num_tiles; segments = (int)parsed.num_segments;
            entry0 = parsed.entry_points[0];
            for (size_t i = 0; i < parsed.num_segments; i++) code_size += parsed.segments[i].filesz;
        }
    } else {
        code_size = s_fw_len;
    }

    char json[384];
    snprintf(json, sizeof(json),
        "{\"ok\":true,\"size\":%zu,\"type\":\"%s\",\"tiles\":%d,\"segments\":%d,"
        "\"entry0\":\"0x%08lx\",\"entry1\":\"0x%08lx\",\"code_size\":%zu}",
        s_fw_len, ftype, tiles, segments,
        (unsigned long)entry0, (unsigned long)entry1, code_size);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

static void flash_task(void *arg)
{
    int mode = (int)(intptr_t)arg;
    s_flash_progress = 0; s_flash_result = 0;
    snprintf(s_flash_status, sizeof(s_flash_status), "Starting...");
    esp_err_t err;

    if (mode == 0) {
        snprintf(s_flash_status, sizeof(s_flash_status), "Loading to RAM via JTAG...");
        s_flash_progress = 10;
        if (s_fw_len >= 4 && (s_fw_buf[0] == 'X' || s_fw_buf[0] == 0x7F))
            err = xmos_jtag_load_xe(s_jtag, s_fw_buf, s_fw_len, true);
        else
            err = xmos_jtag_load_raw(s_jtag, 0, s_fw_buf, s_fw_len, 0x00040000, 0x00080000);
        if (err == ESP_OK) {
            s_flash_progress = 100;
            snprintf(s_flash_status, sizeof(s_flash_status), "Loaded to RAM OK");
        } else {
            s_flash_result = -1;
            snprintf(s_flash_status, sizeof(s_flash_status), "Failed: %s", esp_err_to_name(err));
        }
    } else {
        s_flash_result = -1;
        snprintf(s_flash_status, sizeof(s_flash_status), "SPI flash: provide stub or use direct SPI");
    }
    s_flash_progress = s_flash_result == 0 ? 100 : -1;
    vTaskDelete(NULL);
}

static esp_err_t handler_flash(httpd_req_t *req)
{
    if (!s_fw_ready) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware"); return ESP_FAIL; }
    if (!s_identified) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not identified"); return ESP_FAIL; }
    if (s_flash_progress >= 0 && s_flash_progress < 100) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "In progress"); return ESP_FAIL;
    }
    char qs[16] = "ram";
    httpd_req_get_url_query_str(req, qs, sizeof(qs));
    xTaskCreate(flash_task, "flash", 8192, (void *)(intptr_t)(strstr(qs,"flash") ? 1 : 0), 5, NULL);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t handler_status(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json), "{\"progress\":%d,\"result\":%d,\"status\":\"%s\"}",
             s_flash_progress, s_flash_result, s_flash_status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * HTTP server
 * ---------------------------------------------------------------------- */
static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t uris[] = {
        { .uri = "/",              .method = HTTP_GET,  .handler = handler_index },
        { .uri = "/api/identify",  .method = HTTP_GET,  .handler = handler_identify },
        { .uri = "/api/chain",     .method = HTTP_GET,  .handler = handler_chain },
        { .uri = "/api/bscan",     .method = HTTP_GET,  .handler = handler_bscan },
        { .uri = "/api/upload",    .method = HTTP_POST, .handler = handler_upload },
        { .uri = "/api/flash",     .method = HTTP_POST, .handler = handler_flash },
        { .uri = "/api/status",    .method = HTTP_GET,  .handler = handler_status },
    };
    for (int i = 0; i < sizeof(uris)/sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_IDF_TARGET_ESP32P4
    /* Set LDO_VO4 to 1.8V -- powers GPIO39-44 used for JTAG.
     * This matches XMOS JTAG I/O levels directly, no level shifters. */
    esp_ldo_channel_handle_t ldo_handle;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = JTAG_LDO_CHAN,
        .voltage_mv = JTAG_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_handle));
    ESP_LOGI(TAG, "LDO_VO4 set to %d mV (GPIO39-44 = 1.8V JTAG)", JTAG_LDO_MV);
#endif

    xmos_jtag_pins_t pins = {
        .tck = PIN_TCK, .tms = PIN_TMS, .tdi = PIN_TDI, .tdo = PIN_TDO,
        .trst_n = PIN_TRST, .srst_n = PIN_SRST,
    };
    ESP_ERROR_CHECK(xmos_jtag_init(&pins, &s_jtag));

#if CONFIG_IDF_TARGET_ESP32P4
    net_init_ethernet();
    ESP_LOGI(TAG, "Ethernet started. Check DHCP lease for IP address.");
#elif SOC_WIFI_SUPPORTED
    net_init_wifi_ap();
#else
    ESP_LOGW(TAG, "No network interface available on this chip.");
#endif

    start_webserver();
}
