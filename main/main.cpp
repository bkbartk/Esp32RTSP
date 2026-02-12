#include <cstring>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "rtsp_server.hpp"      // C++ RTSP API
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
// Provide your credentials in main/config.h (copy from config.h.example)
#include "config.h"
#include "ota.hpp"
#include "mdns.h"

#ifndef OTA_URL_DEFAULT
#define OTA_URL_DEFAULT "http://example.com/firmware.bin"
#endif

static const char *TAG = "RTSP_CAM";

// ---------------- WIFI INIT (C++ SAFE) ----------------
static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config{};
    std::memset(&wifi_config, 0, sizeof(wifi_config));

    // Copy SSID/password from `main/wifi_config.h` (keep that file out of VCS)
    std::strncpy((char *)wifi_config.sta.ssid,
                 WIFI_SSID,
                 sizeof(wifi_config.sta.ssid));

    std::strncpy((char *)wifi_config.sta.password,
                 WIFI_PASSWORD,
                 sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI("WIFI", "Connecting to WiFi...");
}
// ------------------------------------------------------


// ---------------- CAMERA CONFIG -----------------------
static camera_config_t camera_config = {
    .pin_pwdn       = -1,
    .pin_reset      = -1,
    .pin_xclk       = 15,
    .pin_sccb_sda   = 4,
    .pin_sccb_scl   = 5,
    .pin_d7         = 16,
    .pin_d6         = 17,
    .pin_d5         = 18,
    .pin_d4         = 12,
    .pin_d3         = 10,
    .pin_d2         = 8,
    .pin_d1         = 9,
    .pin_d0         = 11,
    .pin_vsync      = 6,
    .pin_href       = 7,
    .pin_pclk       = 13,

    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_VGA,
    .jpeg_quality   = 10,
    .fb_count       = 1,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port  = 0,
};

// ------------------------------------------------------


static void mdns_setup(void)
{
#ifdef MDNS_NAME
    if (strlen(MDNS_NAME) > 0) {
        esp_err_t err = mdns_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "mdns_init failed: %d", err);
        } else {
            mdns_hostname_set(MDNS_NAME);
            mdns_instance_name_set("ESP32 RTSP Camera");
            mdns_service_add(NULL, "_rtsp._tcp", NULL, 8554, NULL, 0);
            ESP_LOGI(TAG, "mDNS started: %s.local", MDNS_NAME);
        }
    }
#endif
}

extern "C" void app_main(void)
{
    wifi_init();

    // Initialize mDNS (if configured)
    mdns_setup();

    ESP_LOGI(TAG, "Initialising camera...");
    if (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }
    ESP_LOGI(TAG, "Camera initialised");

    // ---- Correct RTSP API ----
    espp::RtspServer::Config rtsp_cfg;
    rtsp_cfg.port = 8554;          // only required field

    espp::RtspServer server{rtsp_cfg};
    server.start();

    ESP_LOGI(TAG, "RTSP server started");
    ESP_LOGI(TAG, "Stream URL: rtsp://<IP>:8554/stream");
    // --------------------------

    // OTA control: triggered via HTTP endpoint or button (no automatic start)

    // HTTP server: provide `/ota` endpoint to trigger OTA
    httpd_handle_t server_handle = NULL;
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 8080;

    auto ota_http_handler = [](httpd_req_t *req) -> esp_err_t {
        char urlbuf[256] = {0};
        // Support optional URL in query: /ota?url=...
        char qsbuf[256] = {0};
        if (httpd_req_get_url_query_str(req, qsbuf, sizeof(qsbuf)) == ESP_OK) {
            char value[256];
            if (httpd_query_key_value(qsbuf, "url", value, sizeof(value)) == ESP_OK) {
                strncpy(urlbuf, value, sizeof(urlbuf)-1);
            }
        }
        const char *chosen = urlbuf[0] ? urlbuf : OTA_URL_DEFAULT;

        if (start_ota_from_url(chosen) == ESP_OK) {
            const char *resp = "OTA triggered\n";
            httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        } else {
            const char *resp = "OTA failed to start\n";
            httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        }
        return ESP_OK;
    };

    if (httpd_start(&server_handle, &http_config) == ESP_OK) {
        httpd_uri_t ota_uri = {
            .uri = "/ota",
            .method = HTTP_GET,
            .handler = ota_http_handler,
            .user_ctx = nullptr,
        };
        httpd_register_uri_handler(server_handle, &ota_uri);
    }

    // Button poll task: triggers OTA when button held for ~300ms
    const gpio_num_t ota_button_gpio = GPIO_NUM_0;
    gpio_set_direction(ota_button_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ota_button_gpio, GPIO_PULLUP_ONLY);
    xTaskCreate(+[](void *arg){
        (void)arg;
        int stable = 0;
        while (true) {
            int level = gpio_get_level(ota_button_gpio);
            if (level == 0) { // pressed (active low)
                stable++;
                if (stable >= 3) { // ~300ms (3 * 100ms)
                    start_ota_from_url(OTA_URL_DEFAULT);
                    // wait until released
                    while (gpio_get_level(ota_button_gpio) == 0) vTaskDelay(pdMS_TO_TICKS(100));
                }
            } else {
                stable = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }, "ota_button", 2048, NULL, 5, NULL);

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Failed to get frame");
            continue;
        }

        // Convert raw buffer → vector<uint8_t>
        std::vector<uint8_t> jpeg_data(fb->buf, fb->buf + fb->len);

        // Construct JpegFrame using the ONLY valid constructor
        espp::JpegFrame frame(jpeg_data);

        server.send_frame(frame);

        esp_camera_fb_return(fb);
    }
}

