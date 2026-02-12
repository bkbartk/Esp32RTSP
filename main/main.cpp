#include <cstring>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "rtsp_server.hpp"      // C++ RTSP API
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
// Provide your credentials in main/wifi_config.h (copy from wifi_config.h.example)
#include "wifi_config.h"

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


extern "C" void app_main(void)
{
    wifi_init();

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

