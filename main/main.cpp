#include <cstring>
#include <string>
#include <memory>
#include <span>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_camera.h"
#include "esp_heap_caps.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mdns.h"

#include "rtsp_server.hpp"
#include "config.h"
#include "img_converters.h"

static const char *TAG = "PIPE";

// ---------------- CONFIG ----------------

#define MD_W 40
#define MD_H 30
#define FBQ_LEN 4
#define MOTIONQ_LEN 1
#define MOTION_INTERVAL 5

// ---------------- GLOBALS ----------------

static QueueHandle_t fbq_rtsp = nullptr;      // camera_fb_t*
static QueueHandle_t motion_gray_q = nullptr; // uint8_t[MD_W*MD_H]

static uint8_t md_prev[MD_W * MD_H];
static bool md_prev_valid = false;

static std::shared_ptr<espp::RtspServer> rtsp_server;
static bool rtsp_started = false;
static bool wifi_got_ip = false;

// ---------------- CAMERA CONFIG (YUV422 + SVGA) ----------------

static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk  = 15,

    .pin_sccb_sda = 4,
    .pin_sccb_scl = 5,

    .pin_d7 = 16,
    .pin_d6 = 17,
    .pin_d5 = 18,
    .pin_d4 = 12,
    .pin_d3 = 10,
    .pin_d2 = 8,
    .pin_d1 = 9,
    .pin_d0 = 11,

    .pin_vsync = 6,
    .pin_href  = 7,
    .pin_pclk  = 13,

    .xclk_freq_hz = 20000000,

    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_YUV422,
    .frame_size   = FRAMESIZE_SVGA,
    .jpeg_quality = 10,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
    .sccb_i2c_port = 0,
};

// ---------------- MOTION DETECTION ----------------

static bool motion_detect(const uint8_t *curr)
{
    if (!md_prev_valid) {
        memcpy(md_prev, curr, sizeof(md_prev));
        md_prev_valid = true;
        return false;
    }

    int changed = 0;
    const int diff_threshold = 20;
    const int min_pixels = 50;

    for (int i = 0; i < MD_W * MD_H; ++i) {
        if (abs((int)curr[i] - (int)md_prev[i]) > diff_threshold)
            changed++;
    }

    memcpy(md_prev, curr, sizeof(md_prev));

    return changed > min_pixels;
}

// ---------------- RTSP SEND ----------------

static void rtsp_send_jpeg(const uint8_t *jpeg_buf, size_t jpeg_len)
{
    if (!rtsp_server) return;

    std::span<const uint8_t> jpeg_span(jpeg_buf, jpeg_len);
    rtsp_server->send_frame(espp::JpegFrame(jpeg_span));
}

// ---------------- WIFI ----------------

static void wifi_event_handler(void *, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto *event = (ip_event_got_ip_t *)data;

        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));

        ESP_LOGI("WIFI", "IP Address: %s", ip_str);
        wifi_got_ip = true;

        if (rtsp_server && !rtsp_started) {
            rtsp_server->start();
            rtsp_started = true;
            ESP_LOGI(TAG, "RTSP server started: rtsp://%s:8554", ip_str);
        }
    }
}

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               nullptr));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               nullptr));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config{};
    memset(&wifi_config, 0, sizeof(wifi_config));

    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI("WIFI", "Connecting to WiFi...");
}

// ---------------- CAMERA INIT ----------------

static bool camera_init_safe()
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Camera initialized");
    return true;
}

// ---------------- TASKS ----------------

// 1. Capture Task: only grabs fb and pushes to RTSP queue
static void capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (xQueueSend(fbq_rtsp, &fb, 0) != pdTRUE) {
            esp_camera_fb_return(fb);
        }
    }
}

// 2. Motion Task: consumes downscaled grayscale frames
static void motion_task(void *arg)
{
    ESP_LOGI(TAG, "Motion task started");

    uint8_t md_frame[MD_W * MD_H];

    while (true) {
        if (xQueueReceive(motion_gray_q, md_frame, portMAX_DELAY) == pdTRUE) {
            if (motion_detect(md_frame)) {
                ESP_LOGI(TAG, "Motion detected");
                // TODO: MQTT publish if you want
            }
        }
    }
}

// 3. RTSP Task: highest priority, does JPEG + motion every 5th frame
static void rtsp_task(void *arg)
{
    ESP_LOGI(TAG, "RTSP task started");

    while (!wifi_got_ip || !rtsp_started) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    int motion_frame_counter = 0;

    while (true) {
        camera_fb_t *fb = nullptr;
        if (xQueueReceive(fbq_rtsp, &fb, portMAX_DELAY) != pdTRUE)
            continue;

        // --- Motion every Nth frame (Y downscale from YUV422) ---
        motion_frame_counter++;
        if (motion_frame_counter >= MOTION_INTERVAL) {
            motion_frame_counter = 0;

            uint8_t md_frame[MD_W * MD_H];
            int src_w = fb->width;
            int src_h = fb->height;
            int step_x = src_w / MD_W;
            int step_y = src_h / MD_H;
            int idx = 0;
            const uint8_t *src = fb->buf; // YUV422: Y at even bytes

            for (int y = 0; y < src_h && idx < MD_W * MD_H; y += step_y) {
                for (int x = 0; x < src_w && idx < MD_W * MD_H; x += step_x) {
                    int pixel_index = y * src_w + x;
                    md_frame[idx++] = src[pixel_index * 2]; // Y component
                }
            }

            xQueueOverwrite(motion_gray_q, md_frame);
        }

        // --- JPEG for RTSP ---
        uint8_t *jpeg_buf = nullptr;
        size_t jpeg_len = 0;

        bool ok = fmt2jpg(fb->buf, fb->len,
                          fb->width, fb->height,
                          PIXFORMAT_YUV422,
                          camera_config.jpeg_quality,
                          &jpeg_buf, &jpeg_len);

        if (ok && jpeg_buf) {
            ESP_LOGI(TAG, "JPEG size = %u", (unsigned)jpeg_len);
            rtsp_send_jpeg(jpeg_buf, jpeg_len);
            free(jpeg_buf);
        } else {
            ESP_LOGE(TAG, "JPEG encode failed");
        }

        esp_camera_fb_return(fb);
        vTaskDelay(1);  // avoid starving idle/WDT
    }
}

// ---------------- MAIN ----------------

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();
    if (!camera_init_safe()) {
        ESP_LOGE(TAG, "Camera not available, stopping");
        return;
    }

    espp::RtspServer::Config cfg;
    cfg.port = 8554;
    rtsp_server = std::make_shared<espp::RtspServer>(cfg);

    fbq_rtsp = xQueueCreate(FBQ_LEN, sizeof(camera_fb_t *));
    motion_gray_q = xQueueCreate(MOTIONQ_LEN, MD_W * MD_H);
    if (!fbq_rtsp || !motion_gray_q) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    // Priorities: RTSP highest
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(motion_task,  "motion",  4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(rtsp_task,    "rtsp",    8192, nullptr, 6, nullptr, 1);

    ESP_LOGI(TAG, "Pipeline running (YUV422 motion + JPEG RTSP, motion every %d frames)", MOTION_INTERVAL);
}
