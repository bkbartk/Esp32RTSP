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
#include "config.h"
#include "rtsp_server.hpp"

#include "img_converters.h"

static const char *TAG = "PIPE";

// ---------------- CONFIG ----------------

#define MD_W 40
#define MD_H 30
#define FBQ_LEN 4
#define MOTIONQ_LEN 1
#define MOTION_INTERVAL 5

static bool wifi_got_ip = false;

// ---------------- CAMERA CONFIG (YUV422 + SVGA) ----------------

static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 15,

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
    .pin_href = 7,
    .pin_pclk = 13,

    .xclk_freq_hz = 20000000,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA,
    .jpeg_quality = 10,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
    .sccb_i2c_port = 0,
};

// ---------------- MOTION DETECTION ----------------
#if ENABLE_MOTION
static QueueHandle_t motion_gray_q = nullptr; // uint8_t[MD_W*MD_H]
static uint8_t md_prev[MD_W * MD_H];
static bool md_prev_valid = false;
// Tuned thresholds for JPEG-based grayscale

static uint8_t *rgb_buf = NULL;
static size_t rgb_buf_size = 0;

void init_motion()
{
    int max_w = 800;
    int max_h = 600;

    rgb_buf_size = max_w * max_h * 2; // RGB565

    rgb_buf = (uint8_t *)heap_caps_malloc(
        rgb_buf_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!rgb_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate rgb_buf (%u bytes)", rgb_buf_size);
    }
}

const int diff_threshold = 15; // ignore small changes
const int min_pixels = 150;    // require meaningful motion
static bool motion_detect(const uint8_t *curr)
{
    if (!md_prev_valid)
    {
        memcpy(md_prev, curr, sizeof(md_prev));
        md_prev_valid = true;
        return false;
    }

    int changed = 0;

    for (int i = 0; i < MD_W * MD_H; ++i)
    {
        int graychange = abs((int)curr[i] - (int)md_prev[i]);
        if (graychange > diff_threshold)
            changed++;
    }

    // Smooth update
    for (int i = 0; i < MD_W * MD_H; i++)
        md_prev[i] = (md_prev[i] * 3 + curr[i]) / 4;

    return changed > min_pixels;
}

// 2. Motion Task: consumes grayscale buffers, runs detection
static void motion_task(void *arg)
{
    ESP_LOGI(TAG, "Motion task started");

    uint8_t md_frame[MD_W * MD_H];

    bool motion_state = false; // current ON/OFF state
    uint32_t last_motion_time = 0;
    const uint32_t MOTION_TIMEOUT_MS = 5000; // 5 seconds

    while (true)
    {
        if (xQueueReceive(motion_gray_q, md_frame, portMAX_DELAY) == pdTRUE)
        {
            bool moving = motion_detect(md_frame);
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (moving)
            {
                last_motion_time = now;

                if (!motion_state)
                {
                    motion_state = true;
                    ESP_LOGI(TAG, "Motion ON");
                    // TODO: MQTT publish ON
                }
            }
            else
            {
                if (motion_state && (now - last_motion_time > MOTION_TIMEOUT_MS))
                {
                    motion_state = false;
                    ESP_LOGI(TAG, "Motion OFF");
                    // TODO: MQTT publish OFF
                }
            }
        }
    }
}

#endif
// ---------------- RTSP SEND ----------------
#if ENABLE_RTSP
static std::shared_ptr<espp::RtspServer> rtsp_server;
static bool rtsp_started = false;
static QueueHandle_t fbq_rtsp = nullptr; // camera_fb_t*
static void rtsp_send_jpeg(const uint8_t *jpeg_buf, size_t jpeg_len)
{
    if (!rtsp_server)
        return;

    std::span<const uint8_t> jpeg_span(jpeg_buf, jpeg_len);
    rtsp_server->send_frame(espp::JpegFrame(jpeg_span));
}

// 3. RTSP Task: highest priority, only JPEG + send + fb_return
static void rtsp_task(void *arg)
{
    ESP_LOGI(TAG, "RTSP task started");

    while (!wifi_got_ip || !rtsp_started)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (true)
    {
        camera_fb_t *fb = nullptr;
        if (xQueueReceive(fbq_rtsp, &fb, portMAX_DELAY) != pdTRUE)
            continue;
        rtsp_send_jpeg(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        vTaskDelay(1); // yield to avoid starving idle/WDT
    }
}

#endif
// ---------------- WIFI ----------------

static void wifi_event_handler(void *, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        auto *event = (ip_event_got_ip_t *)data;

        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));

        ESP_LOGI("WIFI", "IP Address: %s", ip_str);
        wifi_got_ip = true;
#if ENABLE_RTSP
        if (rtsp_server && !rtsp_started)
        {
            rtsp_server->start();
            rtsp_started = true;
            ESP_LOGI(TAG, "RTSP server started: rtsp://%s:8554", ip_str);
        }
#endif
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

    // optional: disable power save for smoother streaming
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI("WIFI", "Connecting to WiFi...");
}

// ---------------- CAMERA INIT ----------------

static bool camera_init_safe()
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1); // vertical flip
    // s->set_hmirror(s, 0); // horizontal mirror (optional)

    ESP_LOGI(TAG, "Camera initialized");
    return true;
}

// ---------------- TASKS ----------------

static void capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started");
    init_motion();

    int motion_frame_counter = 0;

    while (true)
    {
        // uint32_t t0 = esp_timer_get_time() / 1000;
        camera_fb_t *fb = esp_camera_fb_get();
        // uint32_t t1 = esp_timer_get_time() / 1000;
        // ESP_LOGI(TAG, "fb_get took %u ms", t1 - t0);

        if (!fb)
        {
            ESP_LOGI(TAG, "Capture Failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Skip invalid JPEG frames
        if (fb->len < 4 ||
            fb->buf[0] != 0xFF || fb->buf[1] != 0xD8 ||                   // SOI
            fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9) // EOI
        {
            ESP_LOGI(TAG, "Ivalid JPEG frame, dropping");
            esp_camera_fb_return(fb);
            continue;
        }

#if ENABLE_MOTION
        motion_frame_counter++;
        if (!rgb_buf)
        {
            ESP_LOGE(TAG, "rgb_buf is NULL, skipping motion decode");
        }
        else if (motion_frame_counter >= MOTION_INTERVAL)
        {
            motion_frame_counter = 0;

            esp_jpeg_image_cfg_t jpeg_cfg = {
                .indata = fb->buf,
                .indata_size = fb->len,
                .outbuf = rgb_buf,
                .outbuf_size = rgb_buf_size,
                .out_format = JPEG_IMAGE_FORMAT_RGB565,
            };

            esp_jpeg_image_output_t out;
            esp_err_t err = esp_jpeg_decode(&jpeg_cfg, &out);

            if (err == ESP_OK)
            {
                static uint8_t *gray = NULL;
                static size_t gray_size = 0;

                if (!gray)
                {
                    gray_size = out.width * out.height;
                    gray = (uint8_t *)heap_caps_malloc(gray_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                }
                // Convert RGB565 → grayscale
                for (int i = 0; i < out.width * out.height; i++)
                {
                    uint16_t px = ((uint16_t *)rgb_buf)[i];

                    int r = (px >> 11) & 0x1F;
                    int g = (px >> 5) & 0x3F;
                    int b = px & 0x1F;

                    // Expand to 8-bit
                    r = (r * 255) / 31;
                    g = (g * 255) / 63;
                    b = (b * 255) / 31;

                    gray[i] = (r * 30 + g * 59 + b * 11) / 100;
                }

                uint8_t md_frame[MD_W * MD_H];
                int step_x = out.width / MD_W;
                int step_y = out.height / MD_H;

                int idx = 0;
                for (int y = 0; y < out.height && idx < MD_W * MD_H; y += step_y)
                {
                    for (int x = 0; x < out.width && idx < MD_W * MD_H; x += step_x)
                    {
                        md_frame[idx++] = gray[y * out.width + x];
                    }
                }

                // Send downsampled frame
                xQueueOverwrite(motion_gray_q, md_frame);
            }
            else
            {
                ESP_LOGE(TAG, "JPEG decode failed: %s", esp_err_to_name(err));
            }
        }
#endif

#if ENABLE_RTSP
        // --- Send fb pointer to RTSP queue ---
        if (xQueueSend(fbq_rtsp, &fb, 0) != pdTRUE)
        {
            // if RTSP queue is full, drop frame
            esp_camera_fb_return(fb);
        }
#else
        esp_camera_fb_return(fb);
#endif
    }
}

// ---------------- MAIN ----------------

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();
    if (!camera_init_safe())
    {
        ESP_LOGE(TAG, "Camera not available, stopping");
        return;
    }

    // Priorities: RTSP highest
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, nullptr, 4, nullptr, 0);
#if ENABLE_MOTION
    motion_gray_q = xQueueCreate(MOTIONQ_LEN, MD_W * MD_H);
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, nullptr, 3, nullptr, 1);
#endif
#if ENABLE_RTSP
    espp::RtspServer::Config cfg;
    cfg.port = 8554;
    rtsp_server = std::make_shared<espp::RtspServer>(cfg);
    fbq_rtsp = xQueueCreate(FBQ_LEN, sizeof(camera_fb_t *));
    xTaskCreatePinnedToCore(rtsp_task, "rtsp", 8192, nullptr, 6, nullptr, 1);
#endif
    ESP_LOGI(TAG, "Pipeline running (YUV422 motion + JPEG RTSP, motion every %d frames)", MOTION_INTERVAL);
}
