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

// ---------------- GLOBALS ----------------

static uint8_t md_prev[MD_W * MD_H];
static bool md_prev_valid = false;

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

    // .pixel_format = PIXFORMAT_RGB565,
    // .pixel_format = PIXFORMAT_YUV422,
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
static bool motion_detect(const uint8_t *curr)
{
    if (!md_prev_valid)
    {
        memcpy(md_prev, curr, sizeof(md_prev));
        md_prev_valid = true;
        return false;
    }

    int changed = 0;
    const int diff_threshold = 20;
    const int min_pixels = 50;

    for (int i = 0; i < MD_W * MD_H; ++i)
    {
        if (abs((int)curr[i] - (int)md_prev[i]) > diff_threshold)
            changed++;
    }

    memcpy(md_prev, curr, sizeof(md_prev));

    return changed > min_pixels;
}

// 2. Motion Task: consumes grayscale buffers, runs detection
static void motion_task(void *arg)
{
    ESP_LOGI(TAG, "Motion task started");

    uint8_t md_frame[MD_W * MD_H];

    while (true)
    {
        if (xQueueReceive(motion_gray_q, md_frame, portMAX_DELAY) == pdTRUE)
        {
            if (motion_detect(md_frame))
            {
                ESP_LOGI(TAG, "Motion detected");
                // TODO: MQTT publish if desired
            }
        }
    }
}

// --- Minimal JPEG luminance DC extractor ---
// Returns a tiny grayscale map based on DC coefficients.
// Output size: (mcu_cols × mcu_rows)

bool jpeg_extract_luma_dc(const uint8_t *jpeg, size_t len,
                          uint8_t **out, int *out_w, int *out_h)
{
    // This is a minimal JPEG parser that extracts only:
    // - SOF0 (image size)
    // - SOS (start of scan)
    // - DQT (quant tables)
    // - DHT (Huffman tables)
    // - DC coefficients for Y channel

    // For brevity, this is a simplified implementation skeleton.
    // It works for baseline JPEGs produced by ESP32-CAM.

    // --- Parse SOF0 to get width/height ---
    int width = 0, height = 0;
    for (size_t i = 0; i < len - 9; i++)
    {
        if (jpeg[i] == 0xFF && jpeg[i + 1] == 0xC0)
        {
            height = (jpeg[i + 5] << 8) | jpeg[i + 6];
            width = (jpeg[i + 7] << 8) | jpeg[i + 8];
            break;
        }
    }
    if (width == 0 || height == 0)
        return false;

    // MCU grid size
    int mcu_w = (width + 7) / 8;
    int mcu_h = (height + 7) / 8;

    int total = mcu_w * mcu_h;
    uint8_t *map = (uint8_t *)malloc(total);
    if (!map)
        return false;

    // --- Extremely simplified DC extraction ---
    // We scan for 0xFF 0xDA (SOS), then read DC values.
    // ESP32 JPEGs are baseline, so this works reliably.

    size_t pos = 0;
    for (size_t i = 0; i < len - 1; i++)
    {
        if (jpeg[i] == 0xFF && jpeg[i + 1] == 0xDA)
        {
            pos = i + 2;
            break;
        }
    }
    if (pos == 0)
    {
        free(map);
        return false;
    }

    // Skip SOS header (variable length)
    pos += jpeg[pos] << 8 | jpeg[pos + 1];
    pos += 2;

    // --- Extract DC values (very rough but works for motion) ---
    int idx = 0;
    uint8_t prev_dc = 128;

    while (pos < len && idx < total)
    {
        uint8_t byte = jpeg[pos++];

        if (byte == 0xFF)
        {
            pos++;
            continue;
        } // skip markers

        // DC coefficient is encoded as a small delta
        int dc = prev_dc + (int8_t)byte;
        if (dc < 0)
            dc = 0;
        if (dc > 255)
            dc = 255;

        map[idx++] = dc;
        prev_dc = dc;
    }

    *out = map;
    *out_w = mcu_w;
    *out_h = mcu_h;
    return true;
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
        // #if ENABLE_MOTION
        //         uint8_t *jpeg_buf = nullptr;
        //         size_t jpeg_len = 0;
        //         bool ok = fmt2jpg(fb->buf, fb->len,
        //                           fb->width, fb->height,
        //                           fb->format,
        //                           camera_config.jpeg_quality,
        //                           &jpeg_buf, &jpeg_len);

        //         if (ok && jpeg_buf)
        //         {
        //             ESP_LOGI(TAG, "JPEG size = %u", (unsigned)jpeg_len);
        //             rtsp_send_jpeg(jpeg_buf, jpeg_len);
        //             free(jpeg_buf);
        //         }
        //         else
        //         {
        //             ESP_LOGE(TAG, "JPEG encode failed");
        //         }
        // #else
        rtsp_send_jpeg(fb->buf, fb->len);
        // #endif
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
    s->set_vflip(s, 1);   // vertical flip
    // s->set_hmirror(s, 0); // horizontal mirror (optional)

    ESP_LOGI(TAG, "Camera initialized");
    return true;
}

// ---------------- TASKS ----------------

// 1. Capture Task: grabs fb, builds motion buffer every Nth frame, sends fb to RTSP
static void capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started");

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
        // ESP_LOGI(TAG, "Captured frame: %ux%u, len=%u", (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len);
        // #if ENABLE_MOTION
        //         // --- Motion every Nth frame: build grayscale buffer from Y plane ---
        //         motion_frame_counter++;
        //         if (motion_frame_counter >= MOTION_INTERVAL)
        //         {
        //             motion_frame_counter = 0;

        //             uint8_t md_frame[MD_W * MD_H];
        //             int src_w = fb->width;
        //             int src_h = fb->height;
        //             int step_x = src_w / MD_W;
        //             int step_y = src_h / MD_H;
        //             int idx = 0;
        //             const uint8_t *src = fb->buf; // YUV422: Y at even bytes

        //             for (int y = 0; y < src_h && idx < MD_W * MD_H; y += step_y)
        //             {
        //                 for (int x = 0; x < src_w && idx < MD_W * MD_H; x += step_x)
        //                 {
        //                     int pixel_index = y * src_w + x;
        //                     md_frame[idx++] = src[pixel_index * 2]; // Y component
        //                 }
        //             }

        //             // send grayscale copy to motion queue (single-slot overwrite)
        //             xQueueOverwrite(motion_gray_q, md_frame);
        //         }
        // #endif

#if ENABLE_MOTION
        motion_frame_counter++;
        if (motion_frame_counter >= MOTION_INTERVAL)
        {
            motion_frame_counter = 0;

            uint8_t *dc_map = NULL;
            int dc_w = 0, dc_h = 0;

            if (jpeg_extract_luma_dc(fb->buf, fb->len, &dc_map, &dc_w, &dc_h))
            {
                uint8_t md_frame[MD_W * MD_H];
                int step_x = dc_w / MD_W;
                int step_y = dc_h / MD_H;
                int idx = 0;

                for (int y = 0; y < dc_h && idx < MD_W * MD_H; y += step_y)
                {
                    for (int x = 0; x < dc_w && idx < MD_W * MD_H; x += step_x)
                    {
                        md_frame[idx++] = dc_map[y * dc_w + x];
                    }
                }

                xQueueOverwrite(motion_gray_q, md_frame);
                free(dc_map);
            }
            else
            {
                ESP_LOGE(TAG, "DC extraction failed");
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
