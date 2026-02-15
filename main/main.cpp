#include <cstring>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "rtsp_server.hpp" // C++ RTSP API
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
#include "config.h"
#include "ota.hpp"
#include "mdns.h"
#include "mqtt_client.h"
#include "img_converters.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef OTA_URL_DEFAULT
#define OTA_URL_DEFAULT "http://example.com/firmware.bin"
#endif

static const char *TAG = "RTSP_CAM";

static bool motion_active = false;
static uint32_t last_motion_time = 0;
const uint32_t MOTION_TIMEOUT = 3000; // 3 seconds
const uint32_t MOTION_ON_DELAY = 100; // 100 ms debounce
static uint32_t motion_start_candidate = 0;
static esp_mqtt_client_handle_t client;

QueueHandle_t motion_queue;

void mqtt_init()
{
    std::string uri = "mqtt://" + std::string(MQTT_HOST) + ":1883";
    std::string lwt_topic = std::string(MDNS_NAME) + "/status";

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = uri.c_str();
    mqtt_cfg.credentials.username = MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;

    mqtt_cfg.session.last_will.topic = lwt_topic.c_str();
    mqtt_cfg.session.last_will.msg = "offline";
    mqtt_cfg.session.last_will.qos = 1;
    mqtt_cfg.session.last_will.retain = true;

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    // Publish online status
    std::string online_topic = std::string(MDNS_NAME) + "/status";
    esp_mqtt_client_publish(client, online_topic.c_str(), "online", 0, 1, 1);
}

// ---------------- WIFI INIT (C++ SAFE) ----------------
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        auto *event = (ip_event_got_ip_t *)event_data;

        char ip_str[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));

        ESP_LOGI("WIFI", "IP Address: %s", ip_str);
        ESP_LOGI("RTSP_CAM", "Stream URL: rtsp://%s:8554/stream", ip_str);
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    // Register event handlers
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
    std::memset(&wifi_config, 0, sizeof(wifi_config));

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

    .xclk_freq_hz = 10000000, // 10 MHz
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 20,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = 0,
};

// ------------------------------------------------------

static void mdns_setup(void)
{
#ifdef MDNS_NAME
    if (strlen(MDNS_NAME) > 0)
    {
        esp_err_t err = mdns_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(TAG, "mdns_init failed: %d", err);
        }
        else
        {
            mdns_hostname_set(MDNS_NAME);
            mdns_instance_name_set("ESP32 RTSP Camera");
            mdns_service_add(NULL, "_rtsp._tcp", NULL, 8554, NULL, 0);
            ESP_LOGI(TAG, "mDNS started: %s.local", MDNS_NAME);
        }
    }
#endif
}

static bool detect_motion(const std::vector<uint8_t> &prev,
                          const std::vector<uint8_t> &curr,
                          int threshold,
                          float percent)
{
    int diff = 0;
    int total = curr.size();

    for (int i = 0; i < total; i++)
    {
        if (abs(curr[i] - prev[i]) > threshold)
        {
            diff++;
        }
    }

    return diff > total * percent;
}

struct GrayImage
{
    std::vector<uint8_t> data;
    int w = 0;
    int h = 0;
};

static const std::string motion_topic = std::string(MDNS_NAME) + "/motion";

static bool send_motion(bool motion)
{
    uint32_t now = esp_log_timestamp();
    bool changed = false;

    if (motion)
    {
        last_motion_time = now;

        if (!motion_active)
        {
            if (motion_start_candidate == 0)
            {
                motion_start_candidate = now; // first detection
            }

            if (now - motion_start_candidate > MOTION_ON_DELAY)
            {
                motion_active = true;
                changed = true;

                esp_mqtt_client_publish(client, motion_topic.c_str(), "1", 0, 1, 0);
            }
        }
    }
    else
    {
        motion_start_candidate = 0; // reset ON debounce

        if (motion_active && (now - last_motion_time > MOTION_TIMEOUT))
        {
            motion_active = false;
            changed = true;

            esp_mqtt_client_publish(client, motion_topic.c_str(), "0", 0, 1, 0);
        }
    }

    return changed;
}

void motion_task(void *arg)
{
    std::vector<uint8_t> *gray_ptr = nullptr;
    std::vector<uint8_t> prev;

    while (true)
    {
        if (xQueueReceive(motion_queue, &gray_ptr, portMAX_DELAY))
        {

            auto &gray = *gray_ptr;

            if (!prev.empty() && prev.size() == gray.size())
            {
                bool motion = detect_motion(prev, gray, 20, 0.05f);
                send_motion(motion);
            }

            prev = gray;

            delete gray_ptr; // free safely
        }
    }
}

void start_motion_task()
{
    motion_queue = xQueueCreate(1, sizeof(std::vector<uint8_t> *));
    xTaskCreatePinnedToCore(motion_task, "motion_task", 4096, nullptr, 5, nullptr, 1);
}
// RSTP SESSION HOTFIX
#include <fcntl.h>
#include <unistd.h>

void restart_camera()
{
    esp_camera_deinit();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_err_t err = esp_camera_init(&camera_config);

    if (err != ESP_OK)
    {
        ESP_LOGE("CAM", "Camera re-init failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI("CAM", "Camera re-initialized");
    }
}

static constexpr int MAX_POSSIBLE_FDS = 64;

int get_free_fd_count()
{
    int used = 0;

    for (int fd = 0; fd < MAX_POSSIBLE_FDS; fd++)
    {
        if (fcntl(fd, F_GETFD) != -1)
        {
            used++;
        }
    }

    return MAX_POSSIBLE_FDS - used;
}

volatile bool rtsp_error_detected = false;
static char rtsp_log_buffer[256]; // static = no stack usage


void camera_main_loop()
{
    // ---- Correct RTSP API ----
    espp::RtspServer::Config rtsp_cfg;
    rtsp_cfg.port = 8554;

    espp::RtspServer server{rtsp_cfg};
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second
    server.start();

    // --------------------------

    std::vector<uint8_t> rgb;
    std::vector<uint8_t> gray;

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
            continue;

        // --- RTSP send ---
        std::span<const uint8_t> jpeg_span(fb->buf, fb->len);
        server.send_frame(espp::JpegFrame(jpeg_span));

        // --- Resize buffers once ---
        size_t rgb_size = fb->width * fb->height * 3;
        size_t gray_size = fb->width * fb->height;

        if (rgb.size() != rgb_size)
            rgb.resize(rgb_size);
        if (gray.size() != gray_size)
            gray.resize(gray_size);

        // --- JPEG → RGB ---
        if (fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb.data()))
        {

            // --- RGB → grayscale ---
            for (int i = 0, j = 0; i < rgb_size; i += 3, j++)
            {
                uint8_t r = rgb[i];
                uint8_t g = rgb[i + 1];
                uint8_t b = rgb[i + 2];
                gray[j] = (r * 30 + g * 59 + b * 11) / 100;
            }

            // --- Send grayscale to motion task ---
            if (uxQueueMessagesWaiting(motion_queue) == 0)
            {
                auto *frame = new std::vector<uint8_t>(gray); // allocate on heap
                xQueueSend(motion_queue, &frame, 0);
            }
        }

        esp_camera_fb_return(fb);
        vTaskDelay(1);
    }
}

static void ota()
{
    httpd_handle_t server_handle = NULL;
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 8080;

    auto ota_http_handler = [](httpd_req_t *req) -> esp_err_t
    {
        char urlbuf[256] = {0};
        char qsbuf[256] = {0};

        // Optional: /ota?url=http://server/firmware.bin
        if (httpd_req_get_url_query_str(req, qsbuf, sizeof(qsbuf)) == ESP_OK)
        {
            char value[256];
            if (httpd_query_key_value(qsbuf, "url", value, sizeof(value)) == ESP_OK)
            {
                strncpy(urlbuf, value, sizeof(urlbuf) - 1);
            }
        }

        const char *chosen = urlbuf[0] ? urlbuf : OTA_URL_DEFAULT;

        if (start_ota_from_url(chosen) == ESP_OK)
        {
            httpd_resp_send(req, "OTA started\n", HTTPD_RESP_USE_STRLEN);
        }
        else
        {
            httpd_resp_send(req, "OTA failed\n", HTTPD_RESP_USE_STRLEN);
        }

        return ESP_OK;
    };

    if (httpd_start(&server_handle, &http_config) == ESP_OK)
    {
        httpd_uri_t ota_uri = {
            .uri = "/ota2",
            .method = HTTP_GET,
            .handler = ota_http_handler,
            .user_ctx = nullptr,
        };
        httpd_register_uri_handler(server_handle, &ota_uri);
    }
}

extern "C" void app_main(void)
{
    wifi_init();
    ota();
    // Initialize mDNS (if configured)
    mdns_setup();
    mqtt_init();

    ESP_LOGI(TAG, "Initialising camera...");
    if (esp_camera_init(&camera_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_aec2(s, 1);

    ESP_LOGI(TAG, "Warming up camera...");
    for (int i = 0; i < 10; i++)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb)
            esp_camera_fb_return(fb);
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Camera ready.");

    start_motion_task();
    camera_main_loop(); // never returns
}
