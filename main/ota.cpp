#include "ota.hpp"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG_OTA = "OTA";

static void ota_worker(void *arg)
{
    char *url = (char*)arg; // heap-allocated string or nullptr
    if (!url) url = (char*)"";
    ESP_LOGI(TAG_OTA, "Starting OTA from %s", url);

    esp_http_client_config_t config{};
    config.url = url;
    config.timeout_ms = 60000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG_OTA, "Failed to initialise HTTP client");
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "Failed to open HTTP connection");
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    ESP_LOGI(TAG_OTA, "Content length: %d", content_length);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG_OTA, "No update partition found");
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    esp_ota_handle_t ota_handle = 0;
    size_t image_size = (content_length > 0) ? (size_t)content_length : 0xFFFFFFFF;
    if (esp_ota_begin(update_partition, image_size, &ota_handle) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_begin failed");
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    const int BUFSIZE = 1024;
    char *buffer = (char*)malloc(BUFSIZE);
    if (!buffer) {
        ESP_LOGE(TAG_OTA, "malloc failed");
        esp_ota_end(ota_handle);
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    int data_read = 0;
    while ((data_read = esp_http_client_read(client, buffer, BUFSIZE)) > 0) {
        if (esp_ota_write(ota_handle, (const void*)buffer, data_read) != ESP_OK) {
            ESP_LOGE(TAG_OTA, "esp_ota_write failed");
            free(buffer);
            esp_ota_end(ota_handle);
            esp_http_client_cleanup(client);
            if (arg) free(arg);
            vTaskDelete(NULL);
            return;
        }
    }

    free(buffer);

    if (esp_ota_end(ota_handle) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_end failed");
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_set_boot_partition failed");
        esp_http_client_cleanup(client);
        if (arg) free(arg);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_OTA, "Update successful, restarting...");
    esp_http_client_cleanup(client);
    if (arg) free(arg);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

esp_err_t start_ota_from_url(const char *url)
{
    char *arg = nullptr;
    if (url && url[0]) {
        size_t l = strlen(url) + 1;
        arg = (char*)malloc(l);
        if (!arg) return ESP_ERR_NO_MEM;
        memcpy(arg, url, l);
    }

    BaseType_t ok = xTaskCreate(ota_worker, "ota_worker", 8192, arg, 5, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}
