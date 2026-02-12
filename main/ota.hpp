#pragma once
#include "esp_err.h"

// Start OTA from given URL. Returns ESP_OK on task creation success.
esp_err_t start_ota_from_url(const char *url);
