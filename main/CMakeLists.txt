idf_component_register(SRCS "main.cpp" "ota.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES mdns
                    PRIV_REQUIRES esp_wifi esp_netif nvs_flash esp_event app_update esp_http_client esp_http_server)
