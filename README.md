# esp32rtsp

Simple ESP32 RTSP camera project using espp components.

Quick start
- Copy `main/wifi_config.h.example` to `main/wifi_config.h` and fill your SSID/password.
- Build and flash with ESP-IDF (from project root):

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Notes
- `main/wifi_config.h` is listed in `.gitignore` — do not commit credentials.
- The RTSP server listens on port `8554` and serves `/stream`.
