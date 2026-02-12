# esp32rtsp

Simple ESP32 RTSP camera project using espp components.

Quick start
- Copy `main/config.h.example` to `main/config.h` and fill your SSID/password and `OTA_URL_DEFAULT`.
- Build and flash with ESP-IDF (from project root):
- Build and flash with ESP-IDF (from project root):

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Notes
- `main/config.h` is listed in `.gitignore` — do not commit credentials.
- The RTSP server listens on port `8554` and serves `/stream`.

OTA (Over-The-Air)
- Trigger OTA by visiting: `http://<device-ip>:8080/ota`
- Optionally provide a URL: `http://<device-ip>:8080/ota?url=http://host/firmware.bin`
- Hold the onboard OTA button (~300ms) to trigger OTA using `OTA_URL_DEFAULT`.
- The default fallback OTA URL is defined in `main/config.h.example` as `OTA_URL_DEFAULT`.

Board & Button
- Tested board: CH340 Dual Type C ESP32-S3-CAM Development Board (OV3660, onboard ESP32-S3-N16R8).
- Many ESP32-CAM-style boards expose a boot/flash button on `GPIO0` (active-low). On this board `GPIO0` is commonly used as the boot button, but verify your board's silkscreen or schematic before relying on it.

Security
- This OTA implementation allows insecure HTTPS (no certificate verification) for local/LAN updates. For production, enable certificate verification and signed firmware.
