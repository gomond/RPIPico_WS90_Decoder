# ESP32_WS90_Decoder_MQTT (ESP-IDF)

ESP32 packet-mode WS90 decoder with MQTT publishing for Home Assistant.

## Status

- Packet-mode RFM69 receive path ported.
- WS90 decode and JSON formatting ported.
- Wi-Fi and MQTT client integration added.
- Raw PIO fallback is not ported yet.

## Build

1. Install ESP-IDF (v5.x recommended).
2. In this folder, run:
   - `idf.py set-target esp32s3`
   - `idf.py menuconfig` (set Wi-Fi/MQTT values if needed)
   - `idf.py build flash monitor`

## Configuration in app_main.c

- `WIFI_SSID`
- `WIFI_PASS`
- `MQTT_BROKER_URI`
- GPIO pin constants for SPI + RFM69 reset/IRQ lines
