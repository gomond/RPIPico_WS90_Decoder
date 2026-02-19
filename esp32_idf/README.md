# ESP32_WS90_Decoder_MQTT (ESP-IDF)

ESP32 packet-mode WS90 decoder with MQTT publishing for Home Assistant.

## Status

- Packet-mode RFM69 receive path ported.
- WS90 decode and JSON formatting ported.
- Wi-Fi and MQTT client integration added.
- Home Assistant MQTT discovery expanded for key WS90 metrics.
- Raw PIO fallback is not ported yet.

## Build

1. Install ESP-IDF (v5.x recommended).
2. In this folder, run:
   - `idf.py set-target esp32s3`
   - `idf.py menuconfig` (set Wi-Fi/MQTT values)
   - `idf.py build flash monitor`

## Configuration (`idf.py menuconfig`)

Under `WS90 MQTT Configuration`:

- `WS90_WIFI_SSID`
- `WS90_WIFI_PASS`
- `WS90_MQTT_BROKER_URI`
- `WS90_MQTT_USERNAME` (optional)
- `WS90_MQTT_PASSWORD` (optional)
- `WS90_MQTT_STATE_TOPIC`

GPIO pin constants for SPI + RFM69 reset/IRQ lines are still in `main/app_main.c`.
