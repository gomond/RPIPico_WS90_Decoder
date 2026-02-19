#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"

#define WIFI_SSID       CONFIG_WS90_WIFI_SSID
#define WIFI_PASS       CONFIG_WS90_WIFI_PASS
#define MQTT_BROKER_URI CONFIG_WS90_MQTT_BROKER_URI
#define MQTT_USER       CONFIG_WS90_MQTT_USERNAME
#define MQTT_PASS       CONFIG_WS90_MQTT_PASSWORD
#define MQTT_STATE_TOPIC CONFIG_WS90_MQTT_STATE_TOPIC

#define PIN_MISO 16
#define PIN_MOSI 19
#define PIN_SCK  18
#define PIN_CS   17
#define PIN_RST  20

#define RFM69_REG_FIFO          0x00
#define RFM69_REG_OPMODE        0x01
#define RFM69_REG_DATAMODUL     0x02
#define RFM69_REG_BITRATEMSB    0x03
#define RFM69_REG_BITRATELSB    0x04
#define RFM69_REG_FDEVMSB       0x05
#define RFM69_REG_FDEVLSB       0x06
#define RFM69_REG_FRFMSB        0x07
#define RFM69_REG_FRFMID        0x08
#define RFM69_REG_FRFLSB        0x09
#define RFM69_REG_RXBW          0x19
#define RFM69_REG_AFCBW         0x1A
#define RFM69_REG_DIOMAPPING1   0x25
#define RFM69_REG_IRQFLAGS1     0x27
#define RFM69_REG_IRQFLAGS2     0x28
#define RFM69_REG_RSSITHRESH    0x29
#define RFM69_REG_SYNCCONFIG    0x2E
#define RFM69_REG_SYNCVALUE1    0x2F
#define RFM69_REG_SYNCVALUE2    0x30
#define RFM69_REG_SYNCVALUE3    0x31
#define RFM69_REG_SYNCVALUE4    0x32
#define RFM69_REG_PACKETCONFIG1 0x37
#define RFM69_REG_PAYLOADLENGTH 0x38
#define RFM69_REG_FIFOTHRESH    0x3C
#define RFM69_REG_PACKETCONFIG2 0x3D

#define RFM69_MODE_STDBY       0x04
#define RFM69_MODE_RX          0x10
#define RFM69_OPMODE_MASK      0x1C

#define RADIO_BITRATE_BPS      17241u
#define RADIO_FDEV_HZ          33500u
#define RADIO_CENTER_HZ        433920000u

#define WS90_FRAME_BYTES       32u
#define RFM_CAPTURE_BYTES      32u
#define WS90_EXPECTED_ID       0x00C0E4u
#define WS90_REQUIRE_EXPECTED_ID 1
#define HEARTBEAT_IDLE_MS      10000u

static const char *TAG = "WS90_MQTT";
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static spi_device_handle_t rfm_spi;
static esp_mqtt_client_handle_t mqtt_client;
static bool mqtt_connected = false;

static void mqtt_publish_discovery_sensor(const char *topic,
                                          const char *name,
                                          const char *unique_id,
                                          const char *unit,
                                          const char *device_class,
                                          const char *state_class,
                                          const char *value_template) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    char payload[768];
    snprintf(payload,
             sizeof(payload),
             "{"
             "\"name\":\"%s\","
             "\"uniq_id\":\"%s\","
             "\"stat_t\":\"%s\","
             "\"unit_of_meas\":\"%s\","
             "\"dev_cla\":\"%s\","
             "\"state_class\":\"%s\","
             "\"val_tpl\":\"%s\","
             "\"dev\":{"
             "\"ids\":[\"ws90_decoder\"],"
             "\"name\":\"WS90 Decoder\","
             "\"mdl\":\"ESP32_WS90_Decoder_MQTT\","
             "\"mf\":\"Custom\""
             "}"
             "}",
             name,
             unique_id,
             MQTT_STATE_TOPIC,
             unit,
             device_class,
             state_class,
             value_template);

    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
}

static inline esp_err_t rfm_write(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80u), value };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx;
    return spi_device_transmit(rfm_spi, &t);
}

static inline uint8_t rfm_read(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7Fu), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_transmit(rfm_spi, &t) != ESP_OK) {
        return 0;
    }
    return rx[1];
}

static void rfm_read_fifo(uint8_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buffer[i] = rfm_read(RFM69_REG_FIFO);
    }
}

static void rfm_set_mode(uint8_t mode) {
    uint8_t op = rfm_read(RFM69_REG_OPMODE);
    op = (uint8_t)((op & (uint8_t)~RFM69_OPMODE_MASK) | (mode & RFM69_OPMODE_MASK));
    rfm_write(RFM69_REG_OPMODE, op);

    for (uint32_t i = 0; i < 20; i++) {
        uint8_t irq1 = rfm_read(RFM69_REG_IRQFLAGS1);
        if (irq1 & 0x80u) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void rfm_write_bitrate(uint32_t bitrate_bps) {
    uint32_t bitrate_reg = (32000000u + (bitrate_bps / 2u)) / bitrate_bps;
    if (bitrate_reg > 0xFFFFu) {
        bitrate_reg = 0xFFFFu;
    }
    rfm_write(RFM69_REG_BITRATEMSB, (uint8_t)((bitrate_reg >> 8) & 0xFFu));
    rfm_write(RFM69_REG_BITRATELSB, (uint8_t)(bitrate_reg & 0xFFu));
}

static void rfm_write_frequency_hz(uint32_t freq_hz) {
    uint64_t frf = (((uint64_t)freq_hz) << 19) / 32000000ull;
    rfm_write(RFM69_REG_FRFMSB, (uint8_t)((frf >> 16) & 0xFFu));
    rfm_write(RFM69_REG_FRFMID, (uint8_t)((frf >> 8) & 0xFFu));
    rfm_write(RFM69_REG_FRFLSB, (uint8_t)(frf & 0xFFu));
}

static void rfm_set_sync_profile(uint8_t profile) {
    switch (profile) {
        case 0:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x99);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE3, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE4, 0xD4);
            break;
        case 1:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x91);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE3, 0xD4);
            break;
        default:
            rfm_write(RFM69_REG_SYNCCONFIG, 0x88);
            rfm_write(RFM69_REG_SYNCVALUE1, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xD4);
            break;
    }
}

static void rfm_reset(void) {
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void rfm_init(void) {
    rfm_set_mode(RFM69_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(5));

    rfm_write(RFM69_REG_DATAMODUL, 0x00);
    rfm_write_bitrate(RADIO_BITRATE_BPS);

    uint32_t fdev_reg = (RADIO_FDEV_HZ + 30u) / 61u;
    if (fdev_reg > 0x3FFFu) {
        fdev_reg = 0x3FFFu;
    }
    rfm_write(RFM69_REG_FDEVMSB, (uint8_t)((fdev_reg >> 8) & 0x3Fu));
    rfm_write(RFM69_REG_FDEVLSB, (uint8_t)(fdev_reg & 0xFFu));

    rfm_write_frequency_hz(RADIO_CENTER_HZ);
    rfm_write(RFM69_REG_RXBW, 0b10000010);
    rfm_write(RFM69_REG_AFCBW, 0b10000010);

    rfm_set_sync_profile(0);

    rfm_write(RFM69_REG_PACKETCONFIG1, 0x00);
    rfm_write(RFM69_REG_PAYLOADLENGTH, RFM_CAPTURE_BYTES);
    rfm_write(RFM69_REG_FIFOTHRESH, 0x8F);
    rfm_write(RFM69_REG_PACKETCONFIG2, 0x02);

    rfm_write(RFM69_REG_RSSITHRESH, 0xB4);
    rfm_write(RFM69_REG_DIOMAPPING1, 0x00);
    rfm_set_mode(RFM69_MODE_RX);
}

static uint8_t ws90_crc8(const uint8_t *data, size_t len, uint8_t poly, uint8_t init) {
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t ws90_add_bytes(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)sum;
}

static uint8_t ws90_reverse8(uint8_t x) {
    x = (uint8_t)(((x & 0xF0u) >> 4) | ((x & 0x0Fu) << 4));
    x = (uint8_t)(((x & 0xCCu) >> 2) | ((x & 0x33u) << 2));
    x = (uint8_t)(((x & 0xAAu) >> 1) | ((x & 0x55u) << 1));
    return x;
}

static void shift_left_bits_len(const uint8_t *in, uint8_t *out, uint32_t len, uint8_t bits) {
    if (bits == 0u) {
        memcpy(out, in, len);
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        uint8_t next = (i + 1u < len) ? in[i + 1u] : 0u;
        out[i] = (uint8_t)((in[i] << bits) | (next >> (8u - bits)));
    }
}

static void shift_right_bits_len(const uint8_t *in, uint8_t *out, uint32_t len, uint8_t bits) {
    if (bits == 0u) {
        memcpy(out, in, len);
        return;
    }
    for (int i = (int)len - 1; i >= 0; i--) {
        uint8_t prev = (i > 0) ? in[i - 1] : 0u;
        out[i] = (uint8_t)((in[i] >> bits) | (prev << (8u - bits)));
    }
}

static void mqtt_publish_discovery(void) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_temperature/config",
                                  "WS90 Temperature",
                                  "ws90_temp",
                                  "°C",
                                  "temperature",
                                  "measurement",
                                  "{{ value_json.temperature_c }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_humidity/config",
                                  "WS90 Humidity",
                                  "ws90_humidity",
                                  "%",
                                  "humidity",
                                  "measurement",
                                  "{{ value_json.humidity }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_wind_avg/config",
                                  "WS90 Wind Average",
                                  "ws90_wind_avg",
                                  "m/s",
                                  "wind_speed",
                                  "measurement",
                                  "{{ value_json.wind_avg_m_s }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_wind_max/config",
                                  "WS90 Wind Gust",
                                  "ws90_wind_max",
                                  "m/s",
                                  "wind_speed",
                                  "measurement",
                                  "{{ value_json.wind_max_m_s }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_wind_dir/config",
                                  "WS90 Wind Direction",
                                  "ws90_wind_dir",
                                  "°",
                                  "",
                                  "measurement",
                                  "{{ value_json.wind_dir_deg }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_rain/config",
                                  "WS90 Rain",
                                  "ws90_rain",
                                  "mm",
                                  "precipitation",
                                  "measurement",
                                  "{{ value_json.rain_mm }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_uv/config",
                                  "WS90 UV Index",
                                  "ws90_uv",
                                  "",
                                  "",
                                  "measurement",
                                  "{{ value_json.uv_index }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_light/config",
                                  "WS90 Illuminance",
                                  "ws90_light",
                                  "lx",
                                  "illuminance",
                                  "measurement",
                                  "{{ value_json.light_lux }}");

    mqtt_publish_discovery_sensor("homeassistant/sensor/ws90_battery/config",
                                  "WS90 Battery",
                                  "ws90_battery",
                                  "%",
                                  "battery",
                                  "measurement",
                                  "{{ (value_json.battery_level * 100) | round(0) }}");
}

static void mqtt_publish_state(const char *json) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return;
    }
    esp_mqtt_client_publish(mqtt_client, MQTT_STATE_TOPIC, json, 0, 1, 0);
}

static bool ws90_decode_and_publish(const uint8_t *b) {
    if (b[0] != 0x90u) {
        return false;
    }

    int id = (b[1] << 16) | (b[2] << 8) | b[3];
#if WS90_REQUIRE_EXPECTED_ID
    if (id != (int)WS90_EXPECTED_ID) {
        return false;
    }
#endif

    uint8_t crc = ws90_crc8(b, 31, 0x31, 0x00);
    uint8_t chk = ws90_add_bytes(b, 31);
    if ((crc != 0u) || (chk != b[31])) {
        return false;
    }

    int light_raw = (b[4] << 8) | b[5];
    float light_lux = light_raw * 10.0f;
    int battery_mv = b[6] * 20;
    int battery_lvl = (battery_mv < 1400) ? 0 : ((battery_mv - 1400) / 16);
    if (battery_lvl > 100) {
        battery_lvl = 100;
    }
    int flags = b[7];
    int temp_raw = ((b[7] & 0x03) << 8) | b[8];
    float temp_c = (temp_raw - 400) * 0.1f;
    int humidity = b[9];
    int wind_avg = ((b[7] & 0x10) << 4) | b[10];
    int wind_dir = ((b[7] & 0x20) << 3) | b[11];
    int wind_max = ((b[7] & 0x40) << 2) | b[12];
    int uv_index = b[13];
    int rain_raw = (b[19] << 8) | b[20];
    int rain_start = (b[16] & 0x10) >> 4;
    int supercap_v = (b[21] & 0x3f);
    int firmware = b[29];

    char extra[31];
    snprintf(extra, sizeof(extra), "%02x%02x%02x%02x%02x------%02x%02x%02x%02x%02x%02x%02x",
             b[14], b[15], b[16], b[17], b[18], b[22], b[23], b[24], b[25], b[26], b[27], b[28]);

    char json[512];
    snprintf(json, sizeof(json),
             "{\"model\":\"Fineoffset-WS90\",\"id\":\"%06X\",\"battery_level\":%.3f,\"battery_mv\":%d,\"temperature_c\":%.1f,\"humidity\":%d,\"wind_dir_deg\":%d,\"wind_avg_m_s\":%.1f,\"wind_max_m_s\":%.1f,\"uv_index\":%.1f,\"light_lux\":%.1f,\"flags\":\"%02x\",\"rain_mm\":%.1f,\"rain_start\":%d,\"supercap_v\":%.1f,\"firmware\":%d,\"extra\":\"%s\",\"mic\":\"CRC\"}",
             id,
             (double)(battery_lvl * 0.01f),
             battery_mv,
             (double)temp_c,
             humidity,
             wind_dir,
             (double)(wind_avg * 0.1f),
             (double)(wind_max * 0.1f),
             (double)(uv_index * 0.1f),
             (double)light_lux,
             flags,
             (double)(rain_raw * 0.1f),
             rain_start,
             (double)(supercap_v * 0.1f),
             firmware,
             extra);

    printf("%s\n", json);
    mqtt_publish_state(json);
    return true;
}

static bool ws90_decode_with_alignment(const uint8_t *raw) {
    uint8_t base[WS90_FRAME_BYTES];
    uint8_t shifted[WS90_FRAME_BYTES];
    uint8_t rotated[WS90_FRAME_BYTES];

    if (ws90_decode_and_publish(raw)) {
        return true;
    }

    for (uint32_t mode = 0; mode < 4u; mode++) {
        for (uint32_t i = 0; i < WS90_FRAME_BYTES; i++) {
            uint8_t v = raw[i];
            if (mode == 1u) {
                v = (uint8_t)~v;
            } else if (mode == 2u) {
                v = ws90_reverse8(v);
            } else if (mode == 3u) {
                v = (uint8_t)~ws90_reverse8(v);
            }
            base[i] = v;
        }

        if (ws90_decode_and_publish(base)) {
            return true;
        }

        for (uint8_t off = 1u; off < WS90_FRAME_BYTES; off++) {
            for (uint8_t i = 0u; i < WS90_FRAME_BYTES; i++) {
                rotated[i] = base[(uint8_t)((off + i) % WS90_FRAME_BYTES)];
            }
            if (ws90_decode_and_publish(rotated)) {
                return true;
            }
        }

        for (uint8_t shift = 1u; shift <= 7u; shift++) {
            shift_left_bits_len(base, shifted, WS90_FRAME_BYTES, shift);
            if (ws90_decode_and_publish(shifted)) {
                return true;
            }

            shift_right_bits_len(base, shifted, WS90_FRAME_BYTES, shift);
            if (ws90_decode_and_publish(shifted)) {
                return true;
            }
        }
    }

    return false;
}

static bool handle_capture_bytes(const uint8_t *capture, uint32_t capture_len) {
    uint8_t transformed[RFM_CAPTURE_BYTES];
    uint8_t shifted[RFM_CAPTURE_BYTES];

    if (capture_len < WS90_FRAME_BYTES) {
        return false;
    }

    for (uint32_t mode = 0; mode < 4u; mode++) {
        for (uint32_t i = 0; i < capture_len; i++) {
            uint8_t v = capture[i];
            if (mode == 1u) {
                v = (uint8_t)~v;
            } else if (mode == 2u) {
                v = ws90_reverse8(v);
            } else if (mode == 3u) {
                v = (uint8_t)~ws90_reverse8(v);
            }
            transformed[i] = v;
        }

        for (uint8_t bit_shift = 0u; bit_shift <= 7u; bit_shift++) {
            shift_left_bits_len(transformed, shifted, capture_len, bit_shift);
            if (ws90_decode_with_alignment(shifted)) {
                return true;
            }

            if (bit_shift != 0u) {
                shift_right_bits_len(transformed, shifted, capture_len, bit_shift);
                if (ws90_decode_with_alignment(shifted)) {
                    return true;
                }
            }
        }
    }

    return ws90_decode_with_alignment(capture);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected");
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_publish_discovery();
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        default:
            break;
    }
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    if (strlen(MQTT_USER) > 0) {
        mqtt_cfg.credentials.username = MQTT_USER;
        mqtt_cfg.credentials.authentication.password = MQTT_PASS;
    }

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

static void rfm_spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &rfm_spi));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void ws90_radio_task(void *arg) {
    (void)arg;

    rfm_spi_init();
    rfm_reset();
    rfm_init();

    uint8_t opmode = rfm_read(RFM69_REG_OPMODE);
    uint8_t datamodul = rfm_read(RFM69_REG_DATAMODUL);
    uint8_t syncconfig = rfm_read(RFM69_REG_SYNCCONFIG);

    ESP_LOGI(TAG,
             "WS90 RX packet-mode start: bitrate=%u fdev=%u opmode=0x%02X datamodul=0x%02X sync=0x%02X",
             RADIO_BITRATE_BPS,
             RADIO_FDEV_HZ,
             opmode,
             datamodul,
             syncconfig);

    uint8_t frame[RFM_CAPTURE_BYTES];
    uint32_t heartbeat_ms = (uint32_t)esp_log_timestamp();
    uint32_t last_activity_ms = heartbeat_ms;

    while (true) {
        uint8_t opm = rfm_read(RFM69_REG_OPMODE);
        if ((opm & 0x1Cu) != RFM69_MODE_RX) {
            rfm_set_mode(RFM69_MODE_RX);
        }

        uint8_t irq2 = rfm_read(RFM69_REG_IRQFLAGS2);
        if (irq2 & 0x04u) {
            rfm_read_fifo(frame, RFM_CAPTURE_BYTES);
            bool decoded = handle_capture_bytes(frame, RFM_CAPTURE_BYTES);
            if (decoded) {
                last_activity_ms = (uint32_t)esp_log_timestamp();
            }
        } else {
            uint32_t now_ms = (uint32_t)esp_log_timestamp();
            if (((now_ms - heartbeat_ms) >= 1000u) && ((now_ms - last_activity_ms) >= HEARTBEAT_IDLE_MS)) {
                ESP_LOGI(TAG, "alive: waiting packets");
                heartbeat_ms = now_ms;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASS) == 0) {
        ESP_LOGE(TAG, "Wi-Fi credentials missing. Set WS90_WIFI_SSID and WS90_WIFI_PASS in menuconfig.");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (strlen(MQTT_BROKER_URI) == 0) {
        ESP_LOGE(TAG, "MQTT broker URI missing. Set WS90_MQTT_BROKER_URI in menuconfig.");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    mqtt_start();

    xTaskCreatePinnedToCore(ws90_radio_task, "ws90_radio", 8192, NULL, 5, NULL, 1);
}
