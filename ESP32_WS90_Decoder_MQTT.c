#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "raw_capture.pio.h"

/*
 * WS90 decoder for Raspberry Pi Pico + RFM69.
 *
 * Big-picture flow:
 * 1) Configure RFM69 in packet mode and wait for payload-ready events.
 * 2) Try to decode each 32-byte capture using WS90 framing, ID checks, CRC, and checksum.
 * 3) If packet mode keeps failing, fall back to raw mode + PIO sampling.
 * 4) In raw mode, reconstruct bits from oversampled input and periodically retune.
 */

#define PIN_MISO 16
#define PIN_MOSI 19
#define PIN_SCK  18
#define PIN_CS   17
#define PIN_RST  20
#define PIN_DIO2 22

#define RFM69_REG_FIFO         0x00
#define RFM69_REG_OPMODE       0x01
#define RFM69_REG_DATAMODUL    0x02
#define RFM69_REG_BITRATEMSB   0x03
#define RFM69_REG_BITRATELSB   0x04
#define RFM69_REG_FDEVMSB      0x05
#define RFM69_REG_FDEVLSB      0x06
#define RFM69_REG_FRFMSB       0x07
#define RFM69_REG_FRFMID       0x08
#define RFM69_REG_FRFLSB       0x09
#define RFM69_REG_RXBW         0x19
#define RFM69_REG_AFCBW        0x1A
#define RFM69_REG_DIOMAPPING1  0x25
#define RFM69_REG_IRQFLAGS1    0x27
#define RFM69_REG_IRQFLAGS2    0x28
#define RFM69_REG_RSSITHRESH   0x29
#define RFM69_REG_SYNCCONFIG   0x2E
#define RFM69_REG_SYNCVALUE1   0x2F
#define RFM69_REG_SYNCVALUE2   0x30
#define RFM69_REG_SYNCVALUE3   0x31
#define RFM69_REG_SYNCVALUE4   0x32
#define RFM69_REG_PACKETCONFIG1 0x37
#define RFM69_REG_PAYLOADLENGTH 0x38
#define RFM69_REG_FIFOTHRESH   0x3C
#define RFM69_REG_PACKETCONFIG2 0x3D

#define RFM69_MODE_STDBY       0x04
#define RFM69_MODE_RX          0x10
#define RFM69_OPMODE_MASK      0x1C

#define RADIO_BITRATE_BPS      17241u
#define RADIO_FDEV_HZ          33500u
#define RADIO_CENTER_HZ        433920000u
#define OVERSAMPLE_FACTOR      8u
#define SAMPLE_RATE_HZ         (RADIO_BITRATE_BPS * OVERSAMPLE_FACTOR)

#define STREAM_RAW_WORDS       0
#define STREAM_STATUS          0
#define STREAM_INVALID_FRAMES  1
#define WS90_PREAMBLE_MIN_BITS 10u
#define WS90_SYNC_WORD         0x2DD4u
#define WS90_SYNC_BITS         16u
#define WS90_PREAMBLE_SYNC_32  0xAAAA2DD4u
#define WS90_SYNC_SEARCH_WINDOW_BITS 512u
#define WS90_FRAME_BYTES       32u
#define RFM_CAPTURE_BYTES      32u
#define WS90_EXPECTED_ID       0x00C0E4u
#define WS90_REQUIRE_EXPECTED_ID 1
#define HEARTBEAT_IDLE_MS      10000u
#define WS90_OUTPUT_JSON       1

/*
 * Important runtime switches:
 * - WS90_REQUIRE_EXPECTED_ID: only accept frames from your known station ID.
 * - WS90_OUTPUT_JSON: print one machine-readable JSON line per valid frame.
 * - STREAM_INVALID_FRAMES: print undecodable raw frames for troubleshooting.
 */

typedef struct {
    // Decoder state machine and signal-history fields.
    uint8_t state;
    bool have_last;
    bool last_bit;
    uint32_t alt_run;
    uint32_t max_alt_run;
    uint32_t sample_acc;
    uint32_t sample_count;
    uint32_t sync_shift;
    uint32_t sync_window;
    uint8_t frame[WS90_FRAME_BYTES];
    uint32_t frame_bits;
    bool have_last_sample;
    bool last_sample;
    uint32_t status_sample_total;
    uint32_t status_sample_ones;
    uint32_t status_sample_transitions;
    uint32_t frames_seen;
    uint32_t frames_decoded;
} ws90_decoder_t;

enum {
    // Look for alternating preamble bits (AAAA...).
    DECODER_SEARCH_PREAMBLE = 0,
    // After preamble, search for WS90 sync word (2DD4).
    DECODER_SEARCH_SYNC = 1,
    // Once sync is found, collect the fixed 32-byte payload.
    DECODER_COLLECT_PAYLOAD = 2,
};

static inline void rfm_write(uint8_t addr, uint8_t value) {
    // RFM69 write: set MSB in register address to indicate write operation.
    uint8_t reg = addr | 0x80;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &reg, 1);
    spi_write_blocking(spi0, &value, 1);
    gpio_put(PIN_CS, 1);
}

static inline uint8_t rfm_read(uint8_t addr) {
    // RFM69 read: clear MSB in register address to indicate read operation.
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(spi0, tx, rx, 2);
    gpio_put(PIN_CS, 1);
    return rx[1];
}

static void rfm_set_mode(uint8_t mode) {
    // Update only the mode bits inside OPMODE register.
    uint8_t op = rfm_read(RFM69_REG_OPMODE);
    op = (uint8_t)((op & (uint8_t)~RFM69_OPMODE_MASK) | (mode & RFM69_OPMODE_MASK));
    rfm_write(RFM69_REG_OPMODE, op);

    // Wait for ModeReady so later register/data operations are stable.
    for (uint32_t i = 0; i < 20; i++) {
        uint8_t irq1 = rfm_read(RFM69_REG_IRQFLAGS1);
        if (irq1 & 0x80u) {
            break;
        }
        sleep_ms(1);
    }
}

static inline void rfm_read_fifo(uint8_t *buffer, size_t len) {
    // Sequential read from FIFO starting at address 0x00.
    uint8_t reg = (uint8_t)(RFM69_REG_FIFO & 0x7F);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &reg, 1);
    spi_read_blocking(spi0, 0x00, buffer, len);
    gpio_put(PIN_CS, 1);
}

static void rfm_write_bitrate(uint32_t bitrate_bps) {
    // RFM69 bitrate register uses: bitrate = FXOSC / bitrate_reg (FXOSC=32MHz).
    uint32_t bitrate_reg = (32000000u + (bitrate_bps / 2u)) / bitrate_bps;
    if (bitrate_reg > 0xFFFFu) {
        bitrate_reg = 0xFFFFu;
    }
    rfm_write(RFM69_REG_BITRATEMSB, (uint8_t)((bitrate_reg >> 8) & 0xFFu));
    rfm_write(RFM69_REG_BITRATELSB, (uint8_t)(bitrate_reg & 0xFFu));
}

static void rfm_write_frequency_hz(uint32_t freq_hz) {
    // Convert Hz to FRF register value (datasheet formula).
    uint64_t frf = (((uint64_t)freq_hz) << 19) / 32000000ull;
    rfm_write(RFM69_REG_FRFMSB, (uint8_t)((frf >> 16) & 0xFFu));
    rfm_write(RFM69_REG_FRFMID, (uint8_t)((frf >> 8) & 0xFFu));
    rfm_write(RFM69_REG_FRFLSB, (uint8_t)(frf & 0xFFu));
}

static void rfm_set_sync_profile(uint8_t profile) {
    // Different sync profiles are useful when field conditions vary.
    switch (profile) {
        case 0:
            // 4-byte sync AA AA 2D D4, tolerance 1
            rfm_write(RFM69_REG_SYNCCONFIG, 0x99);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE3, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE4, 0xD4);
            break;
        case 1:
            // 3-byte sync AA 2D D4, tolerance 1
            rfm_write(RFM69_REG_SYNCCONFIG, 0x91);
            rfm_write(RFM69_REG_SYNCVALUE1, 0xAA);
            rfm_write(RFM69_REG_SYNCVALUE2, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE3, 0xD4);
            break;
        default:
            // 2-byte sync 2D D4, strict
            rfm_write(RFM69_REG_SYNCCONFIG, 0x88);
            rfm_write(RFM69_REG_SYNCVALUE1, 0x2D);
            rfm_write(RFM69_REG_SYNCVALUE2, 0xD4);
            break;
    }
}

static void rfm_set_continuous_data_mode(void) {
    // Raw mode: disable packet/sync engine and output demodulated stream on DIO2.
    rfm_set_mode(RFM69_MODE_STDBY);
    rfm_write(RFM69_REG_DATAMODUL, 0x40);
    rfm_write(RFM69_REG_SYNCCONFIG, 0x00);
    rfm_write(RFM69_REG_DIOMAPPING1, 0x00);
    rfm_set_mode(RFM69_MODE_RX);
}

static void rfm_set_raw_bitrate(uint32_t bitrate_bps) {
    // In raw mode we still tune demod clock by changing bitrate register.
    rfm_set_mode(RFM69_MODE_STDBY);
    rfm_write_bitrate(bitrate_bps);
    rfm_set_mode(RFM69_MODE_RX);
}

static void rfm_reset(void) {
    // Hardware reset pulse for known-good startup state.
    gpio_put(PIN_RST, 1);
    sleep_ms(1);
    gpio_put(PIN_RST, 0);
    sleep_ms(10);
}

static void rfm_init(void) {
    // Main packet-mode configuration used during normal operation.
    // Standby
    rfm_set_mode(RFM69_MODE_STDBY);
    sleep_ms(5);

    // FSK, no shaping, packet mode
    rfm_write(RFM69_REG_DATAMODUL, 0x00);

    // Bitrate derived from WS90 symbol period (~58 us => ~17.2 kbps)
    rfm_write_bitrate(RADIO_BITRATE_BPS);

    // Deviation from SDR captures (about +/-33 to +/-35 kHz)
    uint32_t fdev_reg = (RADIO_FDEV_HZ + 30u) / 61u;
    if (fdev_reg > 0x3FFFu) {
        fdev_reg = 0x3FFFu;
    }
    rfm_write(RFM69_REG_FDEVMSB, (uint8_t)((fdev_reg >> 8) & 0x3Fu));
    rfm_write(RFM69_REG_FDEVLSB, (uint8_t)(fdev_reg & 0xFFu));

    // Frequency 433.92 MHz
    rfm_write_frequency_hz(RADIO_CENTER_HZ);

    // RXBW 125 kHz (Carson BW for dev~33.5k and br~17.2k is ~84 kHz)
    rfm_write(RFM69_REG_RXBW, 0b10000010);

    // AFCBW 125 kHz to allow practical frequency error during lock
    rfm_write(RFM69_REG_AFCBW, 0b10000010);

    // Sync ON, 4-byte sync AA AA 2D D4 for tighter framing
    rfm_set_sync_profile(0);

    // Fixed packet length, no hardware line decoding
    rfm_write(RFM69_REG_PACKETCONFIG1, 0x00);
    rfm_write(RFM69_REG_PAYLOADLENGTH, RFM_CAPTURE_BYTES);
    rfm_write(RFM69_REG_FIFOTHRESH, 0x8F);
    rfm_write(RFM69_REG_PACKETCONFIG2, 0x02);

    // Require stronger signal before sync/payload detection (-90 dBm threshold)
    rfm_write(RFM69_REG_RSSITHRESH, 0xB4);

    // DIO0 default mapping (PayloadReady in packet mode)
    rfm_write(RFM69_REG_DIOMAPPING1, 0x00);

    // RX mode
    rfm_set_mode(RFM69_MODE_RX);
}

static void decoder_reset(ws90_decoder_t *decoder) {
    // Reset state machine and working buffers for a fresh frame search.
    memset(decoder, 0, sizeof(*decoder));
    decoder->state = DECODER_SEARCH_PREAMBLE;
}

static inline void decoder_track_sample(ws90_decoder_t *decoder, bool sample_bit) {
    decoder->status_sample_total++;
    if (sample_bit) {
        decoder->status_sample_ones++;
    }
    if (decoder->have_last_sample) {
        if (sample_bit != decoder->last_sample) {
            decoder->status_sample_transitions++;
        }
    } else {
        decoder->have_last_sample = true;
    }
    decoder->last_sample = sample_bit;
}

static uint8_t ws90_crc8(const uint8_t *data, size_t len, uint8_t poly, uint8_t init) {
    // Bitwise CRC8 (rtl_433 compatible settings for WS90).
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
    // WS90 also carries an additive checksum in the last byte.
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)sum;
}

static uint8_t ws90_reverse8(uint8_t x) {
    // Reverse bit order in one byte (e.g. abcdefgh -> hgfedcba).
    x = (uint8_t)(((x & 0xF0u) >> 4) | ((x & 0x0Fu) << 4));
    x = (uint8_t)(((x & 0xCCu) >> 2) | ((x & 0x33u) << 2));
    x = (uint8_t)(((x & 0xAAu) >> 1) | ((x & 0x55u) << 1));
    return x;
}

static void shift_left_bits_len(const uint8_t *in, uint8_t *out, uint32_t len, uint8_t bits) {
    // Bit-shift an entire byte array left by N bits (N=1..7) for re-alignment tests.
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
    // Bit-shift an entire byte array right by N bits (N=1..7) for re-alignment tests.
    if (bits == 0u) {
        memcpy(out, in, len);
        return;
    }
    for (int i = (int)len - 1; i >= 0; i--) {
        uint8_t prev = (i > 0) ? in[i - 1] : 0u;
        out[i] = (uint8_t)((in[i] >> bits) | (prev << (8u - bits)));
    }
}

static void ws90_shift_left_bits(const uint8_t *in, uint8_t *out, uint8_t bits) {
    shift_left_bits_len(in, out, WS90_FRAME_BYTES, bits);
}

static void ws90_shift_right_bits(const uint8_t *in, uint8_t *out, uint8_t bits) {
    shift_right_bits_len(in, out, WS90_FRAME_BYTES, bits);
}

static bool ws90_decode_and_print(const uint8_t *b) {
    // Fast header check: WS90 family byte must be 0x90.
    if (b[0] != 0x90u) {
        return false;
    }

    int id = (b[1] << 16) | (b[2] << 8) | b[3];
#if WS90_REQUIRE_EXPECTED_ID
    // Optional hard filter: ignore other nearby weather stations.
    if (id != (int)WS90_EXPECTED_ID) {
        return false;
    }
#endif

    uint8_t crc = ws90_crc8(b, 31, 0x31, 0x00);
    uint8_t chk = ws90_add_bytes(b, 31);
    if ((crc != 0u) || (chk != b[31])) {
        return false;
    }

    // At this point frame integrity is good; map payload bytes into engineering values.

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
#if WS90_OUTPUT_JSON
    // JSON mode: one compact line per valid packet for easy ingestion/logging.
    printf("{\"model\":\"Fineoffset-WS90\",\"id\":\"%06X\",\"battery_level\":%.3f,\"battery_mv\":%d,\"temperature_c\":%.1f,\"humidity\":%d,\"wind_dir_deg\":%d,\"wind_avg_m_s\":%.1f,\"wind_max_m_s\":%.1f,\"uv_index\":%.1f,\"light_lux\":%.1f,\"flags\":\"%02x\",\"rain_mm\":%.1f,\"rain_start\":%d,\"supercap_v\":%.1f,\"firmware\":%d,\"extra\":\"%s\",\"mic\":\"CRC\"}\n",
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
#else
    printf("model     : Fineoffset-WS90                        ID        : %06X\n", id);
    printf("Battery level: %.3f     Battery Voltage: %d mV  Temperature: %.1f C       Humidity  : %d %%          Wind direction: %d       Wind speed: %.1f m/s       Gust speed: %.1f m/s       UV Index  : %.1f\n",
           (double)(battery_lvl * 0.01f),
           battery_mv,
           (double)temp_c,
           humidity,
           wind_dir,
           (double)(wind_avg * 0.1f),
           (double)(wind_max * 0.1f),
           (double)(uv_index * 0.1f));
    printf("Light     : %.1f lux      Flags     : %02x            Total Rain: %.1f mm        Rain Start: %d             Supercap Voltage: %.1f V   Firmware Version: %d     Extra Data: %s\n",
           (double)light_lux,
           flags,
           (double)(rain_raw * 0.1f),
           rain_start,
           (double)(supercap_v * 0.1f),
           firmware,
           extra);
    printf("Integrity : CRC\n");
#endif
    return true;
}

static bool ws90_decode_expected_id_fallback(const uint8_t *b, const char *tag) {
    // Diagnostic path: same field mapping, but allows CRC/checksum failure printouts.
    if (b[0] != 0x90u) {
        return false;
    }

    int id = (b[1] << 16) | (b[2] << 8) | b[3];
    if (id != (int)WS90_EXPECTED_ID) {
        return false;
    }

    uint8_t crc = ws90_crc8(b, 31, 0x31, 0x00);
    uint8_t chk = ws90_add_bytes(b, 31);

    int light_raw = (b[4] << 8) | b[5];
    float light_lux = light_raw * 10.0f;
    int battery_mv = b[6] * 20;
    int battery_lvl = (battery_mv < 1400) ? 0 : ((battery_mv - 1400) / 16);
    if (battery_lvl > 100) battery_lvl = 100;
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

    printf("model     : Fineoffset-WS90 [%s]                   ID        : %06X\n", tag, id);
    printf("Battery level: %.3f     Battery Voltage: %d mV  Temperature: %.1f C       Humidity  : %d %%          Wind direction: %d       Wind speed: %.1f m/s       Gust speed: %.1f m/s       UV Index  : %.1f\n",
           (double)(battery_lvl * 0.01f),
           battery_mv,
           (double)temp_c,
           humidity,
           wind_dir,
           (double)(wind_avg * 0.1f),
           (double)(wind_max * 0.1f),
           (double)(uv_index * 0.1f));
    printf("Light     : %.1f lux      Flags     : %02x            Total Rain: %.1f mm        Rain Start: %d             Supercap Voltage: %.1f V   Firmware Version: %d     Extra Data: %s\n",
           (double)light_lux,
           flags,
           (double)(rain_raw * 0.1f),
           rain_start,
           (double)(supercap_v * 0.1f),
           firmware,
           extra);
    printf("Integrity : fail (crc=%02X chk=%02X exp=%02X)\n", crc, chk, b[31]);
    return true;
}

static bool ws90_decode_id_anchored_stream(const uint8_t *buf, uint32_t len, const char *tag) {
    // Search all 32-byte windows for the known station ID at bytes [1..3].
    // This is useful when sync/framing is close but not perfect.
    if (len < WS90_FRAME_BYTES) {
        return false;
    }

    for (uint32_t start = 0; start + WS90_FRAME_BYTES <= len; start++) {
        int id = (buf[start + 1] << 16) | (buf[start + 2] << 8) | buf[start + 3];
        if (id != (int)WS90_EXPECTED_ID) {
            continue;
        }

        if (ws90_decode_and_print(&buf[start])) {
            return true;
        }
        if (ws90_decode_expected_id_fallback(&buf[start], tag)) {
            return true;
        }
    }

    return false;
}

static bool ws90_decode_with_alignment(const uint8_t *raw) {
    // Try multiple transform families to recover frames from slight bit/byte misalignment.
    // Order: raw -> invert -> bit-reverse -> bit-reverse+invert, with rotations/shifts.
    uint8_t base[WS90_FRAME_BYTES];
    uint8_t shifted[WS90_FRAME_BYTES];
    uint8_t rotated[WS90_FRAME_BYTES];

    if (ws90_decode_and_print(raw)) {
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

        if (ws90_decode_and_print(base)) {
            return true;
        }

        const char *tag = (mode == 0u) ? "RAW" : (mode == 1u) ? "INV" : (mode == 2u) ? "BITREV" : "BITREV_INV";
        if (ws90_decode_expected_id_fallback(base, tag)) {
            return true;
        }

        for (uint8_t off = 1u; off < WS90_FRAME_BYTES; off++) {
            for (uint8_t i = 0u; i < WS90_FRAME_BYTES; i++) {
                rotated[i] = base[(uint8_t)((off + i) % WS90_FRAME_BYTES)];
            }
            if (ws90_decode_and_print(rotated)) {
                return true;
            }
            if (ws90_decode_expected_id_fallback(rotated, "ROT")) {
                return true;
            }
        }

        for (uint8_t shift = 1u; shift <= 7u; shift++) {
            ws90_shift_left_bits(base, shifted, shift);
            if (ws90_decode_and_print(shifted)) {
                return true;
            }
            if (ws90_decode_expected_id_fallback(shifted, "SHIFTL")) {
                return true;
            }

            for (uint8_t off = 1u; off < WS90_FRAME_BYTES; off++) {
                for (uint8_t i = 0u; i < WS90_FRAME_BYTES; i++) {
                    rotated[i] = shifted[(uint8_t)((off + i) % WS90_FRAME_BYTES)];
                }
                if (ws90_decode_and_print(rotated)) {
                    return true;
                }
                if (ws90_decode_expected_id_fallback(rotated, "SHIFTL_ROT")) {
                    return true;
                }
            }

            ws90_shift_right_bits(base, shifted, shift);
            if (ws90_decode_and_print(shifted)) {
                return true;
            }
            if (ws90_decode_expected_id_fallback(shifted, "SHIFTR")) {
                return true;
            }

            for (uint8_t off = 1u; off < WS90_FRAME_BYTES; off++) {
                for (uint8_t i = 0u; i < WS90_FRAME_BYTES; i++) {
                    rotated[i] = shifted[(uint8_t)((off + i) % WS90_FRAME_BYTES)];
                }
                if (ws90_decode_and_print(rotated)) {
                    return true;
                }
                if (ws90_decode_expected_id_fallback(rotated, "SHIFTR_ROT")) {
                    return true;
                }
            }
        }
    }

    return false;
}

static void print_codes_line(const uint8_t *payload, uint32_t payload_bytes) {
    uint32_t bit_count = 32u + (payload_bytes * 8u);
    printf("codes     : {%lu}%08lX",
           (unsigned long)bit_count,
           (unsigned long)WS90_PREAMBLE_SYNC_32);
    for (uint32_t i = 0; i < payload_bytes; i++) {
        printf("%02X", payload[i]);
    }
    printf("\n");
}

static bool decoder_emit_frame(ws90_decoder_t *decoder) {
    decoder->frames_seen++;
    if (ws90_decode_with_alignment(decoder->frame)) {
        decoder->frames_decoded++;
        return true;
    }

#if STREAM_INVALID_FRAMES
    printf("FRAME_RAW len=%u sync=0x%04X data=", WS90_FRAME_BYTES, WS90_SYNC_WORD);
    for (uint32_t i = 0; i < WS90_FRAME_BYTES; i++) {
        printf("%02X", decoder->frame[i]);
    }
    printf("\n");
    print_codes_line(decoder->frame, WS90_FRAME_BYTES);
#endif
    return false;
}

static bool handle_frame_bytes(const uint8_t *frame) {
    if (ws90_decode_with_alignment(frame)) {
        return true;
    }

#if STREAM_INVALID_FRAMES
    printf("FRAME_RAW len=%u sync=0x%04X data=", WS90_FRAME_BYTES, WS90_SYNC_WORD);
    for (uint32_t i = 0; i < WS90_FRAME_BYTES; i++) {
        printf("%02X", frame[i]);
    }
    printf("\n");
    print_codes_line(frame, WS90_FRAME_BYTES);
#endif
    return false;
}

static bool handle_capture_bytes(const uint8_t *capture, uint32_t capture_len) {
    // Packet-mode capture handler:
    // 1) apply transform/shift search,
    // 2) attempt ID-anchored decoding,
    // 3) attempt full alignment decode,
    // 4) print raw frame for debug if still invalid.
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
            if (ws90_decode_id_anchored_stream(shifted, capture_len, "ID_ANCHOR")) {
                return true;
            }
            for (uint32_t start = 0; start + WS90_FRAME_BYTES <= capture_len; start++) {
                if (ws90_decode_with_alignment(&shifted[start])) {
                    return true;
                }
            }

            if (bit_shift != 0u) {
                shift_right_bits_len(transformed, shifted, capture_len, bit_shift);
                if (ws90_decode_id_anchored_stream(shifted, capture_len, "ID_ANCHOR")) {
                    return true;
                }
                for (uint32_t start = 0; start + WS90_FRAME_BYTES <= capture_len; start++) {
                    if (ws90_decode_with_alignment(&shifted[start])) {
                        return true;
                    }
                }
            }
        }

        if (ws90_decode_id_anchored_stream(transformed, capture_len, "ID_ANCHOR")) {
            return true;
        }
    }

    if (ws90_decode_id_anchored_stream(capture, capture_len, "ID_ANCHOR")) {
        return true;
    }

    for (uint32_t start = 0; start + WS90_FRAME_BYTES <= capture_len; start++) {
        if (ws90_decode_with_alignment(&capture[start])) {
            return true;
        }
    }

    return handle_frame_bytes(capture);
}

static bool decoder_handle_bit(ws90_decoder_t *decoder, bool bit) {
    // Core state machine that converts a bit stream into a 32-byte frame.
    switch (decoder->state) {
        case DECODER_SEARCH_PREAMBLE:
            if (!decoder->have_last) {
                decoder->have_last = true;
                decoder->last_bit = bit;
                decoder->alt_run = 1;
                return false;
            }

            // Preamble is alternating bits (101010...), so transitions are important.
            if (bit != decoder->last_bit) {
                decoder->alt_run++;
            } else {
                decoder->alt_run = 1;
            }
            if (decoder->alt_run > decoder->max_alt_run) {
                decoder->max_alt_run = decoder->alt_run;
            }
            decoder->last_bit = bit;

            decoder->sync_shift = (decoder->sync_shift << 1) | (bit ? 1u : 0u);

            // Fast path: sometimes we see full AAAA2DD4 in one rolling window.
            if (decoder->sync_shift == WS90_PREAMBLE_SYNC_32) {
                decoder->state = DECODER_COLLECT_PAYLOAD;
                decoder->frame_bits = 0;
                memset(decoder->frame, 0, sizeof(decoder->frame));
                break;
            }

            // Once enough alternating bits are seen, start sync search phase.
            if (decoder->alt_run >= WS90_PREAMBLE_MIN_BITS) {
                decoder->state = DECODER_SEARCH_SYNC;
                decoder->sync_shift = 0;
                decoder->sync_window = WS90_SYNC_SEARCH_WINDOW_BITS;
            }
            return false;

        case DECODER_SEARCH_SYNC:
            // Slide a 16-bit window to find sync word 0x2DD4.
            decoder->sync_shift = ((decoder->sync_shift << 1) | (bit ? 1u : 0u)) & ((1u << WS90_SYNC_BITS) - 1u);
            if ((decoder->sync_shift & 0xFFFFu) == WS90_SYNC_WORD) {
                decoder->state = DECODER_COLLECT_PAYLOAD;
                decoder->frame_bits = 0;
                memset(decoder->frame, 0, sizeof(decoder->frame));
                break;
            }

            if (decoder->sync_window > 0u) {
                decoder->sync_window--;
            }
            // Give up after a bounded window to avoid getting stuck forever.
            if (decoder->sync_window == 0u) {
                decoder->state = DECODER_SEARCH_PREAMBLE;
                decoder->have_last = true;
                decoder->last_bit = bit;
                decoder->alt_run = 1;
            }
            return false;

        case DECODER_COLLECT_PAYLOAD: {
            // Pack bits MSB-first into a fixed 32-byte frame buffer.
            uint32_t bit_index = decoder->frame_bits;
            uint32_t byte_index = bit_index >> 3;
            uint32_t bit_in_byte = 7u - (bit_index & 7u);
            if (byte_index < WS90_FRAME_BYTES) {
                if (bit) {
                    decoder->frame[byte_index] |= (1u << bit_in_byte);
                }
                decoder->frame_bits++;
            }

            if (decoder->frame_bits >= WS90_FRAME_BYTES * 8u) {
                bool decoded = decoder_emit_frame(decoder);
                decoder_reset(decoder);
                return decoded;
            }
            return false;
        }
        default:
            decoder_reset(decoder);
            return false;
    }
}

static inline bool decoder_handle_sample(ws90_decoder_t *decoder, bool sample_bit) {
    // Raw mode oversamples each symbol; majority vote converts samples -> one bit.
    decoder_track_sample(decoder, sample_bit);
    decoder->sample_acc += sample_bit ? 1u : 0u;
    decoder->sample_count++;
    if (decoder->sample_count >= OVERSAMPLE_FACTOR) {
        bool reconstructed = (decoder->sample_acc * 2u) >= OVERSAMPLE_FACTOR;
        decoder->sample_acc = 0;
        decoder->sample_count = 0;
        return decoder_handle_bit(decoder, reconstructed);
    }
    return false;
}

static bool process_sample_word(ws90_decoder_t *decoder, uint32_t word) {
    // PIO provides 32 sampled bits at a time; process from MSB to LSB.
    bool decoded = false;
    for (int bit = 31; bit >= 0; bit--) {
        bool sample = ((word >> bit) & 1u) != 0;
        if (decoder_handle_sample(decoder, sample)) {
            decoded = true;
        }
    }
    return decoded;
}

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // SPI init
    spi_init(spi0, 4 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    // Reset + init radio in packet mode (normal path).
    rfm_reset();
    rfm_init();

    uint8_t opmode = rfm_read(RFM69_REG_OPMODE);
    uint8_t datamodul = rfm_read(RFM69_REG_DATAMODUL);
    uint8_t syncconfig = rfm_read(RFM69_REG_SYNCCONFIG);
        printf("WS90 RX packet-mode start: bitrate=%uHz fdev=%uHz opmode=0x%02X datamodul=0x%02X sync=0x%02X\n",
           RADIO_BITRATE_BPS,
            RADIO_FDEV_HZ,
           opmode,
           datamodul,
           syncconfig);

    uint8_t frame[RFM_CAPTURE_BYTES];
    uint32_t no_packet_loops = 0;
    uint32_t undecoded_packets = 0;
    bool raw_mode = false;
    bool raw_ready = false;
    static const uint32_t raw_freq_profiles[] = {433912000u, 433920000u, 433928000u};
    static const uint32_t raw_bitrate_profiles[] = {17094u, 17241u, 17422u};
    uint32_t raw_freq_profile = 1u;
    uint32_t raw_bitrate_profile = 1u;
    uint32_t raw_last_seen_frames = 0u;
    uint32_t raw_last_decoded_frames = 0u;
    PIO pio = pio0;
    uint sm = 0;
    ws90_decoder_t raw_decoder;
    uint32_t heartbeat_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_activity_ms = heartbeat_ms;
    while (true) {
        if (!raw_mode) {
            // Packet-mode path: rely on RFM69 payload-ready interrupt flag.
            uint8_t opm = rfm_read(RFM69_REG_OPMODE);
            if ((opm & 0x1Cu) != RFM69_MODE_RX) {
                rfm_set_mode(RFM69_MODE_RX);
            }

            uint8_t irq2 = rfm_read(RFM69_REG_IRQFLAGS2);
            if (irq2 & 0x04u) {
                rfm_read_fifo(frame, RFM_CAPTURE_BYTES);
                bool decoded = handle_capture_bytes(frame, RFM_CAPTURE_BYTES);
                no_packet_loops = 0;
                last_activity_ms = to_ms_since_boot(get_absolute_time());
                if (decoded) {
                    undecoded_packets = 0;
                } else {
                    undecoded_packets++;
                    // If packet mode repeatedly fails, switch to raw PIO decode path.
                    if (undecoded_packets >= 10u) {
                        printf("fallback: switching to raw PIO decode mode\n");
                        raw_mode = true;
                    }
                }
            } else {
                no_packet_loops++;

                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                if (((now_ms - heartbeat_ms) >= 1000u) && ((now_ms - last_activity_ms) >= HEARTBEAT_IDLE_MS)) {
                    const char *p = "AAAA2DD4";
                    printf("alive: waiting packets (sync=%s)\n", p);
                    heartbeat_ms = now_ms;
                }
                sleep_ms(1);
            }
        } else {
            if (!raw_ready) {
                // Raw fallback setup: continuous demod stream + PIO sampler.
                rfm_set_continuous_data_mode();
                rfm_write_frequency_hz(raw_freq_profiles[raw_freq_profile]);
                rfm_set_raw_bitrate(raw_bitrate_profiles[raw_bitrate_profile]);

                uint offset = pio_add_program(pio, &raw_capture_program);
                sm = pio_claim_unused_sm(pio, true);
                pio_sm_config c = raw_capture_program_get_default_config(offset);
                sm_config_set_in_pins(&c, PIN_DIO2);
                sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
                sm_config_set_in_shift(&c, true, true, 32);
                float clkdiv = (float)clock_get_hz(clk_sys) / (float)SAMPLE_RATE_HZ;
                sm_config_set_clkdiv(&c, clkdiv);
                pio_gpio_init(pio, PIN_DIO2);
                gpio_set_dir(PIN_DIO2, GPIO_IN);
                pio_sm_init(pio, sm, offset, &c);
                pio_sm_set_enabled(pio, sm, true);

                decoder_reset(&raw_decoder);
                raw_ready = true;
                  printf("raw mode active (freq=%lu bitrate=%lu)\n",
                      (unsigned long)raw_freq_profiles[raw_freq_profile],
                      (unsigned long)raw_bitrate_profiles[raw_bitrate_profile]);
            }

            uint32_t sample_word = pio_sm_get_blocking(pio, sm);
            bool raw_decoded = process_sample_word(&raw_decoder, sample_word);
            if (raw_decoded) {
                raw_last_seen_frames = raw_decoder.frames_seen;
                raw_last_decoded_frames = raw_decoder.frames_decoded;
            }

            uint32_t seen_delta = raw_decoder.frames_seen - raw_last_seen_frames;
            uint32_t decoded_delta = raw_decoder.frames_decoded - raw_last_decoded_frames;
            if ((seen_delta >= 12u) && (decoded_delta == 0u)) {
                // Auto-retune in raw mode: sweep bitrate, then nudge center frequency.
                raw_bitrate_profile = (raw_bitrate_profile + 1u) % (sizeof(raw_bitrate_profiles) / sizeof(raw_bitrate_profiles[0]));
                if (raw_bitrate_profile == 0u) {
                    raw_freq_profile = (raw_freq_profile + 1u) % (sizeof(raw_freq_profiles) / sizeof(raw_freq_profiles[0]));
                }
                rfm_write_frequency_hz(raw_freq_profiles[raw_freq_profile]);
                rfm_set_raw_bitrate(raw_bitrate_profiles[raw_bitrate_profile]);
                decoder_reset(&raw_decoder);
                raw_last_seen_frames = 0u;
                raw_last_decoded_frames = 0u;
                printf("raw retune: freq=%lu bitrate=%lu\n",
                       (unsigned long)raw_freq_profiles[raw_freq_profile],
                       (unsigned long)raw_bitrate_profiles[raw_bitrate_profile]);
            }
        }
    }
}