# ESP32_WS90_Decoder_MQTT

Decode FineOffset WS90 weather sensor packets on Raspberry Pi Pico using an RFM69 receiver.

## ESP32 standalone project

The fully functional ESP32-S3 ESP-IDF version is split into branch `esp32-idf-standalone`.
Use that branch as the source for a separate GitHub project/repo dedicated to ESP32.

## What this project does

- Receives WS90 packets around 433.92 MHz.
- Validates payloads with WS90 CRC8 + additive checksum.
- Filters to the expected station ID (`00C0E4`) by default.
- Prints valid packets as single-line JSON (default output mode).
- Falls back to raw PIO sampling if packet mode repeatedly fails.

## Runtime flow (high level)

1. **Packet mode**: Configure RFM69 and wait for `PayloadReady`.
2. **Decode attempt**: Try normal and recovery alignments (invert/bit-reverse/bit-shift/rotation).
3. **Validation**: Accept only frames with expected ID + valid CRC/checksum.
4. **Fallback**: After repeated undecodable packets, switch to **raw mode**.
5. **Raw retune**: Sweep bitrate and frequency profiles until decode recovers.

## Function map for new contributors

### Radio / hardware helpers

- `rfm_write`, `rfm_read`  
  Low-level SPI register access for the RFM69.
- `rfm_set_mode`  
  Sets radio mode (standby/RX) and waits for `ModeReady`.
- `rfm_read_fifo`  
  Reads packet bytes from the RFM69 FIFO.
- `rfm_write_bitrate`, `rfm_write_frequency_hz`  
  Convert engineering units into register values.
- `rfm_set_sync_profile`  
  Selects sync pattern profile (`AA AA 2D D4` is current default path).
- `rfm_set_continuous_data_mode`, `rfm_set_raw_bitrate`  
  Configure raw streaming mode for fallback decoding.
- `rfm_reset`, `rfm_init`  
  Hardware reset and main packet-mode initialization.

### WS90 protocol helpers

- `ws90_crc8`  
  CRC8 implementation compatible with WS90 framing.
- `ws90_add_bytes`  
  Additive checksum helper.
- `ws90_reverse8`, `shift_left_bits_len`, `shift_right_bits_len`  
  Bit-manipulation utilities used for alignment recovery.

### Decode and alignment

- `ws90_decode_and_print`  
  Primary strict decoder: header + ID + CRC/checksum + field extraction + output.
- `ws90_decode_expected_id_fallback`  
  Diagnostic decode path that prints near-matches with checksum failures.
- `ws90_decode_id_anchored_stream`  
  Scans windows for expected ID placement to improve lock chances.
- `ws90_decode_with_alignment`  
  Tries transform variants to recover valid frame alignment.
- `handle_capture_bytes`, `handle_frame_bytes`  
  Packet-mode entry points for decode attempts.

### Raw decoder state machine

- `decoder_reset`  
  Resets state and frame buffers.
- `decoder_handle_bit`  
  Three-state parser: preamble search -> sync search -> payload collect.
- `decoder_handle_sample`  
  Oversample majority vote (`OVERSAMPLE_FACTOR`) from raw samples to bits.
- `process_sample_word`  
  Processes one 32-bit word from PIO RX FIFO.

### Main loop

- `main`  
  Initializes Pico + radio, runs packet mode, triggers raw fallback, applies retune profiles, and prints output.

## Key compile-time switches (`ESP32_WS90_Decoder_MQTT.c`)

- `WS90_OUTPUT_JSON`  
  `1` = JSON output, `0` = human-readable multi-line output.
- `WS90_REQUIRE_EXPECTED_ID`  
  `1` = only accept expected station ID.
- `STREAM_INVALID_FRAMES`  
  `1` = print undecodable frame dumps for debugging.
- `HEARTBEAT_IDLE_MS`  
  Idle time before heartbeat message appears.

## Notes for junior contributors

- Start from `main()` and follow one packet through `handle_capture_bytes()` into `ws90_decode_and_print()`.
- If debugging RF issues, enable `STREAM_INVALID_FRAMES` and compare with SDR captures.
- Avoid changing multiple RF parameters at once; tune one variable at a time (bitrate, bandwidth, frequency, sync).
