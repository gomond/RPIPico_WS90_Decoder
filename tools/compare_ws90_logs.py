#!/usr/bin/env python3
import argparse
import json
import re
from collections import Counter
from pathlib import Path

EXPECTED_ID = bytes.fromhex("00C0E4")
EXPECTED_HDR_ID = bytes.fromhex("9000C0E4")


def reverse8(value: int) -> int:
    value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4)
    value = ((value & 0xCC) >> 2) | ((value & 0x33) << 2)
    value = ((value & 0xAA) >> 1) | ((value & 0x55) << 1)
    return value


def shift_left(data: bytes, bits: int) -> bytes:
    if bits == 0:
        return data
    out = bytearray(len(data))
    for index in range(len(data)):
        nxt = data[index + 1] if index + 1 < len(data) else 0
        out[index] = ((data[index] << bits) & 0xFF) | (nxt >> (8 - bits))
    return bytes(out)


def shift_right(data: bytes, bits: int) -> bytes:
    if bits == 0:
        return data
    out = bytearray(len(data))
    for index in range(len(data) - 1, -1, -1):
        prev = data[index - 1] if index > 0 else 0
        out[index] = ((data[index] >> bits) | ((prev << (8 - bits)) & 0xFF)) & 0xFF
    return bytes(out)


def transform(data: bytes, mode: str, shift_dir: str, shift_bits: int) -> bytes:
    result = data
    if mode == "inv":
        result = bytes((~value) & 0xFF for value in result)
    elif mode == "bitrev":
        result = bytes(reverse8(value) for value in result)
    elif mode == "bitrev_inv":
        result = bytes((~reverse8(value)) & 0xFF for value in result)

    if shift_dir == "l":
        result = shift_left(result, shift_bits)
    elif shift_dir == "r":
        result = shift_right(result, shift_bits)
    return result


def load_pico_frames(path: Path):
    raw = path.read_bytes()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        text = raw.decode("utf-16", errors="replace")
    hex_frames = re.findall(r"data=([0-9A-Fa-f]{64})", text)
    return [bytes.fromhex(item) for item in hex_frames]


def load_sdr(path: Path):
    raw = path.read_bytes()
    if raw.startswith(b"\xff\xfe") or raw.startswith(b"\xfe\xff"):
        text = raw.decode("utf-16", errors="replace")
    else:
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            text = raw.decode("utf-16", errors="replace")

    records = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        records.append(obj)
    return records


def analyze_frames(frames):
    hit_rows = []
    mode_counter = Counter()

    modes = ["raw", "inv", "bitrev", "bitrev_inv"]
    shift_dirs = ["none", "l", "r"]

    for frame_index, frame in enumerate(frames):
        frame_hit = False
        for mode in modes:
            for shift_dir in shift_dirs:
                for shift_bits in range(0, 8):
                    if shift_dir == "none" and shift_bits != 0:
                        continue
                    transformed = transform(frame, mode, shift_dir, shift_bits)

                    hdr_pos = transformed.find(EXPECTED_HDR_ID)
                    id_pos = transformed.find(EXPECTED_ID)

                    if hdr_pos >= 0 or id_pos >= 0:
                        frame_hit = True
                        hit_type = "hdr_id" if hdr_pos >= 0 else "id_only"
                        hit_pos = hdr_pos if hdr_pos >= 0 else id_pos
                        mode_key = f"{mode}/{shift_dir}{shift_bits}"
                        mode_counter[mode_key] += 1
                        hit_rows.append(
                            {
                                "frame": frame_index,
                                "hit_type": hit_type,
                                "pos": hit_pos,
                                "mode": mode,
                                "shift_dir": shift_dir,
                                "shift_bits": shift_bits,
                                "preview": transformed.hex().upper(),
                            }
                        )
        if not frame_hit:
            pass

    return hit_rows, mode_counter


def main():
    parser = argparse.ArgumentParser(description="Compare Pico WS90 raw frames with expected WS90 ID patterns and SDR log metadata.")
    parser.add_argument("--pico", default="pico.log", help="Path to Pico serial log")
    parser.add_argument("--sdr", default="sdr.jsonl", help="Path to rtl_433 JSONL log")
    args = parser.parse_args()

    pico_path = Path(args.pico)
    sdr_path = Path(args.sdr)

    pico_frames = load_pico_frames(pico_path)
    sdr_records = load_sdr(sdr_path)

    hit_rows, mode_counter = analyze_frames(pico_frames)

    sdr_ids = Counter(rec.get("id") for rec in sdr_records if "id" in rec)
    sdr_count = len(sdr_records)

    print(f"Pico frames: {len(pico_frames)}")
    print(f"SDR records: {sdr_count}")
    if sdr_ids:
        print("SDR IDs:", ", ".join(f"{int(k):06X}({v})" for k, v in sorted(sdr_ids.items())))

    print(f"ID/hdr hits in Pico frames: {len(hit_rows)}")
    if mode_counter:
        print("Best transform modes:")
        for mode_name, count in mode_counter.most_common(6):
            print(f"  {mode_name}: {count}")

    for row in hit_rows[:20]:
        print(
            f"hit frame={row['frame']} type={row['hit_type']} pos={row['pos']} "
            f"mode={row['mode']} shift={row['shift_dir']}{row['shift_bits']}"
        )


if __name__ == "__main__":
    main()
