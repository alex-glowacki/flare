#!/usr/bin/env python3
"""
FLARE blackbox log parser.
Decodes integer-scaled columns back to physical units and computes
mag hard-iron calibration offsets from a calibration spin log.

Usage:
    python parse_log.py LOG003.CSV
"""

import csv
import sys
from pathlib import Path


def decode(path: str) -> list[dict]:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            try:
                rows.append({
                    "time_ms":    int(raw["time_ms"]),
                    "roll":       int(raw["roll_x100"])  / 100.0,
                    "pitch":      int(raw["pitch_x100"]) / 100.0,
                    "yaw":        int(raw["yaw_x100"])   / 100.0,
                    "acc_x":      int(raw["acc_x"]),
                    "acc_y":      int(raw["acc_y"]),
                    "acc_z":      int(raw["acc_z"]),
                    "gyr_x":      int(raw["gyr_x"]),
                    "gyr_y":      int(raw["gyr_y"]),
                    "gyr_z":      int(raw["gyr_z"]),
                    "throttle":   int(raw["throttle"]),
                    "m1":         int(raw["m1"]),
                    "m2":         int(raw["m2"]),
                    "m3":         int(raw["m3"]),
                    "m4":         int(raw["m4"]),
                    "armed":      int(raw["armed"]),
                    "rc_ok":      int(raw["rc_ok"]),
                    "gps_fix":    int(raw["gps_fix"]),
                    "gps_sats":   int(raw["gps_sats"]),
                    "lat":        int(raw["lat_x1e6"])  / 1_000_000.0,
                    "lon":        int(raw["lon_x1e6"])  / 1_000_000.0,
                    "alt_ft":     int(raw["alt_x10"])   / 10.0,
                    "spd_mph":    int(raw["spd_x10"])   / 10.0,
                    "mag_x":      int(raw["mag_x"]),
                    "mag_y":      int(raw["mag_y"]),
                })
            except (ValueError, KeyError) as e:
                print(f"  [skip] bad row: {e}")
    return rows


def mag_calibration(rows: list[dict]) -> tuple[float, float]:
    xs = [r["mag_x"] for r in rows if r["mag_x"] != 0 or r["mag_y"] != 0]
    ys = [r["mag_y"] for r in rows if r["mag_x"] != 0 or r["mag_y"] != 0]
    if not xs:
        print("  [error] no valid mag samples found")
        return 0.0, 0.0
    offset_x = (max(xs) + min(xs)) / 2.0
    offset_y = (max(ys) + min(ys)) / 2.0
    print(f"  Samples : {len(xs)}")
    print(f"  X  min={min(xs):6d}  max={max(xs):6d}  → offset_x = {offset_x:.1f}")
    print(f"  Y  min={min(ys):6d}  max={max(ys):6d}  → offset_y = {offset_y:.1f}")
    return offset_x, offset_y


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <LOG.CSV>")
        sys.exit(1)

    path = sys.argv[1]
    print(f"\nFLARE Log Parser — {Path(path).name}")
    print("=" * 48)

    rows = decode(path)
    print(f"Rows decoded: {len(rows)}")

    if "mag_x" in rows[0]:
        print("\n── Mag Hard-Iron Calibration ───────────────────")
        offset_x, offset_y = mag_calibration(rows)
        print(f"\nPaste into main.c (USER CODE BEGIN 2):")  # noqa: F541
        print(f"  mag_cal.offset_x = {offset_x:.1f}f;")
        print(f"  mag_cal.offset_y = {offset_y:.1f}f;")