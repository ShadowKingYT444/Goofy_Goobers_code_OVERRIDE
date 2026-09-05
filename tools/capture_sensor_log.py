#!/usr/bin/env python3
"""Capture synchronized D4 robot telemetry and summarize sensor health."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

import serial


NUMBER_RE = r"[-+]?(?:\d+(?:\.\d+)?|inf|nan)"
D4_RE = re.compile(
    r"D4 s=(?P<sample>\d+) t=(?P<robot_ms>\d+) "
    r"p1=(?P<p1_mm>-?\d+),(?P<p1_conf>-?\d+),(?P<p1_installed>[01]) "
    rf"m17=(?P<m17>{NUMBER_RE}) m18=(?P<m18>{NUMBER_RE}) "
    rf"m11=(?P<m11>{NUMBER_RE}) m13=(?P<m13>{NUMBER_RE}) "
    rf"h(?:5|15)=(?P<h5>-?\d+)(?: h(?:5|15)abs={NUMBER_RE})? "
    rf"imu=(?P<imu>{NUMBER_RE}) rawimu=(?P<rawimu>{NUMBER_RE}) "
    r"imust=(?P<imust>-?\d+) "
    rf"(?:imugyro=(?P<imu_gyro_x>{NUMBER_RE}),(?P<imu_gyro_y>{NUMBER_RE}),"
    rf"(?P<imu_gyro_z>{NUMBER_RE}) imuacc=(?P<imu_acc_x>{NUMBER_RE}),"
    rf"(?P<imu_acc_y>{NUMBER_RE}),(?P<imu_acc_z>{NUMBER_RE}) )?"
    rf"gps7=(?P<gps_x>{NUMBER_RE}),(?P<gps_y>{NUMBER_RE}),"
    rf"(?P<gps_heading>{NUMBER_RE}),(?P<gps_error>{NUMBER_RE}),"
    r"(?P<gps_installed>[01]) errno=(?P<errno>-?\d+)"
    rf"(?: gpsgyro=(?P<gps_gyro_z>{NUMBER_RE}))?"
)

INT_FIELDS = {
    "sample",
    "robot_ms",
    "p1_mm",
    "p1_conf",
    "p1_installed",
    "h5",
    "imust",
    "gps_installed",
    "errno",
}

FIELDNAMES = [
    "host_s",
    "sample",
    "robot_ms",
    "p1_mm",
    "p1_conf",
    "p1_installed",
    "m17",
    "m18",
    "m11",
    "m13",
    "h5",
    "imu",
    "rawimu",
    "imust",
    "imu_gyro_x",
    "imu_gyro_y",
    "imu_gyro_z",
    "imu_acc_x",
    "imu_acc_y",
    "imu_acc_z",
    "gps_x",
    "gps_y",
    "gps_heading",
    "gps_error",
    "gps_installed",
    "errno",
    "gps_gyro_z",
]


def parse_scalar(value: str) -> float | int | str:
    value = value.rstrip(",")
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def parse_fuse_frame(line: str, host_s: float) -> dict[str, object] | None:
    marker = "FUSE_TEST"
    if marker not in line:
        return None
    payload = line[line.index(marker):].strip()
    frame: dict[str, object] = {"host_s": host_s, "line": payload}
    for token in payload.split()[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        frame[key] = parse_scalar(value)
    return frame


def parse_frame(line: str, host_s: float) -> dict[str, float | int] | None:
    match = D4_RE.search(line)
    if not match:
        return None
    frame: dict[str, float | int] = {"host_s": host_s}
    for key, value in match.groupdict().items():
        if value is None:
            frame[key] = math.nan
        else:
            frame[key] = int(value) if key in INT_FIELDS else float(value)
    return frame


def finite_values(rows: list[dict[str, float | int]], key: str) -> list[float]:
    return [float(row[key]) for row in rows if math.isfinite(float(row[key]))]


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "min": None, "median": None, "max": None, "std": None}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "max": max(values),
        "std": statistics.pstdev(values),
    }


def summarize(rows: list[dict[str, float | int]], elapsed_s: float) -> dict[str, object]:
    p1_valid = [
        row
        for row in rows
        if int(row["p1_installed"]) == 1
        and 20 <= int(row["p1_mm"]) < 9999
        and int(row["p1_conf"]) > 0
    ]
    gps_valid = [
        row
        for row in rows
        if int(row["gps_installed"]) == 1
        and math.isfinite(float(row["gps_error"]))
        and 0.0 <= float(row["gps_error"]) <= 0.25
    ]
    count = len(rows)
    host_times = [float(row["host_s"]) for row in rows]
    observed_span_s = (
        host_times[-1] - host_times[0] if len(host_times) >= 2 else 0.0
    )
    maximum_interframe_gap_s = (
        max(b - a for a, b in zip(host_times, host_times[1:]))
        if len(host_times) >= 2 else 0.0
    )
    summary: dict[str, object] = {
        "elapsed_s": elapsed_s,
        "observed_frame_span_s": observed_span_s,
        "maximum_interframe_gap_s": maximum_interframe_gap_s,
        "frames": count,
        "rate_hz": count / elapsed_s if elapsed_s > 0 else 0.0,
        "p1_valid_fraction": len(p1_valid) / count if count else 0.0,
        "gps_valid_fraction": len(gps_valid) / count if count else 0.0,
        "p1_mm_valid": stats([float(row["p1_mm"]) for row in p1_valid]),
        "p1_conf_valid": stats([float(row["p1_conf"]) for row in p1_valid]),
        "gps_x_m_valid": stats([float(row["gps_x"]) for row in gps_valid]),
        "gps_y_m_valid": stats([float(row["gps_y"]) for row in gps_valid]),
        "gps_heading_deg_valid": stats([float(row["gps_heading"]) for row in gps_valid]),
        "gps_error_m_valid": stats([float(row["gps_error"]) for row in gps_valid]),
        "imu_deg": stats(finite_values(rows, "imu")),
        "imu_gyro_x_dps": stats(finite_values(rows, "imu_gyro_x")),
        "imu_gyro_y_dps": stats(finite_values(rows, "imu_gyro_y")),
        "imu_gyro_z_dps": stats(finite_values(rows, "imu_gyro_z")),
        "imu_acc_x_g": stats(finite_values(rows, "imu_acc_x")),
        "imu_acc_y_g": stats(finite_values(rows, "imu_acc_y")),
        "imu_acc_z_g": stats(finite_values(rows, "imu_acc_z")),
        "gps_gyro_z": stats(finite_values(rows, "gps_gyro_z")),
    }
    for key in ("m17", "m18", "m11", "m13", "h5"):
        values = finite_values(rows, key)
        summary[f"{key}_span"] = max(values) - min(values) if values else None
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=5.0,
        help="stop early if no D4 or fused-control frame arrives for this many seconds",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "telemetry.csv"
    fusion_path = args.output / "fusion.jsonl"
    raw_path = args.output / "raw.log"
    summary_path = args.output / "summary.json"

    rows: list[dict[str, float | int]] = []
    fusion_rows: list[dict[str, object]] = []
    start = time.monotonic()
    deadline = start + args.duration
    last_frame_s = start
    device: serial.Serial | None = None
    reconnects = 0
    with raw_path.open("w", encoding="utf-8") as raw_handle:
        while time.monotonic() < deadline:
            if device is None:
                # A fully removed USB device never reaches the normal read
                # branch below. Enforce the same idle deadline here so a long
                # capture saves its evidence promptly instead of retrying the
                # missing path until the original duration expires.
                if time.monotonic() - last_frame_s >= args.idle_timeout:
                    print(
                        f"telemetry idle for {args.idle_timeout:.1f}s; "
                        "the serial device is unavailable",
                        file=sys.stderr,
                    )
                    break
                try:
                    device = serial.Serial(args.port, args.baud, timeout=0.1)
                    # Upload/run resets briefly re-enumerate the Brain's USB
                    # interfaces. Give a successfully reopened interface a
                    # fresh idle window instead of treating that expected gap
                    # as a stopped user program.
                    last_frame_s = time.monotonic()
                    if reconnects:
                        print(
                            f"serial reconnected on {args.port} "
                            f"(attempt {reconnects})",
                            file=sys.stderr,
                        )
                except (serial.SerialException, OSError) as exc:
                    reconnects += 1
                    # At 100-ms retry cadence, report at most about once per
                    # ten seconds so a missing cable cannot flood long-run
                    # campaign logs or the supervising terminal.
                    if reconnects == 1 or reconnects % 100 == 0:
                        print(
                            f"serial unavailable on {args.port}; retrying: {exc}",
                            file=sys.stderr,
                        )
                    time.sleep(0.1)
                    continue

            try:
                line = device.readline().decode("utf-8", errors="replace")
            except (serial.SerialException, OSError) as exc:
                print(f"serial link reset; reopening: {exc}", file=sys.stderr)
                try:
                    device.close()
                except (serial.SerialException, OSError):
                    pass
                device = None
                reconnects += 1
                time.sleep(0.1)
                continue

            now = time.monotonic()
            if line:
                raw_handle.write(line)
                raw_handle.flush()
            frame = parse_frame(line, now - start)
            fusion_frame = parse_fuse_frame(line, now - start)
            if frame is not None:
                rows.append(frame)
                last_frame_s = now
            if fusion_frame is not None:
                fusion_rows.append(fusion_frame)
                # Blocking autonomous helpers emit FUSE_TEST telemetry while
                # opcontrol's normal D4 loop is paused. Those are proof that
                # the user program and serial link are alive, so they must
                # refresh the same watchdog as synchronized D4 frames.
                last_frame_s = now
            if now - last_frame_s >= args.idle_timeout:
                print(
                    f"telemetry idle for {args.idle_timeout:.1f}s; "
                    "the user program may be stopped",
                    file=sys.stderr,
                )
                break
    if device is not None:
        device.close()

    elapsed_s = time.monotonic() - start
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows, elapsed_s)
    summary["serial_reconnect_attempts"] = reconnects
    summary["fusion_frames"] = len(fusion_rows)
    summary["fusion_rate_hz"] = (
        len(fusion_rows) / elapsed_s if elapsed_s > 0 else 0.0
    )
    summary["fusion_phases"] = dict(
        Counter(str(row.get("phase", "unknown")) for row in fusion_rows)
    )
    summary["gps_reject_counts"] = dict(
        Counter(
            str(row["gps_reject"])
            for row in fusion_rows
            if "gps_reject" in row
        )
    )
    summary["ai_reject_counts"] = dict(
        Counter(
            str(row["ai_reject"])
            for row in fusion_rows
            if "ai_reject" in row
        )
    )
    with fusion_path.open("w", encoding="utf-8") as handle:
        for row in fusion_rows:
            handle.write(json.dumps(row, separators=(",", ":")) + "\n")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"wrote {csv_path}")
    print(f"wrote {fusion_path}")
    print(f"wrote {raw_path}")
    print(f"wrote {summary_path}")
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
