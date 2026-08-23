#!/usr/bin/env python3
"""Summarize and validate BRAIN_LOCALIZATION telemetry logs."""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path


PAIR = r"(-?(?:\d+(?:\.\d*)?|\.\d+|inf))"
LOC_RE = re.compile(
    rf"^LOC t=(\d+) x={PAIR} y={PAIR} h={PAIR} imu={PAIR} enc_ok=(\d) "
    rf"m={PAIR},{PAIR},{PAIR},{PAIR} side={PAIR} "
    r"l8=(-?\d+),(-?\d+) l9=(-?\d+),(-?\d+) "
    rf"ltheta={PAIR},lvalid=(\d) "
    rf"c19=(-?\d+),{PAIR},{PAIR},(\d) "
    rf"c20=(-?\d+),{PAIR},{PAIR},(\d) "
    rf"corr=(\d+),(-?\d+),([^,]+),{PAIR}$"
)


@dataclass(frozen=True)
class Sample:
    t_ms: int
    x: float
    y: float
    heading: float
    imu: float
    encoder_ok: bool
    motors: tuple[float, float, float, float]
    side: float
    lidar8_mm: int
    lidar8_confidence: int
    lidar9_mm: int
    lidar9_confidence: int
    lidar_valid: bool
    camera19_id: int
    camera19_valid: bool
    camera20_id: int
    camera20_valid: bool
    correction_port: int
    correction_id: int
    correction_goal: str
    correction_innovation: float


def parse_sample(line: str) -> Sample | None:
    match = LOC_RE.match(line.strip())
    if not match:
        return None
    g = match.groups()
    # Indices include every capturing group in PAIR.
    return Sample(
        t_ms=int(g[0]),
        x=float(g[1]),
        y=float(g[2]),
        heading=float(g[3]),
        imu=float(g[4]),
        encoder_ok=g[5] == "1",
        motors=tuple(float(value) for value in g[6:10]),
        side=float(g[10]),
        lidar8_mm=int(g[11]),
        lidar8_confidence=int(g[12]),
        lidar9_mm=int(g[13]),
        lidar9_confidence=int(g[14]),
        lidar_valid=g[16] == "1",
        camera19_id=int(g[17]),
        camera19_valid=g[20] == "1",
        camera20_id=int(g[21]),
        camera20_valid=g[24] == "1",
        correction_port=int(g[25]),
        correction_id=int(g[26]),
        correction_goal=g[27],
        correction_innovation=float(g[28]),
    )


def finite_samples(samples: list[Sample]) -> list[Sample]:
    return [
        sample
        for sample in samples
        if all(
            math.isfinite(value)
            for value in (sample.x, sample.y, sample.heading, sample.imu)
        )
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--require-route",
        action="store_true",
        help="fail unless a completed movement route is present",
    )
    parser.add_argument(
        "--require-cameras",
        default="",
        help="comma-separated AI-camera correction ports required in the log",
    )
    args = parser.parse_args()
    text = args.log.read_text(errors="replace")
    samples = [sample for line in text.splitlines() if (sample := parse_sample(line))]
    usable = finite_samples(samples)
    failures: list[str] = []
    if len(usable) < 10:
        failures.append("fewer than 10 finite localization samples")
    route_complete = "LOC_ROUTE_DONE" in text
    if args.require_route and not route_complete:
        failures.append("route did not complete")
    if not usable:
        print("FAIL: no finite LOC telemetry")
        return 1

    first, last = usable[0], usable[-1]
    displacement = math.hypot(last.x - first.x, last.y - first.y)
    heading_change = abs(last.heading - first.heading)
    motor_travel = [
        max(sample.motors[index] for sample in usable)
        - min(sample.motors[index] for sample in usable)
        for index in range(4)
    ]
    moving_route = max(motor_travel) >= 20.0
    if args.require_route and not moving_route:
        failures.append("motor telemetry does not prove movement")
    if args.require_route and displacement > 2.0:
        failures.append("closed-route position residual exceeds 2.0 inches")
    if args.require_route and heading_change > 3.0:
        failures.append("closed-route heading residual exceeds 3.0 degrees")
    encoder_bad = sum(not sample.encoder_ok for sample in usable)
    if encoder_bad > max(3, len(usable) // 5):
        failures.append("encoder/IMU consistency gate rejected too many samples")
    detections19 = sum(sample.camera19_valid for sample in usable)
    detections20 = sum(sample.camera20_valid for sample in usable)
    correction_ports = sorted(
        {
            sample.correction_port
            for sample in usable
            if sample.correction_port in (19, 20)
        }
    )
    finite_innovations = [
        sample.correction_innovation
        for sample in usable
        if sample.correction_port in (19, 20)
        and math.isfinite(sample.correction_innovation)
    ]
    if not correction_ports:
        failures.append("no accepted AI-camera position correction")
    required_cameras = {
        int(port) for port in args.require_cameras.split(",") if port.strip()
    }
    missing_cameras = required_cameras.difference(correction_ports)
    if missing_cameras:
        failures.append(
            "missing accepted correction from camera port(s) "
            + ", ".join(str(port) for port in sorted(missing_cameras))
        )

    print(f"log: {args.log}")
    print(f"finite samples: {len(usable)} / {len(samples)}")
    print(f"route complete: {route_complete}")
    print(f"motor travel (deg): {', '.join(f'{value:.1f}' for value in motor_travel)}")
    print(
        "endpoint: "
        f"({first.x:.2f}, {first.y:.2f}, {first.heading:.2f} deg) -> "
        f"({last.x:.2f}, {last.y:.2f}, {last.heading:.2f} deg)"
    )
    print(
        f"endpoint delta: {displacement:.2f} in, {heading_change:.2f} deg; "
        f"encoder-gate rejects: {encoder_bad}"
    )
    print(f"valid tag frames: port19={detections19}, port20={detections20}")
    print(f"accepted correction sources: {correction_ports or 'none'}")
    if finite_innovations:
        print(
            "reported correction innovation: "
            f"last={finite_innovations[-1]:.2f} in, "
            f"max={max(finite_innovations):.2f} in"
        )
    lidar_good = sum(sample.lidar_valid for sample in usable)
    print(f"accepted paired-LiDAR geometry frames: {lidar_good}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
