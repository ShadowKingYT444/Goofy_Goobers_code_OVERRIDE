#!/usr/bin/env python3
"""Stress nominal P8 AprilTag PnP against plausible modeling perturbations.

This is a sensitivity analysis, not a calibrated uncertainty claim. VEX
publishes 320x240 and 74x63-degree FOV, but not a per-unit intrinsic calibration
tolerance. The +/-1-degree FOV and +/-1-pixel principal-point bands below are
therefore explicit engineering what-if assumptions.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


WIDTH = 320.0
HEIGHT = 240.0
HFOV_DEG = 74.0
VFOV_DEG = 63.0
# P8 corners span the inner five-cell detection square, not the 0.875-in
# seven-cell printed outer square.
TAG_SIZE_IN = 0.625


def field(line: str, key: str) -> str | None:
    match = re.search(rf"(?:^|\s){re.escape(key)}=([^\s\x00-\x1f]+)", line)
    return match.group(1) if match else None


def unique_tag_corners(raw: Path) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    seen: set[tuple[float, ...]] = set()
    text = raw.read_bytes().decode("utf-8", errors="ignore")
    for line in text.splitlines():
        if "VISION_SHADOW" not in line or field(line, "tag") != "4":
            continue
        encoded = field(line, "corners")
        if encoded is None:
            continue
        values = tuple(float(value) for value in encoded.split(","))
        if len(values) != 8 or values in seen:
            continue
        seen.add(values)
        frames.append(np.asarray(values, dtype=np.float64).reshape(4, 2))
    return frames


def solve(corners: np.ndarray, tag_size: float, hfov: float, vfov: float,
          cx: float, cy: float) -> tuple[float, float, float] | None:
    fx = WIDTH / (2.0 * math.tan(math.radians(hfov) / 2.0))
    fy = HEIGHT / (2.0 * math.tan(math.radians(vfov) / 2.0))
    camera = np.asarray([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    half = tag_size / 2.0
    object_points = np.asarray(
        [[-half, -half, 0.0], [half, -half, 0.0],
         [half, half, 0.0], [-half, half, 0.0]],
        dtype=np.float64,
    )
    ok, _rotation, translation = cv2.solvePnP(
        object_points,
        corners,
        camera,
        np.zeros((5, 1), dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or translation[2, 0] <= 0.0:
        return None
    right, down, forward = (float(value) for value in translation[:, 0])
    horizontal = math.hypot(forward, right)
    bearing = math.degrees(math.atan2(right, forward))
    elevation = math.degrees(math.atan2(-down, horizontal))
    return horizontal, bearing, elevation


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=20260823)
    parser.add_argument(
        "--campaign",
        type=Path,
        default=Path("reports/sensor_campaign_2026-08-23"),
    )
    args = parser.parse_args()
    frames = unique_tag_corners(args.campaign / "ai_scan_live_03" / "raw.log")
    if not frames:
        raise SystemExit("no unique Tag 4 corner frames")

    rng = np.random.default_rng(args.seed)
    baseline = []
    for corners in frames:
        result = solve(corners, TAG_SIZE_IN, HFOV_DEG, VFOV_DEG,
                       WIDTH / 2.0, HEIGHT / 2.0)
        if result is not None:
            baseline.append(result)
    if not baseline:
        raise SystemExit("no baseline PnP solutions")

    records: list[tuple[float, float, float, float, float, float]] = []
    for _ in range(args.trials):
        frame_index = int(rng.integers(0, len(frames)))
        corners = frames[frame_index] + rng.uniform(-0.5, 0.5, (4, 2))
        tag_size = TAG_SIZE_IN * rng.uniform(0.99, 1.01)
        hfov = rng.uniform(HFOV_DEG - 1.0, HFOV_DEG + 1.0)
        vfov = rng.uniform(VFOV_DEG - 1.0, VFOV_DEG + 1.0)
        cx = WIDTH / 2.0 + rng.uniform(-1.0, 1.0)
        cy = HEIGHT / 2.0 + rng.uniform(-1.0, 1.0)
        result = solve(corners, tag_size, hfov, vfov, cx, cy)
        if result is None:
            continue
        base = baseline[frame_index]
        records.append((*result,
                        result[0] - base[0],
                        result[1] - base[1],
                        result[2] - base[2]))

    values = np.asarray(records)
    horizontal, bearing, elevation = values[:, :3].T
    range_delta, bearing_delta, elevation_delta = values[:, 3:].T
    summary = {
        "seed": args.seed,
        "requested_trials": args.trials,
        "solved_trials": len(records),
        "unique_live_corner_frames": len(frames),
        "published_nominal_model": {
            "resolution_px": [320, 240],
            "horizontal_fov_deg": HFOV_DEG,
            "vertical_fov_deg": VFOV_DEG,
        "detected_tag_size_in": TAG_SIZE_IN,
        },
        "what_if_perturbations": {
            "corner_quantization_px": "+/-0.5 uniform per coordinate",
            "horizontal_fov_deg": "+/-1.0 uniform",
            "vertical_fov_deg": "+/-1.0 uniform",
            "principal_point_px": "+/-1.0 uniform per axis",
            "printed_tag_scale_percent": "+/-1.0 uniform",
            "lens_distortion": "not modeled",
        },
        "horizontal_range_in": {
            "median": float(np.median(horizontal)),
            "p05": percentile(horizontal, 5),
            "p95": percentile(horizontal, 95),
            "max_abs_delta_from_same_frame_baseline": float(
                np.max(np.abs(range_delta))
            ),
            "abs_delta_p95": percentile(np.abs(range_delta), 95),
        },
        "bearing_deg": {
            "median": float(np.median(bearing)),
            "abs_delta_p95": percentile(np.abs(bearing_delta), 95),
            "max_abs_delta": float(np.max(np.abs(bearing_delta))),
        },
        "elevation_deg": {
            "median": float(np.median(elevation)),
            "abs_delta_p95": percentile(np.abs(elevation_delta), 95),
            "max_abs_delta": float(np.max(np.abs(elevation_delta))),
        },
        "interpretation": (
            "Sensitivity under explicit what-if bands only; it does not include "
            "unknown lens distortion, motion blur, tag bending, or a tape-measure "
            "ground-truth error. Keep AI absolute correction disabled until live "
            "multi-range calibration validates the model."
        ),
    }
    output_json = args.campaign / "ai_pnp_sensitivity_summary.json"
    output_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "axes.grid": True,
        "grid.alpha": 0.20,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })
    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.suptitle("P8 AprilTag PnP Sensitivity", fontsize=19, weight="bold")
    fig.text(
        0.5, 0.925,
        "Live Tag 4 corners + explicit intrinsic/print what-if bands; not a calibrated confidence interval",
        ha="center", color="#475569",
    )
    axes[0, 0].hist(horizontal, bins=80, color="#0891b2")
    axes[0, 0].set(title="Horizontal camera-to-tag range",
                   xlabel="Inches", ylabel="Trials")
    axes[0, 1].hist(range_delta, bins=80, color="#f97316")
    axes[0, 1].set(title="Range change vs same-frame nominal solve",
                   xlabel="Inches", ylabel="Trials")
    axes[1, 0].hist(bearing_delta, bins=80, color="#7c3aed")
    axes[1, 0].set(title="Bearing sensitivity",
                   xlabel="Degrees from nominal", ylabel="Trials")
    axes[1, 1].scatter(horizontal, bearing, s=4, alpha=0.12, color="#16a34a")
    axes[1, 1].set(title="Coupled range/bearing cloud",
                   xlabel="Horizontal range (in)", ylabel="Bearing (deg)")
    fig.text(
        0.5, 0.03,
        f"95% |delta|: {summary['horizontal_range_in']['abs_delta_p95']:.2f} in range, "
        f"{summary['bearing_deg']['abs_delta_p95']:.2f} deg bearing",
        ha="center", weight="bold",
    )
    fig.tight_layout(rect=(0.04, 0.07, 0.98, 0.89), h_pad=2.5, w_pad=2.5)
    fig.savefig(args.campaign / "ai_pnp_sensitivity_dashboard.png",
                dpi=220, facecolor="white")
    fig.savefig(args.campaign / "ai_pnp_sensitivity_dashboard.svg",
                facecolor="white")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
