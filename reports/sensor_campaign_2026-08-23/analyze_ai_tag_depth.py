#!/usr/bin/env python3
"""Compare simple known-size depth with a four-corner planar PnP solve."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
RAW = ROOT / "ai_scan_live_03" / "raw.log"
OUT_CSV = ROOT / "ai_tag4_depth_samples.csv"
OUT_JSON = ROOT / "ai_tag4_depth_summary.json"
OUT_PNG = ROOT / "ai_tag4_depth_dashboard.png"

# The P8-returned corner quad spans the inner five-cell detection square.
# The medium print's seven-cell outer black outline is 0.875 in, but using
# that outline with these corners scales every translated distance by 7/5.
TAG_SIZE_IN = 0.625
FX = 212.34
FY = 195.82
CX = 160.0
CY = 120.0
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
DISTORTION = np.zeros((5, 1), dtype=np.float64)
HALF = TAG_SIZE_IN / 2.0
# Corner order follows the live P8 stream around the quad. Absolute tag roll
# may differ by 90/180 degrees if the sensor chooses a different first corner;
# translation and plane obliquity are unaffected.
OBJECT_POINTS = np.array(
    [[-HALF, -HALF, 0.0], [HALF, -HALF, 0.0],
     [HALF, HALF, 0.0], [-HALF, HALF, 0.0]],
    dtype=np.float64,
)


def value(line: str, key: str) -> str | None:
    match = re.search(rf"(?:^|\s){re.escape(key)}=([^\s\x00-\x1f]+)", line)
    return match.group(1) if match else None


def parse() -> list[dict[str, float]]:
    text = RAW.read_bytes().decode("utf-8", errors="ignore")
    rows: list[dict[str, float]] = []
    for line in text.splitlines():
        if "VISION_SHADOW" not in line or value(line, "tag") != "4":
            continue
        corners_raw = value(line, "corners")
        if corners_raw is None:
            continue
        corners = np.array([float(item) for item in corners_raw.split(",")], dtype=np.float64)
        if corners.size != 8:
            continue
        image_points = corners.reshape(4, 2)
        success, rotation_vector, translation = cv2.solvePnP(
            OBJECT_POINTS,
            image_points,
            K,
            DISTORTION,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success or translation[2, 0] <= 0.0:
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        projected, _ = cv2.projectPoints(
            OBJECT_POINTS, rotation_vector, translation, K, DISTORTION
        )
        reprojection_rmse = float(
            np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1)))
        )
        tx, ty, tz = (float(component) for component in translation[:, 0])
        pnp_range = math.sqrt(tx * tx + ty * ty + tz * tz)
        pnp_horizontal = math.hypot(tx, tz)
        pnp_bearing = math.degrees(math.atan2(tx, tz))
        pnp_elevation = math.degrees(math.atan2(-ty, math.hypot(tx, tz)))
        tag_normal = rotation @ np.array([0.0, 0.0, 1.0])
        toward_camera = -translation[:, 0]
        if float(np.dot(tag_normal, toward_camera)) < 0.0:
            tag_normal = -tag_normal
        outward_normal_bearing = math.degrees(
            math.atan2(float(tag_normal[0]), float(tag_normal[2]))
        )
        outward_normal_elevation = math.degrees(
            math.atan2(-float(tag_normal[1]),
                       math.hypot(float(tag_normal[0]), float(tag_normal[2])))
        )
        plane_obliquity = math.degrees(
            math.acos(min(1.0, max(0.0, abs(float(tag_normal[2])))))
        )

        edges = np.linalg.norm(image_points - np.roll(image_points, -1, axis=0), axis=1)
        mean_horizontal = float((edges[0] + edges[2]) / 2.0)
        mean_vertical = float((edges[1] + edges[3]) / 2.0)
        center = image_points.mean(axis=0)
        z_simple = 0.5 * (
            FX * TAG_SIZE_IN / mean_horizontal +
            FY * TAG_SIZE_IN / mean_vertical
        )
        simple_right = z_simple * (center[0] - CX) / FX
        simple_up = -z_simple * (center[1] - CY) / FY
        simple_range = math.sqrt(
            z_simple * z_simple + simple_right * simple_right + simple_up * simple_up
        )
        simple_horizontal = math.hypot(z_simple, simple_right)
        simple_bearing = math.degrees(math.atan2(simple_right, z_simple))
        simple_elevation = math.degrees(
            math.atan2(simple_up, math.hypot(z_simple, simple_right))
        )
        rows.append(
            {
                "brain_ms": float(value(line, "t") or 0),
                "center_x_px": float(center[0]),
                "center_y_px": float(center[1]),
                "simple_forward_depth_in": z_simple,
                "simple_right_in": simple_right,
                "simple_up_in": simple_up,
                "simple_range_in": simple_range,
                "simple_horizontal_in": simple_horizontal,
                "simple_bearing_deg": simple_bearing,
                "simple_elevation_deg": simple_elevation,
                "pnp_forward_depth_in": tz,
                "pnp_right_in": tx,
                "pnp_up_in": -ty,
                "pnp_range_in": pnp_range,
                "pnp_horizontal_in": pnp_horizontal,
                "pnp_bearing_deg": pnp_bearing,
                "pnp_elevation_deg": pnp_elevation,
                "plane_obliquity_deg": plane_obliquity,
                "outward_normal_bearing_deg": outward_normal_bearing,
                "outward_normal_elevation_deg": outward_normal_elevation,
                "pnp_reprojection_rmse_px": reprojection_rmse,
            }
        )
    return rows


def stats(values: np.ndarray) -> dict[str, float]:
    return {
        "median": float(np.median(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "std": float(np.std(values)),
    }


def circular_stats_deg(values: np.ndarray) -> dict[str, float]:
    radians = np.radians(values)
    mean = math.degrees(math.atan2(float(np.sin(radians).sum()),
                                   float(np.cos(radians).sum())))
    residuals = (values - mean + 180.0) % 360.0 - 180.0
    return {
        "circular_mean": mean,
        "circular_std": float(np.std(residuals)),
        "minimum_residual": float(np.min(residuals)),
        "maximum_residual": float(np.max(residuals)),
    }


def main() -> None:
    rows = parse()
    if not rows:
        raise SystemExit("no solvable Tag 4 observations")
    with OUT_CSV.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    arrays = {key: np.array([row[key] for row in rows]) for key in rows[0]}
    summary = {
        "samples": len(rows),
        "tag_id": 4,
        "assumed_detected_square_size_in": TAG_SIZE_IN,
        "print_outer_black_square_in": 0.875,
        "camera_intrinsics": {"fx_px": FX, "fy_px": FY, "cx_px": CX, "cy_px": CY},
        "simple_range_in": stats(arrays["simple_range_in"]),
        "simple_horizontal_in": stats(arrays["simple_horizontal_in"]),
        "pnp_range_in": stats(arrays["pnp_range_in"]),
        "pnp_horizontal_in": stats(arrays["pnp_horizontal_in"]),
        "pnp_minus_simple_range_in": stats(
            arrays["pnp_range_in"] - arrays["simple_range_in"]),
        "pnp_minus_simple_horizontal_in": stats(
            arrays["pnp_horizontal_in"] - arrays["simple_horizontal_in"]),
        "pnp_forward_depth_in": stats(arrays["pnp_forward_depth_in"]),
        "pnp_right_in": stats(arrays["pnp_right_in"]),
        "pnp_up_in": stats(arrays["pnp_up_in"]),
        "pnp_bearing_deg": stats(arrays["pnp_bearing_deg"]),
        "pnp_elevation_deg": stats(arrays["pnp_elevation_deg"]),
        "plane_obliquity_deg": stats(arrays["plane_obliquity_deg"]),
        "outward_normal_bearing_deg": circular_stats_deg(
            arrays["outward_normal_bearing_deg"]),
        "outward_normal_elevation_deg": stats(arrays["outward_normal_elevation_deg"]),
        "reprojection_rmse_px": stats(arrays["pnp_reprojection_rmse_px"]),
        "firmware_model": "The Brain currently emits the simple edge/focal estimate. Planar solvePnP and plane-normal values in this report are offline reanalysis of the emitted corners.",
        "caveat": "The P8 corner quad is modeled as the medium print's 0.625-in inner five-cell detection square, not its 0.875-in outer black outline. P8 intrinsics are nominal and lens distortion is assumed zero. One later 18-in tape observation re-solved to 18.60 in, but multi-range and multi-angle truth is still required before absolute pose correction.",
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n")

    index = np.arange(len(rows))
    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle("P8 AprilTag 4 — Known-Size Depth and Four-Corner Pose", fontsize=17)

    ax = axes[0, 0]
    ax.plot(index, arrays["simple_range_in"], "o-", label="edge-size range")
    ax.plot(index, arrays["pnp_range_in"], "o-", label="planar PnP range")
    ax.plot(index, arrays["pnp_forward_depth_in"], "o--", label="PnP forward depth")
    ax.set(title="Depth estimates", xlabel="valid P8 frame", ylabel="inches")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(index, arrays["pnp_bearing_deg"], "o-", label="horizontal bearing")
    ax.plot(index, arrays["pnp_elevation_deg"], "o-", label="vertical elevation")
    ax.set(title="Camera-relative angles", xlabel="valid P8 frame", ylabel="degrees")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[1, 0]
    ax.plot(index, arrays["pnp_right_in"], "o-", label="right")
    ax.plot(index, arrays["pnp_up_in"], "o-", label="up")
    ax.set(title="Lateral/vertical tag offset", xlabel="valid P8 frame", ylabel="inches")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(index, arrays["plane_obliquity_deg"], "o-", label="plane obliquity (deg)")
    ax.plot(index, arrays["pnp_reprojection_rmse_px"], "o-", label="reprojection RMSE (px)")
    ax.set(title="Pose geometry quality", xlabel="valid P8 frame", ylabel="angle / pixels")
    ax.grid(alpha=0.2)
    ax.legend()

    fig.savefig(OUT_PNG, dpi=180)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
