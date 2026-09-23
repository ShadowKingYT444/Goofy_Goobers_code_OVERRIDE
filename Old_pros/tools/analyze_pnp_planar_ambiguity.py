#!/usr/bin/env python3
"""Quantify two-solution planar-PnP ambiguity in saved P8 tag corners."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
RAW = REPORT / "ai_scan_live_03" / "raw.log"
OUT_CSV = REPORT / "ai_planar_pnp_ambiguity.csv"
OUT_JSON = REPORT / "ai_planar_pnp_ambiguity_summary.json"
OUT_BASE = REPORT / "ai_planar_pnp_ambiguity_dashboard"

# P8 corners span the inner five-cell detection square, not the 0.875-in
# seven-cell printed outer square.
TAG_SIZE_IN = 0.625
FX = 212.34
FY = 195.82
CX = 160.0
CY = 120.0
K = np.asarray([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
DISTORTION = np.zeros(5)
HALF = TAG_SIZE_IN / 2.0
# IPPE_SQUARE requires this exact object-point order. The analysis assumes the
# P8's cyclic x0..x3 stream starts at the corresponding image corner; PROS does
# not document that semantic correspondence, which is itself a qualification
# limitation recorded in the output.
OBJECT_POINTS = np.asarray(
    [[-HALF, HALF, 0.0], [HALF, HALF, 0.0],
     [HALF, -HALF, 0.0], [-HALF, -HALF, 0.0]],
    dtype=np.float64,
)


def angle_diff_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def unique_tag4_corners() -> list[np.ndarray]:
    text = RAW.read_bytes().decode("utf-8", errors="ignore")
    found: list[tuple[float, ...]] = []
    for line in text.splitlines():
        if "VISION_SHADOW" not in line:
            continue
        if not re.search(r"(?:^|\s)tag=4(?:\s|$)", line):
            continue
        match = re.search(r"corners=([^\s\x00-\x1f]+)", line)
        if not match:
            continue
        values = tuple(float(value) for value in match.group(1).split(","))
        if len(values) == 8 and values not in found:
            found.append(values)
    return [np.asarray(values).reshape(4, 2) for values in found]


def oriented_normal(rotation_vector: np.ndarray,
                    translation_vector: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(rotation_vector)
    normal = rotation @ np.asarray([0.0, 0.0, 1.0])
    toward_camera = -translation_vector[:, 0]
    if float(normal @ toward_camera) < 0.0:
        normal = -normal
    return normal / np.linalg.norm(normal)


def main() -> None:
    rows = []
    for frame_index, corners in enumerate(unique_tag4_corners()):
        count, rotations, translations, errors = cv2.solvePnPGeneric(
            OBJECT_POINTS,
            corners,
            K,
            DISTORTION,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if count != 2:
            continue
        solutions = []
        for rotation, translation, error in zip(rotations, translations, errors):
            normal = oriented_normal(rotation, translation)
            tx, ty, tz = (float(value) for value in translation[:, 0])
            solutions.append(
                {
                    "horizontal_range_in": math.hypot(tx, tz),
                    "range_3d_in": math.sqrt(tx * tx + ty * ty + tz * tz),
                    "normal_bearing_deg": math.degrees(math.atan2(normal[0], normal[2])),
                    "normal_elevation_deg": math.degrees(
                        math.atan2(-normal[1], math.hypot(normal[0], normal[2]))
                    ),
                    "reprojection_rmse_px": float(error[0]),
                    "normal": normal,
                }
            )
        best, alternate = solutions
        normal_separation_deg = math.degrees(
            math.acos(float(np.clip(best["normal"] @ alternate["normal"], -1.0, 1.0)))
        )
        rows.append(
            {
                "frame": frame_index,
                "corners": ";".join(f"{x:.0f},{y:.0f}" for x, y in corners),
                "best_horizontal_range_in": best["horizontal_range_in"],
                "alternate_horizontal_range_in": alternate["horizontal_range_in"],
                "range_difference_in": abs(
                    best["horizontal_range_in"] - alternate["horizontal_range_in"]
                ),
                "best_normal_bearing_deg": best["normal_bearing_deg"],
                "alternate_normal_bearing_deg": alternate["normal_bearing_deg"],
                "normal_bearing_separation_deg": angle_diff_deg(
                    best["normal_bearing_deg"], alternate["normal_bearing_deg"]
                ),
                "normal_3d_separation_deg": normal_separation_deg,
                "best_reprojection_rmse_px": best["reprojection_rmse_px"],
                "alternate_reprojection_rmse_px": alternate["reprojection_rmse_px"],
                "reprojection_gap_px": (
                    alternate["reprojection_rmse_px"] - best["reprojection_rmse_px"]
                ),
            }
        )

    with OUT_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "unique_frames": len(rows),
        "solutions_per_frame": 2,
        "max_horizontal_range_difference_in": max(
            row["range_difference_in"] for row in rows
        ),
        "normal_bearing_separation_deg": {
            "min": min(row["normal_bearing_separation_deg"] for row in rows),
            "median": float(np.median(
                [row["normal_bearing_separation_deg"] for row in rows]
            )),
            "max": max(row["normal_bearing_separation_deg"] for row in rows),
        },
        "normal_3d_separation_deg": {
            "min": min(row["normal_3d_separation_deg"] for row in rows),
            "median": float(np.median(
                [row["normal_3d_separation_deg"] for row in rows]
            )),
            "max": max(row["normal_3d_separation_deg"] for row in rows),
        },
        "max_reprojection_gap_px": max(row["reprojection_gap_px"] for row in rows),
        "interpretation": (
            "Known-size range is nearly unchanged between the two IPPE square "
            "solutions, but the inferred tag-plane normal is not unique at this "
            "small image size. Range can be useful diagnostics while Goal-face "
            "orientation remains unqualified."
        ),
        "additional_limit": (
            "PROS documents x0..x3 as four tag corners but does not document the "
            "semantic first-corner/orientation correspondence required by "
            "IPPE_SQUARE."
        ),
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    frames = [row["frame"] for row in rows]
    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5), constrained_layout=True)
    fig.suptitle("P8 planar-tag pose ambiguity — range stable, face normal ambiguous",
                 fontsize=16, fontweight="bold")

    axes[0].plot(frames, [row["best_horizontal_range_in"] for row in rows],
                 "o-", label="lowest reprojection error")
    axes[0].plot(frames, [row["alternate_horizontal_range_in"] for row in rows],
                 "o--", label="second IPPE solution")
    axes[0].set(title="Horizontal range", xlabel="Unique corner frame", ylabel="in")
    axes[0].legend(frameon=False, fontsize=8)

    axes[1].bar(np.asarray(frames) - 0.18,
                [row["normal_bearing_separation_deg"] for row in rows], 0.36,
                label="horizontal bearing separation")
    axes[1].bar(np.asarray(frames) + 0.18,
                [row["normal_3d_separation_deg"] for row in rows], 0.36,
                label="3D normal separation")
    axes[1].set(title="Two plausible face normals", xlabel="Unique corner frame",
                ylabel="angular separation (deg)")
    axes[1].legend(frameon=False)

    axes[2].plot(frames, [row["best_reprojection_rmse_px"] for row in rows],
                 "o-", label="best")
    axes[2].plot(frames, [row["alternate_reprojection_rmse_px"] for row in rows],
                 "o--", label="alternate")
    axes[2].set(title="Both solutions fit nearly equally well",
                xlabel="Unique corner frame", ylabel="reprojection RMSE (px)")
    axes[2].legend(frameon=False)
    for axis in axes:
        axis.grid(alpha=0.18)
    fig.text(
        0.5,
        0.005,
        "Six unique live Tag 4 quads; nominal FOV, zero distortion, assumed x0→point0 correspondence. Diagnostic evidence only.",
        ha="center",
        color="#adb5bd",
        fontsize=9,
    )
    fig.savefig(OUT_BASE.with_suffix(".png"), dpi=180)
    fig.savefig(OUT_BASE.with_suffix(".svg"))
    plt.close(fig)


if __name__ == "__main__":
    main()
