#!/usr/bin/env python3
"""Quantify geometric bias in the onboard opposite-edge/focal tag range."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
WIDTH = 320.0
HEIGHT = 240.0
FX = 212.34
FY = 195.82
CX = WIDTH / 2.0
CY = HEIGHT / 2.0
# P8 corners span the inner five-cell detection square, not the 0.875-in
# seven-cell printed outer square.
TAG_IN = 0.625
MIN_EDGE_PX = 5.0
MAX_EDGE_RATIO = 1.8
MIN_AREA_PX2 = 40.0
MIN_FILL = 0.45
EDGE_MARGIN_PX = 2.0
DISTANCES_IN = (12.0, 20.0, 28.0)
PITCHES_DEG = (0.0, 15.0, 30.0)
YAWS_DEG = np.linspace(0.0, 65.0, 131)


def rotation(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)
    ry = np.asarray([
        [np.cos(yaw), 0.0, np.sin(yaw)],
        [0.0, 1.0, 0.0],
        [-np.sin(yaw), 0.0, np.cos(yaw)],
    ])
    rx = np.asarray([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(pitch), -np.sin(pitch)],
        [0.0, np.sin(pitch), np.cos(pitch)],
    ])
    return ry @ rx


def shoelace_area(points: np.ndarray) -> float:
    x = points[:, 0]
    y = points[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def evaluate(distance_in: float, yaw_deg: float, pitch_deg: float) -> dict[str, float | bool]:
    half = TAG_IN / 2.0
    local = np.asarray([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ])
    camera = (rotation(yaw_deg, pitch_deg) @ local.T).T
    camera[:, 2] += distance_in
    pixels = np.column_stack((
        FX * camera[:, 0] / camera[:, 2] + CX,
        FY * camera[:, 1] / camera[:, 2] + CY,
    ))
    edges = np.linalg.norm(np.roll(pixels, -1, axis=0) - pixels, axis=1)
    horizontal_edge = 0.5 * (edges[0] + edges[2])
    vertical_edge = 0.5 * (edges[1] + edges[3])
    z_width = FX * TAG_IN / horizontal_edge
    z_height = FY * TAG_IN / vertical_edge
    estimated_range = 0.5 * (z_width + z_height)
    area = shoelace_area(pixels)
    span = np.ptp(pixels, axis=0)
    bounding_area = float(span[0] * span[1])
    fill = area / bounding_area if bounding_area > 0.0 else 0.0
    edge_ratio = float(np.max(edges) / np.min(edges))
    in_bounds = bool(
        np.all(pixels[:, 0] > EDGE_MARGIN_PX)
        and np.all(pixels[:, 0] < WIDTH - 1.0 - EDGE_MARGIN_PX)
        and np.all(pixels[:, 1] > EDGE_MARGIN_PX)
        and np.all(pixels[:, 1] < HEIGHT - 1.0 - EDGE_MARGIN_PX)
    )
    usable = bool(
        in_bounds
        and np.min(edges) >= MIN_EDGE_PX
        and edge_ratio <= MAX_EDGE_RATIO
        and area >= MIN_AREA_PX2
        and fill >= MIN_FILL
    )
    return {
        "distance_in": distance_in,
        "yaw_deg": yaw_deg,
        "pitch_deg": pitch_deg,
        "estimated_range_in": estimated_range,
        "range_error_in": estimated_range - distance_in,
        "relative_error_fraction": estimated_range / distance_in - 1.0,
        "minimum_edge_px": float(np.min(edges)),
        "edge_ratio": edge_ratio,
        "area_px2": area,
        "fill_ratio": fill,
        "usable": usable,
    }


def main() -> None:
    rows = [
        evaluate(distance, float(yaw), pitch)
        for distance in DISTANCES_IN
        for pitch in PITCHES_DEG
        for yaw in YAWS_DEG
    ]
    with (REPORT / "ai_edge_obliquity.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    usable = [row for row in rows if row["usable"]]
    reference_cases = {
        f"yaw_{int(yaw)}_pitch_{int(pitch)}": evaluate(20.0, yaw, pitch)
        for yaw, pitch in ((0, 0), (30, 0), (45, 0), (0, 30), (30, 30))
    }
    summary = {
        "model": "nominal distortion-free pinhole projection into production opposite-edge/focal estimator",
        "rows": len(rows),
        "usable_rows": len(usable),
        "tag_detected_square_in": TAG_IN,
        "intrinsics_px": {"fx": FX, "fy": FY, "cx": CX, "cy": CY},
        "production_geometry_gates": {
            "minimum_edge_px": MIN_EDGE_PX,
            "maximum_edge_ratio": MAX_EDGE_RATIO,
            "minimum_area_px2": MIN_AREA_PX2,
            "minimum_fill_ratio": MIN_FILL,
        },
        "front_on_area_gate_range_in": float(
            TAG_IN * np.sqrt(FX * FY / MIN_AREA_PX2)
        ),
        "worst_usable_absolute_relative_error_fraction": float(
            max(abs(float(row["relative_error_fraction"])) for row in usable)
        ),
        "worst_usable_absolute_error_in": float(
            max(abs(float(row["range_error_in"])) for row in usable)
        ),
        "reference_cases_at_20_in": reference_cases,
        "interpretation": (
            "Even with perfect pixels and exact nominal intrinsics, the edge/focal "
            "average overestimates range as a square becomes oblique. Production "
            "geometry gates reject the most foreshortened/small cases but do not "
            "make the accepted range unbiased. This is deterministic model bias, "
            "not a measured accuracy distribution."
        ),
        "not_modeled": [
            "lens distortion or per-unit intrinsic error",
            "integer-corner quantization, blur, rolling exposure, or detector error",
            "tag print/placement tolerance or camera extrinsics",
        ],
    }
    (REPORT / "ai_edge_obliquity_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(15.5, 10), constrained_layout=True)
    fig.suptitle(
        "P8 Edge/Focal Range — Deterministic Obliquity Bias",
        fontsize=17,
        fontweight="bold",
    )
    colors = {0.0: "#58d6ff", 15.0: "#5cffad", 30.0: "#ffb347"}
    for ax, distance in zip(axes.flat[:3], DISTANCES_IN):
        for pitch in PITCHES_DEG:
            selected = [
                row for row in rows
                if row["distance_in"] == distance and row["pitch_deg"] == pitch
            ]
            x = np.asarray([row["yaw_deg"] for row in selected])
            y = 100.0 * np.asarray([row["relative_error_fraction"] for row in selected], dtype=float)
            valid = np.asarray([row["usable"] for row in selected], dtype=bool)
            ax.plot(x, y, color=colors[pitch], alpha=0.25, linestyle=":")
            ax.plot(x[valid], y[valid], color=colors[pitch], linewidth=2.4, label=f"pitch {pitch:.0f}°")
        ax.axhline(0.0, color="white", alpha=0.35, linewidth=1)
        ax.set(
            title=f"True lens-to-tag depth {distance:.0f} in",
            xlabel="Tag yaw from front-facing (deg)",
            ylabel="Estimated range bias (%)",
            xlim=(0, 65),
        )
        ax.grid(alpha=0.16)
        ax.legend(loc="upper left")

    ax = axes.flat[3]
    ax.axis("off")
    case30 = reference_cases["yaw_30_pitch_0"]
    case45 = reference_cases["yaw_45_pitch_0"]
    note = (
        "WHAT THIS IS\n"
        "Perfect synthetic corners, nominal 320×240 pinhole,\n"
        "then the exact onboard opposite-edge calculation.\n\n"
        "20-IN EXAMPLES (PITCH 0°)\n"
        f"yaw 30°: {100*float(case30['relative_error_fraction']):+.1f}% "
        f"({float(case30['range_error_in']):+.2f} in), usable={case30['usable']}\n"
        f"yaw 45°: {100*float(case45['relative_error_fraction']):+.1f}% "
        f"({float(case45['range_error_in']):+.2f} in), usable={case45['usable']}\n\n"
        "PRODUCTION GATES\n"
        f"edge ≥ {MIN_EDGE_PX:.0f}px, area ≥ {MIN_AREA_PX2:.0f}px², "
        f"edge ratio ≤ {MAX_EDGE_RATIO:.1f}\n"
        f"front-on area-gate ceiling ≈ {summary['front_on_area_gate_range_in']:.1f} in\n\n"
        "INTERPRETATION\n"
        "Solid segments pass current geometry gates; dotted portions fail.\n"
        "Gating removes extreme views but accepted range is still biased.\n"
        "Use this as an error mechanism, not a sensor accuracy claim."
    )
    ax.text(
        0.02, 0.98, note, va="top", fontsize=11.3, linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#58d6ff"},
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"ai_edge_obliquity_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
