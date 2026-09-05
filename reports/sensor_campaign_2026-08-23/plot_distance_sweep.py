#!/usr/bin/env python3
"""Parse the live distance sweep and render a compact calibration dashboard."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
RAW = ROOT / "distance_sweep_live_01" / "raw.log"
OUT_CSV = ROOT / "distance_sweep_live_results.csv"
OUT_JSON = ROOT / "distance_sweep_live_summary.json"
OUT_PNG = ROOT / "distance_sweep_live_dashboard.png"
OUT_SVG = ROOT / "distance_sweep_live_dashboard.svg"

BACK_RE = re.compile(
    r"SWEEP target_in=(?P<target>[\d.]+) phase=back "
    r"encoder_in=(?P<encoder>[\d.-]+) gps_in=(?P<gps>[\d.-]+) "
    r"difference_in=(?P<difference>[\d.-]+) "
    r"gps_heading_delta=(?P<gps_heading>[\d.-]+).*?"
    r"imu_delta_deg=(?P<imu_heading>[\d.-]+) "
    r"imu_max_abs_deg=(?P<imu_max>[\d.-]+)"
)
RETURN_RE = re.compile(
    r"SWEEP target_in=(?P<target>[\d.]+) phase=return "
    r"encoder_residual_in=(?P<encoder_residual>[\d.-]+) "
    r"gps_residual_in=(?P<gps_residual>[\d.-]+) "
    r"gps_heading_delta=(?P<gps_heading_residual>[\d.-]+).*?"
    r"imu_residual_deg=(?P<imu_heading_residual>[\d.-]+)"
)


def parse() -> list[dict[str, float]]:
    text = RAW.read_bytes().decode("utf-8", errors="ignore")
    rows: dict[float, dict[str, float]] = {}
    for match in BACK_RE.finditer(text):
        values = {key: float(value) for key, value in match.groupdict().items()}
        rows[values["target"]] = values
    for match in RETURN_RE.finditer(text):
        values = {key: float(value) for key, value in match.groupdict().items()}
        rows.setdefault(values["target"], {}).update(values)
    return [rows[target] for target in sorted(rows)]


def main() -> None:
    rows = parse()
    if len(rows) != 3:
        raise SystemExit(f"expected 3 complete sweep rows, found {len(rows)}")

    fields = list(rows[0])
    with OUT_CSV.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    encoder = np.array([row["encoder"] for row in rows])
    gps = np.array([row["gps"] for row in rows])
    targets = np.array([row["target"] for row in rows])
    scale = float(np.dot(encoder, gps) / np.dot(encoder, encoder))
    gps_fit = scale * encoder
    residual = gps - gps_fit
    rmse = float(np.sqrt(np.mean(residual**2)))
    ratios = gps / encoder
    return_gps = np.array([row["gps_residual"] for row in rows])
    return_encoder = np.array([row["encoder_residual"] for row in rows])
    max_heading = np.array([row["imu_max"] for row in rows])

    summary = {
        "encoder_to_gps_scale": scale,
        "effective_wheel_diameter_in": 2.75 * scale,
        "fit_rmse_in": rmse,
        "per_leg_gps_encoder_ratio": ratios.tolist(),
        "max_gps_return_residual_in": float(return_gps.max()),
        "max_encoder_return_residual_in": float(return_encoder.max()),
        "max_heading_excursion_deg": float(max_heading.max()),
        "note": "One low-speed out-and-back run; retain as evidence, not final calibration.",
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n")

    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle("VEX Motion Characterization — Low-Speed Distance Sweep", fontsize=17)

    ax = axes[0, 0]
    ax.plot(targets, encoder, "o-", label="drive encoder")
    ax.plot(targets, gps, "o-", label="GPS displacement")
    ax.plot(targets, targets, "--", color="0.55", label="commanded")
    ax.set(title="Outward displacement", xlabel="commanded (in)", ylabel="measured (in)")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[0, 1]
    xx = np.linspace(0, encoder.max() * 1.05, 100)
    ax.scatter(encoder, gps, s=70, color="#68d7ff")
    ax.plot(xx, scale * xx, label=f"GPS = {scale:.4f} × encoder")
    ax.plot(xx, xx, "--", color="0.55", label="1:1")
    ax.set(title=f"Encoder scale fit (RMSE {rmse:.3f} in)", xlabel="encoder (in)", ylabel="GPS (in)")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[1, 0]
    width = 0.34
    ax.bar(targets - width / 2, return_encoder, width, label="encoder")
    ax.bar(targets + width / 2, return_gps, width, label="GPS")
    ax.set(title="Residual after returning to start", xlabel="outward target (in)", ylabel="position residual (in)")
    ax.grid(axis="y", alpha=0.2)
    ax.legend()

    ax = axes[1, 1]
    gps_heading = np.array([row["gps_heading"] for row in rows])
    imu_heading = np.array([row["imu_heading"] for row in rows])
    ax.plot(targets, gps_heading, "o-", label="GPS endpoint")
    ax.plot(targets, imu_heading, "o-", label="IMU endpoint")
    ax.plot(targets, max_heading, "o--", label="IMU max excursion")
    ax.set(title="Unwanted heading during straight travel", xlabel="commanded (in)", ylabel="absolute heading change (deg)")
    ax.grid(alpha=0.2)
    ax.legend()

    fig.text(
        0.5,
        0.005,
        f"Estimated effective wheel diameter: {2.75 * scale:.3f} in  •  max GPS return error: {return_gps.max():.3f} in  •  one-run estimate",
        ha="center",
        color="0.8",
    )
    fig.savefig(OUT_PNG, dpi=180)
    fig.savefig(OUT_SVG)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
