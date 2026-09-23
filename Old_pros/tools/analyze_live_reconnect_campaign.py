#!/usr/bin/env python3
"""Analyze the 2026-08-23 reconnect/P8 live qualification captures."""

from __future__ import annotations

import bisect
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from capture_sensor_log import parse_frame


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
DATASETS = [
    REPORT / "p8_scan_reconnect_02",
    REPORT / "p8_heading_sweep_live_01",
    REPORT / "p8_full_heading_sweep_live_01",
]
# The current Override field landmark table uses IDs 0-4. Circle21h7 can
# decode IDs through 37, so a syntactically valid family ID is not necessarily
# a landmark that exists on this field.
OFFICIAL_FIELD_TAG_IDS = frozenset(range(5))
# These captures were produced when firmware multiplied the P8 corner span by
# the medium print's 0.875-in outer outline. The returned quad actually spans
# the 0.625-in inner detection square, so rescale historical distances by 5/7.
P8_HISTORICAL_RANGE_SCALE = 0.625 / 0.875
NUMBER = r"[-+]?(?:\d+(?:\.\d+)?|nan|inf)"


def scalar(payload: str, key: str, cast=float):
    match = re.search(rf"(?:^|\s){re.escape(key)}=({NUMBER})", payload)
    if not match:
        return math.nan if cast is float else None
    try:
        return cast(match.group(1))
    except ValueError:
        return math.nan if cast is float else None


def parse_vision(line: str) -> dict[str, object] | None:
    marker = "VISION_SHADOW"
    if marker not in line:
        return None
    payload = line[line.index(marker):]
    # Serial packets occasionally interleave one byte into a key name. Match
    # the stable suffix of "bearing" while keeping all other fields strict.
    bearing_match = re.search(rf"\S*earing=({NUMBER})", payload)
    center_match = re.search(rf"(?:^|\s)center=({NUMBER}),({NUMBER})", payload)
    reason_match = re.search(r"(?:^|\s)reason=([^\s]+)", payload)
    required = {
        "robot_ms": scalar(payload, "t", int),
        "poll": scalar(payload, "poll", int),
        "count": scalar(payload, "count", int),
        "tag": scalar(payload, "tag", int),
        "area": scalar(payload, "area"),
        "horizontal": scalar(payload, "horizontal") * P8_HISTORICAL_RANGE_SCALE,
        "range_3d": scalar(payload, "range") * P8_HISTORICAL_RANGE_SCALE,
        "repeat": scalar(payload, "repeat", int),
        "valid": scalar(payload, "valid", int),
    }
    if any(value is None for value in required.values()):
        return None
    if bearing_match:
        bearing = float(bearing_match.group(1))
    elif center_match:
        # The CDC stream occasionally interleaves one byte into the word
        # "bearing". The published center survives and reproduces the onboard
        # pinhole bearing using the same documented 74-degree HFOV focal length.
        center_x = float(center_match.group(1))
        bearing = math.degrees(math.atan2(center_x - 160.0, 212.34))
    else:
        bearing = math.nan
    return {
        **required,
        "bearing": bearing,
        "elevation": scalar(payload, "elevation"),
        "reason": reason_match.group(1) if reason_match else "parse_missing",
    }


def normalize_signed(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def read_raw(dataset: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    text = (dataset / "raw.log").read_bytes().decode("utf-8", errors="replace")
    d4: list[dict[str, object]] = []
    vision: list[dict[str, object]] = []
    d4_segment = 0
    vision_segment = 0
    last_d4_ms = -1
    last_vision_ms = -1
    for line_index, line in enumerate(text.splitlines()):
        frame = parse_frame(line, 0.0)
        if frame is not None:
            robot_ms = int(frame["robot_ms"])
            if robot_ms + 1000 < last_d4_ms:
                d4_segment += 1
            last_d4_ms = robot_ms
            frame.update(dataset=dataset.name, segment=d4_segment, line_index=line_index)
            d4.append(frame)
        optical = parse_vision(line)
        if optical is not None:
            robot_ms = int(optical["robot_ms"])
            if robot_ms + 1000 < last_vision_ms:
                vision_segment += 1
            last_vision_ms = robot_ms
            optical.update(
                dataset=dataset.name,
                segment=vision_segment,
                line_index=line_index,
            )
            vision.append(optical)
    return d4, vision


def join_vision_to_imu(
    d4: list[dict[str, object]], vision: list[dict[str, object]]
) -> list[dict[str, object]]:
    by_segment: dict[int, list[dict[str, object]]] = defaultdict(list)
    for row in d4:
        by_segment[int(row["segment"])].append(row)
    joined: list[dict[str, object]] = []
    for optical in vision:
        segment_rows = by_segment.get(int(optical["segment"]), [])
        if not segment_rows:
            continue
        times = [int(row["robot_ms"]) for row in segment_rows]
        target = int(optical["robot_ms"])
        index = bisect.bisect_left(times, target)
        candidates = []
        if index < len(segment_rows):
            candidates.append(segment_rows[index])
        if index:
            candidates.append(segment_rows[index - 1])
        nearest = min(candidates, key=lambda row: abs(int(row["robot_ms"]) - target))
        age_ms = abs(int(nearest["robot_ms"]) - target)
        if age_ms > 120:
            continue
        imu = float(nearest["imu"])
        bearing = float(optical["bearing"])
        row = {**optical, "imu": imu, "join_age_ms": age_ms}
        row["landmark_direction"] = (
            normalize_signed(imu + bearing)
            if math.isfinite(imu) and math.isfinite(bearing)
            else math.nan
        )
        joined.append(row)
    return joined


def stats(values) -> dict[str, float | int | None]:
    finite = np.asarray([float(v) for v in values if math.isfinite(float(v))])
    if not len(finite):
        return {"count": 0, "min": None, "median": None, "max": None, "std": None}
    return {
        "count": int(len(finite)),
        "min": float(np.min(finite)),
        "median": float(np.median(finite)),
        "max": float(np.max(finite)),
        "std": float(np.std(finite)),
    }


def cluster_directions(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    clusters: list[list[dict[str, object]]] = []
    for tag in sorted({int(row["tag"]) for row in rows if int(row["tag"]) >= 0}):
        tagged = sorted(
            [
                row
                for row in rows
                if int(row["tag"]) == tag
                and math.isfinite(float(row["landmark_direction"]))
            ],
            key=lambda row: float(row["landmark_direction"]),
        )
        for row in tagged:
            if not clusters or int(clusters[-1][0]["tag"]) != tag:
                clusters.append([row])
            elif abs(
                float(row["landmark_direction"])
                - float(np.median([r["landmark_direction"] for r in clusters[-1]]))
            ) <= 12.0:
                clusters[-1].append(row)
            else:
                clusters.append([row])
    result = []
    for index, cluster in enumerate(clusters, start=1):
        valid = [row for row in cluster if int(row["valid"]) == 1]
        result.append(
            {
                "cluster": index,
                "tag_id": int(cluster[0]["tag"]),
                "candidate_frames": len(cluster),
                "strict_valid_frames": len(valid),
                "strict_valid_fraction_of_candidates": len(valid) / len(cluster),
                "repeated_fraction": sum(int(row["repeat"]) for row in cluster)
                / len(cluster),
                "landmark_direction_deg": stats(
                    row["landmark_direction"] for row in cluster
                ),
                "horizontal_range_in": stats(row["horizontal"] for row in cluster),
                "bearing_deg": stats(row["bearing"] for row in cluster),
                "area_px2": stats(row["area"] for row in cluster),
                "reasons": dict(Counter(str(row["reason"]) for row in cluster)),
            }
        )
    return result


def stationary_summary() -> dict[str, object]:
    path = REPORT / "reconnect_stationary_01" / "telemetry.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    def col(name: str):
        return np.asarray([float(row[name]) for row in rows])
    gps_x = col("gps_x") * 39.37007874
    gps_y = col("gps_y") * 39.37007874
    imu = col("imu")
    return {
        "frames": len(rows),
        "gps_x_span_in": float(np.ptp(gps_x)),
        "gps_y_span_in": float(np.ptp(gps_y)),
        "gps_endpoint_displacement_in": float(
            math.hypot(gps_x[-1] - gps_x[0], gps_y[-1] - gps_y[0])
        ),
        "imu_span_deg": float(np.ptp(imu)),
        "imu_std_deg": float(np.std(imu)),
        "drive_encoder_max_span_deg": max(
            float(np.ptp(col(name))) for name in ("m17", "m18", "m11", "m13")
        ),
        "p5_span_centideg": float(np.ptp(col("h5"))),
    }


def rotation_summary() -> dict[str, object]:
    path = REPORT / "p8_heading_sweep_live_01" / "telemetry.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    def col(name: str):
        return np.asarray([float(row[name]) for row in rows])
    gps_x = col("gps_x") * 39.37007874
    gps_y = col("gps_y") * 39.37007874
    imu = col("imu")
    wheel_circumference = math.pi * 2.433055
    left = (col("m17") + col("m18")) / 2.0
    right = (col("m11") + col("m13")) / 2.0
    center_deg = (left + right) / 2.0
    center_in = (center_deg - center_deg[0]) * wheel_circumference / 360.0
    h5_in = (col("h5") - col("h5")[0]) / 100.0 / 360.0 * math.pi * 2.0
    p1 = col("p1_mm")
    encoder_span = float(np.ptp(center_in))
    encoder_endpoint = float(center_in[-1])
    gps_max_displacement = float(
        np.max(np.hypot(gps_x - gps_x[0], gps_y - gps_y[0]))
    )
    gps_endpoint_displacement = float(
        math.hypot(gps_x[-1] - gps_x[0], gps_y[-1] - gps_y[0])
    )
    return {
        "frames": len(rows),
        "imu_min_deg": float(np.min(imu)),
        "imu_max_deg": float(np.max(imu)),
        "imu_endpoint_residual_deg": float(imu[-1] - imu[0]),
        "gps_x_span_in": float(np.ptp(gps_x)),
        "gps_y_span_in": float(np.ptp(gps_y)),
        "gps_max_observed_displacement_in": gps_max_displacement,
        "gps_endpoint_observed_displacement_in": gps_endpoint_displacement,
        # A scalar wheel-center path cannot reproduce the GPS field-space
        # vector, but subtracting its full span gives a conservative lower
        # bound on the disagreement during this nominal turn/return sweep.
        "gps_min_excess_over_encoder_span_in": max(
            0.0, gps_max_displacement - encoder_span
        ),
        "encoder_center_span_in": encoder_span,
        "encoder_center_endpoint_in": encoder_endpoint,
        "p5_span_in": float(np.ptp(h5_in)),
        "p1_physical_return_fraction": float(np.mean((p1 >= 20) & (p1 < 9999))),
    }


def main() -> None:
    all_joined: list[dict[str, object]] = []
    dataset_counts = {}
    for dataset in DATASETS:
        d4, vision = read_raw(dataset)
        joined = join_vision_to_imu(d4, vision)
        all_joined.extend(joined)
        dataset_counts[dataset.name] = {
            "d4_frames": len(d4),
            "p8_polls": len(vision),
            "joined_p8_polls": len(joined),
            "candidate_polls": sum(int(row["tag"]) >= 0 for row in joined),
            "strict_valid_polls": sum(int(row["valid"]) == 1 for row in joined),
        }

    candidates = [row for row in all_joined if int(row["tag"]) >= 0]
    non_field_candidates = [
        row for row in candidates if int(row["tag"]) not in OFFICIAL_FIELD_TAG_IDS
    ]
    clusters = cluster_directions(candidates)
    summary = {
        "scope": "successful live reconnect/P8 runs only; failed focused routine excluded",
        "datasets": dataset_counts,
        "stationary": stationary_summary(),
        "reversible_rotation": rotation_summary(),
        "p8": {
            "polls": len(all_joined),
            "candidate_polls": len(candidates),
            "strict_valid_polls": sum(int(row["valid"]) == 1 for row in all_joined),
            "candidate_fraction_all_headings": len(candidates) / len(all_joined),
            "strict_valid_fraction_all_headings": sum(
                int(row["valid"]) == 1 for row in all_joined
            ) / len(all_joined),
            "candidate_ids": dict(Counter(str(row["tag"]) for row in candidates)),
            "non_field_candidate_ids": dict(
                Counter(str(row["tag"]) for row in non_field_candidates)
            ),
            "non_field_candidates_strict_valid": sum(
                int(row["valid"]) == 1 for row in non_field_candidates
            ),
            "candidate_reasons": dict(
                Counter(str(row["reason"]) for row in candidates)
            ),
            "clusters": clusters,
            "limitations": [
                "No tape/laser range truth was available.",
                "Camera-to-robot extrinsics and lens distortion remain unmeasured.",
                "Duplicate tag IDs are separated only by IMU+bearing direction clusters.",
                "Horizontal-range spread combines pixel quantization, obliquity bias, and camera offset during rotation.",
            ],
        },
    }

    csv_path = REPORT / "live_p8_heading_samples.csv"
    fields = [
        "dataset", "segment", "robot_ms", "poll", "count", "tag", "valid",
        "reason", "repeat", "imu", "bearing", "landmark_direction",
        "horizontal", "range_3d", "elevation", "area", "join_age_ms",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in all_joined:
            writer.writerow({field: row.get(field, "") for field in fields})

    summary_path = REPORT / "live_reconnect_sensor_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=False)
    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.13, top=0.90,
                        hspace=0.34, wspace=0.22)
    fig.suptitle("Live reconnect sensor qualification — 2026-08-23", fontsize=16, weight="bold")

    ax = axes[0, 0]
    colors = {0: "#f59e0b", 3: "#22c55e"}
    for tag in sorted({int(row["tag"]) for row in candidates}):
        rows = [row for row in candidates if int(row["tag"]) == tag]
        ax.scatter(
            [row["imu"] for row in rows],
            [row["bearing"] for row in rows],
            s=14,
            alpha=0.65,
            label=(
                f"Tag {tag}"
                if tag in OFFICIAL_FIELD_TAG_IDS
                else f"non-field raw ID {tag}"
            ),
            color=colors.get(tag),
        )
    ax.set_title("P8 bearing versus P6 heading")
    ax.set_xlabel("P6 relative heading (deg)")
    ax.set_ylabel("P8 bearing (deg, right positive)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[0, 1]
    for tag in sorted({int(row["tag"]) for row in candidates}):
        rows = [row for row in candidates if int(row["tag"]) == tag]
        ax.scatter(
            [row["bearing"] for row in rows],
            [row["horizontal"] for row in rows],
            s=14,
            alpha=0.65,
            label=(
                f"Tag {tag}"
                if tag in OFFICIAL_FIELD_TAG_IDS
                else f"non-field raw ID {tag}"
            ),
            color=colors.get(tag),
        )
    ax.set_title("P8 edge/focal horizontal range")
    ax.set_xlabel("Bearing (deg)")
    ax.set_ylabel("Reported horizontal range (in)")
    ax.grid(alpha=0.25)

    ax = axes[1, 0]
    reason_counts = Counter(str(row["reason"]) for row in candidates)
    labels, values = zip(*reason_counts.most_common()) if reason_counts else ([], [])
    ax.bar(labels, values, color="#38bdf8")
    ax.set_title("P8 raw candidates by strict geometry result")
    ax.set_ylabel("Polls")
    ax.tick_params(axis="x", rotation=25)
    ax.grid(axis="y", alpha=0.25)

    ax = axes[1, 1]
    stationary = summary["stationary"]
    rotation = summary["reversible_rotation"]
    labels = [
        "GPS X\nstationary",
        "GPS Y\nstationary",
        "GPS path\nturn sweep",
        "P5 travel\nturn sweep",
    ]
    values = [
        stationary["gps_x_span_in"],
        stationary["gps_y_span_in"],
        rotation["gps_max_observed_displacement_in"],
        rotation["p5_span_in"],
    ]
    bars = ax.bar(labels, values, color=["#f97316", "#f97316", "#ef4444", "#64748b"])
    ax.set_title("Absolute sensors can look healthy while wrong")
    ax.set_ylabel("Observed span / displacement (in)")
    ax.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.2f}", ha="center", va="bottom")

    note = (
        "P8 correction remained disabled. The turn/return accumulated 2.10 in of apparent "
        "encoder-center travel, but GPS moved 24.22 in (at least 19.06 in excess).\n"
        "P5 is mechanically inactive. Values are consistency evidence, not surveyed accuracy."
    )
    fig.text(0.5, 0.025, note, ha="center", va="bottom", fontsize=9)
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"live_reconnect_sensor_dashboard.{suffix}", dpi=180)
    plt.close(fig)

    print(json.dumps(summary, indent=2))
    print(f"wrote {csv_path}")
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
