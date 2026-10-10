#!/usr/bin/env python3
"""Fit a VEX GPS lens offset from surveyed fixed-center heading samples.

Accepted input rows:
    GPS_MOUNT_SAMPLE,index,x_in,y_in,robot_heading_deg
    index,x_in,y_in,robot_heading_deg
    x_in,y_in,robot_heading_deg

The robot's rotation center must be returned to the same marked floor point
before every sample. The fit estimates the lens offset in robot coordinates:
positive forward and positive right.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Sample:
    index: int
    x_in: float
    y_in: float
    heading_deg: float


@dataclass(frozen=True)
class Fit:
    center_x_in: float
    center_y_in: float
    forward_in: float
    right_in: float
    rms_in: float
    max_residual_in: float
    heading_resultant: float
    jackknife_forward_span_in: float
    jackknife_right_span_in: float
    opposite_pair_count: int
    opposite_forward_span_in: float
    opposite_right_span_in: float


def _solve(matrix: list[list[float]], vector: list[float]) -> list[float]:
    n = len(vector)
    augmented = [matrix[row][:] + [vector[row]] for row in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) < 1e-10:
            raise ValueError("sample headings do not provide a solvable fit")
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        scale = augmented[col][col]
        for item in range(col, n + 1):
            augmented[col][item] /= scale
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            for item in range(col, n + 1):
                augmented[row][item] -= factor * augmented[col][item]
    return [augmented[row][n] for row in range(n)]


def _basic_fit(samples: Sequence[Sample]) -> tuple[float, float, float, float]:
    if len(samples) < 4:
        raise ValueError("need at least four samples")
    ata = [[0.0] * 4 for _ in range(4)]
    atb = [0.0] * 4
    for sample in samples:
        h = math.radians(sample.heading_deg)
        sn, cs = math.sin(h), math.cos(h)
        rows = (
            ((1.0, 0.0, sn, cs), sample.x_in),
            ((0.0, 1.0, cs, -sn), sample.y_in),
        )
        for coefficients, observed in rows:
            for row in range(4):
                atb[row] += coefficients[row] * observed
                for col in range(4):
                    ata[row][col] += coefficients[row] * coefficients[col]
    return tuple(_solve(ata, atb))


def _residuals(
    samples: Sequence[Sample], cx: float, cy: float, forward: float, right: float
) -> list[float]:
    result: list[float] = []
    for sample in samples:
        h = math.radians(sample.heading_deg)
        predicted_x = cx + forward * math.sin(h) + right * math.cos(h)
        predicted_y = cy + forward * math.cos(h) - right * math.sin(h)
        result.append(math.hypot(sample.x_in - predicted_x, sample.y_in - predicted_y))
    return result


def _signed_heading_delta(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def _opposite_pairs(samples: Sequence[Sample]) -> list[tuple[float, float]]:
    pairs: list[tuple[float, float]] = []
    used: set[tuple[int, int]] = set()
    for i, first in enumerate(samples):
        candidates = [
            (abs(abs(_signed_heading_delta(other.heading_deg, first.heading_deg)) - 180.0), j)
            for j, other in enumerate(samples)
            if j != i
        ]
        if not candidates:
            continue
        error, j = min(candidates)
        if error > 20.0:
            continue
        key = tuple(sorted((i, j)))
        if key in used:
            continue
        used.add(key)
        second = samples[j]
        dx = 0.5 * (first.x_in - second.x_in)
        dy = 0.5 * (first.y_in - second.y_in)
        h = math.radians(first.heading_deg)
        forward = dx * math.sin(h) + dy * math.cos(h)
        right = dx * math.cos(h) - dy * math.sin(h)
        pairs.append((forward, right))
    return pairs


def fit_mount(samples: Sequence[Sample]) -> Fit:
    if len(samples) < 6:
        raise ValueError("need at least six well-spaced headings; eight is preferred")
    sum_sin = sum(math.sin(math.radians(s.heading_deg)) for s in samples)
    sum_cos = sum(math.cos(math.radians(s.heading_deg)) for s in samples)
    resultant = math.hypot(sum_sin, sum_cos) / len(samples)
    if resultant > 0.75:
        raise ValueError(
            "headings are too concentrated; cover the full 360 degrees around the same center"
        )

    cx, cy, forward, right = _basic_fit(samples)
    residuals = _residuals(samples, cx, cy, forward, right)
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))

    jackknife: list[tuple[float, float]] = []
    for omitted in range(len(samples)):
        subset = [sample for index, sample in enumerate(samples) if index != omitted]
        _, _, fwd, rgt = _basic_fit(subset)
        jackknife.append((fwd, rgt))
    fwd_values = [value[0] for value in jackknife]
    right_values = [value[1] for value in jackknife]

    opposite = _opposite_pairs(samples)
    if opposite:
        opposite_forward = [value[0] for value in opposite]
        opposite_right = [value[1] for value in opposite]
        opposite_forward_span = max(opposite_forward) - min(opposite_forward)
        opposite_right_span = max(opposite_right) - min(opposite_right)
    else:
        opposite_forward_span = math.inf
        opposite_right_span = math.inf

    return Fit(
        center_x_in=cx,
        center_y_in=cy,
        forward_in=forward,
        right_in=right,
        rms_in=rms,
        max_residual_in=max(residuals),
        heading_resultant=resultant,
        jackknife_forward_span_in=max(fwd_values) - min(fwd_values),
        jackknife_right_span_in=max(right_values) - min(right_values),
        opposite_pair_count=len(opposite),
        opposite_forward_span_in=opposite_forward_span,
        opposite_right_span_in=opposite_right_span,
    )


def parse_samples(lines: Iterable[str]) -> list[Sample]:
    samples: list[Sample] = []
    for line_number, raw in enumerate(lines, start=1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        row = next(csv.reader([text]))
        if row[0].strip() == "GPS_MOUNT_SAMPLE":
            row = row[1:]
        lowered = [cell.strip().lower() for cell in row]
        if any("heading" in cell or cell in {"x", "x_in", "y", "y_in"} for cell in lowered):
            continue
        try:
            if len(row) >= 4:
                index = int(float(row[0]))
                x_in, y_in, heading = map(float, row[1:4])
            elif len(row) == 3:
                index = len(samples) + 1
                x_in, y_in, heading = map(float, row)
            else:
                raise ValueError("expected 3 or 4 numeric columns")
        except ValueError as error:
            raise ValueError(f"line {line_number}: {error}") from error
        if not all(math.isfinite(value) for value in (x_in, y_in, heading)):
            raise ValueError(f"line {line_number}: values must be finite")
        samples.append(Sample(index, x_in, y_in, heading % 360.0))
    return samples


def quality(fit: Fit) -> tuple[str, list[str]]:
    issues: list[str] = []
    if fit.rms_in > 1.0:
        issues.append(f"fit RMS is {fit.rms_in:.2f} in (>1.00 in)")
    if fit.max_residual_in > 2.0:
        issues.append(f"largest stop residual is {fit.max_residual_in:.2f} in (>2.00 in)")
    if fit.jackknife_forward_span_in > 1.0 or fit.jackknife_right_span_in > 1.0:
        issues.append("leave-one-out offset changes by more than 1.00 in")
    if fit.opposite_pair_count < 2:
        issues.append("fewer than two near-opposite heading pairs were captured")
    elif fit.opposite_forward_span_in > 1.5 or fit.opposite_right_span_in > 1.5:
        issues.append("opposite-heading offset estimates disagree by more than 1.50 in")
    return ("PASS" if not issues else "REJECT / REPEAT TEST", issues)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", help="sample file; omit to read stdin")
    parser.add_argument("--port", type=int, default=10, help="GPS smart port for paste-ready output")
    parser.add_argument("--facing", type=float, default=90.0, help="camera facing_deg already verified by a straight-line test")
    args = parser.parse_args(argv)

    lines = Path(args.csv).read_text().splitlines() if args.csv else sys.stdin
    try:
        samples = parse_samples(lines)
        fit = fit_mount(samples)
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    verdict, issues = quality(fit)
    print(f"samples: {len(samples)}")
    print(f"center:  x={fit.center_x_in:.3f} in  y={fit.center_y_in:.3f} in")
    print(f"offset:  forward={fit.forward_in:.3f} in  right={fit.right_in:.3f} in")
    print(f"fit:     RMS={fit.rms_in:.3f} in  max={fit.max_residual_in:.3f} in")
    print(
        "stability: leave-one-out span "
        f"forward={fit.jackknife_forward_span_in:.3f} in "
        f"right={fit.jackknife_right_span_in:.3f} in"
    )
    if fit.opposite_pair_count:
        print(
            f"opposites: {fit.opposite_pair_count} pairs; span "
            f"forward={fit.opposite_forward_span_in:.3f} in "
            f"right={fit.opposite_right_span_in:.3f} in"
        )
    else:
        print("opposites: none")
    print(f"coverage resultant: {fit.heading_resultant:.3f} (0 is ideal)")
    print(f"quality: {verdict}")
    for issue in issues:
        print(f"  - {issue}")
    print(
        "paste: gpsreset::init(chassis, "
        f"{args.port}, /*forward_in=*/{fit.forward_in:.3f}, "
        f"/*right_in=*/{fit.right_in:.3f}, /*facing_deg=*/{args.facing:.1f});"
    )
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
