#!/usr/bin/env python3
"""Plot AI Vision reset diagnostics over a schematic VEX Override field."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle, Polygon

FIELD = 70.205
GOALS = [
    (0.0, 0.0),
    (-23.55, 47.10), (23.54, 47.10),
    (-47.10, 23.55), (47.09, 23.55),
    (-47.10, -23.54), (47.09, -23.54),
    (-23.55, -47.09), (23.54, -47.09),
]
PINS = [
    (0.0, 23.55), (23.55, 23.55), (47.10, 47.10), (70.0, 23.55),
    (0.0, -23.55), (-23.55, -23.55), (-47.10, -47.10), (-70.0, -23.55),
]


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else math.nan
    except (TypeError, ValueError):
        return math.nan


def draw_override(ax):
    ax.add_patch(Rectangle((-FIELD, -FIELD), 2 * FIELD, 2 * FIELD,
                           fill=False, linewidth=2.0))
    # Midfield diamond.
    d = 23.55
    ax.add_patch(Polygon([(0, d), (d, 0), (0, -d), (-d, 0)],
                         closed=True, fill=False, linewidth=1.5))
    # Four diagonal autonomous-line segments / object lanes.
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        ax.plot([sx * d, sx * 2 * d], [sy * d, sy * 2 * d], linewidth=1.2)
    # Goals and the pins most useful for reset/path diagnostics.
    for x, y in GOALS:
        ax.add_patch(Circle((x, y), 2.8, fill=False, linewidth=1.2))
    for x, y in PINS:
        ax.add_patch(Circle((x, y), 0.9, fill=False, linewidth=1.0))
    # Approximate toggle spans on the four walls.
    t = 12.9
    ax.plot([-t, t], [FIELD, FIELD], linewidth=4.0)
    ax.plot([-t, t], [-FIELD, -FIELD], linewidth=4.0)
    ax.plot([FIELD, FIELD], [-t, t], linewidth=4.0)
    ax.plot([-FIELD, -FIELD], [-t, t], linewidth=4.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="aivision_reset.csv copied from the V5 SD card")
    ap.add_argument("--out")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    log = Path(args.log)
    rows = []
    with log.open(newline="") as f:
        for r in csv.DictReader(f):
            for k in ("odom_x", "odom_y", "odom_theta", "vision_x", "vision_y", "vision_theta"):
                r[k] = num(r.get(k))
            r["accepted"] = r.get("accepted", "0") == "1"
            rows.append(r)
    if not rows:
        raise SystemExit("log contains no rows")

    fig, ax = plt.subplots(figsize=(8, 8))
    draw_override(ax)

    ox = np.array([r["odom_x"] for r in rows])
    oy = np.array([r["odom_y"] for r in rows])
    ot = np.deg2rad(np.array([r["odom_theta"] for r in rows]))
    good = np.isfinite(ox) & np.isfinite(oy)
    ax.plot(ox[good], oy[good], linewidth=2, label="LemLib before reset")

    vision = [r for r in rows if math.isfinite(r["vision_x"]) and math.isfinite(r["vision_y"])]
    if vision:
        ax.scatter([r["vision_x"] for r in vision], [r["vision_y"] for r in vision],
                   s=10, alpha=0.35, label="AI Vision pose")

    accepted = [r for r in rows if r["accepted"] and math.isfinite(r["vision_x"])]
    if accepted:
        ax.scatter([r["vision_x"] for r in accepted], [r["vision_y"] for r in accepted],
                   marker="x", s=75, linewidths=2, label="Applied reset")
        for r in accepted:
            if math.isfinite(r["odom_x"]) and math.isfinite(r["odom_y"]):
                ax.plot([r["odom_x"], r["vision_x"]],
                        [r["odom_y"], r["vision_y"]], linewidth=1, alpha=0.75)
            if math.isfinite(r["vision_theta"]):
                th = math.radians(r["vision_theta"])
                ax.arrow(r["vision_x"], r["vision_y"],
                         4.0 * math.sin(th), 4.0 * math.cos(th),
                         width=0.12, head_width=1.1, length_includes_head=True, alpha=0.75)

    ids = np.flatnonzero(good)
    if len(ids):
        step = max(1, len(ids) // 30)
        ids = ids[::step]
        ax.quiver(ox[ids], oy[ids], np.sin(ot[ids]), np.cos(ot[ids]),
                  angles="xy", scale_units="xy", scale=0.20, width=0.003, alpha=0.45)

    ax.set_xlim(-72, 72)
    ax.set_ylim(-72, 72)
    ax.set_aspect("equal")
    ax.set_xlabel("X (inches)")
    ax.set_ylabel("Y (inches)")
    ax.set_title(f"AI Vision Reset Analysis: {log.name}")
    ax.grid(alpha=0.15)
    ax.legend(loc="upper right")
    out = Path(args.out) if args.out else log.with_name("aivision_reset_diagnosis.png")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    print(out)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
