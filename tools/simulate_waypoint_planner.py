#!/usr/bin/env python3
"""Offline A* waypoint-planner study using production's provisional clearances."""

from __future__ import annotations

import csv
import heapq
import json
import math
import random
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include" / "localization_config.hpp"
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
GRID_STEP = 2.0
TRIALS = 3000
SEED = 560823


def config_number(source: str, name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([-+0-9.]+)", source)
    if not match:
        raise RuntimeError(f"missing {name}")
    return float(match.group(1))


def load_geometry() -> tuple[float, float, list[tuple[float, float]]]:
    source = CONFIG.read_text()
    wall = config_number(source, "kPhysicalWallHalfSpanIn")
    wall_clearance = config_number(source, "kNavigationProvisionalWallClearanceIn")
    goal_clearance = config_number(source, "kNavigationProvisionalGoalClearanceIn")
    table = source.split("kGoalTagLandmarks{{", 1)[1].split("}};", 1)[0]
    goals = [
        (float(x), float(y))
        for x, y in re.findall(
            r'\{"[^"]+",\s*\d+,\s*GoalColor::\w+,\s*([-+0-9.]+),\s*([-+0-9.]+)\}',
            table,
        )
    ]
    return wall - wall_clearance, goal_clearance, goals


LIMIT, CLEARANCE, GOALS = load_geometry()
GRID_COORDS = np.arange(math.ceil(-LIMIT / GRID_STEP) * GRID_STEP,
                        math.floor(LIMIT / GRID_STEP) * GRID_STEP + 0.1,
                        GRID_STEP)


def point_clear(point: tuple[float, float]) -> bool:
    x, y = point
    return (
        abs(x) <= LIMIT
        and abs(y) <= LIMIT
        and all(math.hypot(x - gx, y - gy) >= CLEARANCE for gx, gy in GOALS)
    )


def point_segment_distance(
    point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segment_clear(a: tuple[float, float], b: tuple[float, float]) -> bool:
    return point_clear(a) and point_clear(b) and all(
        point_segment_distance(goal, a, b) >= CLEARANCE for goal in GOALS
    )


N = len(GRID_COORDS)
POINTS = [(float(x), float(y)) for y in GRID_COORDS for x in GRID_COORDS]
FREE = [point_clear(point) for point in POINTS]


def grid_index(ix: int, iy: int) -> int:
    return iy * N + ix


NEIGHBORS: list[list[tuple[int, float]]] = [[] for _ in POINTS]
for iy in range(N):
    for ix in range(N):
        current = grid_index(ix, iy)
        if not FREE[current]:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < N and 0 <= ny < N):
                continue
            neighbor = grid_index(nx, ny)
            if FREE[neighbor] and segment_clear(POINTS[current], POINTS[neighbor]):
                NEIGHBORS[current].append((neighbor, GRID_STEP * math.hypot(dx, dy)))


def connector(point: tuple[float, float]) -> int | None:
    candidates = sorted(
        (math.hypot(point[0] - candidate[0], point[1] - candidate[1]), index)
        for index, candidate in enumerate(POINTS)
        if FREE[index]
    )
    for _, index in candidates[:64]:
        if segment_clear(point, POINTS[index]):
            return index
    return None


def plan(start: tuple[float, float], target: tuple[float, float]) -> list[tuple[float, float]] | None:
    if segment_clear(start, target):
        return [start, target]
    source = connector(start)
    destination = connector(target)
    if source is None or destination is None:
        return None
    g = [math.inf] * len(POINTS)
    parent = [-1] * len(POINTS)
    g[source] = 0.0
    heap = [(math.hypot(POINTS[source][0] - target[0], POINTS[source][1] - target[1]), source)]
    closed: set[int] = set()
    while heap:
        _, current = heapq.heappop(heap)
        if current in closed:
            continue
        if current == destination:
            break
        closed.add(current)
        for neighbor, cost in NEIGHBORS[current]:
            candidate = g[current] + cost
            if candidate + 1e-9 >= g[neighbor]:
                continue
            g[neighbor] = candidate
            parent[neighbor] = current
            heuristic = math.hypot(
                POINTS[neighbor][0] - target[0], POINTS[neighbor][1] - target[1]
            )
            heapq.heappush(heap, (candidate + heuristic, neighbor))
    if source != destination and parent[destination] < 0:
        return None
    indices = [destination]
    while indices[-1] != source:
        indices.append(parent[indices[-1]])
    raw = [start] + [POINTS[index] for index in reversed(indices)] + [target]

    # Greedy line-of-sight smoothing minimizes turns without weakening the
    # exact same circular-obstacle clearance check.
    smoothed = [raw[0]]
    cursor = 0
    while cursor < len(raw) - 1:
        furthest = cursor + 1
        for candidate in range(len(raw) - 1, cursor, -1):
            if segment_clear(raw[cursor], raw[candidate]):
                furthest = candidate
                break
        smoothed.append(raw[furthest])
        cursor = furthest
    return smoothed


def route_length(route: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route, route[1:]))


def random_point(rng: random.Random) -> tuple[float, float]:
    while True:
        point = (rng.uniform(-LIMIT, LIMIT), rng.uniform(-LIMIT, LIMIT))
        if point_clear(point):
            return point


def main() -> None:
    rng = random.Random(SEED)
    rows = []
    examples: list[tuple[list[tuple[float, float]], float]] = []
    for trial in range(TRIALS):
        start, target = random_point(rng), random_point(rng)
        direct = segment_clear(start, target)
        route = plan(start, target)
        direct_distance = math.hypot(target[0] - start[0], target[1] - start[1])
        planned_distance = route_length(route) if route else math.nan
        row = {
            "trial": trial,
            "start_x_in": start[0],
            "start_y_in": start[1],
            "target_x_in": target[0],
            "target_y_in": target[1],
            "direct_safe": int(direct),
            "planned": int(route is not None),
            "waypoints_including_endpoints": len(route) if route else 0,
            "direct_distance_in": direct_distance,
            "planned_distance_in": planned_distance,
            "stretch": planned_distance / direct_distance if route and direct_distance > 1e-9 else math.nan,
        }
        rows.append(row)
        if route and not direct:
            examples.append((route, row["stretch"]))

    planned = [row for row in rows if row["planned"]]
    indirect = [row for row in planned if not row["direct_safe"]]
    stretches = np.asarray([row["stretch"] for row in indirect])
    waypoint_counts = np.asarray([row["waypoints_including_endpoints"] for row in indirect])
    summary = {
        "trials": TRIALS,
        "seed": SEED,
        "grid_step_in": GRID_STEP,
        "field_center_limit_in": LIMIT,
        "goal_center_clearance_in": CLEARANCE,
        "direct_safe_percent": 100.0 * sum(row["direct_safe"] for row in rows) / TRIALS,
        "planner_success_percent": 100.0 * len(planned) / TRIALS,
        "indirect_routes": len(indirect),
        "indirect_median_stretch": float(np.median(stretches)),
        "indirect_p95_stretch": float(np.percentile(stretches, 95)),
        "indirect_max_stretch": float(np.max(stretches)),
        "indirect_median_waypoints_including_endpoints": float(np.median(waypoint_counts)),
        "indirect_p95_waypoints_including_endpoints": float(np.percentile(waypoint_counts, 95)),
        "warning": (
            "Host-only geometry study. It models robot centerlines around fixed Goal "
            "centers, not the measured/expanded footprint, turn sweep, movable objects, "
            "other robots, controller error, or localization uncertainty."
        ),
    }
    with (REPORT / "waypoint_planner_trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (REPORT / "waypoint_planner_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    examples.sort(key=lambda item: item[1], reverse=True)
    chosen = [examples[0], examples[len(examples) // 2], examples[-1]]
    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16, 6), constrained_layout=True)
    fig.patch.set_facecolor("#07111f")
    for axis, (route, stretch) in zip(axes, chosen):
        axis.set_facecolor("#0c1b2e")
        axis.add_patch(Rectangle((-LIMIT, -LIMIT), 2 * LIMIT, 2 * LIMIT,
                                 fill=False, edgecolor="#60a5fa", linewidth=1.5))
        for gx, gy in GOALS:
            axis.add_patch(Circle((gx, gy), CLEARANCE, color="#ef476f", alpha=0.18))
            axis.plot(gx, gy, "o", color="#ef476f", markersize=4)
        x, y = zip(*route)
        axis.plot(x, y, "-o", color="#38d9a9", linewidth=2, markersize=4)
        axis.plot(x[0], y[0], "s", color="#f9c74f", markersize=7, label="start")
        axis.plot(x[-1], y[-1], "*", color="#ffffff", markersize=10, label="target")
        axis.set_xlim(-60, 60)
        axis.set_ylim(-60, 60)
        axis.set_aspect("equal")
        axis.grid(alpha=0.15)
        axis.set_title(f"{len(route) - 2} intermediate • {stretch:.2f}× stretch")
        axis.set_xlabel("field X (in)")
        axis.set_ylabel("field Y (in)")
    fig.suptitle(
        f"Host-only waypoint study — {summary['planner_success_percent']:.1f}% planned vs "
        f"{summary['direct_safe_percent']:.1f}% direct-safe\n"
        "2-in A* grid + line-of-sight smoothing; red disks are provisional Goal-center exclusions",
        fontsize=15,
        fontweight="bold",
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"waypoint_planner_dashboard.{suffix}", dpi=180)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
