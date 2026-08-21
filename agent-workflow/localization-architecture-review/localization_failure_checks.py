import math


WALL = 70.2
LEFT_OFFSET = 5.29


def signed_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def candidates(theta, distance, x, y, heading):
    headings = [(theta + 90 * k) % 360 for k in range(4)]

    def ox(h):
        return -LEFT_OFFSET * math.sin(math.radians(h))

    def oy(h):
        return LEFT_OFFSET * math.cos(math.radians(h))

    values = [
        ("red", headings[0], "y", WALL - distance - oy(headings[0])),
        ("audience", headings[1], "x", -WALL + distance - ox(headings[1])),
        ("blue", headings[2], "y", -WALL + distance - oy(headings[2])),
        ("zero", headings[3], "x", WALL - distance - ox(headings[3])),
    ]
    scored = []
    for wall, candidate_heading, axis, observed in values:
        predicted = x if axis == "x" else y
        scored.append(
            {
                "wall": wall,
                "heading": candidate_heading,
                "axis": axis,
                "observed": observed,
                "score": abs(observed - predicted)
                + 0.35 * abs(signed_diff(candidate_heading, heading)),
            }
        )
    return sorted(scored, key=lambda item: item["score"])


def test_four_candidates_survive_large_imu_error():
    scored = candidates(0.0, 16.91, -48.0, 0.0, 179.0)
    assert scored[0]["wall"] == "audience", scored
    assert scored[1]["score"] - scored[0]["score"] >= 8.0, scored


def test_mount_offset_is_required_for_absolute_x():
    observed_with_offset = candidates(0.0, 16.91, -48.0, 0.0, 90.0)[0]["observed"]
    observed_without_offset = -WALL + 16.91
    assert abs(observed_with_offset + 48.0) < 0.1
    assert abs(observed_without_offset + 48.0) > 5.0


def test_wall_distance_changes_only_observable_axis():
    pose = {"x": -45.0, "y": 7.25}
    selected = candidates(0.0, 16.91, pose["x"], pose["y"], 90.0)[0]
    assert selected["axis"] == "x"
    corrected = dict(pose)
    corrected[selected["axis"]] = selected["observed"]
    assert corrected["y"] == pose["y"]
    assert corrected["x"] != pose["x"]


def test_opposite_wall_is_not_directionally_consistent():
    assert abs(signed_diff(90.0, 270.0)) == 180.0
    assert abs(signed_diff(90.0, 90.5)) == 0.5


def test_literal_field_center_is_out_of_range():
    assert WALL > 50.0


def main():
    test_four_candidates_survive_large_imu_error()
    test_mount_offset_is_required_for_absolute_x()
    test_wall_distance_changes_only_observable_axis()
    test_opposite_wall_is_not_directionally_consistent()
    test_literal_field_center_is_out_of_range()
    print("localization failure checks passed")


if __name__ == "__main__":
    main()
