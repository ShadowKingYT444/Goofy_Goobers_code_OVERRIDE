import math


WHEEL_DIAMETER_IN = 2.75
WHEEL_CIRCUMFERENCE_IN = math.pi * WHEEL_DIAMETER_IN
TRACK_WIDTH_IN = 10
LEFT_ENCODER_SIGN = 1
RIGHT_ENCODER_SIGN = -1
HORIZONTAL_ODOM_DIAMETER_IN = 2
HORIZONTAL_ODOM_CIRCUMFERENCE_IN = math.pi * HORIZONTAL_ODOM_DIAMETER_IN
HORIZONTAL_ODOM_OFFSET_BACK_IN = 2.8812
HORIZONTAL_ODOM_SIGN = 1


def step_pose(pose, delta_left_deg, delta_right_deg, delta_side_centideg=0):
    delta_left_in = (delta_left_deg / 360) * WHEEL_CIRCUMFERENCE_IN * LEFT_ENCODER_SIGN
    delta_right_in = (delta_right_deg / 360) * WHEEL_CIRCUMFERENCE_IN * RIGHT_ENCODER_SIGN
    delta_side_wheel_in = (
        (delta_side_centideg / 100) / 360
    ) * HORIZONTAL_ODOM_CIRCUMFERENCE_IN * HORIZONTAL_ODOM_SIGN
    delta_center_in = (delta_left_in + delta_right_in) / 2
    delta_heading_rad = (delta_right_in - delta_left_in) / TRACK_WIDTH_IN
    delta_side_center_in = delta_side_wheel_in - HORIZONTAL_ODOM_OFFSET_BACK_IN * delta_heading_rad
    mid_heading = pose["heading_rad"] + delta_heading_rad / 2
    pose["x"] += delta_center_in * math.cos(mid_heading) + delta_side_center_in * math.sin(mid_heading)
    pose["y"] += delta_center_in * math.sin(mid_heading) - delta_side_center_in * math.cos(mid_heading)
    pose["heading_rad"] += delta_heading_rad
    return pose


def heading_deg(pose):
    return (math.degrees(pose["heading_rad"]) + 360) % 360


def main():
    start = {"x": 0.0, "y": 0.0, "heading_rad": math.pi / 2}

    forward = step_pose(start.copy(), 360, -360)
    assert abs(forward["x"]) < 0.001, forward
    assert forward["y"] > 8.5, forward
    assert abs(heading_deg(forward) - 90) < 0.001, forward

    clockwise = step_pose(start.copy(), 360, 360)
    assert clockwise["heading_rad"] < start["heading_rad"], clockwise

    counterclockwise = step_pose(start.copy(), -360, -360)
    assert counterclockwise["heading_rad"] > start["heading_rad"], counterclockwise

    # A rear-center lateral wheel moves during a pure turn. Supplying the
    # physically consistent wheel arc must cancel translation at robot center.
    clockwise_theta = (-(2 * WHEEL_CIRCUMFERENCE_IN)) / TRACK_WIDTH_IN
    clockwise_side_in = HORIZONTAL_ODOM_OFFSET_BACK_IN * clockwise_theta
    clockwise_side_centideg = (
        clockwise_side_in / HORIZONTAL_ODOM_CIRCUMFERENCE_IN * 360 * 100
    )
    clockwise_cancelled = step_pose(
        start.copy(), 360, 360, clockwise_side_centideg
    )
    assert math.hypot(clockwise_cancelled["x"], clockwise_cancelled["y"]) < 1e-9, clockwise_cancelled

    counterclockwise_theta = -clockwise_theta
    counterclockwise_side_in = HORIZONTAL_ODOM_OFFSET_BACK_IN * counterclockwise_theta
    counterclockwise_side_centideg = (
        counterclockwise_side_in / HORIZONTAL_ODOM_CIRCUMFERENCE_IN * 360 * 100
    )
    counterclockwise_cancelled = step_pose(
        start.copy(), -360, -360, counterclockwise_side_centideg
    )
    assert math.hypot(counterclockwise_cancelled["x"], counterclockwise_cancelled["y"]) < 1e-9, counterclockwise_cancelled

    slide_right = step_pose(start.copy(), 0, 0, 36000)
    assert slide_right["x"] > 6.2, slide_right
    assert abs(slide_right["y"]) < 0.001, slide_right

    slide_left = step_pose(start.copy(), 0, 0, -36000)
    assert slide_left["x"] < -6.2, slide_left
    assert abs(slide_left["y"]) < 0.001, slide_left

    print("odometry sign checks passed")


if __name__ == "__main__":
    main()
