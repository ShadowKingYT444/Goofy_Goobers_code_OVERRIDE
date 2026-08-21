from pathlib import Path
import json

import cv2


ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = cv2.imread(
    str(ROOT / "agent-workflow/localization-sensor-repair/camera_robot_marker.png"),
    cv2.IMREAD_GRAYSCALE,
)


def video_frame(path, seconds):
    capture = cv2.VideoCapture(str(path))
    capture.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
    ok, frame = capture.read()
    capture.release()
    assert ok, (path, seconds)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def marker_match(gray):
    search = gray[300:520, 80:1200]
    result = cv2.matchTemplate(search, TEMPLATE, cv2.TM_CCOEFF_NORMED)
    _, score, _, location = cv2.minMaxLoc(result)
    return 80 + location[0], 300 + location[1], score


def main():
    assert TEMPLATE is not None
    video = ROOT / "agent-workflow/localization-sensor-repair/camera_half_inch_test.mp4"
    start = marker_match(video_frame(video, 0.0))
    final = marker_match(video_frame(video, 13.5))
    assert start[2] >= 0.90, start
    assert final[2] >= 0.62, final
    pixel_delta = final[0] - start[0]
    inch_delta = pixel_delta / 30.0
    assert 12 <= pixel_delta <= 20, (start, final, pixel_delta)
    assert 0.4 <= inch_delta <= 0.7, inch_delta

    server = (ROOT / "tools/lidar_bar_server.py").read_text(encoding="utf-8")
    calibration = json.loads(
        (ROOT / "agent-workflow/localization-sensor-repair/camera_calibration.json").read_text(
            encoding="utf-8"
        )
    )
    assert 700 <= calibration["baseline_x_px"] <= 730
    assert calibration["baseline_y_in"] == 0.5
    assert calibration["pixels_per_inch"] == 30.0
    for token in (
        "CAMERA_PIXELS_PER_INCH = 30.0",
        "CAMERA_MIN_SCORE = 0.62",
        "def track_fixed_camera():",
        '"camera_pose": state["camera_pose"]',
        "cameraFresh",
        "Fused pose + fixed camera Y",
        "def load_camera_calibration():",
        "def save_camera_calibration(",
        'state["camera_reset_token"] += 1',
    ):
        assert token in server, token
    print(
        f"camera tracker checks passed: start={start[0]} final={final[0]} "
        f"delta={pixel_delta}px ({inch_delta:.2f}in)"
    )


if __name__ == "__main__":
    main()
