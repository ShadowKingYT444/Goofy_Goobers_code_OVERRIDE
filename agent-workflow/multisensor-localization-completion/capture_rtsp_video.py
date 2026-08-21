import argparse
import time
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="rtsp://127.0.0.1:8554/cam")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    if not capture.isOpened():
        raise SystemExit("RTSP stream unavailable")
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not 5.0 <= fps <= 120.0:
        fps = 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    output = cv2.VideoWriter(
        str(Path(args.output)), cv2.VideoWriter_fourcc(*"mp4v"), fps,
        (width, height)
    )
    deadline = time.monotonic() + args.seconds
    frames = 0
    while time.monotonic() < deadline:
        ok, frame = capture.read()
        if not ok:
            break
        output.write(frame)
        frames += 1
    capture.release()
    output.release()
    print(f"captured {frames} frames to {args.output}")
    return 0 if frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
