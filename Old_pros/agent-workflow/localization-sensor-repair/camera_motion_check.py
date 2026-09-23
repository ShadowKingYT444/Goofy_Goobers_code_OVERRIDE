import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("--known-peak-in", type=float, default=1.51)
    parser.add_argument("--reference-image", type=Path)
    parser.add_argument("--pixels-per-in", type=float)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.video))
    if args.reference_image:
        reference = cv2.imread(str(args.reference_image))
        if reference is None:
            raise RuntimeError(f"cannot read {args.reference_image}")
    else:
        ok, reference = capture.read()
        if not ok:
            raise RuntimeError(f"cannot read {args.video}")

    # The fixed black phone/Brain display on the vertical tower is a high-
    # contrast rigid target. Coordinates are from the fixed 1280x720 Brio view.
    template = cv2.cvtColor(reference[15:255, 395:485], cv2.COLOR_BGR2GRAY)
    search_x0, search_x1 = 80, 650
    search_y0, search_y1 = 0, 300
    reference_center_x = 395 + template.shape[1] / 2.0
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0

    samples = []
    frame_index = 1
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        search = gray[search_y0:search_y1, search_x0:search_x1]
        result = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
        _, score, _, location = cv2.minMaxLoc(result)
        center_x = search_x0 + location[0] + template.shape[1] / 2.0
        samples.append(
            {
                "time_s": frame_index / fps,
                "center_x_px": center_x,
                "delta_x_px": center_x - reference_center_x,
                "score": score,
            }
        )
        frame_index += 1

    capture.release()
    reliable = [sample for sample in samples if sample["score"] >= 0.70]
    if len(reliable) < 10:
        raise RuntimeError("not enough reliable camera template matches")
    peak = max(reliable, key=lambda sample: sample["delta_x_px"])
    tail = reliable[-min(60, len(reliable)):]
    final_delta_px = float(np.median([sample["delta_x_px"] for sample in tail]))
    pixels_per_in = args.pixels_per_in or peak["delta_x_px"] / args.known_peak_in
    if pixels_per_in <= 1.0:
        raise RuntimeError(f"invalid camera scale {pixels_per_in}")

    result = {
        "video": str(args.video),
        "reference_center_x_px": reference_center_x,
        "peak_time_s": peak["time_s"],
        "peak_delta_px": peak["delta_x_px"],
        "known_peak_in": args.known_peak_in,
        "pixels_per_in": pixels_per_in,
        "final_delta_px": final_delta_px,
        "final_displacement_in": final_delta_px / pixels_per_in,
        "minimum_match_score": min(sample["score"] for sample in reliable),
        "reliable_frames": len(reliable),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
