import argparse
import sys

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=int, default=0)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Could not open camera index {args.index}")
        return 1

    print("Press q to quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("No frame")
            break
        cv2.imshow(f"Camera {args.index}", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
