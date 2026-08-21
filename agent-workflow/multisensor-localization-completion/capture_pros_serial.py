import argparse
import time
from pathlib import Path

import serial
from cobs import cobs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    deadline = time.monotonic() + args.seconds
    pending = bytearray()
    packet_count = 0
    # Stream directly to disk so long captures use constant memory and can be
    # tailed live by the safety monitor/analyzer.
    with Path(args.output).open("w", encoding="utf-8", buffering=1) as output:
        with serial.Serial(args.port, 115200, timeout=0.05) as device:
            while time.monotonic() < deadline:
                pending.extend(device.read(device.in_waiting or 1))
                while b"\0" in pending:
                    encoded, _, remainder = pending.partition(b"\0")
                    pending = bytearray(remainder)
                    if not encoded:
                        continue
                    try:
                        decoded = cobs.decode(bytes(encoded))
                    except Exception:
                        continue
                    if decoded.startswith((b"sout", b"serr")):
                        decoded = decoded[4:]
                    text = decoded.rstrip(b"\0").decode("utf-8", errors="replace")
                    if text:
                        output.write(text)
                        packet_count += 1

    print(f"captured {packet_count} packets to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
