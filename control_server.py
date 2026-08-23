#!/usr/bin/env python3
"""Localhost-only browser/gamepad bridge for the V5 Brain USB user serial port."""

import argparse
import json
import re
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parent
PAGE = (ROOT / "control.html").read_bytes()
MAX_COMMAND = 63
HOST_WATCHDOG_S = 0.25
FRAME_RE = re.compile(r"AIV_FRAME port=(\d+) count=(-?\d+)")
TAG_RE = re.compile(
    r"TAG id=(\d+) corners=" + r",".join([r"(-?\d+)"] * 8)
)


class Bridge:
    def __init__(self, port: str):
        self.serial = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.1)
        self.lock = threading.Lock()
        self.sequence = 0
        self.last_browser_command = 0.0
        self.active = False
        self.running = True
        self.latest_lines: list[str] = []
        self.detections: dict[int, dict] = {}
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.reader.start()
        self.watchdog.start()

    def _write(self, message: str) -> None:
        with self.lock:
            self.serial.write(message.encode("ascii"))
            self.serial.flush()

    def drive(self, left: int, right: int, enabled: bool) -> None:
        left = max(-MAX_COMMAND, min(MAX_COMMAND, int(left)))
        right = max(-MAX_COMMAND, min(MAX_COMMAND, int(right)))
        self.sequence += 1
        self.last_browser_command = time.monotonic()
        self.active = enabled
        if enabled:
            self._write(f"D {self.sequence} {left} {right}\n")
        else:
            self._write(f"S {self.sequence}\n")

    def stop(self) -> None:
        self.sequence += 1
        self.active = False
        try:
            self._write(f"S {self.sequence}\n")
        except (OSError, serial.SerialException):
            pass

    def _watchdog_loop(self) -> None:
        while self.running:
            if self.active and time.monotonic() - self.last_browser_command > HOST_WATCHDOG_S:
                self.stop()
            time.sleep(0.025)

    def _read_loop(self) -> None:
        while self.running:
            try:
                raw = self.serial.readline()
            except serial.SerialException:
                self.running = False
                return
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.latest_lines.append(line)
                del self.latest_lines[:-20]
                frame_match = FRAME_RE.search(line)
                if frame_match:
                    port = int(frame_match.group(1))
                    tags = []
                    for tag_match in TAG_RE.finditer(line):
                        values = [int(value) for value in tag_match.groups()]
                        tags.append({"id": values[0], "corners": values[1:]})
                    self.detections[port] = {
                        "updated": time.monotonic(),
                        "count": int(frame_match.group(2)),
                        "tags": tags,
                    }

    def status(self) -> dict:
        now = time.monotonic()
        detections = {
            str(port): {
                "age_ms": round((now - frame["updated"]) * 1000),
                "count": frame["count"],
                "tags": frame["tags"],
            }
            for port, frame in self.detections.items()
            if now - frame["updated"] < 1.0
        }
        return {
            "serial_open": self.serial.is_open,
            "sequence": self.sequence,
            "active": self.active,
            "max_command": MAX_COMMAND,
            "telemetry": self.latest_lines[-8:],
            "detections": detections,
        }

    def close(self) -> None:
        self.running = False
        self.stop()
        self.serial.close()


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge

    def do_GET(self):
        if self.path == "/":
            self._respond(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/api/status":
            body = json.dumps(self.bridge.status()).encode()
            self._respond(200, body, "application/json")
        else:
            self._respond(404, b"Not found", "text/plain")

    def do_POST(self):
        if self.path != "/api/drive" or self.headers.get_content_type() != "application/json":
            self._respond(404, b"Not found", "text/plain")
            return
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 1024)
            data = json.loads(self.rfile.read(length))
            self.bridge.drive(data.get("left", 0), data.get("right", 0), data.get("enabled") is True)
        except (ValueError, TypeError, json.JSONDecodeError, OSError, serial.SerialException) as exc:
            self.bridge.stop()
            self._respond(400, str(exc).encode(), "text/plain")
            return
        self._respond(200, b'{"ok":true}', "application/json")

    def _respond(self, status: int, body: bytes, content_type: str):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyACM2")
    parser.add_argument("--port", type=int, default=8088)
    args = parser.parse_args()

    bridge = Bridge(args.serial)
    Handler.bridge = bridge
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)

    def shutdown(_signum, _frame):
        bridge.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"Control UI: http://127.0.0.1:{args.port}")
    print(f"Serial: {args.serial}; hard limit: {MAX_COMMAND}/127")
    try:
        server.serve_forever()
    finally:
        bridge.close()
        server.server_close()


if __name__ == "__main__":
    main()
