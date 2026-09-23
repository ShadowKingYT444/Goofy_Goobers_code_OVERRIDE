import json
import os
import re
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import serial
from PIL import Image


SERIAL_PORT = "COM6"
CAPTURE_PORT = "COM7"
BAUD_RATE = 115200
HTTP_PORT = 8771
MAX_SAMPLES = 600
ROOT = Path(__file__).resolve().parents[1]
PROS = ROOT / ".venv" / "Scripts" / "pros.exe"
APPDATA_DIR = ROOT / ".pros-appdata"

DIST_RE = re.compile(
    r"DIST (?:sample=(?P<sample>\d+) )?port=(?P<port>\d+) installed=(?P<installed>\d+) "
    r"mm=(?P<mm>-?\d+) in=(?P<inch>-?\d+(?:\.\d+)?) "
    r"confidence=(?P<confidence>-?\d+) size=(?P<size>-?\d+) "
    r"velocity=(?P<velocity>-?\d+(?:\.\d+)?) errno=(?P<errno>-?\d+)"
)

state_lock = threading.Lock()
state = {
    "connected": False,
    "latest": None,
    "samples": [],
    "message": "Waiting for V5 console...",
}

last_serial_time = 0.0

HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Distance Sensor Graph</title>
    <style>
      :root { color-scheme: dark; --bg:#08110f; --panel:#0d1d19; --green:#5dffb1; --blue:#61c4ff; --muted:#9bb9ad; --warn:#ffd166; }
      * { box-sizing:border-box; }
      html, body { margin:0; min-height:100%; background:var(--bg); color:white; font-family:Arial, sans-serif; }
      body { padding:18px; }
      main { display:grid; grid-template-columns:1fr 280px; gap:16px; max-width:1200px; margin:0 auto; }
      section, aside { border:1px solid rgba(93,255,177,.24); background:var(--panel); }
      .chart { position:relative; min-height:620px; overflow:hidden; }
      canvas { position:absolute; inset:0; width:100%; height:100%; }
      aside { padding:18px; display:grid; align-content:start; gap:16px; }
      h1 { margin:0; font-size:20px; }
      .metric { border-top:1px solid rgba(255,255,255,.08); padding-top:12px; }
      .label { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.08em; }
      .value { color:var(--green); font-size:34px; line-height:1.1; font-weight:700; font-variant-numeric:tabular-nums; }
      .small { font-size:18px; color:white; }
      .status { color:var(--muted); line-height:1.4; }
      button { font:inherit; background:#123026; color:white; border:1px solid rgba(93,255,177,.35); padding:8px 10px; }
      @media (max-width:760px) { body{padding:10px;} main{grid-template-columns:1fr;} .chart{min-height:440px;} }
    </style>
  </head>
  <body>
    <main>
      <section class="chart"><canvas id="chart"></canvas></section>
      <aside>
        <h1>Distance Sensor</h1>
        <div class="metric"><div class="label">Distance</div><div class="value" id="distance">--.- in</div></div>
        <div class="metric"><div class="label">Millimeters</div><div class="value small" id="mm">---- mm</div></div>
        <div class="metric"><div class="label">Confidence</div><div class="value small" id="confidence">-- / 63</div></div>
        <div class="metric"><div class="label">Stats</div><div class="value small" id="stats">min -- avg -- max --</div></div>
        <div class="metric"><div class="label">Velocity</div><div class="value small" id="velocity">-- m/s</div></div>
        <button id="clear">Clear graph</button>
        <div class="status" id="status">Connecting...</div>
      </aside>
    </main>
    <script>
      const canvas = document.getElementById("chart");
      const ctx = canvas.getContext("2d");
      const distanceEl = document.getElementById("distance");
      const mmEl = document.getElementById("mm");
      const confidenceEl = document.getElementById("confidence");
      const statsEl = document.getElementById("stats");
      const velocityEl = document.getElementById("velocity");
      const statusEl = document.getElementById("status");
      document.getElementById("clear").onclick = () => fetch("/clear", { method: "POST" });
      let data = null;

      function resize() {
        const ratio = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = Math.max(1, Math.round(rect.width * ratio));
        canvas.height = Math.max(1, Math.round(rect.height * ratio));
        ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      }

      async function poll() {
        try {
          const res = await fetch("/data", { cache: "no-store" });
          data = await res.json();
          const latest = data.latest;
          if (latest) {
            distanceEl.textContent = latest.in.toFixed(2) + " in";
            mmEl.textContent = latest.mm + " mm";
            confidenceEl.textContent = latest.confidence + " / 63";
            velocityEl.textContent = latest.velocity.toFixed(3) + " m/s";
          } else {
            distanceEl.textContent = "--.- in";
            mmEl.textContent = "---- mm";
            confidenceEl.textContent = "-- / 63";
            velocityEl.textContent = "-- m/s";
          }
          if (data.stats) {
            statsEl.textContent = `min ${data.stats.min.toFixed(2)}  avg ${data.stats.avg.toFixed(2)}  max ${data.stats.max.toFixed(2)}`;
          }
          statusEl.textContent = data.message;
        } catch (err) {
          statusEl.textContent = `Server error: ${err.name}`;
        }
      }

      function draw() {
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = "#08110f";
        ctx.fillRect(0, 0, w, h);

        const pad = 48;
        const plotW = Math.max(1, w - pad * 1.5);
        const plotH = Math.max(1, h - pad * 1.4);
        const samples = (data?.samples || []).filter(s => s.mm >= 0 && s.mm < 9999);
        const values = samples.map(s => s.in);
        const maxY = Math.max(24, Math.ceil((Math.max(...values, 24) + 4) / 12) * 12);
        const minY = 0;

        ctx.strokeStyle = "rgba(93,255,177,.18)";
        ctx.lineWidth = 1;
        ctx.fillStyle = "rgba(255,255,255,.56)";
        ctx.font = "12px Arial";
        ctx.textAlign = "right";
        for (let i = 0; i <= 6; i++) {
          const y = pad + plotH * i / 6;
          const value = maxY - (maxY - minY) * i / 6;
          ctx.beginPath();
          ctx.moveTo(pad, y);
          ctx.lineTo(pad + plotW, y);
          ctx.stroke();
          ctx.fillText(value.toFixed(0) + " in", pad - 8, y + 4);
        }

        ctx.strokeStyle = "rgba(255,255,255,.25)";
        ctx.beginPath();
        ctx.moveTo(pad, pad);
        ctx.lineTo(pad, pad + plotH);
        ctx.lineTo(pad + plotW, pad + plotH);
        ctx.stroke();

        if (samples.length > 1) {
          const startT = samples[0].t;
          const endT = samples[samples.length - 1].t;
          const span = Math.max(1, endT - startT);
          ctx.strokeStyle = "#5dffb1";
          ctx.lineWidth = 2;
          ctx.beginPath();
          samples.forEach((sample, index) => {
            const x = pad + ((sample.t - startT) / span) * plotW;
            const y = pad + (1 - (sample.in - minY) / (maxY - minY)) * plotH;
            if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
          });
          ctx.stroke();
        }

        if (data?.latest) {
          ctx.fillStyle = "#61c4ff";
          ctx.textAlign = "left";
          ctx.font = "14px Arial";
          ctx.fillText(`samples ${samples.length}`, pad, h - 16);
        }
        requestAnimationFrame(draw);
      }

      window.addEventListener("resize", resize);
      resize();
      setInterval(poll, 100);
      poll();
      draw();
    </script>
  </body>
</html>"""


def current_payload():
    with state_lock:
        samples = list(state["samples"])
        latest = state["latest"]
        payload = {
            "connected": state["connected"],
            "latest": latest,
            "samples": samples,
            "message": state["message"],
            "stats": None,
        }
    valid = [sample["in"] for sample in samples if 0 <= sample["mm"] < 9999]
    if valid:
        payload["stats"] = {
            "min": min(valid),
            "avg": statistics.fmean(valid),
            "max": max(valid),
        }
    return payload


def update_sample(sample):
    global last_serial_time
    with state_lock:
      state["connected"] = True
      state["latest"] = sample
      state["samples"].append(sample)
      del state["samples"][:-MAX_SAMPLES]
      state["message"] = sample.get("source_message", "Live distance readings.")


def decode_bits(image, y, bit_count):
    value = 0
    for bit in range(bit_count):
        x = 20 + bit * 14 + 5
        r, g, b = image.getpixel((x, y + 5))[:3]
        if r + g + b > 380:
            value |= 1 << bit
    return value


def decode_capture(path):
    with Image.open(path) as image:
        image = image.convert("RGB")
        mm = decode_bits(image, 172, 16)
        confidence = decode_bits(image, 192, 8)
        sample_count = decode_bits(image, 212, 16)
    return mm, confidence, sample_count


def read_screen_capture():
    last_seen_sample = None
    env = os.environ.copy()
    env["APPDATA"] = str(APPDATA_DIR)
    while True:
        if time.time() - last_serial_time < 2.0:
            time.sleep(0.5)
            continue
        try:
            with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as temp:
                capture_path = temp.name
            result = subprocess.run(
                [str(PROS), "v5", "capture", "--force", capture_path, CAPTURE_PORT],
                cwd=str(ROOT),
                env=env,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=8,
            )
            if result.returncode != 0:
                with state_lock:
                    state["message"] = f"No serial data; screen capture failed: {result.stderr.strip()[-120:]}"
                time.sleep(1.0)
                continue

            mm, confidence, sample_count = decode_capture(capture_path)
            if sample_count != last_seen_sample:
                sample = {
                    "t": time.time(),
                    "port": 1,
                    "installed": True,
                    "mm": mm,
                    "in": mm / 25.4,
                    "confidence": confidence,
                    "size": -1,
                    "velocity": 0.0,
                    "errno": 0,
                    "source_message": f"Reading Brain screen barcode on {CAPTURE_PORT}; USB console is silent.",
                }
                update_sample(sample)
                last_seen_sample = sample_count
            time.sleep(0.35)
        except Exception as exc:
            with state_lock:
                state["connected"] = False
                state["message"] = f"Capture fallback error: {exc}"
            time.sleep(1.0)
        finally:
            try:
                os.unlink(capture_path)
            except Exception:
                pass


def read_serial():
    global last_serial_time
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2) as ser:
                with state_lock:
                    state["connected"] = True
                    state["message"] = f"Reading V5 console on {SERIAL_PORT}."
                buffer = ""
                while True:
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    buffer += chunk.decode("utf-8", errors="ignore").replace("\x00", "")
                    lines = buffer.splitlines()
                    buffer = lines.pop() if lines else ""
                    for line in lines:
                        match = DIST_RE.search(line)
                        if not match:
                            continue
                        last_serial_time = time.time()
                        sample = {
                            "t": time.time(),
                            "port": int(match.group("port")),
                            "installed": bool(int(match.group("installed"))),
                            "mm": int(match.group("mm")),
                            "in": float(match.group("inch")),
                            "confidence": int(match.group("confidence")),
                            "size": int(match.group("size")),
                            "velocity": float(match.group("velocity")),
                            "errno": int(match.group("errno")),
                            "source_message": f"Reading V5 console on {SERIAL_PORT}.",
                        }
                        update_sample(sample)
        except serial.SerialException as exc:
            with state_lock:
                state["connected"] = False
                state["message"] = str(exc)
            time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/data":
            body = json.dumps(current_payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/clear":
            self.send_error(404)
            return
        with state_lock:
            state["samples"].clear()
        self.send_response(204)
        self.end_headers()

    def log_message(self, *_):
        return


if __name__ == "__main__":
    threading.Thread(target=read_serial, daemon=True).start()
    threading.Thread(target=read_screen_capture, daemon=True).start()
    HTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
