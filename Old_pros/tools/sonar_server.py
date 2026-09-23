import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import serial


SERIAL_PORT = "COM6"
BAUD_RATE = 115200
HTTP_PORT = 8770

POSE_RE = re.compile(
    r"pose tag_size=(?P<tag_size>[-0-9.]+)in .*?"
    r"bearing=(?P<bearing>[-0-9.]+)deg .*?"
    r"range=(?P<range>[-0-9.]+)in .*?"
    r"right=(?P<right>[-0-9.]+)in .*?"
    r"up=(?P<up>[-0-9.]+)in"
)
TAG_RE = re.compile(r"TAG id=(?P<id>\d+)")
DETECT_RE = re.compile(r"DETECT type=(?P<type>\w+) id=(?P<id>\d+) (?P<body>.*)")
KEY_VALUE_RE = re.compile(r"(\w+)=([-0-9.]+)")

state_lock = threading.Lock()
state = {
    "connected": False,
    "tagVisible": False,
    "tagId": None,
    "bearingDeg": None,
    "rangeIn": None,
    "rightIn": None,
    "upIn": None,
    "tagSizeIn": None,
    "detections": [],
    "updatedAt": 0.0,
    "message": "Waiting for V5 console...",
}
tracks = []
next_track_id = 1


HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>AI Vision Sonar</title>
    <style>
      :root { color-scheme: dark; --bg:#06110d; --panel:#0a1d16; --green:#4cffa6; --muted:#8fb8a2; --warn:#ffd166; }
      * { box-sizing: border-box; }
      html, body { margin:0; min-height:100%; background:var(--bg); color:white; font-family:Arial, sans-serif; }
      body { display:grid; place-items:center; padding:20px; }
      main { width:min(1180px,100%); display:grid; grid-template-columns:minmax(320px,1fr) 280px; gap:18px; align-items:stretch; }
      .camera { position:relative; min-height:360px; border:1px solid rgba(76,255,166,.28); background:#020403; overflow:hidden; }
      video, #overlay { position:absolute; inset:0; width:100%; height:100%; object-fit:contain; }
      #cameraStatus, #cameraControls { position:absolute; left:10px; padding:7px 9px; background:rgba(2,4,3,.76); z-index:3; color:var(--muted); font-size:13px; }
      #cameraStatus { top:10px; }
      #cameraControls { bottom:10px; display:flex; gap:8px; align-items:center; }
      select, button { font:inherit; background:#10271e; color:white; border:1px solid rgba(76,255,166,.35); padding:5px 7px; }
      .sonar { position:relative; min-height:min(72vw,680px); border:1px solid rgba(76,255,166,.28); background:radial-gradient(circle at 50% 100%, rgba(76,255,166,.14), rgba(6,17,13,.2) 55%, rgba(6,17,13,.9)); overflow:hidden; }
      canvas { position:absolute; inset:0; width:100%; height:100%; }
      aside { grid-row:span 2; border:1px solid rgba(76,255,166,.24); background:var(--panel); padding:18px; display:grid; align-content:start; gap:16px; }
      h1 { margin:0; font-size:20px; font-weight:700; }
      .readout { display:grid; gap:10px; }
      .metric { border-top:1px solid rgba(255,255,255,.08); padding-top:12px; }
      .label { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.08em; }
      .value { font-size:34px; line-height:1.1; font-weight:700; color:var(--green); font-variant-numeric:tabular-nums; }
      .small { font-size:18px; color:white; }
      .status { color:var(--muted); line-height:1.4; min-height:42px; }
      .lost .value { color:var(--warn); }
      @media (max-width:760px) { body{padding:12px;} main{grid-template-columns:1fr;} aside{grid-row:auto;} .camera{min-height:300px;} .sonar{min-height:70vh;} }
    </style>
  </head>
  <body>
    <main>
      <section class="camera" aria-label="Camera feed">
        <video id="video" autoplay playsinline muted></video>
        <canvas id="overlay"></canvas>
        <div id="cameraStatus">Starting camera...</div>
        <div id="cameraControls"><select id="cameraSelect"></select><button id="restartCamera">Restart</button></div>
      </section>
      <section class="sonar" aria-label="Sonar display"><canvas id="canvas"></canvas></section>
      <aside id="panel">
        <h1>AI Vision Sonar</h1>
        <div class="readout">
          <div class="metric"><div class="label">Angle</div><div class="value" id="angle">--.- deg</div></div>
          <div class="metric"><div class="label">Distance</div><div class="value" id="range">--.- in</div></div>
          <div class="metric"><div class="label">Offset</div><div class="value small" id="offset">--.- in right</div></div>
          <div class="metric"><div class="label">Tag</div><div class="value small" id="tag">No tag</div></div>
        </div>
        <div class="status" id="status">Connecting...</div>
      </aside>
    </main>
    <script>
      const canvas = document.getElementById("canvas");
      const ctx = canvas.getContext("2d");
      const video = document.getElementById("video");
      const overlay = document.getElementById("overlay");
      const overlayCtx = overlay.getContext("2d");
      const cameraStatus = document.getElementById("cameraStatus");
      const cameraSelect = document.getElementById("cameraSelect");
      const restartCamera = document.getElementById("restartCamera");
      const panel = document.getElementById("panel");
      const angleEl = document.getElementById("angle");
      const rangeEl = document.getElementById("range");
      const offsetEl = document.getElementById("offset");
      const tagEl = document.getElementById("tag");
      const statusEl = document.getElementById("status");
      let latest = null;
      let sweep = -65;
      let stream = null;

      function fmt(value, suffix) {
        return Number.isFinite(value) ? `${value.toFixed(1)} ${suffix}` : `--.- ${suffix}`;
      }

      async function poll() {
        try {
          const res = await fetch("/data", { cache: "no-store" });
          latest = await res.json();
          const fresh = Date.now() / 1000 - latest.updatedAt < 1.2;
          const visible = latest.tagVisible && fresh;
          panel.classList.toggle("lost", !visible);
          angleEl.textContent = visible ? fmt(latest.bearingDeg, "deg") : "--.- deg";
          rangeEl.textContent = visible ? fmt(latest.rangeIn, "in") : "--.- in";
          offsetEl.textContent = visible ? `${latest.rightIn.toFixed(1)} in ${latest.rightIn >= 0 ? "right" : "left"}` : "--.- in";
          tagEl.textContent = visible ? `ID ${latest.tagId}` : "No tag";
          statusEl.textContent = latest.connected ? latest.message : "No V5 console connection on COM6.";
        } catch (err) {
          statusEl.textContent = `Server error: ${err.name}`;
        }
      }

      async function loadCameras() {
        const devices = await navigator.mediaDevices.enumerateDevices();
        const cams = devices.filter((d) => d.kind === "videoinput");
        const previous = cameraSelect.value;
        cameraSelect.innerHTML = "";
        cams.forEach((cam, i) => {
          const option = document.createElement("option");
          option.value = cam.deviceId;
          option.textContent = cam.label || `Camera ${i + 1}`;
          cameraSelect.appendChild(option);
        });
        if (previous) cameraSelect.value = previous;
      }

      async function startCamera() {
        if (stream) stream.getTracks().forEach((track) => track.stop());
        const deviceId = cameraSelect.value;
        stream = await navigator.mediaDevices.getUserMedia({
          video: deviceId ? { deviceId: { exact: deviceId } } : true,
          audio: false
        });
        video.srcObject = stream;
        await video.play();
        await loadCameras();
        cameraStatus.textContent = `Camera live (${video.videoWidth}x${video.videoHeight})`;
      }

      function drawCameraOverlay() {
        if (!video.videoWidth) return;
        overlay.width = video.videoWidth;
        overlay.height = video.videoHeight;
        overlayCtx.clearRect(0, 0, overlay.width, overlay.height);
        overlayCtx.lineWidth = 2;
        overlayCtx.font = "14px Arial";
        for (const det of latest?.detections || []) {
          if (det.type !== "tag") continue;
          const confidence = Number.isFinite(det.confidence) ? det.confidence : 0;
          overlayCtx.globalAlpha = Math.max(0.35, Math.min(1, confidence / 100));
          overlayCtx.strokeStyle = "#4cffa6";
          overlayCtx.fillStyle = "#4cffa6";
          overlayCtx.beginPath();
          overlayCtx.moveTo(det.corners[0][0], det.corners[0][1]);
          for (let i = 1; i < 4; i++) overlayCtx.lineTo(det.corners[i][0], det.corners[i][1]);
          overlayCtx.closePath();
          overlayCtx.stroke();
          overlayCtx.fillText(`tag ${det.id} ${confidence.toFixed(0)}%`, det.corners[0][0], Math.max(14, det.corners[0][1] - 5));
        }
        overlayCtx.globalAlpha = 1;
      }

      function resize() {
        const rect = canvas.getBoundingClientRect();
        const ratio = window.devicePixelRatio || 1;
        canvas.width = Math.max(1, Math.round(rect.width * ratio));
        canvas.height = Math.max(1, Math.round(rect.height * ratio));
        ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      }

      function draw() {
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        ctx.clearRect(0, 0, w, h);
        const cx = w / 2;
        const cy = h * 0.92;
        const maxR = Math.min(w * 0.48, h * 0.86);

        ctx.strokeStyle = "rgba(76,255,166,.24)";
        ctx.lineWidth = 1;
        for (let i = 1; i <= 5; i++) {
          ctx.beginPath();
          ctx.arc(cx, cy, maxR * i / 5, Math.PI, 0);
          ctx.stroke();
        }
        for (let deg = -60; deg <= 60; deg += 15) {
          const a = (-90 + deg) * Math.PI / 180;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + Math.cos(a) * maxR, cy + Math.sin(a) * maxR);
          ctx.stroke();
          ctx.fillStyle = "rgba(210,255,229,.65)";
          ctx.font = "12px Arial";
          ctx.textAlign = "center";
          ctx.fillText(`${deg}`, cx + Math.cos(a) * (maxR + 18), cy + Math.sin(a) * (maxR + 18));
        }

        sweep += 1.8;
        if (sweep > 65) sweep = -65;
        const sa = (-90 + sweep) * Math.PI / 180;
        const grad = ctx.createLinearGradient(cx, cy, cx + Math.cos(sa) * maxR, cy + Math.sin(sa) * maxR);
        grad.addColorStop(0, "rgba(76,255,166,.78)");
        grad.addColorStop(1, "rgba(76,255,166,0)");
        ctx.strokeStyle = grad;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(sa) * maxR, cy + Math.sin(sa) * maxR);
        ctx.stroke();

        const fresh = latest && Date.now() / 1000 - latest.updatedAt < 1.2;
        if (latest && latest.tagVisible && fresh) {
          const maxRange = 48;
          const bearing = Math.max(-65, Math.min(65, latest.bearingDeg));
          const r = Math.min(maxR, maxR * latest.rangeIn / maxRange);
          const a = (-90 + bearing) * Math.PI / 180;
          const x = cx + Math.cos(a) * r;
          const y = cy + Math.sin(a) * r;
          ctx.fillStyle = "#4cffa6";
          ctx.shadowColor = "#4cffa6";
          ctx.shadowBlur = 18;
          ctx.beginPath();
          ctx.arc(x, y, 8, 0, Math.PI * 2);
          ctx.fill();
          ctx.shadowBlur = 0;
          ctx.fillStyle = "white";
          ctx.font = "14px Arial";
          ctx.textAlign = "left";
          ctx.fillText(`${latest.rangeIn.toFixed(1)} in`, x + 14, y + 4);
        }
        requestAnimationFrame(draw);
      }

      window.addEventListener("resize", resize);
      restartCamera.onclick = () => startCamera().catch((err) => { cameraStatus.textContent = `Camera error: ${err.name}`; });
      cameraSelect.onchange = restartCamera.onclick;
      resize();
      startCamera().catch((err) => { cameraStatus.textContent = `Camera error: ${err.name}`; });
      setInterval(poll, 100);
      poll();
      draw();
      function overlayFrame() {
        drawCameraOverlay();
        requestAnimationFrame(overlayFrame);
      }
      overlayFrame();
    </script>
  </body>
</html>"""


def update_state(**changes):
    with state_lock:
        state.update(changes)


def detection_center(detection):
    if detection["type"] == "tag":
        xs = [point[0] for point in detection["corners"]]
        ys = [point[1] for point in detection["corners"]]
        return sum(xs) / 4.0, sum(ys) / 4.0
    return detection["x"] + detection["w"] * 0.5, detection["y"] + detection["h"] * 0.5


def detection_size(detection):
    if detection["type"] == "tag":
        xs = [point[0] for point in detection["corners"]]
        ys = [point[1] for point in detection["corners"]]
        return max(xs) - min(xs), max(ys) - min(ys)
    return detection["w"], detection["h"]


def smooth_detection(old_detection, new_detection, alpha=0.45):
    smoothed = dict(new_detection)
    if new_detection["type"] == "tag":
        smoothed["corners"] = [
            [
                old_detection["corners"][i][0] * (1.0 - alpha) + new_detection["corners"][i][0] * alpha,
                old_detection["corners"][i][1] * (1.0 - alpha) + new_detection["corners"][i][1] * alpha,
            ]
            for i in range(4)
        ]
        return smoothed
    for key in ("x", "y", "w", "h"):
        smoothed[key] = old_detection[key] * (1.0 - alpha) + new_detection[key] * alpha
    return smoothed


def publish_frame_detections(frame_detections):
    global next_track_id
    now = time.time()
    frame_detections = [detection for detection in frame_detections if detection["type"] == "tag"]
    matched_tracks = set()
    deduped = []

    for detection in frame_detections:
        center_x, center_y = detection_center(detection)
        width, height = detection_size(detection)
        max_dist = max(22.0, 0.55 * math.hypot(width, height))
        duplicate_index = None
        for index, existing in enumerate(deduped):
            if existing["type"] != detection["type"] or existing["id"] != detection["id"]:
                continue
            existing_x, existing_y = detection_center(existing)
            if math.hypot(center_x - existing_x, center_y - existing_y) <= max_dist:
                duplicate_index = index
                break
        if duplicate_index is None:
            deduped.append(detection)
            continue
        existing = deduped[duplicate_index]
        existing_area = detection_size(existing)[0] * detection_size(existing)[1]
        area = width * height
        existing_score = existing.get("score", 0)
        score = detection.get("score", 0)
        if score > existing_score or (score == existing_score and area > existing_area):
            deduped[duplicate_index] = detection

    for detection in deduped:
        center_x, center_y = detection_center(detection)
        width, height = detection_size(detection)
        max_dist = max(28.0, 0.9 * math.hypot(width, height))
        best_track = None
        best_dist = max_dist

        for track in tracks:
            if track["index"] in matched_tracks:
                continue
            if track["detection"]["type"] != detection["type"] or track["detection"]["id"] != detection["id"]:
                continue
            track_x, track_y = detection_center(track["detection"])
            dist = math.hypot(center_x - track_x, center_y - track_y)
            if dist < best_dist:
                best_track = track
                best_dist = dist

        if best_track is None:
            track = {
                "index": next_track_id,
                "detection": detection,
                "hits": 1,
                "misses": 0,
                "lastSeen": now,
            }
            next_track_id += 1
            tracks.append(track)
            matched_tracks.add(track["index"])
            continue

        best_track["detection"] = smooth_detection(best_track["detection"], detection)
        if "score" in detection:
            best_track["detection"]["score"] = detection["score"]
        best_track["hits"] = min(20, best_track["hits"] + 1)
        best_track["misses"] = 0
        best_track["lastSeen"] = now
        matched_tracks.add(best_track["index"])

    for track in tracks:
        if track["index"] not in matched_tracks:
            track["misses"] += 1

    tracks[:] = [track for track in tracks if track["misses"] <= 4]

    visible = []
    for track in tracks:
        detection = dict(track["detection"])
        detection["trackId"] = track["index"]
        detection["hits"] = track["hits"]
        detection["misses"] = track["misses"]
        if detection["type"] == "object" and "score" in detection:
            confidence = detection["score"]
        else:
            confidence = min(100.0, 25.0 + track["hits"] * 15.0 - track["misses"] * 18.0)
        detection["confidence"] = max(0.0, min(100.0, confidence))

        min_hits = 1 if detection["type"] == "tag" else 2
        if track["hits"] >= min_hits and detection["confidence"] >= 35.0:
            visible.append(detection)

    update_state(
        detections=visible,
        updatedAt=now,
        message=f"Live filtered detections: {len(visible)} shown.",
    )


def parse_detection(line):
    match = DETECT_RE.search(line)
    if not match:
        return None
    values = {key: float(value) for key, value in KEY_VALUE_RE.findall(match.group("body"))}
    detection = {"type": match.group("type"), "id": int(match.group("id"))}
    if detection["type"] == "tag":
        try:
            detection["corners"] = [
                [values["x0"], values["y0"]],
                [values["x1"], values["y1"]],
                [values["x2"], values["y2"]],
                [values["x3"], values["y3"]],
            ]
        except KeyError:
            return None
        return detection
    if {"x", "y", "w", "h"}.issubset(values):
        detection.update({"x": values["x"], "y": values["y"], "w": values["w"], "h": values["h"]})
        if "score" in values:
            detection["score"] = values["score"]
        if "angle" in values:
            detection["angle"] = values["angle"]
        return detection
    return None


def read_serial():
    last_tag_id = None
    detections = []
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2) as ser:
                update_state(connected=True, message=f"Reading V5 console on {SERIAL_PORT}.")
                buffer = ""
                while True:
                    chunk = ser.read(4096)
                    if not chunk:
                        if time.time() - state["updatedAt"] > 1.5:
                            update_state(tagVisible=False, detections=[], message="No recent tag pose.")
                        continue
                    buffer += chunk.decode("utf-8", errors="ignore").replace("\x00", "")
                    lines = buffer.splitlines()
                    buffer = lines.pop() if lines else ""
                    for line in lines:
                        if "AI Vision installed=" in line:
                            if detections:
                                publish_frame_detections(detections)
                            detections = []
                        tag_match = TAG_RE.search(line)
                        if tag_match:
                            last_tag_id = int(tag_match.group("id"))
                        detection = parse_detection(line)
                        if detection:
                            detections.append(detection)
                        pose_match = POSE_RE.search(line)
                        if not pose_match:
                            continue
                        update_state(
                            connected=True,
                            tagVisible=True,
                            tagId=last_tag_id,
                            bearingDeg=float(pose_match.group("bearing")),
                            rangeIn=float(pose_match.group("range")),
                            rightIn=float(pose_match.group("right")),
                            upIn=float(pose_match.group("up")),
                            tagSizeIn=float(pose_match.group("tag_size")),
                            updatedAt=time.time(),
                            message="Live tag pose.",
                        )
        except serial.SerialException as exc:
            update_state(connected=False, tagVisible=False, message=str(exc))
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
            with state_lock:
                body = json.dumps(state).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, *_):
        return


if __name__ == "__main__":
    threading.Thread(target=read_serial, daemon=True).start()
    HTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
