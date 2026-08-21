import json
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np


PARAMS = cv2.aruco.DetectorParameters()
PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
DETECTORS = [
    ("tag16h5", cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16H5), PARAMS)),
    ("tag25h9", cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25H9), PARAMS)),
    ("tag36h10", cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H10), PARAMS)),
    ("tag36h11", cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11), PARAMS)),
]
QR_DETECTOR = cv2.QRCodeDetector()


HTML = """<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>AprilTag Preview</title>
    <style>
      html, body { margin: 0; height: 100%; background: #111; color: white; font-family: Arial, sans-serif; }
      #wrap { position: fixed; inset: 0; display: grid; place-items: center; }
      video, canvas { position: absolute; max-width: 100vw; max-height: 100vh; }
      video { background: #000; }
      #status, #controls { position: fixed; left: 12px; padding: 8px 10px; background: rgba(0,0,0,.72); z-index: 2; }
      #status { top: 12px; }
      #controls { bottom: 12px; }
      select, button { font: inherit; }
    </style>
  </head>
  <body>
    <div id="status">Starting camera...</div>
    <div id="controls"><select id="camera"></select><button id="restart">Restart</button></div>
    <div id="wrap"><video id="video" autoplay playsinline muted></video><canvas id="canvas"></canvas></div>
    <script>
      const video = document.getElementById("video");
      const canvas = document.getElementById("canvas");
      const ctx = canvas.getContext("2d");
      const status = document.getElementById("status");
      const camera = document.getElementById("camera");
      const restart = document.getElementById("restart");
      let tags = [];
      let busy = false;
      let stream = null;

      function fitOverlay() {
        const rect = video.getBoundingClientRect();
        canvas.style.width = `${rect.width}px`;
        canvas.style.height = `${rect.height}px`;
        canvas.style.left = `${rect.left}px`;
        canvas.style.top = `${rect.top}px`;
      }

      function drawOverlay() {
        if (video.videoWidth) {
          if (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight) {
            canvas.width = video.videoWidth;
            canvas.height = video.videoHeight;
          }
          fitOverlay();
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.lineWidth = Math.max(3, canvas.width / 180);
          ctx.font = `${Math.max(22, canvas.width / 28)}px Arial`;
          for (const tag of tags) {
            const c = tag.corners;
            ctx.strokeStyle = "#00ff66";
            ctx.fillStyle = "#00ff66";
            ctx.beginPath();
            ctx.moveTo(c[0][0], c[0][1]);
            for (let i = 1; i < 4; i++) ctx.lineTo(c[i][0], c[i][1]);
            ctx.closePath();
            ctx.stroke();
            ctx.fillText(`${tag.family} ${tag.id}`, c[0][0], Math.max(24, c[0][1] - 8));
          }
        }
        requestAnimationFrame(drawOverlay);
      }

      async function detect() {
        if (busy || !video.videoWidth) return;
        busy = true;
        const capture = document.createElement("canvas");
        capture.width = video.videoWidth;
        capture.height = video.videoHeight;
        capture.getContext("2d").drawImage(video, 0, 0, capture.width, capture.height);
        capture.toBlob(async (blob) => {
          try {
            const res = await fetch("/detect", { method: "POST", headers: { "Content-Type": "image/jpeg" }, body: blob });
            const data = await res.json();
            tags = data.tags;
            const qr = data.qr.length ? ` QR: ${data.qr.join(", ")}` : "";
            status.textContent = tags.length || data.qr.length
              ? `Tags: ${tags.map(t => `${t.family}:${t.id}`).join(", ")}${qr}`
              : `No tag detected (${video.videoWidth}x${video.videoHeight})`;
          } catch (err) {
            status.textContent = `Detection error: ${err.name}`;
          } finally {
            busy = false;
          }
        }, "image/jpeg", 0.8);
      }

      async function loadCameras() {
        const devices = await navigator.mediaDevices.enumerateDevices();
        const cams = devices.filter((d) => d.kind === "videoinput");
        camera.innerHTML = "";
        cams.forEach((cam, i) => {
          const option = document.createElement("option");
          option.value = cam.deviceId;
          option.textContent = cam.label || `Camera ${i + 1}`;
          camera.appendChild(option);
        });
      }

      async function startCamera() {
        if (stream) stream.getTracks().forEach((track) => track.stop());
        const deviceId = camera.value;
        stream = await navigator.mediaDevices.getUserMedia({
          video: deviceId ? { deviceId: { exact: deviceId } } : true,
          audio: false
        });
        video.srcObject = stream;
        await video.play();
        await loadCameras();
        status.textContent = `Camera live (${video.videoWidth}x${video.videoHeight})`;
      }

      restart.onclick = () => startCamera().catch((err) => { status.textContent = `Camera error: ${err.name}`; });
      camera.onchange = restart.onclick;

      startCamera()
        .then(() => {
          drawOverlay();
          setInterval(detect, 150);
        })
        .catch((err) => { status.textContent = `Camera error: ${err.name}`; });
    </script>
  </body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/detect":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        data = np.frombuffer(self.rfile.read(length), dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        tags = []
        if frame is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            seen = set()
            for family, detector in DETECTORS:
                corners, ids, _ = detector.detectMarkers(gray)
                if ids is None:
                    continue
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    center = tuple(marker_corners.reshape(4, 2).mean(axis=0).round().astype(int))
                    key = (center, int(marker_id))
                    if key in seen:
                        continue
                    seen.add(key)
                    tags.append({
                        "id": int(marker_id),
                        "family": family,
                        "corners": marker_corners.reshape(4, 2).round(1).tolist(),
                    })
            if not tags:
                pass
        ok, decoded, _, _ = QR_DETECTOR.detectAndDecodeMulti(frame) if frame is not None else (False, [], None, None)
        body = json.dumps({"tags": tags, "qr": [x for x in decoded if x] if ok else []}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        return


if __name__ == "__main__":
    HTTPServer(("127.0.0.1", 8766), Handler).serve_forever()
