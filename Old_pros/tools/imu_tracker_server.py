import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:  # pragma: no cover
    serial = None
    list_ports = None

SERIAL_PORT = None  # None = auto-detect VEX V5 serial port.
BAUD_RATE = 115200
HTTP_PORT = 8773
MAX_TRAIL = 260

NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
IMU_RE = re.compile(
    r"IMU sample=(?P<sample>\d+) t=(?P<t>\d+) port=(?P<port>\d+) status=(?P<status>-?\d+) "
    rf"gx=(?P<gx>{NUM_RE}) gy=(?P<gy>{NUM_RE}) gz=(?P<gz>{NUM_RE}) "
    rf"ax=(?P<ax>{NUM_RE}) ay=(?P<ay>{NUM_RE}) az=(?P<az>{NUM_RE}) "
    rf"pitch=(?P<pitch>{NUM_RE}) roll=(?P<roll>{NUM_RE}) yaw=(?P<yaw>{NUM_RE}) "
    rf"rot=(?P<rot>{NUM_RE}) head=(?P<head>{NUM_RE})"
)

state_lock = threading.Lock()
state = {
    "connected": False,
    "latest": None,
    "samples": [],
    "message": "Waiting for V5 serial output..."
}

HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>IMU 3JS Tracker</title>
    <style>
      :root { color-scheme: dark; --bg:#071119; --panel:#0b1723; --text:#eef6ff; --muted:#86a0b6; --accent:#50f0b7; --warn:#ffd166; }
      * { box-sizing: border-box; }
      html, body { margin:0; min-height:100%; background:var(--bg); color:var(--text); font-family: Arial, sans-serif; }
      body { padding:16px; }
      main { max-width:1400px; margin:0 auto; display:grid; grid-template-columns: minmax(360px,1fr) 320px; gap:16px; align-items:stretch; }
      #view { border:1px solid rgba(80,240,183,.28); min-height:560px; background:#03090d; position:relative; }
      canvas { position:absolute; inset:0; width:100%; height:100%; display:block; }
      aside { border:1px solid rgba(80,240,183,.24); background:var(--panel); padding:16px; display:grid; gap:14px; align-content:start; }
      h1 { margin:0; font-size:22px; }
      .metric { border-top:1px solid rgba(255,255,255,.08); padding-top:10px; }
      .label { color:var(--muted); text-transform:uppercase; letter-spacing:.06em; font-size:11px; }
      .value { font-size:31px; font-weight:700; line-height:1.1; color:var(--accent); font-variant-numeric:tabular-nums; }
      .small { font-size:17px; color:white; }
      button { font:inherit; background:#10263a; color:var(--text); border:1px solid rgba(80,240,183,.45); padding:8px 10px; }
      button:hover { background:#163656; }
      .status { color:var(--muted); min-height:44px; white-space:pre-line; }
      @media (max-width:900px) { body{padding:10px;} main{grid-template-columns:1fr;} aside{grid-row:auto;} #view{min-height:62vh;} }
    </style>
  </head>
  <body>
    <main>
      <section id="view" aria-label="IMU 3D tracker"></section>
      <aside>
        <h1>IMU Tracker</h1>
        <div class="metric"><div class="label">Integrated Angle</div><div class="value" id="angle">--.-°</div></div>
        <div class="metric"><div class="label">Yaw Absolute</div><div class="small" id="yaw">--.-°</div></div>
        <div class="metric"><div class="label">Gyro Z</div><div class="small" id="gz">--.--°/s</div></div>
        <div class="metric"><div class="label">Distance est.</div><div class="small" id="distance">--.-- m</div></div>
        <div class="metric"><div class="label">Samples / sec</div><div class="small" id="sampleRate">--</div></div>
        <button id="reset">Reset Pose</button>
        <div class="status" id="status">Waiting for data…</div>
      </aside>
    </main>

    <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js"></script>
    <script>
      const panel = {
        angle: document.getElementById("angle"),
        yaw: document.getElementById("yaw"),
        gz: document.getElementById("gz"),
        distance: document.getElementById("distance"),
        sampleRate: document.getElementById("sampleRate"),
        status: document.getElementById("status"),
        reset: document.getElementById("reset"),
      };

      const state = {
        integratedDeg: 0,
        lastT: null,
        sampleT: null,
        yawZero: null,
        trail: [[0, 0]],
        x: 0,
        z: 0,
        vx: 0,
        vz: 0,
        latest: null,
        rateHistory: [],
      };

      const view = document.getElementById("view");
      if (!window.THREE) {
        panel.status.textContent = "Three.js failed to load.";
        throw new Error("THREE global missing");
      }
      const renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setPixelRatio(window.devicePixelRatio || 1);
      renderer.setSize(view.clientWidth, view.clientHeight);
      view.appendChild(renderer.domElement);

      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x070e16);
      const camera = new THREE.PerspectiveCamera(48, Math.max(1, view.clientWidth) / Math.max(1, view.clientHeight), 0.1, 300);
      camera.position.set(0, 7.5, 9.5);
      camera.lookAt(0, 0, 0);

      scene.add(new THREE.AmbientLight(0x8bb7e2, 0.7));
      const dir = new THREE.DirectionalLight(0xb6e1ff, 1.0);
      dir.position.set(6, 8, 4);
      scene.add(dir);

      const floor = new THREE.GridHelper(40, 40, 0x1f4f62, 0x0e2b3b);
      floor.position.y = 0;
      scene.add(floor);

      const robot = new THREE.Mesh(
        new THREE.CylinderGeometry(0.23, 0.23, 0.26, 24),
        new THREE.MeshStandardMaterial({ color: 0x46d6a8, roughness: 0.24, metalness: 0.05 })
      );
      robot.rotation.x = Math.PI / 2;
      robot.position.y = 0.13;
      scene.add(robot);

      const arrowDir = new THREE.Vector3(0, 0, -1);
      const arrow = new THREE.ArrowHelper(arrowDir, new THREE.Vector3(), 1.0, 0xffc96d, 0.28, 0.18);
      arrow.position.set(0, 0.24, 0);
      scene.add(arrow);

      const trailGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0.01, 0, 0)]);
      const trail = new THREE.Line(
        trailGeom,
        new THREE.LineBasicMaterial({ color: 0x6de2ff })
      );
      scene.add(trail);

      const pointGeom = new THREE.SphereGeometry(0.055, 10, 10);
      const pointMat = new THREE.MeshStandardMaterial({ color: 0xff5e7a });
      const latestPoint = new THREE.Mesh(pointGeom, pointMat);
      scene.add(latestPoint);

      const targetFov = camera.fov;

      function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
      }

      function wrapDeg(value) {
        let angle = value % 360;
        if (angle < -180) angle += 360;
        else if (angle > 180) angle -= 360;
        return angle;
      }

      function formatNumber(value, digits = 1) {
        return Number.isFinite(value) ? value.toFixed(digits) : "--.-";
      }

      function updateTrail(points) {
        const verts = points.map(([x, z]) => new THREE.Vector3(x, 0.02, -z));
        if (verts.length === 0) {
          verts.push(new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(0.001, 0.02, 0.001));
        }
        trail.geometry.setFromPoints(verts);
      }

      function applySample(sample) {
        if (!sample) return;

        const t = Number(sample.t);
        if (!Number.isFinite(state.lastT)) {
          state.lastT = t;
          return;
        }

        let dt = (t - state.lastT) / 1000;
        if (!Number.isFinite(dt) || dt <= 0) {
          state.lastT = t;
          return;
        }
        state.lastT = t;
        dt = clamp(dt, 0, 0.25);

        const gz = Number.isFinite(sample.gz) ? sample.gz : 0;
        if (Number.isFinite(sample.yaw)) {
          if (state.yawZero === null) state.yawZero = sample.yaw;
          state.integratedDeg = wrapDeg(sample.yaw - state.yawZero);
        } else {
          state.integratedDeg = wrapDeg(state.integratedDeg + gz * dt);
        }

        const axBody = clamp(Number.isFinite(sample.ax) ? sample.ax : 0, -2.5, 2.5);
        const ayBody = clamp(Number.isFinite(sample.ay) ? sample.ay : 0, -2.5, 2.5);
        state.vx = state.vx * 0.94 + axBody * dt;
        state.vz = state.vz * 0.94 + ayBody * dt;

        const rad = state.integratedDeg * Math.PI / 180;
        const worldVx = state.vx * Math.cos(rad) - state.vz * Math.sin(rad);
        const worldVz = state.vx * Math.sin(rad) + state.vz * Math.cos(rad);
        state.x += worldVx * dt;
        state.z += worldVz * dt;

        state.trail.push([state.x, state.z]);
        if (state.trail.length > 260) state.trail.shift();

        state.latest = {
          x: state.x,
          z: state.z,
          dist: Math.hypot(state.x, state.z),
        };
      }

      async function poll() {
        try {
          const res = await fetch("/data", { cache: "no-store" });
          const data = await res.json();
          const latest = data.latest;
          if (latest) {
            const sampleTime = latest.t;
            if (state.sampleT != null) {
              const dt = (sampleTime - state.sampleT) / 1000;
              const hz = dt > 0 ? 1 / dt : 0;
              state.rateHistory.push(hz);
              if (state.rateHistory.length > 35) state.rateHistory.shift();
            }
            state.sampleT = sampleTime;
            applySample(latest);
          }

          panel.angle.textContent = `${formatNumber(state.integratedDeg, 1)}°`;
          panel.yaw.textContent = latest && Number.isFinite(latest.yaw) ? `${formatNumber(latest.yaw, 1)}°` : "--.-°";
          panel.gz.textContent = latest && Number.isFinite(latest.gz) ? `${formatNumber(latest.gz, 2)} °/s` : "--.-- °/s";
          panel.distance.textContent = state.latest ? `${formatNumber(state.latest.dist, 2)} m` : "--.-- m";
          panel.status.textContent = data.message || (data.connected ? "Live IMU stream." : "No serial data.");
          if (state.rateHistory.length) {
            const sum = state.rateHistory.reduce((acc, value) => acc + value, 0);
            panel.sampleRate.textContent = `${formatNumber(sum / state.rateHistory.length, 1)} Hz`;
          } else {
            panel.sampleRate.textContent = "--";
          }
        } catch (err) {
          panel.status.textContent = `Server error: ${err.name}`;
        }
        window.setTimeout(poll, 50);
      }

      function render() {
        const p = state.latest || { x: 0, z: 0, dist: 0 };
        robot.position.set(p.x, robot.position.y, -p.z);
        latestPoint.position.set(p.x, 0.32, -p.z);
        latestPoint.visible = true;
        arrow.position.set(p.x, 0.24, -p.z);
        const theta = state.integratedDeg * Math.PI / 180;
        arrow.setDirection(new THREE.Vector3(Math.sin(-theta), 0, Math.cos(theta)).normalize());
        arrow.setLength(Math.max(0.35, Math.min(1.5, 0.4 + p.dist * 0.12)));

        const points = state.trail;
        if (points.length >= 2) updateTrail(points);
        else updateTrail([[0, 0]]);

        const camTarget = new THREE.Vector3(p.x, 0.05, -p.z);
        const dist = Math.max(6, Math.min(22, 10 + Math.abs(p.dist)));
        const camX = p.x + dist * Math.sin(theta + Math.PI / 4);
        const camZ = -p.z + dist * Math.cos(theta + Math.PI / 4);
        camera.position.lerp(new THREE.Vector3(camX, dist * 0.65, camZ), 0.1);
        camera.lookAt(camTarget);
        camera.fov += (targetFov - camera.fov) * 0.1;
        camera.updateProjectionMatrix();

        renderer.render(scene, camera);
        requestAnimationFrame(render);
      }

      function resetPose() {
        state.x = 0;
        state.z = 0;
        state.vx = 0;
        state.vz = 0;
        state.integratedDeg = 0;
        state.lastT = null;
        state.sampleT = null;
        state.yawZero = null;
        state.trail = [[0, 0]];
      }

      panel.reset.onclick = resetPose;
      window.addEventListener("resize", () => {
        camera.aspect = view.clientWidth / Math.max(1, view.clientHeight);
        camera.updateProjectionMatrix();
        renderer.setSize(view.clientWidth, view.clientHeight);
      });

      resetPose();
      poll();
      render();
    </script>
  </body>
</html>
"""


def to_number(value: str) -> float:
    return float(value) if value is not None else float("nan")


def now():
    return time.time()


def current_payload():
    with state_lock:
        latest = state["latest"]
        payload = {
            "connected": state["connected"],
            "latest": latest,
            "message": state["message"],
        }
    return payload


def parse_line(line: str):
    match = IMU_RE.search(line)
    if not match:
        return None
    return {
        "sample": int(match.group("sample")),
        "t": int(match.group("t")),
        "port": int(match.group("port")),
        "status": int(match.group("status")),
        "gx": to_number(match.group("gx")),
        "gy": to_number(match.group("gy")),
        "gz": to_number(match.group("gz")),
        "ax": to_number(match.group("ax")),
        "ay": to_number(match.group("ay")),
        "az": to_number(match.group("az")),
        "pitch": to_number(match.group("pitch")),
        "roll": to_number(match.group("roll")),
        "yaw": to_number(match.group("yaw")),
        "rot": to_number(match.group("rot")),
        "head": to_number(match.group("head")),
        "received": now(),
    }


def update_state(**changes):
    with state_lock:
        state.update(changes)


def serial_candidates():
    if SERIAL_PORT:
        return [SERIAL_PORT]

    ports = []
    for port in list_ports.comports():
        text = f"{port.device} {port.description} {port.hwid}".lower()
        if "vex" not in text and "vid:pid=2888" not in text:
            continue
        priority = 0
        if "user port" in text:
            priority = -2
        elif "communication" in text:
            priority = -1
        ports.append((priority, port.device, port.description))

    ports.sort()
    return [device for _, device, _ in ports]


def read_serial():
    while True:
        candidates = serial_candidates()
        if not candidates:
            update_state(connected=False, message="No VEX V5 serial port found.")
            time.sleep(0.8)
            continue

        for port_name in candidates:
            try:
                with serial.Serial(port_name, BAUD_RATE, timeout=0.2) as ser:
                    update_state(connected=True, message=f"Reading V5 serial on {port_name}.")
                    buffer = ""
                    while True:
                        chunk = ser.read(4096)
                        if not chunk:
                            continue
                        buffer += chunk.decode("utf-8", errors="ignore").replace("\x00", "")
                        lines = buffer.splitlines()
                        buffer = lines.pop() if lines else ""
                        for line in lines:
                            sample = parse_line(line)
                            if sample is None:
                                continue
                            sample["serial_port"] = port_name
                            with state_lock:
                                state["latest"] = sample
                                state["samples"].append(sample)
                                state["samples"] = state["samples"][-MAX_TRAIL:]
                                state["message"] = f"Live IMU on port {sample['port']} via {port_name}."
            except serial.SerialException as exc:
                update_state(connected=False, message=f"{port_name}: {exc}")
                time.sleep(0.2)

        time.sleep(0.6)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/data":
            body = json.dumps(current_payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_error(404)

    def log_message(self, *_):
        return


if __name__ == "__main__":
    if serial is None:
        raise SystemExit("pyserial not installed. Install with: pip install pyserial")
    threading.Thread(target=read_serial, daemon=True).start()
    HTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()

