import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

try:
    import cv2
except ImportError:
    cv2 = None
import serial
from cobs import cobs
from serial.tools import list_ports


BAUD_RATE = 115200
HTTP_PORT = 8774
PORTS = (1,)
DRIVE_MOTORS = (17, 18, 11, 13)
HORIZONTAL_ODOMETER_PORT = 15
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_PIXELS_PER_INCH = 30.0
CAMERA_START_Y_IN = 0.5
CAMERA_MIN_SCORE = 0.62
DEBUG_WEBCAM_ENABLED = os.environ.get("MOREVEX_DEBUG_WEBCAM", "0") == "1"
CAMERA_TEMPLATE_PATH = (
    Path(__file__).resolve().parents[1]
    / "agent-workflow"
    / "localization-sensor-repair"
    / "camera_robot_marker.png"
)
CAMERA_CALIBRATION_PATH = CAMERA_TEMPLATE_PATH.with_name("camera_calibration.json")
MOTOR_VALUE_RE = r"[-+]?(?:\d+(?:\.\d+)?|inf)|nan"
INTEGER_VALUE_RE = r"[-+]?\d+"
D4_RE = re.compile(
    r"D4 s=(?P<sample>\d+) t=(?P<brain_ms>\d+) "
    r"p1=(?P<p1_mm>-?\d+),(?P<p1_conf>-?\d+),(?P<p1_inst>\d+)"
    rf"(?: m17=(?P<m17>{MOTOR_VALUE_RE}) m18=(?P<m18>{MOTOR_VALUE_RE}) "
    rf"m11=(?P<m11>{MOTOR_VALUE_RE}) m13=(?P<m13>{MOTOR_VALUE_RE}))?"
    rf"(?: h(?:5|15)=(?P<h5>{INTEGER_VALUE_RE})"
    rf"(?: h(?:5|15)abs={MOTOR_VALUE_RE})?)?"
    rf"(?: imu=(?P<imu>{MOTOR_VALUE_RE}) rawimu=(?P<rawimu>{MOTOR_VALUE_RE}) "
    rf"imust=(?P<imust>{INTEGER_VALUE_RE}))?"
    rf"(?: imugyro=(?P<imu_gyro_x>{MOTOR_VALUE_RE}),(?P<imu_gyro_y>{MOTOR_VALUE_RE}),"
    rf"(?P<imu_gyro_z>{MOTOR_VALUE_RE}) imuacc=(?P<imu_acc_x>{MOTOR_VALUE_RE}),"
    rf"(?P<imu_acc_y>{MOTOR_VALUE_RE}),(?P<imu_acc_z>{MOTOR_VALUE_RE}))?"
    rf"(?: gps7=(?P<gps_x>{MOTOR_VALUE_RE}),(?P<gps_y>{MOTOR_VALUE_RE}),"
    rf"(?P<gps_heading>{MOTOR_VALUE_RE}),(?P<gps_error>{MOTOR_VALUE_RE}),"
    rf"(?P<gps_inst>{INTEGER_VALUE_RE}) errno=(?P<errno>{INTEGER_VALUE_RE}))?"
    rf"(?: gpsgyro=(?P<gps_gyro_z>{MOTOR_VALUE_RE}))?"
)
FUSE_TEST_PREFIX = "FUSE_TEST"
VISION_SHADOW_PREFIX = "VISION_SHADOW"

state_lock = threading.Lock()
state = {
    "connected": False,
    "serial_port": None,
    "latest": None,
    "message": "Waiting for V5 Brain serial data...",
    "rate_hz": 0.0,
    "last_sample": None,
    "last_sample_time": None,
    "last_brain_ms": None,
    "reset_token": 0,
    "camera_reset_token": 0,
    "onboard_pose": None,
    "last_fuse_line": None,
    "vision": {
        "installed": False,
        "configured": False,
        "valid": False,
        "reason": "waiting",
        "message": "Waiting for onboard AI Vision telemetry...",
    },
    "camera_pose": {
        "available": False,
        "message": "Fixed camera tracker starting...",
    },
}

HTML = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>__PAGE_TITLE__</title>
    <style>
      :root {
        color-scheme: dark;
        --bg: #070909;
        --panel: #101615;
        --line: #263330;
        --text: #f4fbf8;
        --muted: #93a39d;
        --green: #5cffad;
        --cyan: #57c7ff;
        --yellow: #ffd166;
        --red: #ff6b6b;
      }
      * { box-sizing: border-box; }
      html, body { margin: 0; min-height: 100%; background: var(--bg); color: var(--text); font-family: Arial, sans-serif; }
      body { padding: 16px; }
      main { max-width: 1180px; margin: 0 auto; display: grid; gap: 14px; }
      header { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; border-bottom: 1px solid var(--line); padding-bottom: 10px; }
      h1 { margin: 0; font-size: 20px; font-weight: 700; }
      .status { color: var(--muted); font-size: 14px; text-align: right; }
      .tabs { display: flex; gap: 8px; align-items: center; }
      .tab { color: var(--muted); text-decoration: none; border: 1px solid var(--line); background: #0d1312; padding: 7px 10px; font-size: 13px; }
      .tab.active { color: var(--text); border-color: var(--cyan); background: #12201f; }
      .chart { height: min(64vh, 560px); min-height: 360px; position: relative; border: 1px solid var(--line); background: linear-gradient(180deg, #121918, #0b0f0e); overflow: hidden; }
      canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
      .field-panel { display: grid; grid-template-columns: minmax(320px, 560px) 1fr; gap: 14px; align-items: stretch; }
      .field-map { min-height: 420px; position: relative; border: 1px solid var(--line); background: #0c1110; overflow: hidden; }
      .field-hud { position: absolute; left: 12px; top: 12px; z-index: 2; min-width: 220px; border: 1px solid rgba(87,199,255,.55); background: rgba(7,13,12,.86); padding: 10px 12px; font-size: 13px; line-height: 1.45; color: var(--text); font-variant-numeric: tabular-nums; }
      .field-hud strong { display: block; font-size: 18px; line-height: 1.1; margin-bottom: 3px; }
      .field-hud span { color: var(--muted); }
      .pose-card { border: 1px solid var(--line); background: var(--panel); padding: 14px 16px; display: grid; gap: 10px; align-content: start; }
      .pose-header { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
      .pose-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
      .pose-value { font-size: 28px; font-weight: 800; font-variant-numeric: tabular-nums; }
      .small-button { border: 1px solid var(--line); background: #18211f; color: var(--text); padding: 7px 10px; font: inherit; font-size: 12px; cursor: pointer; }
      .small-button:hover { border-color: var(--cyan); }
      .field-reset-button { display: none; position: absolute; top: 12px; right: 12px; z-index: 3; border: 1px solid rgba(87,199,255,.75); background: rgba(10,18,17,.88); color: var(--text); padding: 8px 11px; font: inherit; font-size: 13px; cursor: pointer; }
      .field-reset-button:hover { background: rgba(18,32,31,.95); }
      .readouts { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
      .motor-readouts { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
      .label { display: grid; gap: 4px; padding: 10px; border: 1px solid var(--line); background: var(--panel); min-height: 116px; }
      .port { color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }
      .mm { font-size: 28px; line-height: 1; font-weight: 700; font-variant-numeric: tabular-nums; }
      .inch, .conf { color: var(--muted); font-size: 14px; font-variant-numeric: tabular-nums; }
      .theta-panel { display: grid; grid-template-columns: 1fr; gap: 14px; align-items: stretch; }
      .theta-card { border: 1px solid var(--line); background: var(--panel); padding: 14px 16px; display: grid; gap: 6px; }
      .primary-stats { grid-template-columns: repeat(3, minmax(0, 1fr)); column-gap: 56px; row-gap: 16px; align-items: end; }
      .stat { display: grid; gap: 6px; min-width: 0; padding-right: 36px; border-right: 1px solid var(--line); }
      .stat:nth-child(3) { padding-right: 0; border-right: 0; }
      .theta-label { color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }
      .theta-value { font-size: clamp(42px, 5vw, 72px); line-height: .95; font-weight: 800; font-variant-numeric: tabular-nums; white-space: nowrap; }
      .theta-value.hidden { color: var(--text); font-size: clamp(42px, 5vw, 72px); }
      .theta-detail { color: var(--muted); font-size: 15px; line-height: 1.35; }
      footer { display: flex; justify-content: space-between; gap: 12px; color: var(--muted); font-size: 13px; }
      body.field-only { padding: 0; overflow: hidden; }
      body.field-only main { max-width: none; width: 100vw; height: 100vh; margin: 0; display: block; }
      body.field-only header,
      body.field-only .chart,
      body.field-only .pose-card,
      body.field-only .theta-panel,
      body.field-only .motor-readouts,
      body.field-only .readouts,
      body.field-only footer { display: none; }
      body.field-only .field-panel { display: block; width: 100vw; height: 100vh; }
      body.field-only .field-map { width: 100vw; height: 100vh; min-height: 0; border: 0; }
      body.field-only .field-reset-button { display: block; }
      @media (max-width: 720px) {
        body { padding: 10px; }
        body.field-only { padding: 0; }
        header, footer { display: grid; text-align: left; }
        .status { text-align: left; }
        .chart { height: 360px; min-height: 360px; }
        .theta-panel { grid-template-columns: 1fr; }
        .primary-stats { grid-template-columns: 1fr; }
        .field-panel { grid-template-columns: 1fr; }
        .readouts, .motor-readouts { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      }
    </style>
  </head>
  <body class="__BODY_CLASS__">
    <main>
      <header>
        <h1>VEX Multi-Sensor Localization</h1>
        <nav class="tabs" aria-label="Views">
          <a class="tab __DASHBOARD_TAB_CLASS__" href="/">Dashboard</a>
          <a class="tab __FIELD_TAB_CLASS__" href="/playing-field">Playing Field</a>
          <a class="tab __FIELD_TAB_CLASS__" href="/localization">Localization</a>
        </nav>
        <div class="status" id="status">Connecting...</div>
      </header>
      <section class="chart"><canvas id="line-chart"></canvas></section>
      <section class="field-panel">
        <div class="field-map">
          <canvas id="field-map"></canvas>
          <div class="field-hud" id="field-hud">Waiting for pose...</div>
          <button class="field-reset-button" id="field-reset" type="button">Start Pose Help</button>
        </div>
        <div class="pose-card">
          <div class="pose-header">
            <div class="theta-label">Field Position</div>
            <button class="small-button" id="pose-reset" type="button">Start Pose Help</button>
          </div>
          <div class="theta-detail" id="pose-status">Waiting for drive motor telemetry.</div>
          <div class="theta-detail" id="pose-source">Source: browser odometry fallback.</div>
          <div class="pose-grid">
            <div>
              <div class="theta-label">X</div>
              <div class="pose-value" id="pose-x">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Y</div>
              <div class="pose-value" id="pose-y">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Heading</div>
              <div class="pose-value" id="pose-heading">--.- deg</div>
            </div>
            <div>
              <div class="theta-label">Wheel Travel</div>
              <div class="pose-value" id="pose-travel">--.- in</div>
            </div>
            <div>
              <div class="theta-label">DR Since Fix</div>
              <div class="pose-value" id="pose-dr-travel">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Error Envelope</div>
              <div class="pose-value" id="pose-error-envelope">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Absolute Fix Age</div>
              <div class="pose-value" id="pose-absolute-age">--.- s</div>
            </div>
            <div>
              <div class="theta-label">Left Wheel</div>
              <div class="pose-value" id="pose-left-wheel">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Right Wheel</div>
              <div class="pose-value" id="pose-right-wheel">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Last Delta</div>
              <div class="pose-value" id="pose-last-delta">L --.- / R --.-</div>
            </div>
            <div>
              <div class="theta-label">Turn Delta</div>
              <div class="pose-value" id="pose-turn-delta">--.- deg</div>
            </div>
            <div>
              <div class="theta-label">Side Odom</div>
              <div class="pose-value" id="pose-side-odom">--.- in</div>
            </div>
            <div>
              <div class="theta-label">Legacy Wall Fit</div>
              <div class="pose-value" id="pose-lidar-assist">off</div>
            </div>
            <div>
              <div class="theta-label">AI Vision</div>
              <div class="pose-value" id="pose-ai-vision">waiting</div>
            </div>
            <div>
              <div class="theta-label">Effective Track</div>
              <div class="pose-value" id="cal-track-width">--.-- in</div>
            </div>
            <div>
              <div class="theta-label">Odom Lever Arm</div>
              <div class="pose-value" id="cal-rear-lever">--.-- in</div>
            </div>
            <div>
              <div class="theta-label">Rear Wheel at 15 deg</div>
              <div class="pose-value" id="cal-rear-15">--.-- in</div>
            </div>
            <div>
              <div class="theta-label">L+1 / R-1 Turn</div>
              <div class="pose-value" id="cal-counterturn">--.-- deg</div>
            </div>
            <div>
              <div class="theta-label">Legacy Fit Scale</div>
              <div class="pose-value" id="cal-lidar-scale">--.--- x</div>
            </div>
          </div>
        </div>
      </section>
      <section class="theta-panel">
        <div class="theta-card primary-stats">
          <div class="stat">
            <div class="theta-label">Legacy Wall Range</div>
            <div class="theta-value" id="center-distance">--.-- in</div>
          </div>
          <div class="stat">
            <div class="theta-label">Angle</div>
            <div class="theta-value" id="theta-value">--.- deg</div>
          </div>
          <div class="stat">
            <div class="theta-label">RMSE</div>
            <div class="theta-value" id="rmse-value">--.-- in</div>
          </div>
          <div class="theta-detail" id="theta-detail">Legacy four-sensor wall fit is disabled on this robot.</div>
        </div>
      </section>
      <section class="motor-readouts" id="motor-readouts"></section>
      <section class="readouts" id="readouts"></section>
      <footer>
        <div id="rate">0.0 Hz</div>
        <div id="fit">theta -- deg | fit --</div>
        <div id="scale">Scale: 0-2000 mm</div>
      </footer>
    </main>
    <script>
      const ports = [1];
      const driveMotors = [17, 18, 11, 13];
      const canvas = document.getElementById("line-chart");
      const ctx = canvas.getContext("2d");
      const fieldCanvas = document.getElementById("field-map");
      const fieldCtx = fieldCanvas.getContext("2d");
      const fieldHudEl = document.getElementById("field-hud");
      const readouts = document.getElementById("readouts");
      const motorReadouts = document.getElementById("motor-readouts");
      const statusEl = document.getElementById("status");
      const rateEl = document.getElementById("rate");
      const scaleEl = document.getElementById("scale");
      const fitEl = document.getElementById("fit");
      const thetaValueEl = document.getElementById("theta-value");
      const centerDistanceEl = document.getElementById("center-distance");
      const rmseValueEl = document.getElementById("rmse-value");
      const thetaDetailEl = document.getElementById("theta-detail");
      const gateDetailEl = document.getElementById("gate-detail");
      const poseStatusEl = document.getElementById("pose-status");
      const poseSourceEl = document.getElementById("pose-source");
      const poseXEl = document.getElementById("pose-x");
      const poseYEl = document.getElementById("pose-y");
      const poseHeadingEl = document.getElementById("pose-heading");
      const poseTravelEl = document.getElementById("pose-travel");
      const poseDrTravelEl = document.getElementById("pose-dr-travel");
      const poseErrorEnvelopeEl = document.getElementById("pose-error-envelope");
      const poseAbsoluteAgeEl = document.getElementById("pose-absolute-age");
      const poseLeftWheelEl = document.getElementById("pose-left-wheel");
      const poseRightWheelEl = document.getElementById("pose-right-wheel");
      const poseLastDeltaEl = document.getElementById("pose-last-delta");
      const poseTurnDeltaEl = document.getElementById("pose-turn-delta");
      const poseSideOdomEl = document.getElementById("pose-side-odom");
      const poseLidarAssistEl = document.getElementById("pose-lidar-assist");
      const poseAiVisionEl = document.getElementById("pose-ai-vision");
      const calTrackWidthEl = document.getElementById("cal-track-width");
      const calRearLeverEl = document.getElementById("cal-rear-lever");
      const calRear15El = document.getElementById("cal-rear-15");
      const calCounterturnEl = document.getElementById("cal-counterturn");
      const calLidarScaleEl = document.getElementById("cal-lidar-scale");
      const poseResetEl = document.getElementById("pose-reset");
      const fieldResetEl = document.getElementById("field-reset");
      const sensorSpacingIn = 2;
      const fieldSizeIn = 144;
      const fieldHalfSpanIn = fieldSizeIn / 2;
      const physicalWallHalfSpanIn = 70.2;
      const goalTagLandmarks = [
        { name: "center", id: 0, color: "neutral", x: 0, y: 0 },
        { name: "top red neutral", id: 4, color: "neutral", x: 47.10, y: 23.55 },
        { name: "top blue alliance", id: 3, color: "blue", x: 47.10, y: -23.54 },
        { name: "upper red neutral", id: 1, color: "neutral", x: 23.55, y: 47.10 },
        { name: "upper blue alliance", id: 2, color: "blue", x: 23.55, y: -47.09 },
        { name: "lower red alliance", id: 2, color: "red", x: -23.54, y: 47.10 },
        { name: "lower blue neutral", id: 1, color: "neutral", x: -23.54, y: -47.09 },
        { name: "bottom red alliance", id: 3, color: "red", x: -47.09, y: 23.55 },
        { name: "bottom blue neutral", id: 4, color: "neutral", x: -47.09, y: -23.54 },
      ];
      // Encoder-distance calibration from the 2/5/10-inch straight trials.
      // The physical wheels are 2.75 in, but their measured rolling distance
      // is equivalent to a 2.433055-inch wheel on this drivetrain.
      const effectiveWheelDiameterIn = 2.4330552523;
      const wheelCircumferenceIn = Math.PI * effectiveWheelDiameterIn;
      // Preserve the wheel-diameter/track-width ratio validated by the live
      // turn trials when applying the straight-distance encoder scale.
      const trackWidthIn = 10.6245815677;
      const leftEncoderSign = 1;
      const rightEncoderSign = 1;
      const horizontalOdometerPort = 5;
      const horizontalOdometerDiameterIn = 2;
      const horizontalOdometerCircumferenceIn = Math.PI * horizontalOdometerDiameterIn;
      const horizontalOdometerOffsetBackIn = 5.18;
      const horizontalOdometerSign = 1;
      const horizontalOdometerEnabled = false;
      // Replacement robot: P8 translation/yaw has not been measured. Keep the
      // overlay fail-closed rather than reusing the previous robot's mount.
      const aiCameraForwardOffsetIn = 0;
      const aiCameraRightOffsetIn = 0;
      const aiCameraYawRightDeg = 0;
      const aiCameraExtrinsicsQualified = false;
      const aiGoalFaceOffsetIn = 5.61 / 2;
      const lidarThetaScale = 0.926770;
      const minConfidence = 50;
      const maxRmseIn = 0.16;
      const maxPointErrorIn = 0.55;
      const lidarMaxAssistDistanceIn = 50;
      const lidarThetaErrorInchesPerDeg = 0.5;
      const startPose = { x: 30.18, y: 34.70, headingRad: 151.65 * Math.PI / 180 };
      let latest = null;
      let maxScale = 2000;
      let lastGoodLidarEstimate = null;
      let seenResetToken = null;
      let onboardPath = [];
      let lastOnboardPcTime = null;
      const pose = {
        x: startPose.x,
        y: startPose.y,
        headingRad: startPose.headingRad,
        leftDeg: null,
        rightDeg: null,
        sideCentideg: null,
        leftTravelIn: 0,
        rightTravelIn: 0,
        driveVerticalIn: 0,
        sideTravelIn: 0,
        lastLeftDeltaIn: 0,
        lastRightDeltaIn: 0,
        lastDriveVerticalDeltaIn: 0,
        lastSideDeltaIn: 0,
        lastTurnDeltaDeg: 0,
        lastLidarCorrectionIn: 0,
        lastLidarWall: "off",
        lastLidarThetaErrorDeg: null,
        lastLidarDistanceErrorIn: null,
        lastSample: null,
        travelIn: 0,
        path: [{ x: startPose.x, y: startPose.y }],
        ready: false,
        requiresReset: false,
      };

      function fitLine(samples) {
        if (samples.length < 2) return null;
        const n = samples.length;
        const sumX = samples.reduce((sum, point) => sum + point.arrayXIn, 0);
        const sumY = samples.reduce((sum, point) => sum + point.distanceIn, 0);
        const meanX = sumX / n;
        const meanY = sumY / n;
        let ssXX = 0;
        let ssXY = 0;
        let ssYY = 0;
        for (const point of samples) {
          const dx = point.arrayXIn - meanX;
          const dy = point.distanceIn - meanY;
          ssXX += dx * dx;
          ssXY += dx * dy;
          ssYY += dy * dy;
        }
        const slope = ssXX > 0 ? ssXY / ssXX : 0;
        const intercept = meanY - slope * meanX;
        let sse = 0;
        let maxAbsError = 0;
        for (const point of samples) {
          const error = point.distanceIn - (slope * point.arrayXIn + intercept);
          maxAbsError = Math.max(maxAbsError, Math.abs(error));
          sse += error * error;
        }
        const rmse = Math.sqrt(sse / n);
        const r2 = ssYY > 0 ? Math.max(0, 1 - sse / ssYY) : 1;
        return {
          slope,
          intercept,
          thetaDeg: Math.atan(slope) * 180 / Math.PI * lidarThetaScale,
          rmse,
          maxAbsError,
          r2,
        };
      }

      function coupledSideReading(motors, motorPorts) {
        const values = motorPorts
          .map(port => motors[String(port)]?.position_deg)
          .filter(value => Number.isFinite(value));
        if (values.length !== motorPorts.length) return null;
        if (Math.max(...values) - Math.min(...values) > 15) return null;
        return values.reduce((sum, value) => sum + value, 0) / values.length;
      }

      function driveSensorHealth(motors) {
        const side = (name, ports) => {
          const values = ports.map(port => motors[String(port)]?.position_deg);
          const present = ports.filter((port, index) => Number.isFinite(values[index]));
          const missing = ports.filter(port => !present.includes(port));
          const spread = present.length === ports.length
            ? Math.max(...values) - Math.min(...values)
            : null;
          return `${name} ${present.length}/${ports.length}` +
            (missing.length ? ` missing ${missing.join("/")}` : ` spread ${spread.toFixed(1)} deg`) +
            (Number.isFinite(spread) && spread > 15 ? " INVALID" : "");
        };
        return `${side("L", [17, 18])} | ${side("R", [11, 13])}`;
      }

      function resetPoseBaseline(frame = latest?.latest) {
        const motors = frame?.motors || {};
        const odometer = frame?.odometer || {};
        const leftDeg = coupledSideReading(motors, [17, 18]);
        const rightDeg = coupledSideReading(motors, [11, 13]);
        const sideCentideg = horizontalOdometerEnabled
          ? odometer[String(horizontalOdometerPort)]?.position_centideg
          : null;
        pose.x = startPose.x;
        pose.y = startPose.y;
        pose.headingRad = startPose.headingRad;
        pose.leftDeg = Number.isFinite(leftDeg) ? leftDeg : null;
        pose.rightDeg = Number.isFinite(rightDeg) ? rightDeg : null;
        pose.sideCentideg = Number.isFinite(sideCentideg) ? sideCentideg : null;
        pose.leftTravelIn = 0;
        pose.rightTravelIn = 0;
        pose.driveVerticalIn = 0;
        pose.sideTravelIn = 0;
        pose.lastLeftDeltaIn = 0;
        pose.lastRightDeltaIn = 0;
        pose.lastDriveVerticalDeltaIn = 0;
        pose.lastSideDeltaIn = 0;
        pose.lastTurnDeltaDeg = 0;
        pose.lastLidarCorrectionIn = 0;
        pose.lastLidarWall = "off";
        pose.lastLidarThetaErrorDeg = null;
        pose.lastLidarDistanceErrorIn = null;
        pose.lastSample = frame?.sample ?? null;
        pose.travelIn = 0;
        pose.path = [{ x: startPose.x, y: startPose.y }];
        pose.ready = Number.isFinite(leftDeg) && Number.isFinite(rightDeg);
        pose.requiresReset = !pose.ready;
        onboardPath = [];
        lastOnboardPcTime = null;
        poseStatusEl.textContent = pose.ready
          ? `Browser fallback reset to (${startPose.x}, ${startPose.y}) facing red.`
          : "Waiting for m17/m18 and m11/m13 motor positions.";
      }

      async function requestPoseReset() {
        resetPoseBaseline(latest?.latest);
        try {
          const res = await fetch("/reset-pose", { method: "POST", cache: "no-store" });
          const body = await res.json();
          if (Number.isFinite(body.reset_token)) {
            seenResetToken = body.reset_token;
          }
        } catch (err) {
          poseStatusEl.textContent = `Origin reset locally; reset broadcast failed: ${err.name}`;
        }
      }

      async function requestRuntimeStartPose() {
        window.alert(
          "On the V5 controller, hold X+Y to edit the exact start pose. " +
          "D-pad changes X/Y by 0.5 in; L1/R1 changes heading by 1 deg; " +
          "L2/R2 changes heading by 15 deg; press A while holding Y to save."
        );
      }

      function wallPerpendicularDistanceIn(fit, centerDistanceIn) {
        if (!fit || !Number.isFinite(centerDistanceIn)) return null;
        return centerDistanceIn / Math.sqrt(1 + fit.slope * fit.slope);
      }

      function normalizeDeg(angle) {
        return ((angle % 360) + 360) % 360;
      }

      function angleDiffDeg(a, b) {
        return Math.abs(((a - b + 540) % 360) - 180);
      }

      function lineAngleDiffDeg(a, b) {
        return Math.min(angleDiffDeg(a, b), angleDiffDeg(a + 180, b));
      }

      function fieldPerpendicularWallCandidates(point) {
        const headingDeg = normalizeDeg(pose.headingRad * 180 / Math.PI);
        return [
          { wall: "red", distanceIn: physicalWallHalfSpanIn - point.y, wallThetaDeg: 0 },
          { wall: "blue", distanceIn: point.y + physicalWallHalfSpanIn, wallThetaDeg: 180 },
          { wall: "audience", distanceIn: point.x + physicalWallHalfSpanIn, wallThetaDeg: 90 },
          { wall: "zero", distanceIn: physicalWallHalfSpanIn - point.x, wallThetaDeg: 270 },
        ].map(candidate => ({
          ...candidate,
          expectedSensorThetaDeg: normalizeDeg(candidate.wallThetaDeg - headingDeg),
        }));
      }

      function applyLidarAssist(thetaGate, fit, wallDistanceIn, points) {
        pose.lastLidarCorrectionIn = 0;
        pose.lastLidarWall = "off";
        pose.lastLidarThetaErrorDeg = null;
        pose.lastLidarDistanceErrorIn = null;
        if (!thetaGate?.ok || !fit || !Number.isFinite(wallDistanceIn) || wallDistanceIn <= 0 || wallDistanceIn > lidarMaxAssistDistanceIn) {
          return;
        }
        const maxPointDistanceIn = Math.max(...points.map(point => point.distanceIn));
        if (!Number.isFinite(maxPointDistanceIn) || maxPointDistanceIn > lidarMaxAssistDistanceIn) {
          return;
        }

        const candidates = fieldPerpendicularWallCandidates(pose)
          .map(candidate => {
            const distanceErrorIn = Math.abs(candidate.distanceIn - wallDistanceIn);
            const thetaErrorDeg = lineAngleDiffDeg(normalizeDeg(fit.thetaDeg), candidate.expectedSensorThetaDeg);
            return {
              ...candidate,
              distanceErrorIn,
              thetaErrorDeg,
              score: distanceErrorIn + thetaErrorDeg * lidarThetaErrorInchesPerDeg,
            };
          })
          .sort((a, b) => a.score - b.score);
        if (!candidates.length) return;
        const chosen = candidates[0];
        pose.lastLidarWall = `heading-only ${chosen.wall} d${chosen.distanceErrorIn.toFixed(1)} t${chosen.thetaErrorDeg.toFixed(0)}`;
        pose.lastLidarThetaErrorDeg = chosen.thetaErrorDeg;
        pose.lastLidarDistanceErrorIn = chosen.distanceErrorIn;
      }

      function displayPoseFromLatest() {
        const onboard = latest?.onboard_pose;
        const camera = latest?.camera_pose;
        const d4 = latest?.latest;
        const nowSec = Date.now() / 1000;
        const d4Fresh = Boolean(
          latest?.connected &&
          Number.isFinite(d4?.pc_time) &&
          nowSec - d4.pc_time <= 1.5
        );
        const cameraFresh = Boolean(
          camera?.available &&
          Number.isFinite(camera.field_y_in) &&
          Number.isFinite(camera.score) &&
          camera.score >= 0.62 &&
          Number.isFinite(camera.pc_time) &&
          nowSec - camera.pc_time <= 1.0
        );
        if (
          onboard &&
          Number.isFinite(onboard.x) &&
          Number.isFinite(onboard.y) &&
          Number.isFinite(onboard.heading_deg) &&
          onboard.pose_valid === 1 &&
          Number.isFinite(onboard.pc_time) &&
          nowSec - onboard.pc_time <= 2.5
        ) {
          const displayY = onboard.y;
          if (lastOnboardPcTime !== onboard.pc_time) {
            onboardPath.push({ x: onboard.x, y: displayY });
            if (onboardPath.length > 600) onboardPath.shift();
            lastOnboardPcTime = onboard.pc_time;
          }
          const ageSec = Math.max(0, nowSec - onboard.pc_time);
          return {
            x: onboard.x,
            y: displayY,
            headingRad: onboard.heading_deg * Math.PI / 180,
            path: onboardPath.length ? onboardPath : [{ x: onboard.x, y: onboard.y }],
            ready: true,
            source: "onboard",
            label: "Tournament fused pose",
            detail: `${onboard.phase || "phase"} | age ${ageSec.toFixed(1)}s | lidar ${onboard.lidar || "none"}` +
              (cameraFresh ? ` | webcam debug ${(camera.score * 100).toFixed(0)}%` : " | webcam debug off"),
            phase: onboard.phase || "--",
          };
        }
        const onboardInvalid = Boolean(
          onboard && onboard.pose_valid === 0 && onboard.estimator_valid === 0
        );
        const onboardUninitialized = Boolean(
          onboard && onboard.pose_valid === 0 && onboard.estimator_valid === 1
        );
        return {
          x: pose.x,
          y: pose.y,
          headingRad: pose.headingRad,
          path: pose.path,
          ready: pose.ready && d4Fresh,
          source: "browser",
          label: d4Fresh
            ? "Browser fallback odometry"
            : "Browser fallback odometry — STALE",
          detail: pose.ready && !d4Fresh
            ? "D4 TELEMETRY STALE/DISCONNECTED; frozen browser pose is diagnostic history only."
            : (pose.ready
              ? (onboardInvalid || onboardUninitialized
                ? (onboardInvalid
                    ? "ONBOARD ESTIMATOR INVALID; browser drive-encoder fallback is diagnostic only."
                    : "ONBOARD POSE NOT INITIALIZED; call navigation::init() with a justified field pose.")
                : "D4 drive-encoder fallback only; no onboard fused pose is fresh.")
              : (onboardInvalid
                ? "ONBOARD ESTIMATOR INVALID; waiting for a justified reinitialization."
                : (onboardUninitialized
                    ? "ONBOARD POSE NOT INITIALIZED; call navigation::init() with a justified field pose."
                    : "Waiting for D4 motor telemetry."))),
          phase: "--",
        };
      }

      function updatePose(frame, thetaGate, fit, wallDistanceIn, points) {
        if (!frame || frame.sample === pose.lastSample) return;
        pose.lastSample = frame.sample;
        const motors = frame.motors || {};
        const odometer = frame.odometer || {};
        const leftDeg = coupledSideReading(motors, [17, 18]);
        const rightDeg = coupledSideReading(motors, [11, 13]);
        const sideCentideg = horizontalOdometerEnabled
          ? odometer[String(horizontalOdometerPort)]?.position_centideg
          : null;
        if (!Number.isFinite(leftDeg) || !Number.isFinite(rightDeg)) {
          pose.ready = false;
          pose.requiresReset = true;
          pose.leftDeg = null;
          pose.rightDeg = null;
          poseStatusEl.textContent = "Browser fallback invalid after coupled-encoder loss/spread; reset from a justified start pose.";
          return;
        }
        if (pose.requiresReset) {
          pose.ready = false;
          poseStatusEl.textContent = "Drive encoders recovered, but movement during the gap is unknown; reset the start pose.";
          return;
        }
        if (pose.leftDeg === null || pose.rightDeg === null) {
          pose.leftDeg = leftDeg;
          pose.rightDeg = rightDeg;
          pose.sideCentideg = Number.isFinite(sideCentideg) ? sideCentideg : null;
          pose.ready = true;
          poseStatusEl.textContent = `Browser fallback set to (${startPose.x}, ${startPose.y}) facing red.`;
          return;
        }

        const deltaLeftIn = ((leftDeg - pose.leftDeg) / 360) * wheelCircumferenceIn * leftEncoderSign;
        const deltaRightIn = ((rightDeg - pose.rightDeg) / 360) * wheelCircumferenceIn * rightEncoderSign;
        let deltaSideWheelIn = 0;
        let sideDeltaAccepted = false;
        if (Number.isFinite(sideCentideg) && Number.isFinite(pose.sideCentideg)) {
          deltaSideWheelIn =
            (((sideCentideg - pose.sideCentideg) / 100) / 360) *
            horizontalOdometerCircumferenceIn *
            horizontalOdometerSign;
          sideDeltaAccepted = true;
        }
        pose.leftDeg = leftDeg;
        pose.rightDeg = rightDeg;
        pose.sideCentideg = Number.isFinite(sideCentideg) ? sideCentideg : pose.sideCentideg;

        const inPlaceCounterRotation = deltaLeftIn * deltaRightIn < 0;
        const deltaCenterIn = inPlaceCounterRotation
          ? 0
          : (deltaLeftIn + deltaRightIn) / 2;
        const deltaHeadingRad = (deltaRightIn - deltaLeftIn) / trackWidthIn;
        const deltaSideCenterIn = sideDeltaAccepted
          ? deltaSideWheelIn - horizontalOdometerOffsetBackIn * deltaHeadingRad
          : 0;
        const midHeading = pose.headingRad + deltaHeadingRad / 2;
        const deltaDriveVerticalIn = deltaCenterIn * Math.cos(midHeading);
        pose.x += deltaCenterIn * Math.cos(midHeading) + deltaSideCenterIn * Math.sin(midHeading);
        pose.y += deltaCenterIn * Math.sin(midHeading) - deltaSideCenterIn * Math.cos(midHeading);
        pose.headingRad += deltaHeadingRad;
        applyLidarAssist(thetaGate, fit, wallDistanceIn, points);
        pose.leftTravelIn += deltaLeftIn;
        pose.rightTravelIn += deltaRightIn;
        pose.driveVerticalIn += deltaDriveVerticalIn;
        pose.sideTravelIn += deltaSideWheelIn;
        pose.lastLeftDeltaIn = deltaLeftIn;
        pose.lastRightDeltaIn = deltaRightIn;
        pose.lastDriveVerticalDeltaIn = deltaDriveVerticalIn;
        pose.lastSideDeltaIn = deltaSideCenterIn;
        pose.lastTurnDeltaDeg = deltaHeadingRad * 180 / Math.PI;
        pose.travelIn += Math.hypot(deltaCenterIn, deltaSideCenterIn);

        const last = pose.path[pose.path.length - 1];
        if (!last || Math.hypot(pose.x - last.x, pose.y - last.y) >= 0.5) {
          pose.path.push({ x: pose.x, y: pose.y });
          if (pose.path.length > 600) pose.path.shift();
        }
        const sideStatus = horizontalOdometerEnabled && Number.isFinite(sideCentideg)
          ? "port 5 side odom"
          : "port 5 side odom disabled";
        poseStatusEl.textContent = `Relative drive-encoder odometry; ${sideStatus}.`;
      }

      function drawLidarFieldEstimate(toScreen, fit, thetaGate, wallDistanceIn) {
        if (!fit || !Number.isFinite(wallDistanceIn) || wallDistanceIn <= 0) return;

        const sigmaIn = Math.max(0.08, fit.rmse || 0);
        const offsets = [-3, -2, -1, 0, 1, 2, 3];
        const baseAlpha = thetaGate?.ok ? 0.42 : 0.18;
        const normalizeDeg = angle => ((angle % 360) + 360) % 360;
        const rotate = (vector, degrees) => {
          const radians = degrees * Math.PI / 180;
          const c = Math.cos(radians);
          const s = Math.sin(radians);
          return {
            x: vector.x * c - vector.y * s,
            y: vector.x * s + vector.y * c,
          };
        };
        const clipLineToField = (center, direction) => {
          const hits = [];
          const add = (t, x, y) => {
            if (
              Number.isFinite(t) &&
              x >= -fieldHalfSpanIn - 0.001 &&
              x <= fieldHalfSpanIn + 0.001 &&
              y >= -fieldHalfSpanIn - 0.001 &&
              y <= fieldHalfSpanIn + 0.001
            ) {
              hits.push({
                t,
                x: Math.max(-fieldHalfSpanIn, Math.min(fieldHalfSpanIn, x)),
                y: Math.max(-fieldHalfSpanIn, Math.min(fieldHalfSpanIn, y)),
              });
            }
          };
          if (Math.abs(direction.x) > 0.0001) {
            let t = (-fieldHalfSpanIn - center.x) / direction.x;
            add(t, -fieldHalfSpanIn, center.y + t * direction.y);
            t = (fieldHalfSpanIn - center.x) / direction.x;
            add(t, fieldHalfSpanIn, center.y + t * direction.y);
          }
          if (Math.abs(direction.y) > 0.0001) {
            let t = (-fieldHalfSpanIn - center.y) / direction.y;
            add(t, center.x + t * direction.x, -fieldHalfSpanIn);
            t = (fieldHalfSpanIn - center.y) / direction.y;
            add(t, center.x + t * direction.x, fieldHalfSpanIn);
          }
          hits.sort((a, b) => a.t - b.t);
          if (hits.length < 2) return null;
          return [hits[0], hits[hits.length - 1]];
        };
        const drawArrow = (point, direction, alpha) => {
          const tip = toScreen(point);
          const tail = toScreen({
            x: point.x - direction.x * 7,
            y: point.y - direction.y * 7,
          });
          const angle = Math.atan2(tip.y - tail.y, tip.x - tail.x);
          const head = 7;
          fieldCtx.strokeStyle = `rgba(255,82,82,${alpha})`;
          fieldCtx.fillStyle = `rgba(255,82,82,${alpha})`;
          fieldCtx.lineWidth = 2;
          fieldCtx.beginPath();
          fieldCtx.moveTo(tail.x, tail.y);
          fieldCtx.lineTo(tip.x, tip.y);
          fieldCtx.stroke();
          fieldCtx.beginPath();
          fieldCtx.moveTo(tip.x, tip.y);
          fieldCtx.lineTo(tip.x - head * Math.cos(angle - 0.55), tip.y - head * Math.sin(angle - 0.55));
          fieldCtx.lineTo(tip.x - head * Math.cos(angle + 0.55), tip.y - head * Math.sin(angle + 0.55));
          fieldCtx.closePath();
          fieldCtx.fill();
        };
        const hypotheses = [
          {
            wall: "red",
            wallThetaDeg: 0,
            tangent: { x: 1, y: 0 },
            centerAt: distance => ({ x: 0, y: physicalWallHalfSpanIn - distance }),
            label: { x: 60, y: 66 },
          },
          {
            wall: "blue",
            wallThetaDeg: 180,
            tangent: { x: -1, y: 0 },
            centerAt: distance => ({ x: 0, y: -physicalWallHalfSpanIn + distance }),
            label: { x: 60, y: -66 },
          },
          {
            wall: "audience",
            wallThetaDeg: 90,
            tangent: { x: 0, y: 1 },
            centerAt: distance => ({ x: -physicalWallHalfSpanIn + distance, y: 0 }),
            label: { x: -66, y: 60 },
          },
          {
            wall: "zero",
            wallThetaDeg: 270,
            tangent: { x: 0, y: -1 },
            centerAt: distance => ({ x: physicalWallHalfSpanIn - distance, y: 0 }),
            label: { x: 66, y: 60 },
          },
        ];

        for (const hypothesis of hypotheses) {
          const estimatedHeadingDeg = normalizeDeg(hypothesis.wallThetaDeg - fit.thetaDeg);
          const arrowDirection = rotate({ x: 1, y: 0 }, estimatedHeadingDeg);
          for (const offset of offsets) {
            const weight = Math.exp(-0.5 * offset * offset);
            const distanceIn = wallDistanceIn + offset * sigmaIn;
            if (distanceIn < 0 || distanceIn > fieldSizeIn) continue;
            const center = hypothesis.centerAt(distanceIn);
            const clipped = clipLineToField(center, hypothesis.tangent);
            if (!clipped) continue;
            const p1 = toScreen(clipped[0]);
            const p2 = toScreen(clipped[1]);
            const alpha = Math.max(0.05, baseAlpha * weight).toFixed(3);
            fieldCtx.strokeStyle = `rgba(255,82,82,${alpha})`;
            fieldCtx.lineWidth = offset === 0 ? 4 : 3;
            fieldCtx.beginPath();
            fieldCtx.moveTo(p1.x, p1.y);
            fieldCtx.lineTo(p2.x, p2.y);
            fieldCtx.stroke();

            if (offset === 0) {
              const startT = Math.ceil(clipped[0].t / 12) * 12;
              for (let t = startT; t < clipped[1].t; t += 12) {
                drawArrow(
                  { x: center.x + hypothesis.tangent.x * t, y: center.y + hypothesis.tangent.y * t },
                  arrowDirection,
                  thetaGate?.ok ? 0.95 : 0.55
                );
              }
            }
          }

          const label = toScreen(hypothesis.label);
          fieldCtx.fillStyle = thetaGate?.ok ? "rgba(255,82,82,.95)" : "rgba(255,82,82,.55)";
          fieldCtx.font = "700 12px Arial";
          fieldCtx.textAlign = "left";
          fieldCtx.textBaseline = "bottom";
          fieldCtx.fillText(`${hypothesis.wall} H ${estimatedHeadingDeg.toFixed(1)} deg`, label.x, label.y);
        }
      }

      function drawDrivetrainVerticalEstimate(toScreen) {
        if (!pose.ready || !Number.isFinite(pose.driveVerticalIn)) return;
        const xIn = Math.max(-fieldHalfSpanIn, Math.min(fieldHalfSpanIn, pose.driveVerticalIn));
        const redPoint = toScreen({ x: xIn, y: fieldHalfSpanIn });
        const bluePoint = toScreen({ x: xIn, y: -fieldHalfSpanIn });

        fieldCtx.strokeStyle = "rgba(87,199,255,.92)";
        fieldCtx.lineWidth = 4;
        fieldCtx.beginPath();
        fieldCtx.moveTo(redPoint.x, redPoint.y);
        fieldCtx.lineTo(bluePoint.x, bluePoint.y);
        fieldCtx.stroke();

        fieldCtx.fillStyle = "rgba(87,199,255,.95)";
        fieldCtx.strokeStyle = "rgba(87,199,255,.95)";
        fieldCtx.lineWidth = 2;
        for (let yIn = -60; yIn <= 60; yIn += 12) {
          const p = toScreen({ x: xIn, y: yIn });
          fieldCtx.beginPath();
          fieldCtx.moveTo(p.x - 5, p.y - 4);
          fieldCtx.lineTo(p.x + 5, p.y);
          fieldCtx.lineTo(p.x - 5, p.y + 4);
          fieldCtx.stroke();
        }

        const arrowYIn = Math.max(-64, Math.min(64, pose.y));
        const base = toScreen({ x: 0, y: arrowYIn });
        const tip = toScreen({ x: xIn, y: arrowYIn });
        fieldCtx.setLineDash([6, 6]);
        fieldCtx.beginPath();
        fieldCtx.moveTo(base.x, base.y);
        fieldCtx.lineTo(tip.x, tip.y);
        fieldCtx.stroke();
        fieldCtx.setLineDash([]);
        const arrowSign = pose.driveVerticalIn >= 0 ? 1 : -1;
        fieldCtx.beginPath();
        fieldCtx.moveTo(tip.x, tip.y);
        fieldCtx.lineTo(tip.x - 6, tip.y + arrowSign * 10);
        fieldCtx.lineTo(tip.x + 6, tip.y + arrowSign * 10);
        fieldCtx.closePath();
        fieldCtx.fill();

        fieldCtx.font = "700 12px Arial";
        fieldCtx.textAlign = "left";
        fieldCtx.textBaseline = "bottom";
        fieldCtx.fillText(`drive X ${pose.driveVerticalIn.toFixed(1)} in`, redPoint.x + 8, redPoint.y - 8);
      }

      function drawGoalTagLandmarks(toScreen) {
        const colors = {
          neutral: { fill: "#171b1a", stroke: "#e6ece9" },
          red: { fill: "#c83f49", stroke: "#ffd5d8" },
          blue: { fill: "#2495c5", stroke: "#d3f3ff" },
        };
        for (const goal of goalTagLandmarks) {
          const point = toScreen(goal);
          const palette = colors[goal.color];
          fieldCtx.fillStyle = palette.fill;
          fieldCtx.strokeStyle = palette.stroke;
          fieldCtx.lineWidth = 2;
          fieldCtx.beginPath();
          fieldCtx.arc(point.x, point.y, 9, 0, Math.PI * 2);
          fieldCtx.fill();
          fieldCtx.stroke();
          fieldCtx.fillStyle = "#ffffff";
          fieldCtx.font = "700 11px Arial";
          fieldCtx.textAlign = "center";
          fieldCtx.textBaseline = "middle";
          fieldCtx.fillText(String(goal.id), point.x, point.y + 0.5);
        }
      }

      function drawAiVisionHypotheses(toScreen, displayPose) {
        if (!aiCameraExtrinsicsQualified) return;
        const vision = latest?.vision || {};
        const onboard = latest?.onboard_pose || {};
        if (!vision.valid || !Number.isFinite(vision.horizontal_range_in) ||
            vision.horizontal_range_in <= 0 || !Number.isFinite(vision.bearing_deg)) return;

        const headingRad = displayPose.headingRad;
        if (!Number.isFinite(headingRad)) return;
        const headingDeg = normalizeDeg(headingRad * 180 / Math.PI);
        const cameraHeadingDeg = normalizeDeg(headingDeg - aiCameraYawRightDeg);
        const rayRad = normalizeDeg(cameraHeadingDeg + vision.bearing_deg) * Math.PI / 180;
        const ray = { x: Math.cos(rayRad), y: Math.sin(rayRad) };
        const forward = { x: Math.cos(headingRad), y: Math.sin(headingRad) };
        const right = { x: Math.sin(headingRad), y: -Math.cos(headingRad) };
        const robotFromCamera = {
          x: aiCameraForwardOffsetIn * forward.x + aiCameraRightOffsetIn * right.x,
          y: aiCameraForwardOffsetIn * forward.y + aiCameraRightOffsetIn * right.y,
        };
        const faces = [
          { name: "+x", nx: 1, ny: 0 },
          { name: "+y", nx: 0, ny: 1 },
          { name: "-x", nx: -1, ny: 0 },
          { name: "-y", nx: 0, ny: -1 },
        ];
        const candidates = [];
        for (const goal of goalTagLandmarks.filter(item => item.id === vision.tag_id)) {
          for (const face of faces) {
            const tag = {
              x: goal.x + face.nx * aiGoalFaceOffsetIn,
              y: goal.y + face.ny * aiGoalFaceOffsetIn,
            };
            const camera = {
              x: tag.x - vision.horizontal_range_in * ray.x,
              y: tag.y - vision.horizontal_range_in * ray.y,
            };
            if ((camera.x - tag.x) * face.nx + (camera.y - tag.y) * face.ny <= 0) continue;
            const robot = {
              x: camera.x - robotFromCamera.x,
              y: camera.y - robotFromCamera.y,
            };
            if (Math.abs(robot.x) > fieldHalfSpanIn || Math.abs(robot.y) > fieldHalfSpanIn) continue;
            candidates.push({ goal, face, tag, robot });
          }
        }

        for (const candidate of candidates) {
          const tagPoint = toScreen(candidate.tag);
          const radiusPoint = toScreen({
            x: candidate.tag.x + vision.horizontal_range_in,
            y: candidate.tag.y,
          });
          const radiusPx = Math.abs(radiusPoint.x - tagPoint.x);
          const selected = candidate.goal.name.replaceAll(" ", "_") === onboard.ai_goal &&
            candidate.face.name === onboard.ai_face;
          fieldCtx.save();
          fieldCtx.setLineDash(selected ? [] : [5, 5]);
          fieldCtx.strokeStyle = selected ? "rgba(92,255,173,.75)" : "rgba(255,209,102,.28)";
          fieldCtx.lineWidth = selected ? 2.5 : 1.5;
          fieldCtx.beginPath();
          fieldCtx.arc(tagPoint.x, tagPoint.y, radiusPx, 0, Math.PI * 2);
          fieldCtx.stroke();
          fieldCtx.restore();

          const point = toScreen(candidate.robot);
          const sigmaIn = Math.max(1.0, vision.horizontal_range_in * 0.08);
          const sigmaPoint = toScreen({ x: candidate.robot.x + sigmaIn, y: candidate.robot.y });
          const sigmaPx = Math.abs(sigmaPoint.x - point.x);
          fieldCtx.fillStyle = selected ? "rgba(92,255,173,.28)" : "rgba(255,209,102,.18)";
          fieldCtx.strokeStyle = selected ? "#5cffad" : "#ffd166";
          fieldCtx.lineWidth = selected ? 2.5 : 1.5;
          fieldCtx.beginPath();
          fieldCtx.arc(point.x, point.y, sigmaPx, 0, Math.PI * 2);
          fieldCtx.fill();
          fieldCtx.stroke();
          fieldCtx.fillStyle = selected ? "#5cffad" : "#ffd166";
          fieldCtx.font = "700 10px Arial";
          fieldCtx.textAlign = "left";
          fieldCtx.textBaseline = "bottom";
          fieldCtx.fillText(`${candidate.goal.id}:${candidate.face.name}`, point.x + 5, point.y - 4);
        }
      }

      function drawStartPose(toScreen) {
        const start = toScreen({ x: startPose.x, y: startPose.y });
        const headingTip = toScreen({
          x: startPose.x + Math.cos(startPose.headingRad) * 10,
          y: startPose.y + Math.sin(startPose.headingRad) * 10,
        });
        fieldCtx.fillStyle = "rgba(255,209,102,.18)";
        fieldCtx.strokeStyle = "#ffd166";
        fieldCtx.lineWidth = 2;
        fieldCtx.beginPath();
        fieldCtx.arc(start.x, start.y, 13, 0, Math.PI * 2);
        fieldCtx.fill();
        fieldCtx.stroke();
        fieldCtx.beginPath();
        fieldCtx.moveTo(start.x, start.y);
        fieldCtx.lineTo(headingTip.x, headingTip.y);
        fieldCtx.stroke();
        fieldCtx.fillStyle = "#ffd166";
        fieldCtx.font = "700 11px Arial";
        fieldCtx.textAlign = "center";
        fieldCtx.textBaseline = "top";
        fieldCtx.fillText("START 30.2,34.7", start.x, start.y + 16);
      }

      function drawField(fit, thetaGate, wallDistanceIn, displayPose) {
        const dpr = window.devicePixelRatio || 1;
        const rect = fieldCanvas.getBoundingClientRect();
        const width = Math.max(1, Math.floor(rect.width * dpr));
        const height = Math.max(1, Math.floor(rect.height * dpr));
        if (fieldCanvas.width !== width || fieldCanvas.height !== height) {
          fieldCanvas.width = width;
          fieldCanvas.height = height;
        }
        fieldCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
        fieldCtx.clearRect(0, 0, rect.width, rect.height);

        const pad = 28;
        const size = Math.min(rect.width, rect.height) - pad * 2;
        const left = (rect.width - size) / 2;
        const top = (rect.height - size) / 2;
        const scale = size / fieldSizeIn;
        const toScreen = point => ({
          x: left + (fieldHalfSpanIn + Math.max(-fieldHalfSpanIn, Math.min(fieldHalfSpanIn, point.x))) * scale,
          y: top + (fieldHalfSpanIn - Math.max(-fieldHalfSpanIn, Math.min(fieldHalfSpanIn, point.y))) * scale,
        });

        fieldCtx.fillStyle = "#111817";
        fieldCtx.fillRect(left, top, size, size);
        fieldCtx.strokeStyle = "#3b4a45";
        fieldCtx.lineWidth = 2;
        fieldCtx.strokeRect(left, top, size, size);

        fieldCtx.strokeStyle = "rgba(147,163,157,.22)";
        fieldCtx.lineWidth = 1;
        for (let inch = -48; inch <= 48; inch += 24) {
          const verticalTop = toScreen({ x: inch, y: fieldHalfSpanIn });
          const verticalBottom = toScreen({ x: inch, y: -fieldHalfSpanIn });
          const horizontalLeft = toScreen({ x: -fieldHalfSpanIn, y: inch });
          const horizontalRight = toScreen({ x: fieldHalfSpanIn, y: inch });
          fieldCtx.beginPath();
          fieldCtx.moveTo(verticalTop.x, verticalTop.y);
          fieldCtx.lineTo(verticalBottom.x, verticalBottom.y);
          fieldCtx.moveTo(horizontalLeft.x, horizontalLeft.y);
          fieldCtx.lineTo(horizontalRight.x, horizontalRight.y);
          fieldCtx.stroke();
        }

        fieldCtx.fillStyle = "rgba(244,251,248,.72)";
        fieldCtx.font = "12px Arial";
        fieldCtx.textAlign = "center";
        fieldCtx.fillText("+Y red", left + size / 2, top + 16);
        fieldCtx.fillText("-Y blue", left + size / 2, top + size - 8);
        fieldCtx.textAlign = "left";
        fieldCtx.fillText("-X", left + 8, top + size / 2 - 8);
        fieldCtx.textAlign = "right";
        fieldCtx.fillText("+X / 0 deg", left + size - 8, top + size / 2 - 8);

        drawGoalTagLandmarks(toScreen);
        drawAiVisionHypotheses(toScreen, displayPose);
        drawStartPose(toScreen);
        drawLidarFieldEstimate(toScreen, fit, thetaGate, wallDistanceIn);
        drawDrivetrainVerticalEstimate(toScreen);

        if (displayPose.source !== "onboard" && pose.path.length > 1) {
          fieldCtx.strokeStyle = "#57c7ff";
          fieldCtx.lineWidth = 3;
          fieldCtx.beginPath();
          pose.path.forEach((point, index) => {
            const screen = toScreen(point);
            if (index === 0) fieldCtx.moveTo(screen.x, screen.y);
            else fieldCtx.lineTo(screen.x, screen.y);
          });
          fieldCtx.stroke();
        }

        if (displayPose.source === "onboard" && displayPose.path.length > 1) {
          fieldCtx.strokeStyle = "#5cffad";
          fieldCtx.lineWidth = 3;
          fieldCtx.beginPath();
          displayPose.path.forEach((point, index) => {
            const screen = toScreen(point);
            if (index === 0) fieldCtx.moveTo(screen.x, screen.y);
            else fieldCtx.lineTo(screen.x, screen.y);
          });
          fieldCtx.stroke();
        }

        const robot = toScreen(displayPose);
        fieldCtx.fillStyle = displayPose.ready ? "#5cffad" : "#ffd166";
        fieldCtx.strokeStyle = "#07100d";
        fieldCtx.lineWidth = 3;
        fieldCtx.beginPath();
        fieldCtx.arc(robot.x, robot.y, 9, 0, Math.PI * 2);
        fieldCtx.stroke();
        fieldCtx.fill();

        fieldCtx.strokeStyle = "#5cffad";
        fieldCtx.lineWidth = 3;
        const headingTip = toScreen({
          x: displayPose.x + Math.cos(displayPose.headingRad) * 12,
          y: displayPose.y + Math.sin(displayPose.headingRad) * 12,
        });
        fieldCtx.beginPath();
        fieldCtx.moveTo(robot.x, robot.y);
        fieldCtx.lineTo(headingTip.x, headingTip.y);
        fieldCtx.stroke();

        const displayHeadingDeg = ((displayPose.headingRad * 180 / Math.PI + 360) % 360);
        const imuFrame = latest?.latest?.imu || {};
        const gpsFrame = latest?.latest?.gps || {};
        const motorFrame = latest?.latest?.motors || {};
        const driveHealthLine = driveSensorHealth(motorFrame);
        const cameraFrame = latest?.camera_pose || {};
        const visionFrame = latest?.vision || {};
        const visionAgeSec = Number.isFinite(visionFrame.pc_time)
          ? Math.max(0, Date.now() / 1000 - visionFrame.pc_time)
          : null;
        const visionCandidate = latest?.onboard_pose || {};
        const visionPortLabel = Number.isFinite(visionFrame.port) && visionFrame.port > 0
          ? `P${visionFrame.port}`
          : "no port";
        const visionLine = visionFrame.installed
          ? `AI Vision ${visionPortLabel} ${visionFrame.configured ? "configured" : "CONFIG ERROR"}` +
            (visionFrame.valid
              ? ` | tag ${visionFrame.tag_id} | bearing ${visionFrame.bearing_deg.toFixed(1)} deg` +
                ` | horizontal ${Number.isFinite(visionFrame.horizontal_range_in) ? visionFrame.horizontal_range_in.toFixed(1) + " in" : "--"}` +
                ` | 3D ${Number.isFinite(visionFrame.range_estimate_in) ? visionFrame.range_estimate_in.toFixed(1) + " in" : "--"}` +
                ` | edge ${Number.isFinite(visionFrame.mean_edge_px) ? visionFrame.mean_edge_px.toFixed(1) + " px" : "--"}` +
                ` | shape ${Number.isFinite(visionFrame.edge_ratio) ? visionFrame.edge_ratio.toFixed(2) : "--"}/${Number.isFinite(visionFrame.fill_ratio) ? visionFrame.fill_ratio.toFixed(2) : "--"}` +
                ` | age ${visionAgeSec === null ? "--" : visionAgeSec.toFixed(1)}s`
              : ` | ${visionFrame.reason || "no observation"} | age ${visionAgeSec === null ? "--" : visionAgeSec.toFixed(1)}s`) +
            ` | mount ${aiCameraExtrinsicsQualified ? "qualified" : "UNMEASURED"}`
          : `AI Vision ${visionPortLabel} | ${visionFrame.message || "waiting"}`;
        const visionCandidateLine = Number.isFinite(visionCandidate.ai_id) && visionCandidate.ai_id >= 0
          ? `AI candidate ${visionCandidate.ai_goal || "--"}/${visionCandidate.ai_face || "--"}` +
            ` | residual ${Number.isFinite(visionCandidate.ai_residual) ? visionCandidate.ai_residual.toFixed(1) : "--"} deg` +
            ` | range ${Number.isFinite(visionCandidate.ai_range) ? visionCandidate.ai_range.toFixed(1) : "--"}/${Number.isFinite(visionCandidate.ai_pred_range) ? visionCandidate.ai_pred_range.toFixed(1) : "--"} in` +
            ` | dr ${Number.isFinite(visionCandidate.ai_range_residual) ? visionCandidate.ai_range_residual.toFixed(1) : "--"} in` +
            ` | innovation ${Number.isFinite(visionCandidate.ai_innovation) ? visionCandidate.ai_innovation.toFixed(1) : "--"} in` +
            ` | step ${Number.isFinite(visionCandidate.ai_pos_step) ? visionCandidate.ai_pos_step.toFixed(2) : "--"} in/${Number.isFinite(visionCandidate.ai_heading_step) ? visionCandidate.ai_heading_step.toFixed(2) : "--"} deg` +
            ` | margin ${Number.isFinite(visionCandidate.ai_margin) ? visionCandidate.ai_margin.toFixed(1) : "--"} deg` +
            ` | ${visionCandidate.ai_reject || "shadow-only"}`
          : `AI candidate | ${visionCandidate.ai_reject || "waiting"}`;
        const cameraLine = cameraFrame.available && Number.isFinite(cameraFrame.field_y_in)
          ? `Webcam DEBUG ONLY Y ${cameraFrame.field_y_in.toFixed(1)} in | ${(cameraFrame.score * 100).toFixed(0)}%`
          : `Webcam DEBUG ONLY | ${cameraFrame.message || "disabled"}`;
        const imuLine = Number.isFinite(imuFrame.ez_rotation_deg)
          ? `IMU EZ ${imuFrame.ez_rotation_deg.toFixed(1)} deg | raw ${Number.isFinite(imuFrame.raw_rotation_deg) ? imuFrame.raw_rotation_deg.toFixed(1) : "--"} deg | st ${Number.isFinite(imuFrame.status) ? imuFrame.status : "--"}`
          : "IMU waiting";
        const gpsLine = gpsFrame.installed
          ? `GPS P7 raw ${Number.isFinite(gpsFrame.x_m) ? gpsFrame.x_m.toFixed(3) : "--"},${Number.isFinite(gpsFrame.y_m) ? gpsFrame.y_m.toFixed(3) : "--"} m` +
            ` | RMS ${Number.isFinite(gpsFrame.error_m) ? (gpsFrame.error_m * 39.3701).toFixed(2) : "--"} in` +
            ` | gyro-z ${Number.isFinite(gpsFrame.gyro_z) ? gpsFrame.gyro_z.toFixed(2) : "--"}` +
            ` | fusion ${visionCandidate.gps_reject || "waiting"}`
          : "GPS P7 missing/waiting";
        const onboardConfidence = latest?.onboard_pose || {};
        const confidenceLine = Number.isFinite(onboardConfidence.pos_envelope)
          ? `DR ${Number.isFinite(onboardConfidence.dr_travel) ? onboardConfidence.dr_travel.toFixed(1) : "--"} in since fix` +
            ` | empirical envelope +/-${onboardConfidence.pos_envelope.toFixed(1)} in` +
            ` | fix age ${Number.isFinite(onboardConfidence.abs_age) ? (onboardConfidence.abs_age / 1000).toFixed(1) : "--"}s`
          : "DR confidence waiting";
        poseXEl.textContent = `${displayPose.x.toFixed(1)} in`;
        poseYEl.textContent = `${displayPose.y.toFixed(1)} in`;
        poseHeadingEl.textContent = `${displayHeadingDeg.toFixed(1)} deg`;
        poseSourceEl.textContent = `${displayPose.label}: ${displayPose.detail}`;
        fieldHudEl.innerHTML =
          `<strong>${displayPose.label}</strong>` +
          `X ${displayPose.x.toFixed(1)} in | Y ${displayPose.y.toFixed(1)} in | H ${displayHeadingDeg.toFixed(1)} deg<br>` +
          `L ${pose.leftTravelIn.toFixed(1)} in | R ${pose.rightTravelIn.toFixed(1)} in | L-R ${(pose.leftTravelIn - pose.rightTravelIn).toFixed(1)} in<br>` +
           `Drive sensors ${driveHealthLine}<br>` +
           `${visionLine}<br>` +
           `${visionCandidateLine}<br>` +
          `${cameraLine}<br>` +
          `${imuLine}<br>` +
          `${gpsLine}<br>` +
          `${confidenceLine}<br>` +
          `<span>${displayPose.detail}</span>`;
        poseTravelEl.textContent = `${pose.travelIn.toFixed(1)} in`;
        poseDrTravelEl.textContent = Number.isFinite(onboardConfidence.dr_travel)
          ? `${onboardConfidence.dr_travel.toFixed(1)} in`
          : "--.- in";
        poseErrorEnvelopeEl.textContent = Number.isFinite(onboardConfidence.pos_envelope)
          ? `+/-${onboardConfidence.pos_envelope.toFixed(1)} in est.`
          : "--.- in";
        poseAbsoluteAgeEl.textContent = Number.isFinite(onboardConfidence.abs_age)
          ? `${(onboardConfidence.abs_age / 1000).toFixed(1)} s`
          : "--.- s";
        poseLeftWheelEl.textContent = `${pose.leftTravelIn.toFixed(1)} in`;
        poseRightWheelEl.textContent = `${pose.rightTravelIn.toFixed(1)} in`;
        poseLastDeltaEl.textContent = `L ${pose.lastLeftDeltaIn.toFixed(2)} / R ${pose.lastRightDeltaIn.toFixed(2)}`;
        poseTurnDeltaEl.textContent = `${pose.lastTurnDeltaDeg.toFixed(2)} deg`;
        poseSideOdomEl.textContent = `${pose.sideTravelIn.toFixed(1)} in`;
        poseLidarAssistEl.textContent =
          pose.lastLidarWall !== "off" ? pose.lastLidarWall : "off";
        poseAiVisionEl.textContent = visionFrame.valid
          ? `tag ${visionFrame.tag_id} @ ${visionFrame.bearing_deg.toFixed(1)} deg`
          : (visionFrame.reason || "waiting");
        const onboardCalibration = latest?.onboard_pose || {};
        const displayedTrackWidth = Number.isFinite(onboardCalibration.track)
          ? onboardCalibration.track
          : trackWidthIn;
        const displayedRearLever = Number.isFinite(onboardCalibration.rear)
          ? onboardCalibration.rear
          : horizontalOdometerOffsetBackIn;
        const displayedLidarScale = Number.isFinite(onboardCalibration.lidar_scale)
          ? onboardCalibration.lidar_scale
          : lidarThetaScale;
        const rearTravel15In = Math.abs(displayedRearLever) * Math.PI / 12;
        const rearSensor15Deg = rearTravel15In / horizontalOdometerCircumferenceIn * 360;
        const counterTurnDeg = -2 / displayedTrackWidth * 180 / Math.PI;
        calTrackWidthEl.textContent = `${displayedTrackWidth.toFixed(3)} in`;
        calRearLeverEl.textContent = `${displayedRearLever.toFixed(4)} in`;
        calRear15El.textContent = `${rearTravel15In.toFixed(3)} in / ${rearSensor15Deg.toFixed(1)} deg`;
        calCounterturnEl.textContent = `${counterTurnDeg.toFixed(2)} deg`;
        calLidarScaleEl.textContent = `${displayedLidarScale.toFixed(6)} x`;
      }

      function lineCenterDistanceIn(fit) {
        if (!fit) return null;
        const centerXIn = ((ports.length - 1) * sensorSpacingIn) / 2;
        return fit.slope * centerXIn + fit.intercept;
      }

      function evaluateThetaGate(points, fit, centerDistanceIn) {
        if (ports.length < 2) return { ok: false, reason: "legacy wall fit disabled; current robot has forward P1 only" };
        if (!fit) return { ok: false, reason: "need at least two valid sensors" };
        if (points.length !== ports.length || points.some(point => point.missing)) {
          return { ok: false, reason: "need all four LiDARs installed and returning distance" };
        }
        const lowConfidence = points.filter(point => point.reading.confidence < minConfidence);
        if (lowConfidence.length) {
          return { ok: false, reason: `low confidence on L${ports.indexOf(lowConfidence[0].port) + 1}` };
        }
        if (fit.rmse > maxRmseIn) {
          return { ok: false, reason: `RMSE ${fit.rmse.toFixed(2)} in is above ${maxRmseIn.toFixed(2)} in` };
        }
        if (fit.maxAbsError > maxPointErrorIn) {
          return { ok: false, reason: `one point is ${fit.maxAbsError.toFixed(2)} in off the line` };
        }
        if (!Number.isFinite(centerDistanceIn) || centerDistanceIn <= 0 || centerDistanceIn > lidarMaxAssistDistanceIn) {
          return { ok: false, reason: `wall distance must be under ${lidarMaxAssistDistanceIn} in` };
        }
        const farPoint = points.find(point => point.distanceIn > lidarMaxAssistDistanceIn);
        if (farPoint) {
          return { ok: false, reason: `L${ports.indexOf(farPoint.port) + 1} is over ${lidarMaxAssistDistanceIn} in` };
        }
        return { ok: true, reason: "clean straight-line fit" };
      }

      for (const port of ports) {
        const sensor = document.createElement("article");
        sensor.className = "label";
        sensor.innerHTML = `
          <div class="port">Rear Distance / Port ${port}</div>
          <div class="mm" id="mm-${port}">---- mm</div>
          <div class="inch" id="in-${port}">--.-- in</div>
          <div class="conf" id="conf-${port}">conf -- / 63</div>`;
        readouts.appendChild(sensor);
      }

      for (const port of driveMotors) {
        const motor = document.createElement("article");
        motor.className = "label";
        motor.innerHTML = `
          <div class="port">Drive Motor Port ${port}</div>
          <div class="mm" id="motor-${port}">---- deg</div>
          <div class="inch">${port === 17 || port === 18 ? "left chassis" : "right chassis"}</div>
          <div class="conf">integrated encoder</div>`;
        motorReadouts.appendChild(motor);
      }

      function resizeCanvas() {
        const ratio = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = Math.max(1, Math.round(rect.width * ratio));
        canvas.height = Math.max(1, Math.round(rect.height * ratio));
        ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      }

      function render() {
        const sensors = latest?.sensors || {};
        const validValues = ports
          .map(port => sensors[String(port)]?.mm)
          .filter(mm => Number.isFinite(mm) && mm >= 0);
        const desiredScale = Math.max(2000, Math.ceil((Math.max(...validValues, 0) + 250) / 500) * 500);
        maxScale = maxScale * 0.9 + desiredScale * 0.1;
        scaleEl.textContent = `Scale: 0-${Math.round(maxScale)} mm`;

        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        const padLeft = 58;
        const padRight = 24;
        const padTop = 28;
        const padBottom = 46;
        const plotW = Math.max(1, w - padLeft - padRight);
        const plotH = Math.max(1, h - padTop - padBottom);
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = "#0b0f0e";
        ctx.fillRect(0, 0, w, h);
        ctx.strokeStyle = "rgba(255,255,255,.08)";
        ctx.lineWidth = 1;
        ctx.fillStyle = "rgba(244,251,248,.62)";
        ctx.font = "12px Arial";
        ctx.textAlign = "right";
        ctx.textBaseline = "middle";
        for (let i = 0; i <= 5; i++) {
          const y = padTop + plotH * i / 5;
          const value = Math.round(maxScale - maxScale * i / 5);
          ctx.beginPath();
          ctx.moveTo(padLeft, y);
          ctx.lineTo(padLeft + plotW, y);
          ctx.stroke();
          ctx.fillText(`${value}`, padLeft - 8, y);
        }

        const points = ports.map((port, index) => {
          const reading = sensors[String(port)];
          const missing = !reading || !reading.installed || reading.mm < 0;
          const arrayXIn = index * sensorSpacingIn;
          const sensorSpanIn = (ports.length - 1) * sensorSpacingIn;
          const x = sensorSpanIn > 0
            ? padLeft + (arrayXIn / sensorSpanIn) * plotW
            : padLeft + plotW / 2;
          const y = padTop + (1 - Math.max(0, Math.min(reading?.mm || 0, maxScale)) / Math.max(1, maxScale)) * plotH;
          return {
            port,
            reading,
            missing,
            arrayXIn,
            distanceIn: missing ? 0 : reading.mm / 25.4,
            x,
            y,
          };
        });
        const validPoints = points.filter(point => !point.missing);
        const fit = fitLine(validPoints);
        const centerDistanceIn = lineCenterDistanceIn(fit);
        const wallDistanceIn = wallPerpendicularDistanceIn(fit, centerDistanceIn);
        const thetaGate = evaluateThetaGate(points, fit, centerDistanceIn);
        const currentLidarGood = thetaGate.ok && Number.isFinite(wallDistanceIn);
        if (currentLidarGood) {
          lastGoodLidarEstimate = {
            fit: { ...fit },
            centerDistanceIn,
            wallDistanceIn,
            sample: latest?.latest?.sample ?? null,
          };
        }
        const displayedLidarEstimate = lastGoodLidarEstimate;

        ctx.strokeStyle = "#5cffad";
        ctx.lineWidth = 3;
        ctx.beginPath();
        let started = false;
        for (const point of points) {
          if (point.missing) {
            started = false;
            continue;
          }
          if (!started) {
            ctx.moveTo(point.x, point.y);
            started = true;
          } else {
            ctx.lineTo(point.x, point.y);
          }
        }
        ctx.stroke();

        if (fit) {
          const xMin = 0;
          const xMax = (ports.length - 1) * sensorSpacingIn;
          const yMinIn = fit.slope * xMin + fit.intercept;
          const yMaxIn = fit.slope * xMax + fit.intercept;
          const fitX1 = padLeft;
          const fitX2 = padLeft + plotW;
          const fitY1 = padTop + (1 - Math.max(0, Math.min(yMinIn * 25.4, maxScale)) / Math.max(1, maxScale)) * plotH;
          const fitY2 = padTop + (1 - Math.max(0, Math.min(yMaxIn * 25.4, maxScale)) / Math.max(1, maxScale)) * plotH;
          ctx.strokeStyle = fit.rmse <= maxRmseIn && fit.maxAbsError <= maxPointErrorIn ? "#ffd166" : "rgba(255,209,102,.45)";
          ctx.lineWidth = 2;
          ctx.setLineDash([8, 6]);
          ctx.beginPath();
          ctx.moveTo(fitX1, fitY1);
          ctx.lineTo(fitX2, fitY2);
          ctx.stroke();
          ctx.setLineDash([]);
          fitEl.textContent = `RMSE ${fit.rmse.toFixed(2)} in | max err ${fit.maxAbsError.toFixed(2)} in | R2 ${fit.r2.toFixed(3)}`;
          if (gateDetailEl) gateDetailEl.textContent = `Gate: confidence >= ${minConfidence}, RMSE <= ${maxRmseIn.toFixed(2)} in, point error <= ${maxPointErrorIn.toFixed(2)} in. Current: ${thetaGate.reason}.`;
          if (displayedLidarEstimate) {
            centerDistanceEl.textContent = `${displayedLidarEstimate.wallDistanceIn.toFixed(2)} in`;
            rmseValueEl.textContent = `${displayedLidarEstimate.fit.rmse.toFixed(2)} in`;
          } else {
            centerDistanceEl.textContent = "--.-- in";
            rmseValueEl.textContent = `${fit.rmse.toFixed(2)} in`;
          }
          if (currentLidarGood) {
            thetaValueEl.textContent = `${fit.thetaDeg.toFixed(1)} deg`;
            thetaDetailEl.textContent = `Wall distance uses center distance ${centerDistanceIn.toFixed(2)} in x cos(theta). RMSE ${fit.rmse.toFixed(2)} in, max point error ${fit.maxAbsError.toFixed(2)} in.`;
          } else if (displayedLidarEstimate) {
            thetaValueEl.textContent = `${displayedLidarEstimate.fit.thetaDeg.toFixed(1)} deg`;
            thetaDetailEl.textContent = `Holding last good LiDAR wall because current reading is not confident: ${thetaGate.reason}.`;
          } else {
            thetaValueEl.textContent = "--.- deg";
            thetaDetailEl.textContent = `Hidden: ${thetaGate.reason}.`;
          }
        } else {
          fitEl.textContent = "RMSE -- | max err --";
          if (gateDetailEl) gateDetailEl.textContent = `Gate: confidence >= ${minConfidence}, RMSE <= ${maxRmseIn.toFixed(2)} in, point error <= ${maxPointErrorIn.toFixed(2)} in.`;
          if (displayedLidarEstimate) {
            centerDistanceEl.textContent = `${displayedLidarEstimate.wallDistanceIn.toFixed(2)} in`;
            rmseValueEl.textContent = `${displayedLidarEstimate.fit.rmse.toFixed(2)} in`;
            thetaValueEl.textContent = `${displayedLidarEstimate.fit.thetaDeg.toFixed(1)} deg`;
            thetaDetailEl.textContent = "Holding last good LiDAR wall because there are not enough valid points for a new line fit.";
          } else {
            centerDistanceEl.textContent = "--.-- in";
            rmseValueEl.textContent = "--.-- in";
            thetaValueEl.textContent = "--.- deg";
            thetaDetailEl.textContent = ports.length < 2
              ? "Legacy four-sensor wall fit is disabled; current P1 is forward obstacle range only."
              : "Hidden: not enough valid points for a line fit.";
          }
        }

        ctx.textAlign = "center";
        ctx.textBaseline = "top";
        for (const point of points) {
          const color = point.missing ? "#404746" : point.reading.confidence < 20 ? "#ffd166" : "#57c7ff";
          ctx.fillStyle = color;
          ctx.strokeStyle = "#07100d";
          ctx.lineWidth = 3;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 7, 0, Math.PI * 2);
          ctx.stroke();
          ctx.fill();
          ctx.fillStyle = "rgba(244,251,248,.78)";
          ctx.fillText(`P${point.port}`, point.x, padTop + plotH + 14);
          ctx.fillText("forward", point.x, padTop + plotH + 28);
        }

        for (const port of ports) {
          const reading = sensors[String(port)];
          const mmEl = document.getElementById(`mm-${port}`);
          const inEl = document.getElementById(`in-${port}`);
          const confEl = document.getElementById(`conf-${port}`);
          const missing = !reading || !reading.installed || reading.mm < 0;
          mmEl.textContent = missing ? "---- mm" : `${reading.mm} mm`;
          inEl.textContent = missing ? "--.-- in" : `${(reading.mm / 25.4).toFixed(2)} in`;
          confEl.textContent = reading ? `conf ${reading.confidence} / 63` : "conf -- / 63";
        }
        const frame = latest?.latest;
        updatePose(frame, thetaGate, currentLidarGood ? fit : null, currentLidarGood ? wallDistanceIn : null, points);
        const displayPose = displayPoseFromLatest();
        drawField(
          displayedLidarEstimate?.fit || null,
          displayedLidarEstimate ? { ok: true } : thetaGate,
          displayedLidarEstimate?.wallDistanceIn ?? null,
          displayPose
        );
        const motors = frame?.motors || {};
        for (const port of driveMotors) {
          const pos = motors[String(port)]?.position_deg;
          document.getElementById(`motor-${port}`).textContent =
            Number.isFinite(pos) ? `${pos.toFixed(1)} deg` : "---- deg";
        }
        requestAnimationFrame(render);
      }

      async function poll() {
        try {
          const res = await fetch("/data", { cache: "no-store" });
          latest = await res.json();
          statusEl.textContent = latest.message;
          rateEl.textContent = `${latest.rate_hz.toFixed(1)} Hz`;
          const resetToken = Number.isFinite(latest.reset_token) ? latest.reset_token : 0;
          if (seenResetToken === null) {
            seenResetToken = resetToken;
          } else if (resetToken !== seenResetToken) {
            seenResetToken = resetToken;
            resetPoseBaseline(latest.latest);
          }
        } catch (err) {
          statusEl.textContent = `UI fetch error: ${err.name}`;
        } finally {
          setTimeout(poll, 20);
        }
      }

      window.addEventListener("resize", resizeCanvas);
      poseResetEl.addEventListener("click", requestRuntimeStartPose);
      fieldResetEl.addEventListener("click", requestRuntimeStartPose);
      resizeCanvas();
      poll();
      render();
    </script>
  </body>
</html>"""


def serial_candidates():
    ports = [
        port
        for port in list_ports.comports()
        if "bluetooth" not in " ".join(
            str(part) for part in (port.description, port.manufacturer, port.hwid)
        ).lower()
    ]

    def score(port):
        text = " ".join(str(part) for part in (port.device, port.description, port.manufacturer, port.hwid)).lower()
        value = 0
        if "vex" in text or "v5" in text:
            value += 20
        if "user" in text:
            value += 10
        if "communication" in text or "communications" in text:
            value += 5
        return value

    ranked = sorted(ports, key=score, reverse=True)
    return [port.device for port in ranked if score(port) > 0] + [
        port.device for port in ranked if score(port) == 0
    ]


def parse_d4(line):
    match = D4_RE.search(line)
    if not match:
        return None
    sensors = {}
    for port in PORTS:
        sensors[str(port)] = {
            "mm": int(match.group(f"p{port}_mm")),
            "in": int(match.group(f"p{port}_mm")) / 25.4,
            "confidence": int(match.group(f"p{port}_conf")),
            "installed": bool(int(match.group(f"p{port}_inst"))),
        }
    motors = {}
    for port in DRIVE_MOTORS:
        value = match.group(f"m{port}")
        if value is not None:
            position = float(value)
            if math.isfinite(position):
                motors[str(port)] = {"position_deg": position}
    odometer = {}
    h5 = match.group("h5")
    if h5 is not None:
        odometer[str(HORIZONTAL_ODOMETER_PORT)] = {"position_centideg": int(h5)}
    imu = {}
    ez_imu = parse_number(match.group("imu"))
    raw_imu = parse_number(match.group("rawimu"))
    imu_status = match.group("imust")
    if ez_imu is not None:
        imu["ez_rotation_deg"] = ez_imu
    if raw_imu is not None:
        imu["raw_rotation_deg"] = raw_imu
    if imu_status is not None:
        imu["status"] = int(imu_status)
    for axis in ("x", "y", "z"):
        gyro = parse_number(match.group(f"imu_gyro_{axis}"))
        accel = parse_number(match.group(f"imu_acc_{axis}"))
        if gyro is not None:
            imu[f"gyro_{axis}_dps"] = gyro
        if accel is not None:
            imu[f"accel_{axis}_g"] = accel
    gps = {}
    gps_x = parse_number(match.group("gps_x"))
    gps_y = parse_number(match.group("gps_y"))
    gps_heading = parse_number(match.group("gps_heading"))
    gps_error = parse_number(match.group("gps_error"))
    gps_installed = match.group("gps_inst")
    gps_gyro_z = parse_number(match.group("gps_gyro_z"))
    if gps_x is not None:
        gps["x_m"] = gps_x
    if gps_y is not None:
        gps["y_m"] = gps_y
    if gps_heading is not None:
        gps["heading_deg"] = gps_heading
    if gps_error is not None:
        gps["error_m"] = gps_error
    if gps_installed is not None:
        gps["installed"] = bool(int(gps_installed))
    if gps_gyro_z is not None:
        gps["gyro_z"] = gps_gyro_z
    return {
        "sample": int(match.group("sample")),
        "brain_ms": int(match.group("brain_ms")),
        "pc_time": time.time(),
        "sensors": sensors,
        "motors": motors,
        "odometer": odometer,
        "imu": imu,
        "gps": gps,
    }


def parse_number(value):
    if value is None:
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def parse_fuse_test(line):
    if FUSE_TEST_PREFIX not in line:
        return None

    line = line[line.index(FUSE_TEST_PREFIX):]
    for marker in ("soutD4 ", "D4 s=", "soutFUSE_TEST "):
        marker_index = line.find(marker, len(FUSE_TEST_PREFIX))
        if marker_index >= 0:
            line = line[:marker_index]

    fields = {}
    for token in line.strip().split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value.rstrip(",")

    x = parse_number(fields.get("x"))
    y = parse_number(fields.get("y"))
    heading = parse_number(fields.get("heading") or fields.get("h"))
    if x is None or y is None or heading is None:
        return None

    pose = {
        "phase": fields.get("phase", "unknown"),
        "x": x,
        "y": y,
        "heading_deg": heading % 360.0,
        "pc_time": time.time(),
        "lidar": fields.get("lidar"),
        "reject": fields.get("reject"),
        "line": line.strip(),
    }

    for key in (
        "imu",
        "bias",
        "theta",
        "distance",
        "rmse",
        "axis_step",
        "traveled",
        "target",
        "remaining",
        "error",
        "power",
        "ai_bearing",
        "ai_residual",
        "ai_margin",
        "ai_age",
        "ai_range",
        "ai_pred_range",
        "ai_range_residual",
        "ai_innovation",
        "ai_pos_step",
        "ai_heading_step",
        "track",
        "rear",
        "lidar_scale",
        "gps_x",
        "gps_y",
        "gps_heading",
        "gps_error",
        "gps_innovation",
        "gps_pos_step",
        "gps_heading_step",
        "dr_travel",
        "pos_envelope",
        "abs_age",
        "pose_valid",
        "estimator_valid",
    ):
        value = parse_number(fields.get(key))
        if value is not None:
            pose[key] = value
    ai_id = parse_number(fields.get("ai_id"))
    if ai_id is not None:
        pose["ai_id"] = int(ai_id)
    for source_key, pose_key in (
        ("gps_reject", "gps_reject"),
        ("ai_goal", "ai_goal"),
        ("ai_face", "ai_face"),
        ("ai_reject", "ai_reject"),
    ):
        if fields.get(source_key) is not None:
            pose[pose_key] = fields[source_key]
    return pose


def parse_vision_shadow(line):
    if VISION_SHADOW_PREFIX not in line:
        return None
    line = line[line.index(VISION_SHADOW_PREFIX):]
    for marker in ("soutD4 ", "D4 s=", "soutFUSE_TEST ", "FUSE_TEST "):
        marker_index = line.find(marker, len(VISION_SHADOW_PREFIX))
        if marker_index >= 0:
            line = line[:marker_index]

    fields = {}
    for token in line.strip().split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value.rstrip(",")
    try:
        corners = [int(value) for value in fields.get("corners", "").split(",")]
        center = [float(value) for value in fields.get("center", "").split(",")]
        if len(corners) != 8 or len(center) != 2:
            return None
        return {
            "brain_ms": int(fields["t"]),
            "poll": int(fields["poll"]),
            "port": int(fields.get("port", "0")),
            "installed": bool(int(fields["installed"])),
            "configured": bool(int(fields["configured"])),
            "object_count": int(fields["count"]),
            "tag_id": int(fields["tag"]),
            "corners": corners,
            "center_px": center,
            "area_px2": float(fields["area"]),
            "mean_edge_px": float(fields.get("mean_edge", "0")),
            "forward_depth_in": float(fields.get("depth", "0")),
            "right_offset_in": float(fields.get("right", "0")),
            "up_offset_in": float(fields.get("up", "0")),
            "horizontal_range_in": float(fields.get(
                "horizontal",
                str(math.hypot(
                    float(fields.get("depth", "0")),
                    float(fields.get("right", "0")),
                )),
            )),
            "range_estimate_in": float(fields.get("range", "0")),
            "edge_ratio": float(fields["edge_ratio"]),
            "fill_ratio": float(fields["fill"]),
            "bearing_deg": float(fields["bearing"]),
            "elevation_deg": float(fields.get("elevation", "0")),
            "image_roll_deg": float(fields.get("roll", "0")),
            "repeated_geometry": bool(int(fields["repeat"])),
            "geometry_age_ms": int(fields["geometry_age"]),
            "valid": bool(int(fields["valid"])),
            "reason": fields.get("reason", "unknown"),
            "pc_time": time.time(),
            "line": line.strip(),
        }
    except (KeyError, TypeError, ValueError):
        return None


def update_latest(frame, serial_port):
    now = time.time()
    with state_lock:
        previous_sample = state["last_sample"]
        previous_brain_ms = state["last_brain_ms"]
        rate_hz = state["rate_hz"]
        if (
            previous_sample is not None
            and previous_brain_ms is not None
            and frame["sample"] != previous_sample
            and frame["brain_ms"] > previous_brain_ms
        ):
            sample_delta = frame["sample"] - previous_sample
            time_delta_s = (frame["brain_ms"] - previous_brain_ms) / 1000.0
            instant = sample_delta / max(0.001, time_delta_s)
            rate_hz = (rate_hz * 0.75) + (instant * 0.25) if rate_hz else instant
        state.update(
            {
                "connected": True,
                "serial_port": serial_port,
                "latest": frame,
                "message": f"Live on {serial_port}: P1 Distance, P12 IMU, P7 GPS, P6 AI Vision",
                "rate_hz": rate_hz,
                "last_sample": frame["sample"],
                "last_sample_time": now,
                "last_brain_ms": frame["brain_ms"],
            }
        )


def update_onboard_pose(pose, serial_port):
    with state_lock:
        state["connected"] = True
        state["serial_port"] = serial_port
        state["onboard_pose"] = pose
        state["last_fuse_line"] = pose["line"]
        if state["latest"] is None:
            state["message"] = f"Live on {serial_port}: onboard fused pose, waiting for D4 frames..."


def update_vision(vision, serial_port):
    vision["message"] = (
        f"Port {vision.get('port', 0)} tag {vision['tag_id']} valid"
        if vision["valid"]
        else f"Port {vision.get('port', 0)} {vision['reason']}"
    )
    with state_lock:
        state["connected"] = True
        state["serial_port"] = serial_port
        state["vision"] = vision


def load_camera_calibration():
    try:
        calibration = json.loads(CAMERA_CALIBRATION_PATH.read_text(encoding="utf-8"))
        baseline_x = float(calibration["baseline_x_px"])
        baseline_y = float(calibration["baseline_y_in"])
        pixels_per_inch = float(calibration["pixels_per_inch"])
        if not all(math.isfinite(value) for value in (baseline_x, baseline_y, pixels_per_inch)):
            raise ValueError("camera calibration contains a non-finite number")
        if pixels_per_inch <= 0:
            raise ValueError("pixels_per_inch must be positive")
        return baseline_x, baseline_y, pixels_per_inch
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def save_camera_calibration(baseline_x, baseline_y, pixels_per_inch):
    calibration = {
        "camera": "Brio 101",
        "baseline_x_px": baseline_x,
        "baseline_y_in": baseline_y,
        "pixels_per_inch": pixels_per_inch,
        "updated_unix_s": time.time(),
        "note": "Fixed-camera anchor. Use Reset Start only at the configured robot pose.",
    }
    temporary_path = CAMERA_CALIBRATION_PATH.with_suffix(".json.tmp")
    temporary_path.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(CAMERA_CALIBRATION_PATH)


def track_fixed_camera():
    if not DEBUG_WEBCAM_ENABLED:
        with state_lock:
            state["camera_pose"] = {
                "available": False,
                "message": "disabled in tournament mode; set MOREVEX_DEBUG_WEBCAM=1 for ground truth",
            }
        return
    if cv2 is None:
        with state_lock:
            state["camera_pose"] = {
                "available": False,
                "message": "OpenCV is unavailable.",
            }
        return
    template = cv2.imread(str(CAMERA_TEMPLATE_PATH), cv2.IMREAD_GRAYSCALE)
    if template is None:
        with state_lock:
            state["camera_pose"] = {
                "available": False,
                "message": f"Missing camera marker: {CAMERA_TEMPLATE_PATH}",
            }
        return

    calibration = load_camera_calibration()
    if calibration is None:
        baseline_x = None
        baseline_y = CAMERA_START_Y_IN
        pixels_per_inch = CAMERA_PIXELS_PER_INCH
    else:
        baseline_x, baseline_y, pixels_per_inch = calibration
    with state_lock:
        applied_reset_token = state["camera_reset_token"]

    while True:
        capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, 30)
        if not capture.isOpened():
            with state_lock:
                state["camera_pose"] = {
                    "available": False,
                    "message": "Brio 101 camera is unavailable; using onboard Y.",
                }
            capture.release()
            time.sleep(1.0)
            continue

        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError("camera frame read failed")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                search = gray[300:520, 80:1200]
                result = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
                _, score, _, location = cv2.minMaxLoc(result)
                marker_x = 80 + location[0]
                marker_y = 300 + location[1]
                now = time.time()
                if score >= CAMERA_MIN_SCORE:
                    with state_lock:
                        reset_token = state["camera_reset_token"]
                    if reset_token != applied_reset_token:
                        baseline_x = marker_x
                        baseline_y = CAMERA_START_Y_IN
                        pixels_per_inch = CAMERA_PIXELS_PER_INCH
                        save_camera_calibration(baseline_x, baseline_y, pixels_per_inch)
                        applied_reset_token = reset_token
                    if baseline_x is None:
                        baseline_x = marker_x
                        save_camera_calibration(baseline_x, baseline_y, pixels_per_inch)
                    camera_y_in = baseline_y + (
                        marker_x - baseline_x
                    ) / pixels_per_inch
                    camera_pose = {
                        "available": True,
                        "x_px": marker_x,
                        "y_px": marker_y,
                        "baseline_x_px": baseline_x,
                        "baseline_y_in": baseline_y,
                        "pixels_per_inch": pixels_per_inch,
                        "field_y_in": camera_y_in,
                        "score": float(score),
                        "pc_time": now,
                        "message": "Fixed Brio 101 along-wall correction active.",
                    }
                else:
                    camera_pose = {
                        "available": False,
                        "score": float(score),
                        "pc_time": now,
                        "message": "Camera marker confidence low; using onboard Y.",
                    }
                with state_lock:
                    state["camera_pose"] = camera_pose
                time.sleep(0.03)
        except Exception as exc:
            with state_lock:
                state["camera_pose"] = {
                    "available": False,
                    "message": f"Camera tracker reconnecting: {exc}",
                }
        finally:
            capture.release()
        time.sleep(0.5)


def decode_serial_packets(packet_buffer, chunk):
    """Return complete PROS user-port text chunks and the unfinished packet."""
    packet_buffer.extend(chunk)
    decoded_chunks = []
    while b"\0" in packet_buffer:
        encoded, remainder = packet_buffer.split(b"\0", 1)
        packet_buffer = bytearray(remainder)
        if not encoded:
            continue
        try:
            decoded = cobs.decode(encoded)
        except cobs.DecodeError:
            # Focused smoke images may emit raw text instead of PROS topics.
            decoded = bytes(encoded)
        if len(decoded) >= 4 and decoded[:4] in (b"sout", b"serr"):
            decoded = decoded[4:]
        decoded = decoded.split(b"\0", 1)[0]
        if decoded:
            decoded_chunks.append(decoded.decode("utf-8", errors="ignore"))
    return decoded_chunks, packet_buffer


def read_serial():
    while True:
        candidates = serial_candidates()
        if not candidates:
            with state_lock:
                state["connected"] = False
                state["serial_port"] = None
                state["message"] = "No COM ports found. Replug the Brain USB if this stays here."
            time.sleep(1.0)
            continue

        for port in candidates:
            try:
                with serial.Serial(port, BAUD_RATE, timeout=0.05) as ser:
                    with state_lock:
                        state["connected"] = True
                        state["serial_port"] = port
                        state["message"] = f"Reading {port}; waiting for D4 frames..."
                    buffer = ""
                    packet_buffer = bytearray()
                    last_recognized_time = time.monotonic()
                    while True:
                        chunk = ser.read(4096)
                        if not chunk:
                            with state_lock:
                                latest = state["latest"]
                                if latest and time.time() - latest["pc_time"] > 1.5:
                                    state["message"] = f"{port} connected, but D4 data is stale."
                            if time.monotonic() - last_recognized_time > 6.0:
                                raise serial.SerialException(
                                    "no recognized D4/FUSE_TEST/VISION_SHADOW telemetry for 6 s"
                                )
                            continue
                        decoded_chunks, packet_buffer = decode_serial_packets(
                            packet_buffer, chunk
                        )
                        buffer += "".join(decoded_chunks)
                        lines = buffer.splitlines()
                        buffer = lines.pop() if lines else ""
                        for line in lines:
                            recognized = False
                            frame = parse_d4(line)
                            if frame:
                                update_latest(frame, port)
                                recognized = True
                            fused_pose = parse_fuse_test(line)
                            if fused_pose:
                                update_onboard_pose(fused_pose, port)
                                recognized = True
                            vision = parse_vision_shadow(line)
                            if vision:
                                update_vision(vision, port)
                                recognized = True
                            if recognized:
                                last_recognized_time = time.monotonic()
                        if time.monotonic() - last_recognized_time > 6.0:
                            raise serial.SerialException(
                                "bytes received but no recognized robot telemetry for 6 s"
                            )
            except serial.SerialException as exc:
                with state_lock:
                    state["connected"] = False
                    state["message"] = f"{port}: {exc}"
                time.sleep(0.2)


def payload():
    with state_lock:
        return {
            "connected": state["connected"],
            "serial_port": state["serial_port"],
            "latest": state["latest"],
            "onboard_pose": state["onboard_pose"],
            "camera_pose": state["camera_pose"],
            "vision": state["vision"],
            "sensors": state["latest"]["sensors"] if state["latest"] else {},
            "message": state["message"],
            "rate_hz": state["rate_hz"],
            "reset_token": state["reset_token"],
        }


def request_pose_reset():
    with state_lock:
        state["reset_token"] += 1
        state["camera_reset_token"] += 1
        return state["reset_token"]


def view_for_path(raw_path):
    path = unquote(raw_path).rstrip("/") or "/"
    if path in (
        "/playing-field",
        "/playing_field",
        "/playing field",
        "/playingfield",
        "/pose-grid",
        "/pose_grid",
        "/localization",
        "/localisation",
    ):
        return "field"
    if path in ("/", "/index.html"):
        return "dashboard"
    return None


def render_html(view):
    field_only = view == "field"
    return (
        HTML.replace("__PAGE_TITLE__", "Localization Tracker" if field_only else "Localization Sensor Dashboard")
        .replace("__BODY_CLASS__", "field-only" if field_only else "")
        .replace("__DASHBOARD_TAB_CLASS__", "" if field_only else "active")
        .replace("__FIELD_TAB_CLASS__", "active" if field_only else "")
    )


class Handler(BaseHTTPRequestHandler):
    def send_json(self, status, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_reset_response(self):
        body = json.dumps({"reset_token": request_pose_reset()}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        view = view_for_path(path)
        if view:
            body = render_html(view).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/data":
            body = json.dumps(payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/reset-pose":
            self.send_reset_response()
            return
        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/reset-pose":
            self.send_reset_response()
            return
        self.send_error(404)

    def log_message(self, *_):
        return


class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True


if __name__ == "__main__":
    threading.Thread(target=read_serial, daemon=True).start()
    threading.Thread(target=track_fixed_camera, daemon=True).start()
    print(f"LiDAR line UI: http://127.0.0.1:{HTTP_PORT}/", flush=True)
    ReusableHTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
