# Evaluation plan

- Source/build gates: production kernel/API exposes AI Vision; existing localization and mechanism checks still pass.
- Hardware gate: port 20 installed/enabled; real Circle21h7 IDs and corners captured after a bounded rotation.
- Geometry replay: all nine Goals/four faces, duplicate IDs, equal-score ties, wrong heading, clipping, skew, stale/reordered frames, and wrong camera calibration.
- Sensor outage replay: all LiDAR blocked at startup/mid-route, tags visible/hidden, motor disagreement, port-5 error/jump, IMU wrap, and conflicting LiDAR/tag observations.
- Webcam isolation: absent webcam and injected false debug coordinates leave onboard and official UI pose bit-identical.
- Port-5 calibration: settled endpoints at relative LiDAR angles `0,+12,-12,+12,-12,0`; require safe stop/watchdogs, repeated CW/CCW slope agreement, and honest ratio-only result.
- Live route budget: plan no more than 30 inches and 600 degrees, leaving overshoot headroom below the hard 36-inch/720-degree user cap.
- UI: rendered tournament mode reports onboard X/Y/heading plus IMU/drive/side/LiDAR/tag freshness and explicit degraded states; no webcam-derived official coordinate.
- Accuracy evidence: webcam/video may score synchronized checkpoints offline, but never feeds the estimator.
