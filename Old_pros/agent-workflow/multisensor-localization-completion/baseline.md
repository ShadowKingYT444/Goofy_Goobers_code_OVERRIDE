# Baseline before this completion pass

## Proven

- The IMU, drive encoders, four distance sensors, and port-5 rotation sensor stream live telemetry.
- Bounded software LiDAR heading correction removed the former rapid IMU oscillation.
- Audience-wall heading and perpendicular-axis correction were physically tested.
- A short forward test produced closely matched four-motor deltas.

## Incomplete or contradicted

- Operational UI Y is replaced by a fixed laptop-camera template tracker when fresh; this contradicts the webcam-debug-only requirement.
- No `pros::AIVision` object or AprilTag observation exists in production firmware.
- AI Vision port 1, family 21H7, tag size 0.75 in, and smoke intrinsics are provisional only.
- The rear-center odometer uses an assumed 2 in wheel, implicit sign +1, and assumed 12 cm rear offset. No LiDAR-versus-port-5 turn calibration exists.
- Start pose is compiled into firmware and duplicated in browser JavaScript rather than entered at round start.
- Smart port 9 is claimed by both `distance_9` and `slider_left(-9)` in source, although live telemetry proves a distance sensor is currently present there.
- Only one physical wall was tested; no live LiDAR-occlusion-to-AprilTag recovery exists.
- Existing acceptance artifacts cover the earlier wall-only repair and do not prove this full goal.
