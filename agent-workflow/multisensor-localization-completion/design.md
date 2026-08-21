# Design proposal

1. Centralize rear-wheel diameter/scale, raw sign, and rear offset in `localization_config.hpp` and consume the same values in firmware and UI telemetry.
2. Add a slow paired-turn calibration that records stable same-wall LiDAR headings and raw port-5 endpoints. Fit raw side travel versus signed LiDAR angle to determine sign and effective rear offset. Validate CW/CCW symmetry and pure-rotation translation cancellation.
3. Keep laptop camera position as a separately labeled diagnostic residual. Never substitute it into the operational fused pose.
4. Probe the actual AI Vision hardware with the isolated smoke project before adding production hardware declarations or upgrading the kernel.
5. After hardware identification, add timestamped onboard tag observations in shadow mode. Centralize camera extrinsics and per-face landmark geometry. Reject stale, clipped, back-facing, fast-turn, inconsistent, or duplicate-ID ambiguous candidates.
6. Enable bounded bearing/global corrections only after known-pose physical tag validation. During LiDAR/tag outages, dead-reckon and report growing/degraded confidence rather than reusing stale observations.
7. Add runtime start-pose entry/reset shared by the Brain estimator and UI.
