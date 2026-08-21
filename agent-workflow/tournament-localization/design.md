# Design

1. Upgrade production to an AI-Vision-capable PROS kernel and verify all existing behavior still builds.
2. Instantiate port-20 AI Vision once, configure Circle21h7 tag-only detection after startup, and sample without blocking the 20 ms prediction loop.
3. Add timestamped vision observations and telemetry first. Reject invalid, clipped, tiny, non-convex, repeated/stale, fast-turn, unknown, back-facing, and ambiguous duplicate-ID observations.
4. Expand the field map from Goal centers to explicit Goal/tag-face geometry and add calibrated camera intrinsics/extrinsics. Do not invent values: collect live stationary views at surveyed poses.
5. Start in shadow mode: enumerate all matching faces, score predicted bearing/range/visibility, require an absolute gate, winner margin, and three consistent fresh observations, and log every decision without correcting pose.
6. Enable only bounded corrections after replay/live residuals pass. A single bearing cannot produce full 2D pose; use calibrated square-tag pose/range, two-tag triangulation, or bearing accumulation across motion.
7. Keep the laptop webcam outside the estimator. Tournament UI displays onboard X/Y/heading and sensor freshness; optional webcam data is labeled debug ground truth/error only.
8. Calibrate port-5 rotation ratio with small repeated LiDAR-observed turns. Rotation identifies rear-offset/effective-wheel-diameter ratio, not both independently; translation scale still requires a measured sideways push.
9. Track cumulative commanded travel/rotation across all acceptance routines and abort before 36 inches or 720 degrees.
