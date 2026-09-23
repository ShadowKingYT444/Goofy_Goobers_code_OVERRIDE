# Localization Completion Report

> **SUPERSEDED — different robot/chassis.** This July report describes the
> prior robot with a mechanically active rear tracker and multi-LiDAR setup. It
> is not a competition-readiness claim for the current P1/P6/P7/P8 robot. The
> current authoritative status is
> [reports/sensor_campaign_2026-08-25/goal_completion_audit.md](reports/sensor_campaign_2026-08-25/goal_completion_audit.md),
> with live-resume evidence in
> [reports/sensor_campaign_2026-08-25/live_resume_addendum.md](reports/sensor_campaign_2026-08-25/live_resume_addendum.md).

The fused localization system now propagates continuously from drive encoders, the rear horizontal tracking wheel, and IMU, with independently gated LiDAR-wall and AI Vision/AprilTag corrections. The supervised long-route session covered approximately 120 inches (10 feet). After correcting a safely detected low-power turn stall, the continuation route completed with `1.54 in` raw return-position error and `4.93 deg` raw return-heading error. Tag 3 then reacquired with `0.09 in` range residual and `0.00 deg` bearing residual and bounded the pose to `(22.91,41.15,144.18 deg)`.

Repeated tag IDs remain explicit competing hypotheses: measured range creates candidate circles around matching tag faces, while bearing plus fused heading yields implied robot-position distributions. Corrections require physical-goal separation and temporal consistency. The UI now shows the selected hypothesis in green and alternate paired-prop hypotheses in amber.

Full evidence, known limitations, commands, and manual checks are in [agent-workflow/multisensor-localization-completion/report.md](agent-workflow/multisensor-localization-completion/report.md). Exact operating commands are in [agent-workflow/multisensor-localization-completion/runbook.md](agent-workflow/multisensor-localization-completion/runbook.md).

## Final slow 20-foot campaign

A subsequent slow route completed six 20-inch reverse/return pairs and two bounded 15-degree heading excursions: 240 inches total translation at maximum drive power 35. Raw odometry returned with `6.42 in` position error and `4.32 deg` heading error; the visible Tag 3 then corrected the absolute position to `(30.18,34.70)` with `0.09 in` innovation and `0.07 in` range residual. Camera heading correction is deliberately disabled because one point landmark's range+bearing cannot uniquely determine both position and heading; heading remains IMU/LiDAR-owned. The final compact analysis is in `agent-workflow/multisensor-localization-completion/slow_long_route_final_summary.json`.
