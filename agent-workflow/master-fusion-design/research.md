# Source Map

Accessed 2026-07-09.

| Source | Type | Reliability | Use |
|---|---|---|---|
| `src/autons.cpp` | active firmware | high | Current prediction loop, LiDAR cadence, gates, and gains |
| `include/localization_config.hpp` | active config | high | Field frame, Goal map, repeated IDs, sensor sides |
| `ai_vision_smoke/src/main.cpp` | local experiment | medium | Existing corner-to-bearing/range prototype |
| `tools/lidar_bar_server.py` | active telemetry UI | high | Current observable telemetry contract |
| https://pros.cs.purdue.edu/v5/pros-4/aivision.html | official PROS tutorial | high | Tag ID and four-corner API contract |
| https://pros.cs.purdue.edu/v5/pros-4/group__cpp-aivision.html | official PROS API | high | Main-project integration surface |
| https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-V5-Distance-Sensor | official VEX documentation | high | Distance range and accuracy limitations |

Confirmed: external sensors provide useful absolute-ish observations but not infallible truth. Inference: a small uncertainty-weighted error-state filter is the best complexity/reliability tradeoff for this robot; physical replay and field tests are still required.
