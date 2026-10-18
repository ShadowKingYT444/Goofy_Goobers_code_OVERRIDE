# Autonomous journal

Engineering notes for the autonomous code, team 1862X, V5RC Override.
Newest entries are at the bottom of each file, dated.

| File | What it covers |
|---|---|
| [localization.md](localization.md) | How the robot knows where it is: odometry, goal and loader contact resets, GPS, the distance-sensor wall resets, and what each one taught us |
| [motion_and_pid.md](motion_and_pid.md) | PID gains, LemLib motion behaviour we rely on, chaining, stall endings, curves |
| [three_pin.md](three_pin.md) | The three-pin match autonomous (slot 1) |
| [four_pin.md](four_pin.md) | The four-pin far and close match autonomous (slots 2 and 3), formerly "two-pin" |
| [skills.md](skills.md) | The skills route (slot 4) |
| [solo.md](solo.md) | The solo autonomous (not on a slot yet) |
| [gps_reset_findings.md](gps_reset_findings.md) | 2026-10-10 deep review of the GPS correction code (moved here from `docs/`) |

Related material that is not journal text:
- `docs/autonomous_explainer.html`: animated, judge-friendly explainer.
- `Auto_paths/`: route sketches and HTML visualizers used to lay routes out.
- `CLAUDE.md`: current working notes (hardware, ports, slots, build steps).

## Slots on the Brain (2026-10-17)

| Slot | Name | Routine |
|---|---|---|
| 1 | 3 pin | `three_pin_auton` |
| 2 | 4 pin far | `four_pin_far_auton` |
| 3 | 4 pin close | `four_pin_close_auton` |
| 4 | skills | `skills` (test build: three-pin, then match loads onto G2) |
