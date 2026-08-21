# Report

Date: 2026-07-09

Status: design replacement complete. The proposed estimator is not implemented
in active firmware.

## Outcome

`NEXT_CHANGES.md` was deleted and recreated. It now documents the confirmed
current partial-fusion behavior and an implementation-ready master-fusion
architecture. The design evaluates every fresh external observation, but uses
measurement quality, estimator covariance, normalized innovation, temporal
consistency, and ambiguity gates before changing pose.

The design keeps LiDAR as a heading-bias observer initially, adds AI Vision as a
landmark-bearing observer in shadow mode, preserves the entered center-origin
start-pose contract, and explicitly represents unobservable forward drift as
growing uncertainty when no field reference is available.

## Changed Files

- `NEXT_CHANGES.md`
- `agent-workflow/master-fusion-design/goal.md`
- `agent-workflow/master-fusion-design/assumptions.md`
- `agent-workflow/master-fusion-design/baseline.md`
- `agent-workflow/master-fusion-design/design.md`
- `agent-workflow/master-fusion-design/failure-cases.md`
- `agent-workflow/master-fusion-design/eval-plan.md`
- `agent-workflow/master-fusion-design/progress.md`
- `agent-workflow/master-fusion-design/report.md`
- `agent-workflow/master-fusion-design/runbook.md`
- `agent-workflow/master-fusion-design/acceptance.json`
- `agent-workflow/master-fusion-design/research.md`
- `agent-workflow/master-fusion-design/master_fusion_note_check.py`

No C++ firmware, PROS configuration, autonomous routine, or dashboard source was
changed.

## Commands Run

```text
.\.venv\Scripts\python.exe .\agent-workflow\master-fusion-design\master_fusion_note_check.py
.\.venv\Scripts\python.exe .\agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
.\.venv\Scripts\python.exe -m py_compile .\agent-workflow\master-fusion-design\master_fusion_note_check.py
```

All six checks passed.

## Failed Attempt

`git status --short` failed with `fatal: not a git repository` because the
workspace's `.git` directory is empty. Changed-file scope was therefore reviewed
directly rather than from Git metadata.

## Artifact Review

- The note states that current LiDAR correction is capped at once per 1000 ms.
- The note states that AI Vision is not integrated in the active estimator.
- The obsolete vertical-tracking-wheel architecture and old title are absent.
- Large disagreement is explicitly not treated as automatic external-sensor
  truth.
- LiDAR, camera, stale-frame, duplicate-ID, fast-turn, outage, and 90-second
  failure tests are named with pass criteria.
- The first coding phase preserves current movement behavior and adds shadow
  estimator telemetry before changing correction cadence.

## Remaining Risks

- Sensor noise and latency distributions have not been measured on hardware.
- Camera mounting transform, intrinsics, and Goal tag-face geometry are not yet
  calibrated.
- The proposed covariance and NIS thresholds are not tuned.
- A tag or valid wall will not always be visible, so forward drift remains
  unbounded during sufficiently long reference outages.
- No recorded sensor log currently proves recovery behavior.

## Not Done

- No master estimator C++ code was added.
- No camera correction affects live pose.
- The 1000 ms LiDAR correction throttle was not removed.
- No PROS firmware build or physical robot test was needed for this design-only
  task.

## Manual Verification

Read `NEXT_CHANGES.md`, then run the commands in `runbook.md`. Before enabling
any correction behavior, complete Phase 1 shadow telemetry and collect the
calibration datasets listed in the note.
