# Evaluation Plan

Design artifact checks:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\master-fusion-design\master_fusion_note_check.py
.\.venv\Scripts\python.exe .\agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
```

Required implementation evaluations for the future estimator:

1. Replay recorded sensor logs deterministically through the estimator.
2. Inject a 5-degree IMU bias with a clean wall and verify bounded convergence without overshoot.
3. Present a flat non-wall obstruction and verify no heading update.
4. Present duplicate-ID tag candidates with a small winner margin and verify rejection.
5. Inject forward slip while a side tag is visible and verify forward error decreases.
6. Repeat one stale camera frame and verify it is not counted as multiple observations.
7. Run a fast turn and verify prediction continues while external updates are gated.
8. Remove all external observations and verify uncertainty grows.
9. Reacquire two independent tags and verify recovery without a pose teleport.
10. Drive a measured 90-second route and compare pose against ground truth at fixed checkpoints.

Initial pass criteria to tune with real data:

- clean-wall heading error below 1 degree after convergence;
- forward error below 3 inches when a valid tag correction is available at least every 5 seconds;
- no normal update larger than 1 inch or 0.5 degree;
- no accepted corrections in named obstacle, ambiguity, stale-frame, or fast-turn adversarial cases;
- uncertainty increases during outages and decreases only after accepted independent information.
