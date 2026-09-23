# Runbook

Validate the rewritten design note:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\master-fusion-design\master_fusion_note_check.py
```

Reconfirm current firmware baselines:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\known-start-tag-map\localization_config_check.py
.\.venv\Scripts\python.exe .\agent-workflow\fusion-test-auton\fusion_test_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
```

Validate the new checker itself:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\agent-workflow\master-fusion-design\master_fusion_note_check.py
```

No firmware build is required for this design-only replacement unless source code changes in the same pass.
