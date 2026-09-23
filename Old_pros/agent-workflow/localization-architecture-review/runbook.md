From repo root:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py
.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py
.\.venv\Scripts\pros.exe make
```

For live dashboard inspection:

```powershell
.\.venv\Scripts\python.exe .\tools\lidar_bar_server.py
```

Then open:

```text
http://127.0.0.1:8774/
http://127.0.0.1:8774/playing-field
```
