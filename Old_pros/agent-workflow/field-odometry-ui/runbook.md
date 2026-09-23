Run syntax check:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py
```

Run regression checks:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py
.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
```

Build/upload robot code:

```powershell
.\.venv\Scripts\pros.exe make
.\.venv\Scripts\pros.exe upload --slot 1 --name MoreVex --after run . COM15
```

Run dashboard:

```powershell
.\.venv\Scripts\python.exe .\tools\lidar_bar_server.py
```

Open:

```text
http://127.0.0.1:8774/
http://127.0.0.1:8774/playing-field
http://127.0.0.1:8774/pose-grid
```

Pose-grid behavior:

```text
Onboard fused pose = latest fresh FUSE_TEST x/y/heading from robot code
Browser fallback odometry = D4 motor + port 5 odom estimate when FUSE_TEST is stale/missing
```
