Source checks:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\pid-autotune\pid_autotune_source_check.py
.\.venv\Scripts\python.exe -m py_compile .\agent-workflow\pid-autotune\pid_autotune_source_check.py
```

Build attempt:

```powershell
$env:APPDATA=(Resolve-Path .\.pros-appdata).Path
.\.venv\Scripts\pros.exe make
```

Robot use:

```text
Controller: X + Down
Serial output prefix: PID_TUNE
```
