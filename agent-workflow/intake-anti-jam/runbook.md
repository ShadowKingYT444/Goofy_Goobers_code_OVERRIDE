Build:

```powershell
$env:APPDATA = 'C:\Users\terry\Downloads\MoreVex\.pros-appdata'
.\.venv\Scripts\pros.exe make
```

Driver controls:

- A: intake forward.
- B: intake reverse.
- UP: calibrate normal intake values for about two seconds.

Robot verification:

1. Run calibration with the intake clear.
2. Run intake forward normally and confirm no pulsing.
3. Stall the intake and confirm both intake motors pulse reverse/forward for about one second.
