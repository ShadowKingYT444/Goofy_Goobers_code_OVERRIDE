Build and upload:

```powershell
cd C:\Users\terry\Downloads\MoreVex\distance_working
$env:APPDATA=(Resolve-Path '..\.pros-appdata').Path
..\.venv\Scripts\pros.exe make
..\.venv\Scripts\pros.exe upload --slot 1 --name DistanceSmoke --after run . COM7
```

Start graph UI:

```powershell
cd C:\Users\terry\Downloads\MoreVex
.\.venv\Scripts\python.exe .\tools\distance_server.py
```

Open:

```text
http://127.0.0.1:8771/
```
