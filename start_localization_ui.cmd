@echo off
cd /d "%~dp0"
echo Starting MoreVex localization UI...
echo.
echo Open this URL:
echo   http://127.0.0.1:8774/localization
echo.
echo Keep this window open while using the UI.
echo.
".venv\Scripts\python.exe" "tools\lidar_bar_server.py"
echo.
echo Server stopped.
pause
