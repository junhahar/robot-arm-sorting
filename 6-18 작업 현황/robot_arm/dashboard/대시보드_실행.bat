@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo Sambo Robot Arm Dashboard GPT Server starting...
echo Browser URL: http://127.0.0.1:8766/
echo.

start "" powershell.exe -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Seconds 1; Start-Process 'http://127.0.0.1:8766/'"
python .\sambo_dashboard_gpt_server.py

echo.
echo Dashboard server stopped. Press any key to close.
pause >nul
