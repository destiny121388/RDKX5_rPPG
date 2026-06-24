@echo off
setlocal
cd /d "%~dp0"

set "PY="
set "USE_PY_LAUNCHER="
if exist "%~dp0..\.venv_windows_ble\Scripts\python.exe" set "PY=%~dp0..\.venv_windows_ble\Scripts\python.exe"
if not defined PY (
  for /f "delims=" %%I in ('where.exe python 2^>nul') do (
    echo %%I | find /I "WindowsApps" >nul
    if errorlevel 1 if not defined PY set "PY=%%I"
  )
)
if not defined PY (
  where.exe py >nul 2>nul
  if not errorlevel 1 set "USE_PY_LAUNCHER=1"
)
if not defined PY if not defined USE_PY_LAUNCHER (
  echo [ERROR] Python was not found.
  pause
  exit /b 1
)

echo Starting Xiaomi Smart Band 10 BLE -^> PC bridge.
echo.
echo Keep this window open and keep heart-rate broadcast enabled on the band.
echo   How: swipe up on band -^> Settings -^> Heart-rate broadcast -^> ON
echo.
echo PC page: http://127.0.0.1:8090/
echo.

if defined USE_PY_LAUNCHER (
  py -3 "%~dp0mi_band_10_ble_to_pc.py" --relay-url http://127.0.0.1:8090
) else (
  "%PY%" "%~dp0mi_band_10_ble_to_pc.py" --relay-url http://127.0.0.1:8090
)
echo.
pause
