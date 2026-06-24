@echo off
setlocal
cd /d "%~dp0"

set "BOARD_URL=http://10.77.3.84:8080"
set "RELAY_EXE=%~dp0..\..\windows\tools\reference_hr_relay.exe"

echo Reference HR Relay with board forwarding.
echo Board URL: %BOARD_URL%
echo.

set "PY="
set "USE_PY_LAUNCHER="
if not exist "%RELAY_EXE%" if exist "%~dp0..\.venv_windows_ble\Scripts\python.exe" set "PY=%~dp0..\.venv_windows_ble\Scripts\python.exe"
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

if defined USE_PY_LAUNCHER (
  py -3 "%~dp0reference_hr_relay.py" --listen-host 0.0.0.0 --listen-port 8090 --board-url %BOARD_URL%
) else if exist "%RELAY_EXE%" (
  "%RELAY_EXE%" --listen-host 0.0.0.0 --listen-port 8090 --board-url %BOARD_URL%
) else (
  "%PY%" "%~dp0reference_hr_relay.py" --listen-host 0.0.0.0 --listen-port 8090 --board-url %BOARD_URL%
)

echo Relay exited.
pause
