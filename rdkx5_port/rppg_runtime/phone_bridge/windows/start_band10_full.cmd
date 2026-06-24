@echo off
setlocal
cd /d "%~dp0"

set "BOARD_URL=http://10.77.3.84:8080"
set "SCRIPT_DIR=%~dp0"

echo ==========================================
echo  Xiaomi Smart Band 10 - Full Startup
echo  中继 + BLE 桥接 + 转发到板卡
echo ==========================================
echo.
echo Board URL: %BOARD_URL%
echo.
echo This will open TWO windows.
echo Close both with Ctrl+C or stop script.
echo.

REM ---- Launch relay server with board forwarding ----
echo Starting relay server...
start "HR Relay" "%SCRIPT_DIR%start_relay_with_board.cmd"

timeout /t 3 /nobreak >nul

REM ---- Launch BLE bridge ----
echo Starting BLE bridge...
start "BLE Bridge" "%SCRIPT_DIR%start_mi_band_10_ble_to_pc.cmd"

echo.
echo Both services started.
echo PC page: http://127.0.0.1:8090/
echo Board relay: %BOARD_URL%/reference_hr
echo.
pause
