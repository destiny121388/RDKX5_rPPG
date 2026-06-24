#!/usr/bin/env bash
set -o pipefail

BASE_DIR="/tmp/rdkx5_rppg"
PROJECT_DIR="${PROJECT_DIR:-$HOME/workspace_rdkx5/CV_Project}"
PORT_DIR="$PROJECT_DIR/rdkx5_port"
PYTHON_BIN="$PORT_DIR/.venv_rdkx5/bin/python"
SERVER_PORT=8080
LOCK_DIR="$BASE_DIR/supervisor.lock"
SOURCE_MODE="${SOURCE_MODE:-camera}"
VIDEO_PATH="${VIDEO_PATH:-$PROJECT_DIR/data/vid.avi}"
VIDEO_GT_PATH="${VIDEO_GT_PATH:-$PROJECT_DIR/data/gtdump.xmp}"
SIGNAL_SECONDS="${SIGNAL_SECONDS:-10}"

mkdir -p "$BASE_DIR"

set +u
source /opt/tros/humble/setup.bash
set -u

CAMERA_LOG="$BASE_DIR/mipi_cam.log"
SERVER_LOG="$BASE_DIR/rppg_server.log"
TTS_LOG="$BASE_DIR/tts_service.log"
SUPERVISOR_LOG="$BASE_DIR/supervisor.log"

camera_pid=""
server_pid=""
tts_pid=""
server_started_ts=0

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$SUPERVISOR_LOG"
}

release_lock() {
  if [ -d "$LOCK_DIR" ] && [ -f "$LOCK_DIR/pid" ]; then
    local owner=""
    owner="$(cat "$LOCK_DIR/pid" 2>/dev/null || true)"
    if [ "$owner" = "$$" ]; then
      rm -rf "$LOCK_DIR"
    fi
  fi
}

acquire_lock() {
  local owner=""
  if mkdir "$LOCK_DIR" 2>/dev/null; then
    echo $$ > "$LOCK_DIR/pid"
    echo $$ > "$BASE_DIR/supervisor.pid"
    return 0
  fi

  owner="$(cat "$LOCK_DIR/pid" 2>/dev/null || true)"
  if [ -n "$owner" ] && kill -0 "$owner" 2>/dev/null; then
    log "another supervisor is already running (pid=$owner), exiting"
    return 1
  fi

  log "found stale supervisor lock, clearing it"
  rm -rf "$LOCK_DIR"
  if mkdir "$LOCK_DIR" 2>/dev/null; then
    echo $$ > "$LOCK_DIR/pid"
    echo $$ > "$BASE_DIR/supervisor.pid"
    return 0
  fi

  log "failed to acquire supervisor lock"
  return 1
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_steps="${2:-20}"
  local i=""
  if [ -z "${pid:-}" ]; then
    return 0
  fi
  for ((i=0; i<timeout_steps; i++)); do
    if ! kill -0 "$pid" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

kill_server_listeners() {
  local pids=""
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${SERVER_PORT}/tcp" 2>/dev/null || true
  fi
  if command -v ss >/dev/null 2>&1; then
    pids="$(ss -ltnp 2>/dev/null | awk -v port=":${SERVER_PORT}" '$4 ~ port { if (match($0, /pid=[0-9]+/)) { print substr($0, RSTART + 4, RLENGTH - 4) } }' | sort -u)"
    if [ -n "$pids" ]; then
      for pid in $pids; do
        kill -9 "$pid" 2>/dev/null || true
      done
    fi
  fi
}

stop_children() {
  if [ -n "${server_pid:-}" ]; then
    kill "$server_pid" 2>/dev/null || true
    wait_for_pid_exit "$server_pid" 15 || kill -9 "$server_pid" 2>/dev/null || true
  fi
  if [ -n "${tts_pid:-}" ]; then
    kill "$tts_pid" 2>/dev/null || true
    wait_for_pid_exit "$tts_pid" 15 || kill -9 "$tts_pid" 2>/dev/null || true
  fi
  if [ -n "${camera_pid:-}" ]; then
    kill "$camera_pid" 2>/dev/null || true
    wait_for_pid_exit "$camera_pid" 15 || kill -9 "$camera_pid" 2>/dev/null || true
  fi
  pkill -f '[r]dk_rppg_server.py' 2>/dev/null || true
  pkill -f '[t]ts_service.py' 2>/dev/null || true
  pkill -f '/opt/tros/humble/lib/mipi_cam/[m]ipi_cam' 2>/dev/null || true
  pkill -f '[r]os2 launch mipi_cam' 2>/dev/null || true
  kill_server_listeners
}

wait_for_http_up() {
  local timeout_seconds="${1:-12}"
  local i=""
  for ((i=0; i<timeout_seconds; i++)); do
    if python3 - <<'PY'
import json, urllib.request
try:
    with urllib.request.urlopen('http://127.0.0.1:8080/status', timeout=1.5) as r:
        data = json.loads(r.read().decode('utf-8'))
    if isinstance(data, dict):
        print('ready')
        raise SystemExit(0)
except Exception:
    pass
raise SystemExit(1)
PY
    then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_camera() {
  if [ "$SOURCE_MODE" != "camera" ]; then
    camera_pid=""
    return 0
  fi
  log "starting camera"
  : > "$CAMERA_LOG"
  ros2 launch mipi_cam mipi_cam_640x480_bgr8.launch.py >> "$CAMERA_LOG" 2>&1 &
  camera_pid=$!
  echo "$camera_pid" > "$BASE_DIR/mipi_cam.pid"
}

start_server() {
  log "starting rppg server"
  : > "$SERVER_LOG"
  kill_server_listeners
  sleep 1
  cd "$PROJECT_DIR"
  local server_args=(
    "$PORT_DIR/rppg_runtime/board/rdk_rppg_server.py"
    --source-mode "$SOURCE_MODE"
    --port 8080
    --resize-width 640
    --jpeg-quality 78
    --signal-seconds "$SIGNAL_SECONDS"
  )
  if [ "$SOURCE_MODE" = "camera" ]; then
    server_args+=(--topic /image_raw --title "RDK X5 Camera rPPG")
  else
    server_args+=(--video-path "$VIDEO_PATH" --video-gt-path "$VIDEO_GT_PATH" --video-loop --title "RDK X5 Video rPPG")
  fi
  "$PYTHON_BIN" "${server_args[@]}" >> "$SERVER_LOG" 2>&1 &
  server_pid=$!
  echo "$server_pid" > "$BASE_DIR/rppg_server.pid"
  if ! wait_for_http_up 20; then
    log "rppg server failed to become healthy on port ${SERVER_PORT}"
    kill "$server_pid" 2>/dev/null || true
    wait_for_pid_exit "$server_pid" 15 || kill -9 "$server_pid" 2>/dev/null || true
    kill_server_listeners
    tail -n 20 "$SERVER_LOG" 2>/dev/null | sed 's/^/[server] /' | tee -a "$SUPERVISOR_LOG" >/dev/null
    return 1
  fi
  server_started_ts="$(date +%s)"
  return 0
}

start_tts() {
  log "starting tts service"
  : > "$TTS_LOG"
  cd "$PORT_DIR/rppg_runtime/board"
  "$PYTHON_BIN" tts_service.py >> "$TTS_LOG" 2>&1 &
  tts_pid=$!
  echo "$tts_pid" > "$BASE_DIR/tts_service.pid"
  # 等待 TTS 模型加载完成（模型文件较大，需十几秒）
  local i=""
  for ((i=0; i<30; i++)); do
    if python3 - <<'PY'
import json, urllib.request
try:
    with urllib.request.urlopen('http://127.0.0.1:7878', timeout=1.0) as r:
        data = json.loads(r.read().decode('utf-8'))
    if isinstance(data, dict) and data.get('service') == 'TTS':
        print('ready')
        raise SystemExit(0)
except Exception:
    pass
raise SystemExit(1)
PY
    then
      log "tts service ready"
      return 0
    fi
    sleep 1
  done
  log "tts service failed to become healthy"
  kill "$tts_pid" 2>/dev/null || true
  wait_for_pid_exit "$tts_pid" 15 || kill -9 "$tts_pid" 2>/dev/null || true
  return 1
}

if ! acquire_lock; then
  exit 0
fi

trap 'log "stopping"; stop_children; release_lock; exit 0' INT TERM EXIT

stop_children
sleep 1
start_camera
if [ "$SOURCE_MODE" = "camera" ]; then
  sleep 8
fi
start_tts
if ! start_server; then
  log "initial rppg server start failed"
fi
log "ready: http://10.77.3.84:8080/ mode=$SOURCE_MODE"

while true; do
  if [ "$SOURCE_MODE" = "camera" ] && ! kill -0 "$camera_pid" 2>/dev/null; then
    log "camera died, restarting"
    pkill -f '/opt/tros/humble/lib/mipi_cam/[m]ipi_cam' 2>/dev/null || true
    pkill -f '[r]os2 launch mipi_cam' 2>/dev/null || true
    sleep 2
    start_camera
    sleep 8
  fi

  if ! kill -0 "$server_pid" 2>/dev/null; then
    log "rppg server died, restarting"
    pkill -f '[r]dk_rppg_server.py' 2>/dev/null || true
    kill_server_listeners
    sleep 1
    if ! start_server; then
      log "rppg server restart failed"
    fi
  fi

  if ! kill -0 "$tts_pid" 2>/dev/null; then
    log "tts service died, restarting"
    pkill -f '[t]ts_service.py' 2>/dev/null || true
    sleep 1
    start_tts
  fi

  now_ts="$(date +%s)"
  if [ $((now_ts - server_started_ts)) -lt 35 ]; then
    sleep 5
    continue
  fi

  age="$(python3 - <<'PY'
import json, urllib.request
try:
    with urllib.request.urlopen('http://127.0.0.1:8080/status', timeout=1.5) as r:
        data = json.loads(r.read().decode('utf-8'))
    print(data.get('age_sec') if data.get('age_sec') is not None else 999)
except Exception:
    print(999)
PY
)"
  age_int="${age%.*}"
  if [ "${age_int:-999}" -gt 5 ]; then
    log "stream stale age=${age}s, restarting source and server"
    stop_children
    sleep 2
    start_camera
    if [ "$SOURCE_MODE" = "camera" ]; then
      sleep 8
    fi
    start_tts
    if ! start_server; then
      log "rppg server restart after stale stream failed"
    fi
  fi

  sleep 5
done
