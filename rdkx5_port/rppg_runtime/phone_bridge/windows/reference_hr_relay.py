#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None, socket.AF_INET):
            addresses.add(item[4][0])
    except OSError:
        pass
    return sorted(ip for ip in addresses if not ip.startswith("127."))


def project_root_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parents[3]
    return Path(__file__).resolve().parents[3]


class HeartRateState:
    def __init__(self, max_events: int = 100) -> None:
        self._lock = threading.RLock()
        self._latest: dict[str, object] = {
            "seq": 0,
            "source": "",
            "bpm": None,
            "received_at": None,
            "device_timestamp": None,
            "remote_addr": "",
        }
        self._seq = 0
        self._events: list[dict[str, object]] = []
        self._max_events = max_events
        self._forward_status: dict[str, object] = {"ok": None, "state": "idle"}

    def update(
        self,
        bpm: float,
        source: str,
        device_timestamp: Optional[float],
        remote_addr: str,
    ) -> dict[str, object]:
        with self._lock:
            self._seq += 1
            now = time.time()
            previous_received_at = self._latest.get("received_at")
            interval_sec = None
            if previous_received_at is not None:
                interval_sec = now - float(previous_received_at)
            delay_sec = None
            if device_timestamp is not None:
                delay_sec = now - device_timestamp
            self._latest = {
                "seq": self._seq,
                "source": source or "mi_band_7",
                "bpm": round(float(bpm), 1),
                "received_at": now,
                "device_timestamp": device_timestamp,
                "remote_addr": remote_addr,
                "interval_sec": round(interval_sec, 3) if interval_sec is not None else None,
                "delay_sec": round(delay_sec, 3) if delay_sec is not None else None,
            }
            self._events.append(dict(self._latest))
            self._events = self._events[-self._max_events :]
            return self.status()

    def set_forward_status(self, status: dict[str, object]) -> None:
        with self._lock:
            self._forward_status = dict(status)
            self._forward_status["updated_at"] = time.time()

    def status(self) -> dict[str, object]:
        with self._lock:
            item = dict(self._latest)
            forward_status = dict(self._forward_status)
        received_at = item.get("received_at")
        age = time.time() - float(received_at) if received_at else None
        return {
            "seq": item.get("seq") or 0,
            "source": item.get("source") or "",
            "bpm": item.get("bpm"),
            "age_sec": round(age, 2) if age is not None else None,
            "fresh": bool(age is not None and age <= 30.0),
            "received_at": received_at,
            "device_timestamp": item.get("device_timestamp"),
            "remote_addr": item.get("remote_addr") or "",
            "interval_sec": item.get("interval_sec"),
            "delay_sec": item.get("delay_sec"),
            "board_forward": forward_status,
        }

    def events(self, limit: int = 20) -> list[dict[str, object]]:
        with self._lock:
            return list(reversed(self._events[-limit:]))


class ExperimentLogger:
    def __init__(self, base_dir: Path) -> None:
        self.base_dir = base_dir
        self._lock = threading.RLock()
        self._active = False
        self._session_id = ""
        self._note = ""
        self._started_at: Optional[float] = None
        self._last_written_at: Optional[float] = None
        self._rows = 0
        self._jsonl_path: Optional[Path] = None
        self._csv_path: Optional[Path] = None
        self._last_error = ""

    def start(self, note: str = "") -> dict[str, object]:
        with self._lock:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            session_id = time.strftime("%Y%m%d_%H%M%S")
            self._session_id = session_id
            self._note = note.strip()
            self._started_at = time.time()
            self._last_written_at = None
            self._rows = 0
            self._last_error = ""
            self._jsonl_path = self.base_dir / f"{session_id}.jsonl"
            self._csv_path = self.base_dir / f"{session_id}.csv"
            self._csv_path.write_text(
                ",".join(
                    [
                        "timestamp_iso",
                        "timestamp_unix",
                        "board_ok",
                        "board_source_mode",
                        "board_source_name",
                        "board_seq",
                        "board_frame_idx",
                        "board_total_frames",
                        "board_time_sec",
                        "board_age_sec",
                        "board_fps",
                        "board_face_count",
                        "board_roi_count",
                        "board_bpm",
                        "board_bpm_state",
                        "board_raw_bpm",
                        "board_selected_bpm",
                        "board_tracker_reason",
                        "board_signal_method",
                        "board_candidate_score",
                        "board_selection_reason",
                        "board_peak_bpm",
                        "board_peak_snr",
                        "board_signal_std",
                        "board_peak1_bpm",
                        "board_peak1_snr",
                        "board_peak2_bpm",
                        "board_peak2_snr",
                        "board_peak3_bpm",
                        "board_peak3_snr",
                        "board_buffer_seconds",
                        "board_r",
                        "board_g",
                        "board_b",
                        "signal_seconds",
                        "min_bpm",
                        "max_bpm",
                        "reference_source",
                        "reference_bpm",
                        "reference_age_sec",
                        "pc_seq",
                        "pc_interval_sec",
                        "pc_mode",
                        "delta_bpm",
                        "board_error",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            self._jsonl_path.write_text("", encoding="utf-8")
            self._active = True
            return self.status()

    def stop(self) -> dict[str, object]:
        with self._lock:
            self._active = False
            return self.status()

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "active": self._active,
                "session_id": self._session_id,
                "note": self._note,
                "started_at": self._started_at,
                "last_written_at": self._last_written_at,
                "rows": self._rows,
                "jsonl_path": str(self._jsonl_path) if self._jsonl_path else "",
                "csv_path": str(self._csv_path) if self._csv_path else "",
                "last_error": self._last_error,
            }

    def log_snapshot(self, snapshot: dict[str, object]) -> None:
        with self._lock:
            if not self._active or not self._jsonl_path or not self._csv_path:
                return
            now = time.time()
            board_wrap = snapshot.get("board_wrap") or {}
            board = snapshot.get("board") or {}
            params = board.get("params") or {}
            debug = board.get("bpm_debug") or {}
            reference = snapshot.get("active_ref") or {}
            local_hr = snapshot.get("reference_hr") or {}
            top_peaks = debug.get("top_peaks") if isinstance(debug.get("top_peaks"), list) else []

            def peak_value(index: int, key: str) -> object:
                if index >= len(top_peaks):
                    return None
                peak = top_peaks[index]
                if not isinstance(peak, dict):
                    return None
                return peak.get(key)

            row = {
                "timestamp_iso": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
                "timestamp_unix": round(now, 3),
                "board_ok": bool(board_wrap.get("ok")),
                "board_source_mode": board.get("source_mode"),
                "board_source_name": board.get("source_name"),
                "board_seq": board.get("seq"),
                "board_frame_idx": board.get("frame_idx"),
                "board_total_frames": board.get("total_frames"),
                "board_time_sec": board.get("time_sec"),
                "board_age_sec": board.get("age_sec"),
                "board_fps": board.get("input_fps"),
                "board_face_count": board.get("face_count"),
                "board_roi_count": board.get("roi_count"),
                "board_bpm": board.get("bpm"),
                "board_bpm_state": board.get("bpm_state"),
                "board_raw_bpm": debug.get("raw_bpm"),
                "board_selected_bpm": debug.get("selected_bpm"),
                "board_tracker_reason": debug.get("tracker_reason"),
                "board_signal_method": debug.get("signal_method"),
                "board_candidate_score": debug.get("candidate_score"),
                "board_selection_reason": debug.get("selection_reason"),
                "board_peak_bpm": debug.get("peak_bpm"),
                "board_peak_snr": debug.get("peak_snr"),
                "board_signal_std": debug.get("signal_std"),
                "board_peak1_bpm": peak_value(0, "bpm"),
                "board_peak1_snr": peak_value(0, "snr"),
                "board_peak2_bpm": peak_value(1, "bpm"),
                "board_peak2_snr": peak_value(1, "snr"),
                "board_peak3_bpm": peak_value(2, "bpm"),
                "board_peak3_snr": peak_value(2, "snr"),
                "board_buffer_seconds": board.get("buffer_seconds"),
                "board_r": (board.get("rgb") or {}).get("r"),
                "board_g": (board.get("rgb") or {}).get("g"),
                "board_b": (board.get("rgb") or {}).get("b"),
                "signal_seconds": params.get("signal_seconds"),
                "min_bpm": params.get("min_bpm"),
                "max_bpm": params.get("max_bpm"),
                "reference_source": reference.get("source"),
                "reference_bpm": reference.get("bpm"),
                "reference_age_sec": reference.get("age_sec"),
                "pc_seq": local_hr.get("seq"),
                "pc_interval_sec": local_hr.get("interval_sec"),
                "pc_mode": "pc_only" if snapshot.get("pc_only") else "pc_plus_board",
                "delta_bpm": snapshot.get("delta_bpm"),
                "board_error": board_wrap.get("error") or "",
            }
            try:
                with self._jsonl_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(row, ensure_ascii=False) + "\n")
                with self._csv_path.open("a", encoding="utf-8", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
                    writer.writerow(row)
                self._rows += 1
                self._last_written_at = now
            except Exception as exc:
                self._last_error = str(exc)


def first_value(payload: dict[str, object], *names: str) -> Optional[object]:
    for name in names:
        if name not in payload:
            continue
        value = payload[name]
        if isinstance(value, list):
            return value[0] if value else None
        return value
    return None


def parse_device_timestamp(value: Optional[object]) -> Optional[float]:
    if value in (None, ""):
        return None
    try:
        parsed = float(str(value).strip())
    except ValueError:
        return None
    if parsed > 1_000_000_000_000:
        parsed = parsed / 1000.0
    return parsed


def validate_hr_payload(payload: dict[str, object]) -> tuple[Optional[dict[str, object]], Optional[str]]:
    bpm_raw = first_value(payload, "bpm", "hr", "heart_rate", "heartrate")
    if bpm_raw is None:
        return None, "missing bpm"
    try:
        bpm = float(str(bpm_raw).strip())
    except ValueError:
        return None, "bpm must be numeric"
    if bpm < 30.0 or bpm > 240.0:
        return None, "bpm out of range"

    source = str(first_value(payload, "source", "device") or "mi_band_7").strip() or "mi_band_7"
    device_timestamp = parse_device_timestamp(first_value(payload, "timestamp", "ts", "time"))
    return {"source": source, "bpm": bpm, "device_timestamp": device_timestamp}, None


def read_json_response(response) -> dict[str, object]:
    text = response.read().decode("utf-8", errors="replace")
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("response is not a JSON object")
    return data


def make_handler(
    board_url: str,
    timeout: float,
    state: HeartRateState,
    board_forward_enabled: bool,
    logger: ExperimentLogger,
):
    target_base = board_url.rstrip("/")
    board_base_js = json.dumps(target_base)

    class Handler(BaseHTTPRequestHandler):
        server_version = "ReferenceHrRelay/0.3"

        def log_message(self, fmt: str, *args) -> None:
            print(f"{self.address_string()} - {fmt % args}", flush=True)

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self.send_cors_headers()
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            if path in ("/", "/index.html"):
                self.send_html()
                return
            if path == "/experiment":
                self.send_experiment_html()
                return
            if path == "/ping":
                self.send_json({"ok": True, "relay": "reference_hr", "board_url": target_base})
                return
            if path == "/status":
                self.send_json(
                    {
                        "ok": True,
                        "reference_hr": state.status(),
                        "board_url": target_base,
                        "pc_only": not board_forward_enabled,
                    }
                )
                return
            if path == "/events":
                self.send_json({"ok": True, "events": state.events(), "pc_only": not board_forward_enabled})
                return
            if path == "/experiment_status":
                self.send_json(self.build_experiment_snapshot())
                return
            if path == "/experiment_log":
                self.send_json({"ok": True, "logger": logger.status()})
                return
            if path == "/board_status":
                snapshot = self.build_experiment_snapshot()
                self.send_json(snapshot["board_wrap"])
                return
            if path == "/board_params":
                self.send_json(self.board_request("/params"))
                return
            if path == "/reference_hr":
                values = parse_qs(parsed.query)
                if not values:
                    self.send_json({"ok": True, "reference_hr": state.status(), "board_url": target_base})
                    return
                self.accept_reference_hr(values)
                return
            self.send_json({"ok": False, "error": "not found"}, status=404)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/reference_hr":
                payload = self.read_payload()
                if payload is None:
                    return
                self.accept_reference_hr(payload)
                return
            if parsed.path == "/board_params":
                payload = self.read_payload()
                if payload is None:
                    return
                self.send_json(self.board_request("/params", payload))
                return
            if parsed.path == "/experiment_log":
                payload = self.read_payload(allow_empty=True)
                if payload is None:
                    return
                action = str(payload.get("action") or "").strip().lower()
                if action == "start":
                    note = str(payload.get("note") or "").strip()
                    self.send_json({"ok": True, "logger": logger.start(note=note)})
                    return
                if action == "stop":
                    self.send_json({"ok": True, "logger": logger.stop()})
                    return
                self.send_json({"ok": False, "error": "unknown log action"}, status=400)
                return
            self.send_json({"ok": False, "error": "not found"}, status=404)

        def read_payload(self, allow_empty: bool = False) -> Optional[dict[str, object]]:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self.send_json({"ok": False, "error": "invalid content length"}, status=400)
                return None
            if length <= 0:
                if allow_empty:
                    return {}
                self.send_json({"ok": False, "error": "empty or too large body"}, status=400)
                return None
            if length > 4096:
                self.send_json({"ok": False, "error": "empty or too large body"}, status=400)
                return None
            raw = self.rfile.read(length).decode("utf-8", errors="replace")
            content_type = self.headers.get("Content-Type", "")
            try:
                if "application/json" in content_type:
                    payload = json.loads(raw)
                else:
                    try:
                        payload = json.loads(raw)
                    except json.JSONDecodeError:
                        payload = parse_qs(raw)
            except Exception as exc:
                self.send_json({"ok": False, "error": f"invalid payload: {exc}"}, status=400)
                return None
            if not isinstance(payload, dict):
                self.send_json({"ok": False, "error": "payload must be an object"}, status=400)
                return None
            return payload

        def board_request(self, path: str, payload: Optional[dict[str, object]] = None) -> dict[str, object]:
            if not target_base:
                return {"ok": False, "error": "board url not configured"}
            data = None
            headers: dict[str, str] = {}
            method = "GET"
            if payload is not None:
                data = json.dumps(payload).encode("utf-8")
                headers["Content-Type"] = "application/json"
                method = "POST"
            request = urllib.request.Request(target_base + path, data=data, headers=headers, method=method)
            try:
                with urllib.request.urlopen(request, timeout=timeout) as response:
                    body = read_json_response(response)
                    return {"ok": True, "status": response.status, "board": body, "board_url": target_base}
            except urllib.error.HTTPError as exc:
                try:
                    body = read_json_response(exc)
                except Exception:
                    body = {"ok": False, "error": exc.read().decode("utf-8", errors="replace")[:300]}
                return {
                    "ok": False,
                    "status": exc.code,
                    "error": "board http error",
                    "board": body,
                    "board_url": target_base,
                }
            except urllib.error.URLError as exc:
                return {"ok": False, "error": f"board unavailable: {exc}", "board_url": target_base}
            except TimeoutError as exc:
                return {"ok": False, "error": f"board unavailable: {exc}", "board_url": target_base}
            except Exception as exc:
                return {"ok": False, "error": f"board unavailable: {exc}", "board_url": target_base}

        def build_experiment_snapshot(self) -> dict[str, object]:
            local_hr = state.status()
            board_wrap = self.board_request("/status")
            board = board_wrap.get("board") if board_wrap.get("ok") and isinstance(board_wrap.get("board"), dict) else {}
            board_ref = board.get("reference_hr") if isinstance(board.get("reference_hr"), dict) else {}
            active_ref = None
            active_ref_source = "none"
            if board_ref.get("fresh") and board_ref.get("bpm") is not None:
                active_ref = dict(board_ref)
                active_ref_source = "board"
            elif local_hr.get("fresh") and local_hr.get("bpm") is not None:
                active_ref = dict(local_hr)
                active_ref_source = "pc"
            delta_bpm = None
            if isinstance(board.get("bpm"), (int, float)) and active_ref and isinstance(active_ref.get("bpm"), (int, float)):
                delta_bpm = round(float(board["bpm"]) - float(active_ref["bpm"]), 1)
            snapshot = {
                "ok": True,
                "pc_only": not board_forward_enabled,
                "reference_hr": local_hr,
                "board_wrap": board_wrap,
                "board": board,
                "active_ref": active_ref,
                "active_ref_source": active_ref_source,
                "delta_bpm": delta_bpm,
                "logger": logger.status(),
            }
            logger.log_snapshot(snapshot)
            snapshot["logger"] = logger.status()
            return snapshot

        def accept_reference_hr(self, payload: dict[str, object]) -> None:
            parsed, error = validate_hr_payload(payload)
            if error:
                self.send_json({"ok": False, "error": error}, status=400)
                return
            assert parsed is not None
            local_status = state.update(
                bpm=float(parsed["bpm"]),
                source=str(parsed["source"]),
                device_timestamp=parsed["device_timestamp"] if isinstance(parsed["device_timestamp"], float) else None,
                remote_addr=self.client_address[0],
            )
            if board_forward_enabled and target_base:
                threading.Thread(target=self.forward_to_board_and_store, args=(local_status,), daemon=True).start()
                board_forward = {"state": "queued"}
            else:
                state.set_forward_status({"state": "pc_only", "ok": None})
                board_forward = {"state": "pc_only"}
            self.send_json({"ok": True, "reference_hr": local_status, "board_forward": board_forward})

        def forward_to_board_and_store(self, local_status: dict[str, object]) -> None:
            state.set_forward_status({"state": "forwarding", "ok": None})
            state.set_forward_status(self.forward_to_board(local_status))

        def forward_to_board(self, local_status: dict[str, object]) -> dict[str, object]:
            bpm = local_status.get("bpm")
            if bpm is None:
                return {"ok": False, "error": "no bpm"}
            data = {
                "source": local_status.get("source") or "mi_band_7",
                "bpm": bpm,
                "timestamp": local_status.get("device_timestamp") or local_status.get("received_at"),
            }
            payload = json.dumps(data).encode("utf-8")
            request = urllib.request.Request(
                target_base + "/reference_hr",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=timeout) as response:
                    body = response.read().decode("utf-8", errors="replace")
                    return {"ok": 200 <= response.status < 300, "status": response.status, "body": body[:300]}
            except urllib.error.HTTPError as exc:
                body = exc.read().decode("utf-8", errors="replace")
                return {"ok": False, "status": exc.code, "body": body[:300]}
            except urllib.error.URLError as exc:
                return {"ok": False, "error": f"board unavailable: {exc}"}
            except TimeoutError as exc:
                return {"ok": False, "error": f"board unavailable: {exc}"}

        def send_html(self) -> None:
            body = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Xiaomi Band HR Relay</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #101010; color: #f4f4f4; padding: 24px; }
    .value { font-size: 56px; font-weight: 700; margin: 10px 0; }
    .row { margin: 8px 0; }
    .label { color: #aaa; display: inline-block; min-width: 150px; }
    code, a { color: #8df58d; }
    .muted { color: #aaa; }
  </style>
</head>
<body>
  <h1>Xiaomi Band HR Relay</h1>
  <div class="muted">Latest heart rate received by this PC</div>
  <p><a href="/experiment">打开完整实验页</a></p>
  <div class="value" id="bpm">--</div>
  <div class="row"><span class="label">seq</span><span id="seq">--</span></div>
  <div class="row"><span class="label">source</span><span id="source">--</span></div>
  <div class="row"><span class="label">age</span><span id="age">--</span></div>
  <div class="row"><span class="label">from</span><span id="remote">--</span></div>
  <div class="row"><span class="label">received at</span><span id="receivedAt">--</span></div>
  <div class="row"><span class="label">device timestamp</span><span id="deviceTs">--</span></div>
  <div class="row"><span class="label">send delay</span><span id="delay">--</span></div>
  <div class="row"><span class="label">receive interval</span><span id="interval">--</span></div>
  <div class="row"><span class="label">mode</span><span id="mode">PC only</span></div>
  <div class="row"><span class="label">raw unix seconds</span><span id="raw">--</span></div>
  <h2>Recent events</h2>
  <table style="border-collapse: collapse; width: 100%; max-width: 980px;">
    <thead>
      <tr>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">seq</th>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">bpm</th>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">received</th>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">interval</th>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">delay</th>
        <th style="text-align:left;border-bottom:1px solid #333;padding:6px;">source</th>
      </tr>
    </thead>
    <tbody id="events"></tbody>
  </table>
  <p class="muted">Manual test: <code>/reference_hr?bpm=72&source=phone_test</code></p>
  <script>
    function formatUnixSeconds(value) {
      if (value === null || value === undefined) return '--';
      const date = new Date(Number(value) * 1000);
      if (Number.isNaN(date.getTime())) return '--';
      return date.toLocaleString();
    }

    function formatDelay(receivedAt, deviceTs) {
      if (receivedAt === null || receivedAt === undefined || deviceTs === null || deviceTs === undefined) return '--';
      const delta = Number(receivedAt) - Number(deviceTs);
      if (!Number.isFinite(delta)) return '--';
      return `${delta.toFixed(3)}s`;
    }

    async function tick() {
      const res = await fetch('/status', { cache: 'no-store' });
      const s = await res.json();
      const hr = s.reference_hr || {};
      document.getElementById('bpm').textContent = hr.bpm === null ? '--' : `${hr.bpm} BPM`;
      document.getElementById('seq').textContent = hr.seq || 0;
      document.getElementById('source').textContent = hr.source || '--';
      document.getElementById('age').textContent = hr.age_sec === null || hr.age_sec === undefined ? '--' : `${hr.age_sec}s`;
      document.getElementById('remote').textContent = hr.remote_addr || '--';
      document.getElementById('receivedAt').textContent = formatUnixSeconds(hr.received_at);
      document.getElementById('deviceTs').textContent = formatUnixSeconds(hr.device_timestamp);
      document.getElementById('delay').textContent = formatDelay(hr.received_at, hr.device_timestamp);
      document.getElementById('interval').textContent = hr.interval_sec === null || hr.interval_sec === undefined ? '--' : `${hr.interval_sec}s`;
      document.getElementById('mode').textContent = s.pc_only ? 'PC only' : 'PC + board forward';
      document.getElementById('raw').textContent =
        `received_at=${hr.received_at ?? '--'} device_timestamp=${hr.device_timestamp ?? '--'}`;

      const eventsResp = await fetch('/events', { cache: 'no-store' });
      const eventsData = await eventsResp.json();
      document.getElementById('events').innerHTML = (eventsData.events || []).map(e => `
        <tr>
          <td style="padding:6px;border-bottom:1px solid #222;">${e.seq ?? '--'}</td>
          <td style="padding:6px;border-bottom:1px solid #222;">${e.bpm ?? '--'}</td>
          <td style="padding:6px;border-bottom:1px solid #222;">${formatUnixSeconds(e.received_at)}</td>
          <td style="padding:6px;border-bottom:1px solid #222;">${e.interval_sec ?? '--'}s</td>
          <td style="padding:6px;border-bottom:1px solid #222;">${e.delay_sec ?? '--'}s</td>
          <td style="padding:6px;border-bottom:1px solid #222;">${e.source || '--'}</td>
        </tr>
      `).join('');
    }
    setInterval(tick, 300);
    tick();
  </script>
</body>
</html>"""
            payload = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.send_cors_headers()
            self.end_headers()
            self.wfile.write(payload)

        def send_experiment_html(self) -> None:
            body = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>RDK X5 完整实验页</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: Arial, sans-serif; background: #0f1115; color: #f4f4f4; }}
    header {{ padding: 14px 18px; background: #191c22; border-bottom: 1px solid #2b3039; display: flex; gap: 14px; align-items: center; flex-wrap: wrap; }}
    header a {{ color: #95c7ff; text-decoration: none; }}
    main {{ display: grid; grid-template-columns: minmax(0, 1.3fr) minmax(320px, 440px); min-height: calc(100vh - 60px); }}
    .stage {{ padding: 16px; background: #07090d; position: relative; }}
    .stage img {{ display: block; width: 100%; max-height: calc(100vh - 110px); object-fit: contain; background: #000; border: 1px solid #2b3039; border-radius: 6px; }}
    .stage .placeholder {{ position: absolute; inset: 16px; display: none; align-items: center; justify-content: center; text-align: center; color: #98a1af; border: 1px solid #2b3039; border-radius: 6px; background: rgba(0,0,0,0.72); padding: 24px; }}
    aside {{ padding: 16px; border-left: 1px solid #2b3039; background: #14171d; overflow: auto; }}
    .grid {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }}
    .card {{ padding: 12px; border: 1px solid #2b3039; border-radius: 6px; background: #1a1e26; }}
    .label {{ color: #98a1af; font-size: 12px; margin-bottom: 6px; }}
    .value {{ font-size: 30px; font-weight: 700; line-height: 1.1; }}
    .small {{ font-size: 14px; color: #c8ced8; }}
    .muted {{ color: #8f97a4; }}
    .ok {{ color: #86e27a; }}
    .bad {{ color: #ff8d8d; }}
    .section {{ margin-top: 14px; }}
    .row {{ display: flex; justify-content: space-between; gap: 10px; padding: 6px 0; border-bottom: 1px solid #232832; }}
    .row:last-child {{ border-bottom: 0; }}
    form {{ display: grid; gap: 10px; }}
    input, button {{ width: 100%; padding: 9px 10px; border-radius: 6px; border: 1px solid #384150; font: inherit; }}
    input {{ background: #0f131a; color: #f4f4f4; }}
    button {{ background: #2a6df4; color: white; cursor: pointer; }}
    code {{ color: #9fe39b; }}
    @media (max-width: 1000px) {{
      main {{ display: block; }}
      .stage img {{ max-height: 58vh; }}
      aside {{ border-left: 0; border-top: 1px solid #2b3039; }}
    }}
  </style>
</head>
<body>
  <header>
    <strong>RDK X5 完整实验页</strong>
    <span id="topStatus" class="muted">waiting...</span>
    <a href="/">PC 心率接收页</a>
    <a href="__BOARD_LINK__" target="_blank" rel="noreferrer">板端原始页</a>
  </header>
  <main>
    <section class="stage">
      <img id="camera" alt="RDK X5 camera" />
      <div id="cameraPlaceholder" class="placeholder">等待板端画面...<br />如果这里一直不出图，说明板子 8080 服务还没起来。</div>
    </section>
    <aside>
      <div class="grid">
        <div class="card">
          <div class="label">板端 rPPG BPM</div>
          <div class="value" id="boardBpm">--</div>
          <div class="small" id="boardState">--</div>
        </div>
        <div class="card">
          <div class="label">参考 BPM</div>
          <div class="value" id="refBpm">--</div>
          <div class="small" id="refSource">--</div>
        </div>
        <div class="card">
          <div class="label">差值</div>
          <div class="value" id="delta">--</div>
          <div class="small" id="deltaHint">--</div>
        </div>
          <div class="label">????</div>
          <div class="value" id="forwardState">--</div>
          <div class="small" id="forwardHint">--</div>
        </div>
      </div>

      <div class="section card">
        <div class="label">运行状态</div>
        <div class="row"><span>板端画面</span><span id="boardFresh">--</span></div>
        <div class="row"><span>板端 FPS</span><span id="boardFps">--</span></div>
        <div class="row"><span>检测到人脸</span><span id="faceCount">--</span></div>
        <div class="row"><span>ROI 数量</span><span id="roiCount">--</span></div>
        <div class="row"><span>RGB 均值</span><span id="rgb">--</span></div>
        <div class="row"><span>缓冲长度</span><span id="buffer">--</span></div>
        <div class="row"><span>PC 最新 seq</span><span id="pcSeq">--</span></div>
        <div class="row"><span>PC 接收间隔</span><span id="pcInterval">--</span></div>
      </div>

      <div class="section card">
        <div class="label">实时调参</div>
        <form id="paramsForm">
          <label class="small">信号窗口秒数
            <input id="signalSeconds" name="signal_seconds" type="number" min="8" max="60" step="1" />
          </label>
          <label class="small">最低 BPM
            <input id="minBpm" name="min_bpm" type="number" min="30" max="180" step="1" />
          </label>
          <label class="small">最高 BPM
            <input id="maxBpm" name="max_bpm" type="number" min="60" max="240" step="1" />
          </label>
          <button type="submit">应用到板子</button>
        </form>
        <div id="paramsStatus" class="small muted" style="margin-top: 10px;">等待读取板端参数...</div>
      </div>
    </aside>
  </main>
  <script>
    const boardBase = __BOARD_BASE_JS__;
    const camera = document.getElementById('camera');
    const cameraPlaceholder = document.getElementById('cameraPlaceholder');
    let paramsLoaded = false;
    let nextDelay = 120;

    function fmt(value, suffix = '') {{
      if (value === null || value === undefined || value === '') return '--';
      return `${{value}}${{suffix}}`;
    }}

    function hasValue(value) {{
      return value !== null && value !== undefined && value !== '';
    }}

    function refreshImage() {{
      if (!boardBase) return;
      camera.src = boardBase + '/snapshot.jpg?t=' + Date.now();
    }}
    camera.onload = () => {{
      cameraPlaceholder.style.display = 'none';
      setTimeout(refreshImage, nextDelay);
    }};
    camera.onerror = () => {{
      cameraPlaceholder.style.display = 'flex';
      setTimeout(refreshImage, 900);
    }};

    function applyParamsToInputs(params) {{
      if (!params) return;
      document.getElementById('signalSeconds').value = params.signal_seconds ?? '';
      document.getElementById('minBpm').value = params.min_bpm ?? '';
      document.getElementById('maxBpm').value = params.max_bpm ?? '';
      paramsLoaded = true;
    }}

    async function tick() {{
      try {{
        const [pcResp, boardResp] = await Promise.all([
          fetch('/status', {{ cache: 'no-store' }}),
          fetch('/board_status', {{ cache: 'no-store' }}),
        ]);
        const pcData = await pcResp.json();
        const boardWrap = await boardResp.json();
        const hr = pcData.reference_hr || {{}};
        const board = boardWrap.board || {{}};
        const boardRef = board.reference_hr || {{}};
        const pcOnly = !!pcData.pc_only;
        const activeRef = boardRef.fresh && boardRef.bpm !== null ? boardRef : (hr.fresh && hr.bpm !== null ? hr : null);
        const delta = hasValue(board.bpm) && activeRef && hasValue(activeRef.bpm)
          ? (Number(board.bpm) - Number(activeRef.bpm)).toFixed(1)
          : null;
        const stale = !boardWrap.ok || board.age_sec === null || board.age_sec > 3;
        nextDelay = stale ? 700 : 120;
        cameraPlaceholder.style.display = boardWrap.ok ? 'none' : 'flex';
        document.getElementById('topStatus').textContent = boardWrap.ok
          ? `???? | age ${fmt(board.age_sec, 's')} | ?? seq ${hr.seq || 0}`
          : (hr.fresh
              ? `?????? PC ?????? | seq ${hr.seq || 0} | ${boardWrap.error || '?????'}`
              : (boardWrap.error || '?????'));
        document.getElementById('topStatus').className = boardWrap.ok ? 'ok' : 'bad';

        document.getElementById('boardBpm').textContent = hasValue(board.bpm) ? `${{board.bpm}}` : '--';
        document.getElementById('boardState').textContent = `state=${{board.bpm_state || '--'}}`;
        document.getElementById('refBpm').textContent = activeRef && hasValue(activeRef.bpm) ? `${{activeRef.bpm}}` : '--';
        document.getElementById('refSource').textContent = activeRef
          ? `${{activeRef.source || '--'}} | age ${{fmt(activeRef.age_sec, 's')}}`
          : '暂无新参考心率';
        document.getElementById('delta').textContent = delta === null ? '--' : `${{delta}}`;
        document.getElementById('deltaHint').textContent = delta === null ? '等待两路心率同时有效' : '板端 rPPG - 参考心率';
        const forward = hr.board_forward || {{}};
        document.getElementById('forwardState').textContent = pcOnly ? 'PC ??' : (forward.state || '--');
        document.getElementById('forwardHint').textContent =
          pcOnly
            ? '???? PC ????????'
            : (forward.ok === null || forward.ok === undefined
                ? (forward.error || '????')
                : (forward.ok ? `ok status=${{forward.status ?? '--'}}` : (forward.error || `status=${{forward.status ?? '--'}}`)));

        document.getElementById('boardFresh').textContent = boardWrap.ok ? `${{board.width || '--'}}x${{board.height || '--'}} | age ${{fmt(board.age_sec, 's')}}` : '--';
        document.getElementById('boardFps').textContent = boardWrap.ok ? fmt(board.input_fps) : '--';
        document.getElementById('faceCount').textContent = boardWrap.ok ? fmt(board.face_count) : '--';
        document.getElementById('roiCount').textContent = boardWrap.ok ? fmt(board.roi_count) : '--';
        document.getElementById('rgb').textContent = boardWrap.ok && board.rgb ? `R ${{board.rgb.r}} / G ${{board.rgb.g}} / B ${{board.rgb.b}}` : '--';
        document.getElementById('buffer').textContent = boardWrap.ok ? `${{fmt(board.buffer_seconds, 's')}} | ${{board.bpm_state || '--'}}` : '--';
        document.getElementById('pcSeq').textContent = fmt(hr.seq);
        document.getElementById('pcInterval').textContent = fmt(hr.interval_sec, 's');

        if (!paramsLoaded && board.params) {{
          applyParamsToInputs(board.params);
          document.getElementById('paramsStatus').textContent =
            `当前板端参数：窗口 ${{board.params.signal_seconds}}s，频段 ${{board.params.min_bpm}}-${{board.params.max_bpm}} BPM`;
        }}
      }} catch (err) {{
        document.getElementById('topStatus').textContent = `状态获取失败: ${{err}}`;
        document.getElementById('topStatus').className = 'bad';
      }}
    }}

    document.getElementById('paramsForm').addEventListener('submit', async (event) => {{
      event.preventDefault();
      const payload = {{
        signal_seconds: Number(document.getElementById('signalSeconds').value),
        min_bpm: Number(document.getElementById('minBpm').value),
        max_bpm: Number(document.getElementById('maxBpm').value),
      }};
      document.getElementById('paramsStatus').textContent = '正在提交到板子...';
      try {{
        const resp = await fetch('/board_params', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify(payload),
        }});
        const data = await resp.json();
        if (!data.ok) {{
          document.getElementById('paramsStatus').textContent = `应用失败：${{data.error || 'unknown error'}}`;
          return;
        }}
        const boardPayload = data.board || {{}};
        const params = boardPayload.params || (boardPayload.status ? boardPayload.status.params : null);
        applyParamsToInputs(params);
        document.getElementById('paramsStatus').textContent =
          params
            ? `已应用：窗口 ${{params.signal_seconds}}s，频段 ${{params.min_bpm}}-${{params.max_bpm}} BPM`
            : '已提交到板子';
      }} catch (err) {{
        document.getElementById('paramsStatus').textContent = `应用失败：${{err}}`;
      }}
    }});

    setInterval(tick, 1000);
    tick();
    refreshImage();
  </script>
</body>
</html>""".replace("__BOARD_LINK__", target_base or "#").replace("__BOARD_BASE_JS__", board_base_js).replace("{{", "{").replace("}}", "}")
            payload = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.send_cors_headers()
            self.end_headers()
            self.wfile.write(payload)

        def send_cors_headers(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")

        def send_json(self, data: dict[str, object], status: int = 200) -> None:
            payload = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.send_cors_headers()
            self.end_headers()
            self.wfile.write(payload)

    return Handler


class QuietThreadingHTTPServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address) -> None:
        print(f"Connection closed while serving {client_address}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8090)
    parser.add_argument("--board-url", default="http://10.77.3.84:8080")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--no-board-forward", action="store_true")
    args = parser.parse_args()

    state = HeartRateState()
    logger = ExperimentLogger(project_root_dir() / "artifacts" / "experiment_logs")
    logger.start(note="auto_start")
    print(f"Receiving phone heart-rate data on port {args.listen_port}", flush=True)
    if args.no_board_forward:
        print("PC-only mode: board forwarding disabled", flush=True)
    else:
        print(f"Best-effort board forward target: {args.board_url}/reference_hr", flush=True)
    print(f"Listening on http://{args.listen_host}:{args.listen_port}/", flush=True)
    print(f"Experiment logs: {logger.status().get('csv_path')}", flush=True)
    for ip in local_ipv4_addresses():
        print(f"Phone status URL: http://{ip}:{args.listen_port}/", flush=True)
        print(f"Phone test URL: http://{ip}:{args.listen_port}/reference_hr?bpm=72&source=phone_test", flush=True)

    server = QuietThreadingHTTPServer(
        (args.listen_host, args.listen_port),
        make_handler(args.board_url, args.timeout, state, not args.no_board_forward, logger),
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
