#!/usr/bin/env python3
from __future__ import annotations

import argparse
import html
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from scipy.signal import butter, filtfilt
import subprocess
import threading
import os

from tts_client import tts_say_async
from audio_utils import get_usb_audio_device

VOICE_READY = "/home/sunrise/workspace_rdkx5/CV_Project/rdkx5_port/rppg_runtime/board/ready.wav"
VOICE_WAIT = "/home/sunrise/workspace_rdkx5/CV_Project/rdkx5_port/rppg_runtime/board/wait.wav"
VOICE_OK = "/home/sunrise/workspace_rdkx5/CV_Project/rdkx5_port/rppg_runtime/board/ok.wav"


DEFAULT_TARGET_FPS = 10.0
DEFAULT_MIN_BPM = 45.0
DEFAULT_MAX_BPM = 110.0 #180
DEFAULT_SELECTION_VARIANT = "pos_harmonic"
DEFAULT_MESH_INPUT_WIDTH = 320
MIN_SIGNAL_SECONDS = 8.0
MAX_SIGNAL_SECONDS = 60.0
MIN_SAMPLE_COUNT = 80
MIN_ANALYSIS_SECONDS = 9.0 #8.0 
PEAK_MIN_SEPARATION_BPM = 6.0
TRACK_NEAR_BPM = 18.0
TRACK_MAX_JUMP_BPM = 10 #12.0
TRACK_CONFIRM_FRAMES = 3 #3
TRACK_SMOOTHING = 0.2  #0.35
ESCAPE_NEAR_BPM = 10.0
ESCAPE_MIN_SEPARATION_BPM = 18.0
ESCAPE_POWER_RATIO = 1.15
ESCAPE_CONFIRM_FRAMES = 3
TOP_PEAK_COUNT = 5
LOW_BPM_SUBHARMONIC_MAX = 70.0
LOW_BPM_SUBHARMONIC_RATIO = 0.68
LOW_BPM_BOUNDARY_MARGIN = 7.0
LOW_BPM_BOUNDARY_ALT_RATIO = 0.72
HIGH_BPM_HARMONIC_MIN = 80.0
HIGH_BPM_HALF_SUPPORT_RATIO = 0.8
METHOD_SCORE_BONUS = {"green": 0.0, "pos": 0.25, "chrom": 0.2}
ROI_POLYGONS = {
    "forehead": [9, 107, 66, 105, 104, 103, 67, 109, 10, 338, 297, 332, 333, 334, 296, 336],
    "left cheek": [132, 58, 172, 136, 150, 169, 210, 212, 202, 57],
    "right cheek": [361, 288, 397, 365, 379, 394, 430, 432, 422, 287],
}


def image_to_bgr(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    h, w = int(msg.height), int(msg.width)
    if enc in ("bgr8", "rgb8"):
        frame = data.reshape((h, msg.step))[:, : w * 3].reshape((h, w, 3))
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if enc == "rgb8" else frame
    if enc in ("mono8", "8uc1"):
        frame = data.reshape((h, msg.step))[:, :w]
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if enc in ("bgra8", "rgba8"):
        frame = data.reshape((h, msg.step))[:, : w * 4].reshape((h, w, 4))
        code = cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(frame, code)
    if enc in ("nv12", "yuv420"):
        frame = data.reshape((h * 3 // 2, w))
        return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)
    raise ValueError(f"unsupported ROS image encoding: {msg.encoding}")


def load_ground_truth(path: Path) -> tuple[np.ndarray, np.ndarray]:
    timestamps = []
    heart_rate = []
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for raw_line in handle:
            line = raw_line.strip().replace(";", ",")
            if not line:
                continue
            parts = [item.strip() for item in line.split(",") if item.strip()]
            if len(parts) < 2:
                continue
            try:
                t_ms = float(parts[0])
                bpm = float(parts[1])
            except ValueError:
                continue
            timestamps.append(t_ms / 1000.0)
            heart_rate.append(bpm)
    if not timestamps:
        raise ValueError(f"no ground-truth rows in {path}")
    return np.asarray(timestamps, dtype=np.float64), np.asarray(heart_rate, dtype=np.float64)


def interpolate_ground_truth(gt_times: np.ndarray, gt_hr: np.ndarray, time_sec: float) -> Optional[float]:
    if gt_times.size == 0:
        return None
    if time_sec < float(gt_times[0]) or time_sec > float(gt_times[-1]):
        return None
    return float(np.interp(time_sec, gt_times, gt_hr))


def clamp_rect(rect: tuple[int, int, int, int], width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = rect
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width, x2))
    y2 = max(y1 + 1, min(height, y2))
    return x1, y1, x2, y2


def mean_rgb(frame: np.ndarray, rects: list[tuple[int, int, int, int]]) -> tuple[float, float, float]:
    chunks = []
    for x1, y1, x2, y2 in rects:
        roi = frame[y1:y2, x1:x2]
        if roi.size:
            chunks.append(roi.reshape(-1, 3))
    if not chunks:
        return 0.0, 0.0, 0.0
    pixels = np.concatenate(chunks, axis=0)
    b, g, r = np.mean(pixels, axis=0)
    return float(r), float(g), float(b)


def mean_rgb_polygons(frame: np.ndarray, polygons: list[np.ndarray]) -> tuple[float, float, float]:
    height, width = frame.shape[:2]
    region_means = []
    for pts in polygons:
        if pts.shape[0] < 3:
            continue
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        mean_bgr = cv2.mean(frame, mask=mask)[:3]
        region_means.append(mean_bgr)
    if not region_means:
        return 0.0, 0.0, 0.0
    bgr = np.mean(region_means, axis=0)
    return float(bgr[2]), float(bgr[1]), float(bgr[0])


def empty_bpm_debug(sample_count: int = 0, reason: str = "collecting") -> dict[str, object]:
    return {
        "sample_count": sample_count,
        "duration_sec": 0.0,
        "peak_bpm": None,
        "peak_power": None,
        "band_median_power": None,
        "peak_snr": None,
        "signal_std": None,
        "signal_range": None,
        "raw_bpm": None,
        "selected_bpm": None,
        "signal_method": None,
        "candidate_score": None,
        "candidate_methods": [],
        "selection_reason": "",
        "support_bpm": None,
        "escape_bpm": None,
        "escape_count": 0,
        "tracker_reason": reason,
        "top_peaks": [],
        "reason": reason,
    }


def detrend_signal(signal: np.ndarray, target_fps: float) -> np.ndarray:
    signal = np.asarray(signal, dtype=np.float64)
    if signal.size == 0:
        return signal
    centered = signal - np.mean(signal)
    if centered.size < 5:
        return centered
    window = max(3, int(round(target_fps * 1.6)))
    if window >= centered.size:
        window = centered.size - 1 if centered.size % 2 == 0 else centered.size
    if window < 3:
        return centered
    if window % 2 == 0:
        window -= 1
    baseline = np.convolve(centered, np.ones(window, dtype=np.float64) / float(window), mode="same")
    detrended = centered - baseline
    detrended = detrended - np.mean(detrended)
    std = float(np.std(detrended))
    if std > 1e-6:
        detrended = detrended / std
    return detrended

def bandpass_filter(data, fs, lowcut=0.7, highcut=3.0, order=2):
    """
    fs: 采样频率 (target_fps)
    lowcut: 42 BPM (0.7Hz)
    highcut: 180 BPM (3.0Hz)
    """
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    # 使用 filtfilt 进行零相位滤波，避免信号相位偏移
    return filtfilt(b, a, data)



def build_rppg_signals(rgb: np.ndarray, target_fps: float) -> dict[str, np.ndarray]:
    means = np.mean(rgb, axis=0, keepdims=True) + 1e-6
    normalized = rgb / means - 1.0
    
    # 原始信号提取
    green_raw = normalized[:, 1]
    
    pos_x = normalized[:, 1] - normalized[:, 2]
    pos_y = -2.0 * normalized[:, 0] + normalized[:, 1] + normalized[:, 2]
    pos_alpha = float(np.std(pos_x) / (np.std(pos_y) + 1e-6))
    pos_raw = pos_x + pos_alpha * pos_y

    chrom_x = 3.0 * normalized[:, 0] - 2.0 * normalized[:, 1]
    chrom_y = 1.5 * normalized[:, 0] + normalized[:, 1] - 1.5 * normalized[:, 2]
    chrom_alpha = float(np.std(chrom_x) / (np.std(chrom_y) + 1e-6))
    chrom_raw = chrom_x - chrom_alpha * chrom_y

    # 应用带通滤波替代简单的 detrend
    return {
        "green": bandpass_filter(detrend_signal(green_raw, target_fps), target_fps),
        "pos": bandpass_filter(detrend_signal(pos_raw, target_fps), target_fps),
        "chrom": bandpass_filter(detrend_signal(chrom_raw, target_fps), target_fps)
    }


def analyze_signal_spectrum(
    signal: np.ndarray,
    *,
    target_fps: float,
    min_bpm: float,
    max_bpm: float,
) -> dict[str, object]:
    valid_result: dict[str, object] = {
        "signal_std": round(float(np.std(signal)), 6) if signal.size else None,
        "signal_range": round(float(np.max(signal) - np.min(signal)), 6) if signal.size else None,
        "median_power": None,
        "top_peaks": [],
        "top1": None,
    }
    if signal.size < MIN_SAMPLE_COUNT:
        return valid_result
    
    n_fft = 1024 
    windowed = signal * np.hanning(len(signal))
    spectrum = np.abs(np.fft.rfft(windowed, n=n_fft))
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / target_fps)

    min_hz = min_bpm / 60.0
    max_hz = max_bpm / 60.0
    valid = np.where((freqs >= min_hz) & (freqs <= max_hz))[0]
    if len(valid) == 0:
        return valid_result
    median_power = float(np.median(spectrum[valid]))
    valid_result["median_power"] = round(median_power, 6)
    sorted_valid = list(valid[np.argsort(spectrum[valid])[::-1]])
    top_peaks: list[dict[str, float]] = []
    for peak in sorted_valid:
        peak_bpm = float(freqs[peak] * 60.0)
        if any(abs(peak_bpm - float(item["bpm"])) < PEAK_MIN_SEPARATION_BPM for item in top_peaks):
            continue
        peak_power = float(spectrum[peak])
        top_peaks.append(
            {
                "bpm": round(peak_bpm, 3),
                "power": round(peak_power, 6),
                "snr": round(peak_power / (median_power + 1e-6), 3),
            }
        )
        if len(top_peaks) >= TOP_PEAK_COUNT:
            break
    valid_result["top_peaks"] = top_peaks
    valid_result["top1"] = top_peaks[0] if top_peaks else None
    return valid_result


def best_peak_near_bpm(top_peaks: list[dict[str, float]], target_bpm: float, tolerance_bpm: float) -> Optional[dict[str, float]]:
    matches = [peak for peak in top_peaks if abs(float(peak["bpm"]) - target_bpm) <= tolerance_bpm]
    if not matches:
        return None
    return max(matches, key=lambda peak: (float(peak["snr"]), float(peak["power"])))


def score_peak_candidate(
    peak: dict[str, float],
    *,
    method_name: str,
    method_peaks: list[dict[str, float]],
    min_bpm: float,
) -> tuple[float, list[str]]:
    bpm = float(peak["bpm"])
    snr = float(peak["snr"])
    score = snr + METHOD_SCORE_BONUS.get(method_name, 0.0)
    reasons = [method_name]

    if bpm < LOW_BPM_SUBHARMONIC_MAX:
        doubled = best_peak_near_bpm(method_peaks, bpm * 2.0, 10.0)
        if doubled and float(doubled["snr"]) >= snr * LOW_BPM_SUBHARMONIC_RATIO:
            score *= 0.45
            reasons.append("subharmonic_penalty")

    if bpm <= min_bpm + LOW_BPM_BOUNDARY_MARGIN:
        high_alt = max(
            (
                candidate
                for candidate in method_peaks
                if float(candidate["bpm"]) >= max(HIGH_BPM_HARMONIC_MIN, bpm * 1.7)
            ),
            key=lambda candidate: float(candidate["snr"]),
            default=None,
        )
        if high_alt and float(high_alt["snr"]) >= snr * LOW_BPM_BOUNDARY_ALT_RATIO:
            score *= 0.4
            reasons.append("boundary_penalty")

    if bpm >= HIGH_BPM_HARMONIC_MIN:
        half_peak = best_peak_near_bpm(method_peaks, bpm * 0.5, 8.0)
        if half_peak and float(half_peak["snr"]) >= snr * HIGH_BPM_HALF_SUPPORT_RATIO:
            score *= 1.18
            reasons.append("half_support")

    return score, reasons


def pick_variant_bpm(
    variant: str,
    *,
    method_results: dict[str, dict[str, object]],
    min_bpm: float,
) -> tuple[Optional[float], dict[str, object]]:
    if variant.endswith("_harmonic"):
        method_name = variant.replace("_harmonic", "")
        result = method_results[method_name]
        candidates = []
        for peak in result.get("top_peaks", [])[:3]:
            score, reasons = score_peak_candidate(
                peak,
                method_name=method_name,
                method_peaks=result.get("top_peaks", []),
                min_bpm=min_bpm,
            )
            candidates.append((float(score), float(peak["snr"]), peak, reasons))
        if not candidates:
            return None, {
                "signal_method": method_name,
                "selection_reason": "no_peak",
                "top_peaks": result.get("top_peaks", [])[:3],
                "raw_bpm": None,
                "candidate_score": None,
                "candidate_methods": [method_name],
            }
        score, _, peak, reasons = max(candidates, key=lambda item: (item[0], item[1]))
        return float(peak["bpm"]), {
            "signal_method": method_name,
            "selection_reason": "+".join(reasons),
            "top_peaks": result.get("top_peaks", [])[:3],
            "raw_bpm": float(peak["bpm"]),
            "candidate_score": round(float(score), 3),
            "candidate_methods": [method_name],
        }

    candidates: list[dict[str, object]] = []
    for method_name, result in method_results.items():
        for peak in result.get("top_peaks", [])[:3]:
            score, reasons = score_peak_candidate(
                peak,
                method_name=method_name,
                method_peaks=result.get("top_peaks", []),
                min_bpm=min_bpm,
            )
            candidates.append(
                {
                    "bpm": float(peak["bpm"]),
                    "score": float(score),
                    "method": method_name,
                    "peak": peak,
                    "reasons": reasons,
                }
            )

    if not candidates:
        return None, {
            "signal_method": None,
            "selection_reason": "no_peak",
            "top_peaks": [],
            "raw_bpm": None,
            "candidate_score": None,
            "candidate_methods": [],
        }

    clusters: list[dict[str, object]] = []
    for entry in sorted(candidates, key=lambda item: (float(item["score"]), float(item["peak"]["snr"])), reverse=True):
        cluster = next((item for item in clusters if abs(float(item["bpm"]) - float(entry["bpm"])) <= PEAK_MIN_SEPARATION_BPM), None)
        if cluster is None:
            clusters.append(
                {
                    "bpm": float(entry["bpm"]),
                    "score": float(entry["score"]),
                    "weight_sum": float(entry["score"]),
                    "weighted_bpm": float(entry["bpm"]) * float(entry["score"]),
                    "methods": {str(entry["method"])},
                    "entries": [entry],
                    "primary": entry,
                }
            )
            continue
        cluster["score"] = float(cluster["score"]) + float(entry["score"])
        cluster["weight_sum"] = float(cluster["weight_sum"]) + float(entry["score"])
        cluster["weighted_bpm"] = float(cluster["weighted_bpm"]) + float(entry["bpm"]) * float(entry["score"])
        cluster["methods"].add(str(entry["method"]))
        cluster["entries"].append(entry)
        primary = cluster["primary"]
        if (float(entry["score"]), float(entry["peak"]["snr"])) > (float(primary["score"]), float(primary["peak"]["snr"])):
            cluster["primary"] = entry

    for cluster in clusters:
        cluster["bpm"] = float(cluster["weighted_bpm"]) / max(float(cluster["weight_sum"]), 1e-6)
        cluster["score"] = float(cluster["score"]) * (1.0 + 0.12 * (len(cluster["methods"]) - 1))

    best_cluster = max(clusters, key=lambda item: (float(item["score"]), float(item["primary"]["peak"]["snr"])))
    primary_entry = best_cluster["primary"]
    selected_method = str(primary_entry["method"])
    selected_result = method_results[selected_method]
    return float(best_cluster["bpm"]), {
        "signal_method": selected_method,
        "selection_reason": "+".join(primary_entry["reasons"]),
        "top_peaks": selected_result.get("top_peaks", [])[:3],
        "raw_bpm": round(float(best_cluster["bpm"]), 3),
        "candidate_score": round(float(best_cluster["score"]), 3),
        "candidate_methods": sorted(best_cluster["methods"]),
    }


def rough_bpm(
    samples: list[dict[str, float]],
    *,
    target_fps: float = DEFAULT_TARGET_FPS,
    min_bpm: float = DEFAULT_MIN_BPM,
    max_bpm: float = DEFAULT_MAX_BPM,
    selection_variant: str = DEFAULT_SELECTION_VARIANT,
) -> tuple[Optional[float], str, dict[str, object]]:
    debug = empty_bpm_debug(len(samples), "collecting")
    if len(samples) < MIN_SAMPLE_COUNT:
        return None, "collecting", debug
    times = np.array([s["t"] for s in samples], dtype=np.float64)
    green = np.array([s["g"] for s in samples], dtype=np.float64)
    duration = times[-1] - times[0]
    debug["duration_sec"] = round(float(duration), 3)
    if duration < MIN_ANALYSIS_SECONDS:
        return None, "collecting", debug
    grid = np.arange(times[0], times[-1], 1.0 / target_fps)
    if len(grid) < MIN_SAMPLE_COUNT:
        return None, "collecting", debug
    red = np.array([s["r"] for s in samples], dtype=np.float64)
    blue = np.array([s["b"] for s in samples], dtype=np.float64)
    if max_bpm <= min_bpm:
        debug["reason"] = "invalid_range"
        return None, "invalid_range", debug
    rgb = np.stack(
        [
            np.interp(grid, times, red),
            np.interp(grid, times, green),
            np.interp(grid, times, blue),
        ],
        axis=1,
    )
    signal_map = build_rppg_signals(rgb, target_fps)
    method_results: dict[str, dict[str, object]] = {}

    for method_name, signal in signal_map.items():
        result = analyze_signal_spectrum(signal, target_fps=target_fps, min_bpm=min_bpm, max_bpm=max_bpm)
        method_results[method_name] = result

    selected_bpm, selection = pick_variant_bpm(
        selection_variant,
        method_results=method_results,
        min_bpm=min_bpm,
    )
    if selected_bpm is None:
        debug["reason"] = "no_peak"
        return None, "no_frequency", debug
    selected_method = selection.get("signal_method")
    selected_result = method_results[selected_method] if selected_method in method_results else method_results["green"]
    debug.update(
        {
            "signal_std": method_results["green"].get("signal_std"),
            "signal_range": method_results["green"].get("signal_range"),
            "peak_bpm": selected_result.get("top1", {}).get("bpm") if selected_result.get("top1") else None,
            "peak_power": selected_result.get("top1", {}).get("power") if selected_result.get("top1") else None,
            "band_median_power": selected_result.get("median_power"),
            "peak_snr": selected_result.get("top1", {}).get("snr") if selected_result.get("top1") else None,
            "raw_bpm": selection.get("raw_bpm"),
            "selected_bpm": round(float(selected_bpm), 3),
            "signal_method": selected_method,
            "candidate_score": selection.get("candidate_score"),
            "candidate_methods": selection.get("candidate_methods", []),
            "selection_reason": selection.get("selection_reason", ""),
            "top_peaks": selection.get("top_peaks", [])[:3],
            "reason": "rough",
        }
    )
    return float(selected_bpm), "rough", debug


class VoiceManager:
    def __init__(self, device="plughw:0,0"):
        self.device = device
        # 状态标志
        self.played_wait = False
        self.played_ok = False
        self.last_ready_time = 0.0

        # 计时器
        self.face_start_time = None
        self.hr_start_time = None

        # TTS 周期性播报
        self._latest_bpm = None
        self._last_tts_time = 0.0
        self._tts_active = False

        # 测量稳定性容错
        self._last_valid_measurement_time = None  # 记录最后一次有效测量时间
        self._collecting_timeout = 1.0  # 允许的 collecting 状态最大持续时间

        # 锁，防止多个语音进程冲突
        self._lock = threading.Lock()

    def play_async(self, wav_path, callback=None):
        """开辟新线程播放语音，不阻塞主程序。callback 在播放完成后调用（锁外）。"""
        def _play():
            with self._lock:
                if os.path.exists(wav_path):
                    subprocess.run(["aplay", "-D", self.device, wav_path],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if callback:
                callback()
        threading.Thread(target=_play, daemon=True).start()

    def update(self, face_count, frontal_score, bpm, bpm_state):
        now = time.time()

        # 缓存最新有效心率，供 TTS 周期播报使用
        if bpm is not None:
            self._latest_bpm = bpm

        # 场景 1: 未检测到脸 或 脸未正对摄像头
        if face_count <= 0 or frontal_score < 0.35:
            self.face_start_time = None
            self.hr_start_time = None
            self.played_wait = False
            self.played_ok = False
            self._tts_active = False
            self._last_tts_time = 0.0
            self._last_valid_measurement_time = None

            if now - self.last_ready_time > 6.0:
                print("提示：请对准摄像头", flush=True)
                self.play_async(VOICE_READY)
                self.last_ready_time = now
            return

        # 场景 2: 检测到脸
        if self.face_start_time is None:
            self.face_start_time = now

        if not self.played_wait and (now - self.face_start_time >= 2.0):
            print("状态：检测到人脸，开始测量...")
            self.play_async(VOICE_WAIT)
            self.played_wait = True

        # 场景 3: 确认得出有效心率
        if bpm is not None and bpm_state != "collecting":
            # 有效测量状态
            if self.hr_start_time is None:
                self.hr_start_time = now
            self._last_valid_measurement_time = now  # 记录最后一次有效测量时间

            if not self.played_ok and (now - self.hr_start_time >= 1.5):
                print(f"成功：测量完成，当前心率: {bpm}")
                self.played_ok = True

                # VOICE_OK 播完后触发首次 TTS，回调执行时才读取最新心率
                self.play_async(VOICE_OK, callback=lambda: self._tts_say_bpm(self._latest_bpm))
                self._last_tts_time = now
                self._tts_active = True

            # 每 8 秒周期播报最新心率
            if self._tts_active and self._latest_bpm is not None:
                if now - self._last_tts_time >= 8.0:
                    self._tts_say_bpm(self._latest_bpm)
                    self._last_tts_time = now

        elif bpm is not None and self._last_valid_measurement_time is not None:
            # bpm 有值但 bpm_state == "collecting"
            # 如果 collecting 状态持续时间超过阈值，才重置计时器
            if (now - self._last_valid_measurement_time) > self._collecting_timeout:
                self.hr_start_time = None
                self._last_valid_measurement_time = None
            # 否则保持 hr_start_time 不变，允许短暂波动

        else:
            # bpm 为 None，完全重置
            self.hr_start_time = None
            self._last_valid_measurement_time = None

    def _tts_say_bpm(self, bpm):
        if bpm is None:
            return
        hr = int(round(bpm))
        text = f"您当前的心率为{hr}。"
        tts_say_async(text)

class SharedState:
    def __init__(self, max_signal_seconds: float) -> None:
        self.condition = threading.Condition()
        self.jpeg: Optional[bytes] = None
        self.seq = 0
        self.width = 0
        self.height = 0
        self.encoding = ""
        self.input_fps = 0.0
        self.last_update = 0.0
        self.last_error = ""
        self.face_count = 0
        self.frontal_score = 0.0
        self.roi_count = 0
        self.rgb = {"r": 0.0, "g": 0.0, "b": 0.0}
        self.source_mode = "camera"
        self.source_name = ""
        self.frame_idx = 0
        self.total_frames = 0
        self.frame_time_sec = 0.0
        self.duration_sec = 0.0
        self.video_fps = 0.0
        self.video_loop = False
        self.bpm: Optional[float] = None
        self.last_valid_bpm: Optional[float] = None  # 新增：记录最后一次有效心率
        self.bpm_state = "collecting"
        self.bpm_debug: dict[str, object] = empty_bpm_debug()
        self._pending_jump_bpm: Optional[float] = None
        self._pending_jump_count = 0
        self._escape_bpm: Optional[float] = None
        self._escape_count = 0
        self.reference_hr: dict[str, object] = {
            "source": "",
            "bpm": None,
            "received_at": None,
            "device_timestamp": None,
        }
        self.params_config: dict[str, float] = {
            "signal_seconds": float(max_signal_seconds),
            "target_fps": DEFAULT_TARGET_FPS,
            "min_bpm": DEFAULT_MIN_BPM,
            "max_bpm": DEFAULT_MAX_BPM,
        }
        self.samples: list[dict[str, float]] = []
        self._timestamps: list[float] = []
        self._last_valid_roi_at = 0.0
        self.voice = VoiceManager(device=get_usb_audio_device())

    def configure_source(
        self,
        *,
        source_mode: str,
        source_name: str = "",
        total_frames: int = 0,
        duration_sec: float = 0.0,
        video_fps: float = 0.0,
        video_loop: bool = False,
    ) -> None:
        with self.condition:
            self.source_mode = source_mode
            self.source_name = source_name
            self.total_frames = total_frames
            self.duration_sec = duration_sec
            self.video_fps = video_fps
            self.video_loop = video_loop

    def _params_locked(self) -> dict[str, float]:
        return {
            "signal_seconds": round(float(self.params_config["signal_seconds"]), 2),
            "target_fps": round(float(self.params_config["target_fps"]), 2),
            "min_bpm": round(float(self.params_config["min_bpm"]), 1),
            "max_bpm": round(float(self.params_config["max_bpm"]), 1),
        }

    def params(self) -> dict[str, float]:
        with self.condition:
            return self._params_locked()

    def _trim_samples_locked(self, now: float) -> None:
        cutoff = now - float(self.params_config["signal_seconds"])
        self.samples = [s for s in self.samples if s["t"] >= cutoff]

    def _recompute_bpm_locked(self) -> None:
        raw_bpm, raw_state, raw_debug = rough_bpm(
            self.samples,
            target_fps=float(self.params_config["target_fps"]),
            min_bpm=float(self.params_config["min_bpm"]),
            max_bpm=float(self.params_config["max_bpm"]),
            selection_variant=DEFAULT_SELECTION_VARIANT,
        )
        self.bpm, self.bpm_state, self.bpm_debug = self._stabilize_bpm_locked(raw_bpm, raw_state, raw_debug)

    def _stabilize_bpm_locked(
        self,
        raw_bpm: Optional[float],
        raw_state: str,
        raw_debug: dict[str, object],
    ) -> tuple[Optional[float], str, dict[str, object]]:
        
        # --- 策略 1: 如果 FFT 没测出结果，直接返回上一次的值，不让界面变横线，现在又给这个改掉了 ---
        if raw_bpm is None:
            self._pending_jump_bpm = None
            self._pending_jump_count = 0
            self._escape_bpm = None
            self._escape_count = 0
            raw_debug["selected_bpm"] = self.bpm # 维持旧值
            raw_debug["tracker_reason"] = "keep_last_valid"
            return self.bpm, raw_state, raw_debug

        selected_bpm = float(raw_bpm)

        # --- 策略 2: 硬上限过滤。 ---
        if selected_bpm > 110.0:
            raw_debug["tracker_reason"] = "reject_high_outlier"
            return self.bpm, "high_outlier", raw_debug

        previous_bpm = self.bpm
        
        # 准备寻找候选峰值（原有逻辑保持）
        peaks = [
            item
            for item in raw_debug.get("top_peaks", [])
            if isinstance(item, dict) and isinstance(item.get("bpm"), (int, float))
        ]
        support_peak: Optional[dict[str, float]] = None
        dominant_peak: Optional[dict[str, float]] = peaks[0] if peaks else None

        # 跟踪逻辑
        if previous_bpm is not None:
            # --- 策略 3: 剧烈跳变拦截。---
            if abs(selected_bpm - previous_bpm) > 12.0:
                raw_debug["tracker_reason"] = "reject_huge_jump"
                return previous_bpm, "tracking", raw_debug

            # 原有的寻峰逻辑：尝试找前一次心率附近的峰值
            if peaks:
                strongest_power = float(peaks[0].get("power") or 0.0)
                nearby_peaks = [p for p in peaks if abs(float(p["bpm"]) - previous_bpm) <= TRACK_NEAR_BPM]
                if nearby_peaks:
                    support_peak = min(nearby_peaks, key=lambda p: abs(float(p["bpm"]) - previous_bpm))
                    raw_debug["support_bpm"] = round(float(support_peak["bpm"]), 3)
                    # 如果主峰跳太远，但附近有小峰，就选小峰
                    if abs(selected_bpm - previous_bpm) > TRACK_MAX_JUMP_BPM:
                        if float(support_peak.get("power") or 0.0) >= strongest_power * 0.5:
                            selected_bpm = float(support_peak["bpm"])

            # --- 策略 4: 小幅度跳变确认逻辑 ---
            jump = selected_bpm - previous_bpm
            if abs(jump) > TRACK_MAX_JUMP_BPM:
                if self._pending_jump_bpm is not None and abs(selected_bpm - self._pending_jump_bpm) <= PEAK_MIN_SEPARATION_BPM:
                    self._pending_jump_count += 1
                else:
                    self._pending_jump_bpm = selected_bpm
                    self._pending_jump_count = 1
                
                # 如果没达到连续确认次数，就锁定旧值不动
                if self._pending_jump_count < TRACK_CONFIRM_FRAMES:
                    raw_debug["selected_bpm"] = round(previous_bpm, 3)
                    raw_debug["tracker_reason"] = f"hold_jump_{self._pending_jump_count}"
                    return previous_bpm, "tracking", raw_debug
                
                self._pending_jump_bpm = None
                self._pending_jump_count = 0
            else:
                self._pending_jump_bpm = None
                self._pending_jump_count = 0
                
                # 自适应平滑（利用之前教你的 SNR）
                current_snr = float(raw_debug.get("peak_snr") or 1.0)
                snr_factor = max(0.5, min(current_snr, 3.0)) / 2.0
                adaptive_weight = TRACK_SMOOTHING * snr_factor
                selected_bpm = previous_bpm * (1.0 - adaptive_weight) + selected_bpm * adaptive_weight

        raw_debug["selected_bpm"] = round(selected_bpm, 3)
        raw_debug["tracker_reason"] = "stable_track"
        return selected_bpm, "tracking" if previous_bpm is not None else raw_state, raw_debug

    def update_params(
        self,
        *,
        signal_seconds: Optional[float] = None,
        min_bpm: Optional[float] = None,
        max_bpm: Optional[float] = None,
    ) -> dict[str, float]:
        with self.condition:
            next_signal_seconds = float(self.params_config["signal_seconds"] if signal_seconds is None else signal_seconds)
            next_min_bpm = float(self.params_config["min_bpm"] if min_bpm is None else min_bpm)
            next_max_bpm = float(self.params_config["max_bpm"] if max_bpm is None else max_bpm)

            if next_signal_seconds < MIN_SIGNAL_SECONDS or next_signal_seconds > MAX_SIGNAL_SECONDS:
                raise ValueError(f"signal_seconds must be between {MIN_SIGNAL_SECONDS:.0f} and {MAX_SIGNAL_SECONDS:.0f}")
            if next_min_bpm < 30.0 or next_min_bpm > 180.0:
                raise ValueError("min_bpm must be between 30 and 180")
            if next_max_bpm < 60.0 or next_max_bpm > 240.0:
                raise ValueError("max_bpm must be between 60 and 240")
            if next_max_bpm <= next_min_bpm:
                raise ValueError("max_bpm must be greater than min_bpm")

            self.params_config["signal_seconds"] = next_signal_seconds
            self.params_config["min_bpm"] = next_min_bpm
            self.params_config["max_bpm"] = next_max_bpm
            now = time.time()
            self._trim_samples_locked(now)
            if self.samples:
                self._recompute_bpm_locked()
            else:
                self.bpm = None
                self.bpm_state = "collecting"
                self.bpm_debug = empty_bpm_debug()
                self._pending_jump_bpm = None
                self._pending_jump_count = 0
                self._escape_bpm = None
                self._escape_count = 0
            return self._params_locked()

    def update(
        self,
        jpeg: bytes,
        width: int,
        height: int,
        encoding: str,
        face_count: int,
        frontal_score: float,
        roi_count: int,
        rgb: tuple[float, float, float],
        *,
        frame_idx: Optional[int] = None,
        frame_time_sec: Optional[float] = None,
    ) -> None:
        now = time.time()
        r, g, b = rgb
        with self.condition:
            self.jpeg = jpeg
            self.seq += 1
            self.width = width
            self.height = height
            self.encoding = encoding
            self.face_count = face_count
            self.frontal_score = frontal_score
            self.roi_count = roi_count
            self.rgb = {"r": round(r, 3), "g": round(g, 3), "b": round(b, 3)}
            if frame_idx is not None:
                self.frame_idx = int(frame_idx)
            if frame_time_sec is not None:
                self.frame_time_sec = float(frame_time_sec)
            self.last_error = ""
            self.last_update = now
            self._timestamps.append(now)
            self._timestamps = [t for t in self._timestamps if now - t <= 3.0]
            elapsed = self._timestamps[-1] - self._timestamps[0] if len(self._timestamps) > 1 else 0.0
            self.input_fps = (len(self._timestamps) - 1) / elapsed if elapsed > 0 else 0.0
            if face_count > 0 and frontal_score >= 0.35 and roi_count > 0:
                self._last_valid_roi_at = now
                self.samples.append({"t": now, "r": r, "g": g, "b": b})
                self._trim_samples_locked(now)
                self._recompute_bpm_locked()
            elif self._last_valid_roi_at and (now - self._last_valid_roi_at) >= 1.5:
                self.samples = []
                self.bpm = None
                self.bpm_state = "no_face"
                self.bpm_debug = empty_bpm_debug(reason="no_face")
            self.condition.notify_all()
            self.voice.update(self.face_count, self.frontal_score, self.bpm, self.bpm_state)

    def set_error(self, error: str) -> None:
        with self.condition:
            self.last_error = error

    def update_reference_hr(self, bpm: float, source: str = "mi_band_7", device_timestamp: Optional[float] = None) -> None:
        now = time.time()
        with self.condition:
            self.reference_hr = {
                "source": source or "mi_band_7",
                "bpm": round(float(bpm), 1),
                "received_at": now,
                "device_timestamp": device_timestamp,
            }

    def _reference_status_locked(self) -> dict[str, object]:
        received_at = self.reference_hr.get("received_at")
        age = time.time() - float(received_at) if received_at else None
        bpm = self.reference_hr.get("bpm")
        return {
            "source": self.reference_hr.get("source") or "",
            "bpm": bpm,
            "age_sec": round(age, 2) if age is not None else None,
            "fresh": bool(age is not None and age <= 30.0),
            "device_timestamp": self.reference_hr.get("device_timestamp"),
        }

    def reference_status(self) -> dict[str, object]:
        with self.condition:
            return self._reference_status_locked()

    def status(self) -> dict[str, object]:
        with self.condition:
            age = time.time() - self.last_update if self.last_update else None
            buffer_seconds = self.samples[-1]["t"] - self.samples[0]["t"] if len(self.samples) > 1 else 0.0
            input_fps = self.input_fps if age is not None and age <= 3.0 else 0.0
            reference = self._reference_status_locked()
            reference_bpm = reference.get("bpm") if reference.get("fresh") else None
            progress = (self.frame_idx / self.total_frames) if self.total_frames > 0 else None
            
            active_bpm = self.bpm
            active_bpm_state = self.bpm_state
            active_bpm_debug = self.bpm_debug

            # 人脸丢失或未正对时，BPM 返回 None，前端显示 "-"
            if self.face_count <= 0 or self.roi_count <= 0 or self.frontal_score < 0.35:
                active_bpm = None
                active_bpm_state = "no_face"
                active_bpm_debug = dict(self.bpm_debug)
                active_bpm_debug["selected_bpm"] = None
                active_bpm_debug["tracker_reason"] = "no_face"
                if active_bpm_debug.get("reason") == "rough":
                    active_bpm_debug["reason"] = "no_face"

            bpm_delta = None
            if active_bpm is not None and isinstance(reference_bpm, (int, float)):
                bpm_delta = round(active_bpm - float(reference_bpm), 1)

            # 语音提示文字
            if self.face_count <= 0 or self.frontal_score < 0.35:
                voice_prompt = "请对准摄像头"
            elif self.voice.played_ok:
                voice_prompt = "心率测量成功"
            else:
                voice_prompt = "请稍后"

            return {
                "ok": self.jpeg is not None,
                "seq": self.seq,
                "width": self.width,
                "height": self.height,
                "encoding": self.encoding,
                "source_mode": self.source_mode,
                "source_name": self.source_name,
                "frame_idx": self.frame_idx,
                "total_frames": self.total_frames,
                "time_sec": round(self.frame_time_sec, 3),
                "duration_sec": round(self.duration_sec, 3),
                "progress": round(progress, 4) if progress is not None else None,
                "video_fps": round(self.video_fps, 3) if self.video_fps else None,
                "video_loop": self.video_loop,
                "input_fps": round(input_fps, 2),
                "age_sec": round(age, 3) if age is not None else None,
                "face_count": self.face_count,
                "frontal_score": round(self.frontal_score, 3),
                "roi_count": self.roi_count,
                "rgb": self.rgb,
                "buffer_seconds": round(buffer_seconds, 2),
                "bpm": round(active_bpm, 1) if active_bpm is not None else None,
                "bpm_state": active_bpm_state,
                "bpm_debug": active_bpm_debug,
                "params": self._params_locked(),
                "reference_hr": reference,
                "bpm_delta": bpm_delta,
                "voice_prompt": voice_prompt,
                "last_error": self.last_error,
            }

    def signal(self) -> dict[str, object]:
        with self.condition:
            if not self.samples:
                return {"samples": []}
            t0 = self.samples[-1]["t"]
            samples = [
                {
                    "t": round(s["t"] - t0, 3),
                    "r": round(s["r"], 3),
                    "g": round(s["g"], 3),
                    "b": round(s["b"], 3),
                }
                for s in self.samples[-240:]
            ]
            return {"samples": samples}


class FrameProcessor:
    def __init__(self, resize_width: int, jpeg_quality: int, mesh_input_width: int = DEFAULT_MESH_INPUT_WIDTH) -> None:
        self.resize_width = resize_width
        self.jpeg_quality = jpeg_quality
        self.mesh_input_width = mesh_input_width
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.mesh_spec = self.mp_drawing.DrawingSpec(color=(60, 220, 90), thickness=1, circle_radius=1)
        self.contour_spec = self.mp_drawing.DrawingSpec(color=(40, 190, 255), thickness=1, circle_radius=1)

    def close(self) -> None:
        self.face_mesh.close()

    def build_rois(self, landmarks, width: int, height: int) -> list[tuple[str, np.ndarray]]:
        rois: list[tuple[str, np.ndarray]] = []
        for name, indices in ROI_POLYGONS.items():
            pts = np.array(
                [(int(landmarks.landmark[i].x * width), int(landmarks.landmark[i].y * height)) for i in indices],
                dtype=np.int32,
            )
            rois.append((name, pts))
        return rois

    def process_frame(self, frame: np.ndarray, encoding: str) -> dict[str, object]:
        if self.resize_width > 0 and frame.shape[1] > self.resize_width:
            scale = self.resize_width / frame.shape[1]
            frame = cv2.resize(frame, (self.resize_width, int(frame.shape[0] * scale)))
        sample_frame = frame.copy()
        h, w = frame.shape[:2]
        face_count = 0
        frontal_score = 0.0
        roi_polygons: list[np.ndarray] = []
        rgb_means = (0.0, 0.0, 0.0)

        mesh_frame = frame
        if self.mesh_input_width > 0 and frame.shape[1] > self.mesh_input_width:
            scale = self.mesh_input_width / frame.shape[1]
            mesh_frame = cv2.resize(frame, (self.mesh_input_width, int(frame.shape[0] * scale)))
        mesh_rgb = cv2.cvtColor(mesh_frame, cv2.COLOR_BGR2RGB)
        mesh_rgb.flags.writeable = False
        results = self.face_mesh.process(mesh_rgb)
        if results.multi_face_landmarks:
            face_count = len(results.multi_face_landmarks)
            landmarks = results.multi_face_landmarks[0]

            # 人脸朝向检测：鼻子在脸部bbox中的水平位置比例
            nose_tip = landmarks.landmark[1]
            nose_x = int(nose_tip.x * w)
            xs = [int(lm.x * w) for lm in landmarks.landmark]
            x1, x2 = min(xs), max(xs)
            # 0.5=正脸, <0.35偏左, >0.65偏右
            nose_ratio = (nose_x - x1) / max(x2 - x1, 1)
            frontal_score = 1.0 - 2.0 * abs(nose_ratio - 0.5)

            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=landmarks,
                connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mesh_spec,
            )
            self.mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=landmarks,
                connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.contour_spec,
            )
            for name, pts in self.build_rois(landmarks, w, h):
                roi_polygons.append(pts)
                x, y, _, _ = cv2.boundingRect(pts)
                cv2.polylines(frame, [pts], isClosed=True, color=(255, 220, 60), thickness=2)
                cv2.putText(frame, name, (x, max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 220, 60), 1, cv2.LINE_AA)
            rgb_means = mean_rgb_polygons(sample_frame, roi_polygons)

        self.draw_overlay(frame, face_count, len(roi_polygons), rgb_means)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return {
            "jpeg": encoded.tobytes(),
            "width": w,
            "height": h,
            "encoding": encoding,
            "face_count": face_count,
            "frontal_score": frontal_score,
            "roi_count": len(roi_polygons),
            "rgb_means": rgb_means,
        }

    def draw_overlay(self, frame: np.ndarray, face_count: int, roi_count: int, rgb_means: tuple[float, float, float]) -> None:
        r, g, b = rgb_means
        lines = [
            f"faces: {face_count}  roi: {roi_count}",
            f"RGB mean: R {r:.1f}  G {g:.1f}  B {b:.1f}",
        ]
        y = 28
        for line in lines:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 4, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 255, 120), 2, cv2.LINE_AA)
            y += 28


class CameraSourceNode(Node):
    def __init__(self, args: argparse.Namespace, shared: SharedState, processor: FrameProcessor) -> None:
        super().__init__("rdkx5_rppg_runtime")
        self.shared = shared
        self.processor = processor
        self.sub = self.create_subscription(Image, args.topic, self.on_image, 10)

    def on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            result = self.processor.process_frame(frame, msg.encoding)
            self.shared.update(
                result["jpeg"],
                int(result["width"]),
                int(result["height"]),
                str(result["encoding"]),
                int(result["face_count"]),
                float(result["frontal_score"]),
                int(result["roi_count"]),
                result["rgb_means"],
            )
        except Exception as exc:
            self.shared.set_error(str(exc))


class VideoFileRunner:
    def __init__(self, args: argparse.Namespace, shared: SharedState, processor: FrameProcessor) -> None:
        self.args = args
        self.shared = shared
        self.processor = processor
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, name="rdkx5-video-source", daemon=True)
        self.gt_times: Optional[np.ndarray] = None
        self.gt_hr: Optional[np.ndarray] = None
        if args.video_gt_path:
            gt_path = Path(args.video_gt_path)
            if gt_path.exists():
                self.gt_times, self.gt_hr = load_ground_truth(gt_path)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5.0)

    def run(self) -> None:
        cap = cv2.VideoCapture(str(self.args.video_path))
        if not cap.isOpened():
            self.shared.set_error(f"failed to open video: {self.args.video_path}")
            return
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0) or 30.0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        duration_sec = (total_frames / fps) if total_frames > 0 and fps > 0 else 0.0
        self.shared.configure_source(
            source_mode="video",
            source_name=Path(self.args.video_path).name,
            total_frames=total_frames,
            duration_sec=duration_sec,
            video_fps=fps,
            video_loop=self.args.video_loop,
        )
        frame_duration = 1.0 / max(fps, 1.0)
        try:
            while not self.stop_event.is_set():
                started = time.time()
                ok, frame = cap.read()
                if not ok:
                    if self.args.video_loop:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    self.shared.set_error("video playback finished")
                    time.sleep(0.2)
                    continue
                frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES) or 0)
                time_sec = float(cap.get(cv2.CAP_PROP_POS_MSEC) or 0.0) / 1000.0
                result = self.processor.process_frame(frame, "bgr8")
                self.shared.update(
                    result["jpeg"],
                    int(result["width"]),
                    int(result["height"]),
                    str(result["encoding"]),
                    int(result["face_count"]),
                    float(result["frontal_score"]),
                    int(result["roi_count"]),
                    result["rgb_means"],
                    frame_idx=frame_idx,
                    frame_time_sec=time_sec,
                )
                if self.gt_times is not None and self.gt_hr is not None:
                    gt_bpm = interpolate_ground_truth(self.gt_times, self.gt_hr, time_sec)
                    if gt_bpm is not None:
                        self.shared.update_reference_hr(gt_bpm, source="video_gt", device_timestamp=time_sec)
                elapsed = time.time() - started
                sleep_sec = max(0.0, frame_duration - elapsed)
                if sleep_sec > 0:
                    self.stop_event.wait(timeout=sleep_sec)
        except Exception as exc:
            self.shared.set_error(str(exc))
        finally:
            cap.release()


def make_handler(shared: SharedState, title: str):
    class Handler(BaseHTTPRequestHandler):
        server_version = "RDKRppg/0.1"

        def log_message(self, fmt: str, *args) -> None:
            return

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
            elif path == "/status":
                self.send_json(shared.status())
            elif path == "/params":
                self.send_json({"ok": True, "params": shared.params()})
            elif path == "/signal":
                self.send_json(shared.signal())
            elif path == "/snapshot.jpg":
                self.send_snapshot()
            elif path == "/reference_hr":
                values = parse_qs(parsed.query)
                if values:
                    self.handle_reference_update(values)
                else:
                    self.send_json({"ok": True, "reference_hr": shared.reference_status()})
            else:
                self.send_error(404, "not found")

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            if path == "/reference_hr":
                self.handle_json_or_form(self.handle_reference_update)
                return
            if path == "/params":
                self.handle_json_or_form(self.handle_params_update)
                return
            self.send_error(404, "not found")

        def handle_json_or_form(self, callback) -> None:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self.send_json({"ok": False, "error": "invalid content length"}, status=400)
                return
            if length <= 0 or length > 4096:
                self.send_json({"ok": False, "error": "empty or too large body"}, status=400)
                return
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
                return
            if not isinstance(payload, dict):
                self.send_json({"ok": False, "error": "payload must be an object"}, status=400)
                return
            callback(payload)

        def handle_params_update(self, payload: dict[str, object]) -> None:
            def pick(name: str) -> Optional[float]:
                if name not in payload:
                    return None
                value = payload[name]
                if isinstance(value, list):
                    value = value[0] if value else None
                if value in (None, ""):
                    return None
                try:
                    return float(str(value).strip())
                except ValueError as exc:
                    raise ValueError(f"{name} must be numeric") from exc

            try:
                params = shared.update_params(
                    signal_seconds=pick("signal_seconds"),
                    min_bpm=pick("min_bpm"),
                    max_bpm=pick("max_bpm"),
                )
            except ValueError as exc:
                self.send_json({"ok": False, "error": str(exc)}, status=400)
                return
            self.send_json({"ok": True, "params": params, "status": shared.status()})

        def handle_reference_update(self, payload: dict[str, object]) -> None:
            def pick(*names: str) -> Optional[object]:
                for name in names:
                    if name in payload:
                        value = payload[name]
                        if isinstance(value, list):
                            return value[0] if value else None
                        return value
                return None

            bpm_raw = pick("bpm", "hr", "heart_rate", "heartrate")
            if bpm_raw is None:
                self.send_json({"ok": False, "error": "missing bpm"}, status=400)
                return
            try:
                bpm = float(str(bpm_raw).strip())
            except ValueError:
                self.send_json({"ok": False, "error": "bpm must be numeric"}, status=400)
                return
            if bpm < 30.0 or bpm > 240.0:
                self.send_json({"ok": False, "error": "bpm out of range"}, status=400)
                return

            source = str(pick("source", "device") or "mi_band_7").strip() or "mi_band_7"
            ts_raw = pick("timestamp", "ts", "time")
            device_timestamp: Optional[float] = None
            if ts_raw not in (None, ""):
                try:
                    device_timestamp = float(str(ts_raw).strip())
                    if device_timestamp > 1_000_000_000_000:
                        device_timestamp = device_timestamp / 1000.0
                except ValueError:
                    device_timestamp = None

            shared.update_reference_hr(bpm=bpm, source=source, device_timestamp=device_timestamp)
            self.send_json({"ok": True, "reference_hr": shared.reference_status()})

        def send_html(self) -> None:
            page_title = html.escape(title)
            body = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{page_title}</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: Arial, sans-serif; background: #101010; color: #f4f4f4; }}
    header {{ padding: 10px 14px; background: #202020; display: flex; gap: 14px; align-items: center; flex-wrap: wrap; }}
    code {{ color: #8df58d; }}
    main {{ display: grid; grid-template-columns: minmax(0, 1fr) 360px; min-height: calc(100vh - 48px); }}
    #camera {{ display: block; width: 100%; height: calc(100vh - 48px); object-fit: contain; background: #000; }}
    aside {{ border-left: 1px solid #333; padding: 14px; background: #161616; }}
    .metric {{ margin-bottom: 12px; padding-bottom: 10px; border-bottom: 1px solid #333; }}
    .value {{ font-size: 34px; font-weight: bold; line-height: 1.1; }}
    .label {{ color: #aaa; font-size: 13px; margin-bottom: 4px; }}
    canvas {{ width: 100%; height: 170px; background: #050505; border: 1px solid #333; }}
    .bad {{ color: #ff7777; }}
    @media (max-width: 900px) {{
      main {{ display: block; }}
      #camera {{ height: 62vh; }}
      aside {{ border-left: 0; border-top: 1px solid #333; }}
    }}
  </style>
</head>
<body>
  <header>
    <strong>{page_title}</strong>
    <span id="status">waiting...</span>
    <a href="/snapshot.jpg" style="color:#9fd1ff">snapshot</a>
  </header>
  <main>
    <img id="camera" src="/snapshot.jpg" />
    <aside>
      <div class="metric">
        <div class="label">摄像头 rPPG BPM</div>
        <div class="value" id="bpm">--</div>
      </div>
      <div class="metric">
        <div class="label">语音提示</div>
        <div class="value" id="voicePrompt" style="font-size:22px; color:#ffd966;">--</div>
      </div>
      <div class="metric">
        <div class="label">小米手环参考 BPM</div>
        <div class="value" id="refBpm">--</div>
        <div id="delta">差值 --</div>
      </div>
      <div class="metric">
        <div class="label">RGB 均值</div>
        <div id="rgb">--</div>
      </div>
      <div class="metric">
        <div class="label">缓冲</div>
        <div id="buffer">--</div>
      </div>
      <div class="metric">
        <div class="label">当前参数</div>
        <div id="params">--</div>
      </div>
      <div class="label">绿色通道波形</div>
      <canvas id="wave" width="640" height="220"></canvas>
    </aside>
  </main>
  <script>
    const camera = document.getElementById('camera');
    const canvas = document.getElementById('wave');
    const ctx = canvas.getContext('2d');
    let nextDelay = 90;

    function refreshImage() {{
      camera.src = '/snapshot.jpg?t=' + Date.now();
    }}
    camera.onload = () => setTimeout(refreshImage, nextDelay);
    camera.onerror = () => setTimeout(refreshImage, 500);

    function drawWave(samples) {{
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#333';
      ctx.beginPath();
      ctx.moveTo(0, canvas.height / 2);
      ctx.lineTo(canvas.width, canvas.height / 2);
      ctx.stroke();
      if (!samples || samples.length < 2) return;
      const values = samples.map(s => s.g);
      const minV = Math.min(...values);
      const maxV = Math.max(...values);
      const span = Math.max(1, maxV - minV);
      ctx.strokeStyle = '#78f078';
      ctx.lineWidth = 2;
      ctx.beginPath();
      samples.forEach((s, i) => {{
        const x = i * canvas.width / (samples.length - 1);
        const y = canvas.height - ((s.g - minV) / span * (canvas.height - 28) + 14);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }});
      ctx.stroke();
    }}

    async function tick() {{
      try {{
        const statusResp = await fetch('/status', {{ cache: 'no-store' }});
        const s = await statusResp.json();
        const stale = s.age_sec === null || s.age_sec > 2;
        nextDelay = stale ? 500 : 90;
        document.getElementById('status').innerHTML =
          `frame <code>${{s.seq}}</code> | fps <code>${{s.input_fps}}</code> | faces <code>${{s.face_count}}</code> | roi <code>${{s.roi_count}}</code> | age <span class="${{stale ? 'bad' : ''}}">${{s.age_sec}}s</span>`;
        document.getElementById('bpm').textContent = s.bpm === null ? '-' : s.bpm;
        document.getElementById('voicePrompt').textContent = s.voice_prompt || '-';
        const ref = s.reference_hr || {{}};
        const refFresh = ref.fresh && ref.bpm !== null;
        document.getElementById('refBpm').textContent = refFresh ? ref.bpm : '--';
        document.getElementById('delta').textContent =
          s.bpm_delta === null ? `差值 -- | ${{ref.source || 'mi_band_7'}} age ${{ref.age_sec ?? '--'}}s` : `差值 ${{s.bpm_delta}} BPM | ${{ref.source || 'mi_band_7'}} age ${{ref.age_sec ?? '--'}}s`;
        document.getElementById('rgb').textContent = `R ${{s.rgb.r}} / G ${{s.rgb.g}} / B ${{s.rgb.b}}`;
        document.getElementById('buffer').textContent = `${{s.buffer_seconds}}s · ${{s.bpm_state}}`;
        const params = s.params || {{}};
        document.getElementById('params').textContent =
          `窗口 ${{params.signal_seconds ?? '--'}}s · 频段 ${{params.min_bpm ?? '--'}}-${{params.max_bpm ?? '--'}} BPM`;
        const signalResp = await fetch('/signal', {{ cache: 'no-store' }});
        const signal = await signalResp.json();
        drawWave(signal.samples);
      }} catch (e) {{
        document.getElementById('status').textContent = 'status unavailable';
      }}
    }}
    setInterval(tick, 1000);
    tick();
    refreshImage();
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

        def send_snapshot(self) -> None:
            deadline = time.time() + 5.0
            with shared.condition:
                while shared.jpeg is None:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        self.send_error(503, "no frame yet")
                        return
                    shared.condition.wait(timeout=remaining)
                jpeg = shared.jpeg
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_cors_headers()
            self.end_headers()
            self.wfile.write(jpeg)

    return Handler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-mode", choices=("camera", "video"), default="camera")
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--video-path", default="")
    parser.add_argument("--video-gt-path", default="")
    parser.add_argument("--video-loop", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--resize-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=78)
    parser.add_argument("--signal-seconds", type=float, default=10.0)
    parser.add_argument("--title", default="RDK X5 rPPG Preview")
    args = parser.parse_args()

    shared = SharedState(args.signal_seconds)
    processor = FrameProcessor(args.resize_width, args.jpeg_quality)
    node: Optional[CameraSourceNode] = None
    runner: Optional[VideoFileRunner] = None

    if args.source_mode == "camera":
        shared.configure_source(source_mode="camera", source_name=args.topic)
        rclpy.init()
        node = CameraSourceNode(args, shared, processor)
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
    else:
        if not args.video_path:
            raise SystemExit("--video-path is required when --source-mode=video")
        runner = VideoFileRunner(args, shared, processor)
        runner.start()

    server = ReusableThreadingHTTPServer((args.host, args.port), make_handler(shared, args.title))
    print(f"Serving rPPG preview on http://{args.host}:{args.port}/ mode={args.source_mode}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        if runner is not None:
            runner.stop()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        processor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
