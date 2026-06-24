#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# TTS HTTP 服务 - 常驻运行
# 用法: python tts_service.py
# 启动后会一直运行，监听 http://localhost:7878 的请求

import sys
import sherpa_onnx
import numpy as np
import os
import json
import wave
import tempfile
import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

from audio_utils import get_usb_audio_device


class MatchaBakerManager:
    def __init__(self, model_path, audio_device="plughw:0,0"):
        self.audio_device = audio_device

        # 1. 设置文件路径
        matcha_model = os.path.join(model_path, "model-steps-3.onnx")
        vocoder_model = os.path.join(model_path, "vocos-22khz-univ.onnx")
        lexicon_file = os.path.join(model_path, "lexicon.txt")
        tokens_file = os.path.join(model_path, "tokens.txt")
        dict_dir = os.path.join(model_path, "dict")

        # 2. 规则文件路径（处理数字和日期）
        rule_fsts = ",".join([
            os.path.join(model_path, "phone.fst"),
            os.path.join(model_path, "date.fst"),
            os.path.join(model_path, "number.fst")
        ])

        # 检查关键文件
        for f in [matcha_model, vocoder_model, lexicon_file, tokens_file]:
            if not os.path.exists(f):
                raise FileNotFoundError(f"缺失文件: {f}")

        # 3. 构造 Matcha 配置
        matcha_config = sherpa_onnx.OfflineTtsMatchaModelConfig(
            acoustic_model=matcha_model,
            vocoder=vocoder_model,
            lexicon=lexicon_file,
            tokens=tokens_file,
            dict_dir=dict_dir,
            noise_scale=0.1,
            length_scale=1.0
        )

        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=matcha_config,
                num_threads=4,
                debug=False,
            ),
            rule_fsts=rule_fsts,
            max_num_sentences=1,
        )

        self.tts = sherpa_onnx.OfflineTts(config)
        self.audio_lock = threading.Lock()

    def say(self, text, speed=0.9, volume=1.0):
        with self.audio_lock:
            print(f"正在播放: {text}")
            audio = self.tts.generate(text, sid=0, speed=speed)

            if len(audio.samples) == 0:
                return

            samples = audio.samples
            max_val = np.max(np.abs(samples))
            if max_val > 0:
                samples = samples / max_val
            samples = samples * volume
            samples = np.clip(samples, -1.0, 1.0)

            # 转为 int16 并写入临时 WAV 文件
            samples_int16 = (samples * 32767).astype(np.int16)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                wav_path = f.name

            try:
                with wave.open(wav_path, "w") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)  # 16-bit
                    wf.setframerate(audio.sample_rate)
                    wf.writeframes(samples_int16.tobytes())

                subprocess.run(
                    ["aplay", "-D", self.audio_device, wav_path],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
            finally:
                if os.path.exists(wav_path):
                    os.remove(wav_path)


class TTSRequestHandler(BaseHTTPRequestHandler):
    """HTTP 请求处理器"""

    tts_instance = None

    @classmethod
    def set_tts_instance(cls, tts_instance):
        cls.tts_instance = tts_instance

    def do_POST(self):
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                text = data.get('text', '')

                if text:
                    print(f"收到TTS请求: {text}")
                    self._play_text(text)
                    self._send_response({'status': 'ok', 'message': 'TTS播放完成'})
                else:
                    self._send_response({'status': 'error', 'message': '文本为空'}, 400)
            else:
                self._send_response({'status': 'error', 'message': '无效请求'}, 400)

        except json.JSONDecodeError:
            self._send_response({'status': 'error', 'message': 'JSON解析失败'}, 400)
        except Exception as e:
            self._send_response({'status': 'error', 'message': str(e)}, 500)

    def do_GET(self):
        self._send_response({'status': 'ok', 'service': 'TTS'})

    def _play_text(self, text):
        try:
            if self.tts_instance:
                self.tts_instance.say(text)
        except Exception as e:
            print(f"播放出错: {e}")

    def _send_response(self, data, status_code=200):
        self.send_response(status_code)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        response = json.dumps(data, ensure_ascii=False)
        self.wfile.write(response.encode('utf-8'))

    def log_message(self, format, *args):
        pass


if __name__ == "__main__":
    # TTS 模型路径（相对于脚本所在目录的上级）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "..", "model", "matcha-icefall-zh-baker")

    # 音频设备（自动检测 USB 声卡）
    AUDIO_DEVICE = get_usb_audio_device()

    HOST = '0.0.0.0'
    PORT = 7878

    try:
        print("=" * 50)
        print("TTS HTTP 服务正在启动...")
        print(f"模型路径: {model_path}")
        print(f"音频设备: {AUDIO_DEVICE}")
        print(f"监听地址: http://{HOST}:{PORT}")
        print("=" * 50)

        print("正在加载 TTS 模型（首次加载需要几秒到十几秒）...")
        tts = MatchaBakerManager(model_path, audio_device=AUDIO_DEVICE)
        print("TTS 模型加载完成！")

        TTSRequestHandler.set_tts_instance(tts)

        server = HTTPServer((HOST, PORT), TTSRequestHandler)
        print(f"\n✓ TTS 服务已启动，监听 http://{HOST}:{PORT}")
        print("服务将一直运行，按 Ctrl+C 停止")
        print("=" * 50)

        server.serve_forever()

    except KeyboardInterrupt:
        print("\n\n收到停止信号，正在关闭 TTS 服务...")
        server.shutdown()
        print("TTS 服务已停止")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"TTS 服务启动失败: {e}")
        sys.exit(1)
