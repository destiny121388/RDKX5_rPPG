#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""音频设备自动检测工具。"""

import subprocess
import re


def get_usb_audio_device(fallback="plughw:0,0"):
    """通过 aplay -l 检测 USB 音频设备，返回 plughw:<card>,0。

    如果找不到 USB 设备，返回 fallback (默认 plughw:0,0)。
    """
    try:
        result = subprocess.run(
            ["aplay", "-l"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.splitlines():
            # 匹配: card N: ... [USB ...] 或 card N: ... USB ...
            m = re.match(r"card\s+(\d+)\b.*USB", line, re.IGNORECASE)
            if m:
                return f"plughw:{m.group(1)},0"
    except Exception:
        pass
    return fallback


if __name__ == "__main__":
    print(get_usb_audio_device())
