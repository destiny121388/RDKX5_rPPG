#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# TTS 客户端模块
# 用法:
#   from tts_client import tts_say, tts_say_async
#   tts_say("同步播报文字")
#   tts_say_async("异步播报文字，不阻塞")

import requests
import threading
import time

# TTS 服务配置
TTS_HOST = 'localhost'
TTS_PORT = 7878
TTS_URL = f'http://{TTS_HOST}:{TTS_PORT}'

# 请求超时时间（秒）
REQUEST_TIMEOUT = 30.0


def tts_say(text):
    """
    同步调用 TTS 播报（阻塞，直到请求发送完成）

    Args:
        text (str): 要播报的文本

    Returns:
        bool: 是否成功发送请求
    """
    try:
        response = requests.post(
            TTS_URL,
            json={'text': text},
            timeout=REQUEST_TIMEOUT
        )
        if response.status_code == 200:
            print(f"TTS请求已发送: {text}")
            return True
        else:
            print(f"TTS请求失败: HTTP {response.status_code}")
            return False

    except requests.exceptions.ConnectionError:
        print("TTS服务未启动或连接失败")
        return False
    except requests.exceptions.Timeout:
        print("TTS请求超时")
        return False
    except Exception as e:
        print(f"TTS调用异常: {e}")
        return False


def tts_say_async(text):
    """
    异步调用 TTS 播报（不阻塞，立即返回）

    Args:
        text (str): 要播报的文本

    Returns:
        threading.Thread: 后台线程对象
    """
    def _say():
        tts_say(text)

    thread = threading.Thread(target=_say, daemon=True)
    thread.start()
    return thread


def tts_check_service():
    """
    检查 TTS 服务是否正常运行

    Returns:
        bool: 服务是否可用
    """
    try:
        response = requests.get(TTS_URL, timeout=2.0)
        return response.status_code == 200
    except:
        return False


if __name__ == "__main__":
    # 测试代码
    print("测试 TTS 客户端...")

    # 1. 检查服务状态
    print(f"\n1. 检查 TTS 服务状态...")
    if tts_check_service():
        print("   ✓ TTS 服务运行正常")
    else:
        print("   ✗ TTS 服务未启动")
        print("   请先运行: python tts_service.py")
        exit(1)

    # 2. 同步播报测试
    print("\n2. 同步播报测试...")
    tts_say("你的心率为88。")
    time.sleep(3)

    # 3. 异步播报测试
    # print("\n3. 异步播报测试...")
    # tts_say_async("这是一条异步播报测试消息")
    # print("   异步调用已返回，主线程继续执行")
    # time.sleep(3)

    print("\n测试完成！")