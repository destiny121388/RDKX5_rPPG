# rppg_runtime

> **注意**：由于 GitHub 上传大小限制，本仓库仅包含核心代码文件。完整工程（含虚拟环境、预训练模型等）请前往百度网盘获取：
> - **链接**：[https://pan.baidu.com/s/1sGSuxOXjw0kl4_S2i-_v3A?pwd=hwm1]
> - **提取码**：`hwm1`

基于摄像头视频流的实时非接触式心率（rPPG）检测系统，运行于 ** RDK X5** 嵌入式开发板，配套 PC 端 BLE 参考心率桥接工具。系统通过远程光电容积脉搏波描记法（Remote Photoplethysmography, rPPG）检测面部皮肤因血流引起的微小颜色变化，实时估算心率（BPM），并支持中文语音合成播报。

---

## 目录

- [系统概述](#系统概述)
- [系统架构](#系统架构)
- [目录结构](#目录结构)
- [硬件与软件要求](#硬件与软件要求)
- [依赖项](#依赖项)
- [快速开始](#快速开始)
- [rPPG 算法详解](#rppg-算法详解)
- [HTTP API 参考](#http-api-参考)
- [配置参数](#配置参数)
- [语音交互流程](#语音交互流程)
- [实验数据记录](#实验数据记录)
- [模型来源与许可](#模型来源与许可)


---

## 系统概述

本系统面向非接触式生理信号监测场景，由三个协作节点构成：

| 节点 | 运行平台 | 职责 |
|------|----------|------|
| **rPPG 推理服务器** | RDK X5（Linux / ROS2 Humble） | 摄像头采集 → 人脸检测 → ROI 提取 → 信号处理 → 心率估计 → Web 仪表盘 |
| **TTS 语音服务** | RDK X5（Linux） | 基于 Matcha-TTS 的中文语音合成，通过 USB 声卡播报心率结果 |
| **参考心率桥接** | Windows PC | BLE 连接小米手环 10 获取参考心率，提供实验监控面板与数据记录 |

**核心特性：**

- **纯非接触式测量**：无需任何传感器接触人体，仅通过摄像头画面即可检测心率
- **多方法融合**：同时使用 Green、POS、CHROM 三种 rPPG 信号变体，通过聚类融合提升鲁棒性
- **实时轨迹跟踪**：自适应平滑、跳变检测与确认机制，抑制瞬时波动
- **中文语音播报**：本地 ONNX 模型推理，无需联网，自动检测 USB 音频设备
- **进程自动守护**：Supervisor 脚本持续监控各组件，崩溃自动重启，流数据卡顿自动恢复
- **实验数据记录**：自动将 rPPG 估计值与参考心率记录为 CSV/JSONL，便于离线分析
- **在线参数调优**：通过 Web 面板实时调整信号窗口、BPM 范围等参数，无需重启服务

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                     RDK X5 开发板（Linux / ROS2 Humble）                │
│                                                                     │
│  ┌──────────────┐    ROS2 Topic     ┌──────────────────────────┐   │
│  │  MIPI 摄像头   │─── /image_raw ──▶│  rdk_rppg_server.py      │   │
│  │  (640×480)   │                  │  (端口 8080)              │   │
│  └──────────────┘                  │                          │   │
│                                    │  MediaPipe FaceMesh      │   │
│  ┌──────────────┐                  │  → ROI RGB 提取           │   │
│  │  TTS 服务     │ ◀── HTTP POST ──│  → 去趋势 + 带通滤波      │   │
│  │  (端口 7878)  │                  │  → FFT + 峰值评分        │   │
│  │  sherpa-onnx │                  │  → 多方法聚类融合         │   │
│  └──────┬───────┘                  │  → 轨迹跟踪与平滑         │   │
│         │                          │  → Web 仪表盘             │   │
│         ▼                          └──────────┬───────────────┘   │
│    USB 音频输出                                │                    │
│                                               │ HTTP API           │
│  ┌────────────────────────┐                   │ /status /snapshot  │
│  │  rdk_rppg_supervisor.sh │────── 守护 ──────│ /params /signal    │
│  │  (进程管理 + 健康检查)    │                   │ /reference_hr      │
│  └────────────────────────┘                   └────────┬───────────┘
└───────────────────────────────────────────────────────┼────────────┘
                                                        │
                                           HTTP POST /reference_hr
                                           HTTP GET /status, /snapshot
                                                        │
┌───────────────────────────────────────────────────────┼────────────┐
│                 Windows PC（参考心率桥接）                  │            │
│                                                       │            │
│  ┌─────────────────────┐     BLE      ┌───────────────┴──────────┐ │
│  │  小米手环 10          │ ──────────▶ │  mi_band_10_ble_to_pc.py  │ │
│  │  (心率广播模式)       │             │  (BLE → HTTP 桥接)        │ │
│  └─────────────────────┘             └──────────────┬───────────┘ │
│                                                     │              │
│                                        HTTP POST /reference_hr     │
│                                                     │              │
│                                          ┌──────────┴───────────┐  │
│                                          │  reference_hr_relay.py │  │
│                                          │  (端口 8090)           │  │
│                                          │                       │  │
│                                          │  /experiment 实验面板   │  │
│                                          │  /board_params 参数代理 │  │
│                                          │  实验日志 CSV + JSONL   │  │
│                                          └───────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

**数据流说明：**

1. MIPI 摄像头 → ROS2 Topic `/image_raw` → rPPG 服务器
2. rPPG 服务器 → 人脸检测 → RGB 信号 → 心率估计 → Web 仪表盘
3. 小米手环 10 → BLE 广播 → PC 端 `mi_band_10_ble_to_pc.py` → HTTP → PC 端中继服务
4. PC 端中继服务 → HTTP POST `/reference_hr` → 开发板（转发参考心率作为真值对比）
5. 开发板 → TTS 客户端 → HTTP POST → TTS 服务 → USB 音频输出
6. Supervisor 脚本持续监控进程健康状态，异常时自动重启

---

## 目录结构

```
rdkx5_port/
├── README.md                                 # 项目文档（本文件）
├── .venv_rdkx5/                              # Python 3.10 虚拟环境
│
└── rppg_runtime/                             # rPPG 运行时代码
    │
    ├── board/                                # RDK X5 开发板端代码
    │   ├── rdk_rppg_server.py                # ★ 核心 rPPG 服务器
    │   │                                     #    - ROS2 图像订阅
    │   │                                     #    - MediaPipe FaceMesh 人脸关键点检测
    │   │                                     #    - 多 ROI 区域 RGB 信号提取
    │   │                                     #    - rPPG 信号处理与心率估计
    │   │                                     #    - HTTP 服务器 + Web 仪表盘
    │   │                                     #    - 参考心率接口与参数在线调优
    │   │                                     #    - 语音播报状态机管理
    │   │
    │   ├── rdk_rppg_supervisor.sh            # 进程管理守护脚本
    │   │                                     #    - 单例模式（PID 文件锁）
    │   │                                     #    - 按序启动：摄像头 → TTS → rPPG
    │   │                                     #    - 持续监控，崩溃自动重启
    │   │                                     #    - 流数据卡顿检测（age_sec > 5s）
    │   │
    │   ├── tts_service.py                    # TTS HTTP 服务（独立进程，端口 7878）
    │   │                                     #    - 加载 Matcha-TTS ONNX 模型
    │   │                                     #    - 接收 POST 文本，返回合成音频
    │   │                                     #    - 自动检测 USB 音频设备
    │   │
    │   ├── tts_client.py                     # TTS 客户端模块（供 rPPG 服务器调用）
    │   │                                     #    - tts_say(text) 同步播报
    │   │                                     #    - tts_say_async(text) 异步播报
    │   │                                     #    - tts_check_service() 健康检查
    │   │
    │   ├── audio_utils.py                    # USB 音频设备自动检测工具
    │   │
    │   ├── ready.wav                         # 语音提示："请对准摄像头"
    │   ├── wait.wav                          # 语音提示："请稍后"
    │   └── ok.wav                            # 语音提示："测量成功"
    │
    ├── model/                                # 预训练模型资源
    │   └── matcha-icefall-zh-baker/          # Matcha-TTS 中文女声模型
    │       ├── model-steps-3.onnx            #   声学模型（条件流匹配架构）
    │       ├── vocos-22khz-univ.onnx         #   声码器（Vocos，22kHz 输出）
    │       ├── tokens.txt                    #   音素 → ID 映射表
    │       ├── lexicon.txt                   #   中文词汇 → 音素词典
    │       ├── phone.fst                     #   字符/发音规则 FST
    │       ├── date.fst                      #   日期朗读规则 FST
    │       ├── number.fst                    #   数字朗读规则 FST
    │       ├── README.md                     #   模型来源与许可说明
    │       └── dict/                         #   CppJieba 中文分词词典
    │           ├── jieba.dict.utf8           #     最大概率分词词典
    │           ├── hmm_model.utf8            #     HMM 分词模型
    │           ├── idf.utf8                  #     TF-IDF 关键词提取
    │           ├── stop_words.utf8           #     停用词表
    │           ├── user.dict.utf8            #     用户自定义词典
    │           ├── pos_dict/                 #     词性标注模型
    │           └── README.md                 #     词典说明
    │
    └── phone_bridge/                         # PC 端参考心率桥接
        └── windows/
            ├── mi_band_10_ble_to_pc.py       # BLE 心率桥接
            │                                  #    - 自动扫描并连接小米手环 10
            │                                  #    - 订阅 BLE 心率测量特征 (0x2A37)
            │                                  #    - 解析标准心率测量格式
            │                                  #    - 去重后 POST 到中继服务
            │
            ├── reference_hr_relay.py         # HTTP 中继服务 + 实验面板
            │                                  #    - 接收 BLE 桥接的心率数据（端口 8090）
            │                                  #    - 可选转发至开发板 /reference_hr
            │                                  #    - /experiment 完整实验监控面板
            │                                  #    - 实验数据自动记录（CSV + JSONL）
            │
            ├── start_band10_full.cmd         # 一键启动（两个窗口：中继 + BLE）
            ├── start_relay_with_board.cmd    # 单独启动中继服务（带板端转发）
            └── start_mi_band_10_ble_to_pc.cmd # 单独启动 BLE 桥接
```

---

## 硬件与软件要求

### RDK X5 开发板

| 类别 | 要求 |
|------|------|
| 硬件平台 | RDK X5 开发板 |
| 摄像头 | MIPI CSI 摄像头（ROS2 `mipi_cam` 驱动，640×480） |
| 音频输出 | USB 声卡 / 板载音频（`aplay` 可识别设备） |
| 操作系统 | Linux + TROS Humble |
| Python | Python 3.10（`.venv_rdkx5` 虚拟环境见下方说明） |
| 网络 | 与 PC 处于同一局域网（参考心率转发依赖 HTTP 通信） |

### Windows PC

| 类别 | 要求 |
|------|------|
| 操作系统 | Windows 10 / 11 |
| Python | Python 3 |
| 蓝牙 | 支持 BLE（Bluetooth Low Energy）的蓝牙适配器 |
| 网络 | 与开发板处于同一局域网 |

### 参考心率设备

| 设备 | 说明 |
|------|------|
| 小米手环 10 | 需在手表端开启「心率广播」功能（设置 → 心率广播 → 开启） |

### 开发板虚拟环境（`.venv_rdkx5`）

`rdk_rppg_supervisor.sh` 通过 `$PORT_DIR/.venv_rdkx5/bin/python` 调用 Python。该虚拟环境位于 `rdkx5_port/` 根目录下，与 `rppg_runtime/` 并列：

```
rdkx5_port/
├── .venv_rdkx5/          # Python 3.10 虚拟环境（include-system-site-packages = true）
│   └── lib/python3.10/site-packages/
│       ├── mediapipe/    # 0.10.9（Apache 2.0）
│       ├── absl_py/      # 2.4.0
│       └── flatbuffers/  # 25.12.19
└── rppg_runtime/         # rPPG 运行时代码

```

虚拟环境配置为 `include-system-site-packages = true`，即会继承系统全局安装的包。以下依赖来自 RDK X5 系统环境：OpenCV、NumPy、SciPy、ROS2（`rclpy`、`sensor_msgs`）、`sherpa-onnx`、`requests`。

---

## 依赖项

### 开发板端（RDK X5）

| 依赖 | 用途 |
|------|------|
| `rclpy` | ROS2 Python 客户端库，订阅 MIPI 摄像头话题 |
| `sensor_msgs.msg.Image` | ROS2 图像消息类型 |
| `opencv-python` (`cv2`) | 图像处理、JPEG 编码、多边形 ROI 提取 |
| `mediapipe` | FaceMesh 人脸关键点检测（468 点） |
| `numpy` | 数值计算、信号处理 |
| `scipy` | Butterworth 滤波器设计与零相位滤波 (`butter`, `filtfilt`) |
| `sherpa-onnx` | Matcha-TTS ONNX 模型推理引擎 |
| `requests` | TTS 客户端 HTTP 调用 |
| `aplay` (ALSA) | 音频播放（系统命令） |

### PC 端（Windows）

| 依赖 | 用途 |
|------|------|
| `bleak` | BLE 异步扫描、连接与数据订阅 |
| `asyncio` | 异步事件循环（Python 标准库） |
| `http.server` | HTTP 服务器（Python 标准库） |


---

## 快速开始

### 1. 开发板端（RDK X5）

#### 1.1 使用 Supervisor 一键启动（推荐）

```bash
# 设置项目根目录（根据实际路径修改）
export PROJECT_DIR=$HOME/workspace_rdkx5/CV_Project

# 可选：调整分析窗口时长
export SIGNAL_SECONDS=10

# 启动（摄像头模式）
bash rppg_runtime/board/rdk_rppg_supervisor.sh
```

> ⚠️⚠️⚠️⚠️⚠️ **注意**：本开源工程的目录结构与我们 RDK X5 板端实际部署的目录结构略有差异。`rdk_rppg_supervisor.sh` 内部通过 `$PORT_DIR` 等变量拼装各组件的启动路径，使用前请根据板端实际目录布局调整脚本中以下配置：
>
> - `PORT_DIR` — 项目根目录路径（脚本中默认指向板端实际路径）
> - `PYTHON_BIN` — Python 解释器路径（默认使用 `.venv_rdkx5/bin/python`）
> - 各组件脚本/日志路径
>
> 建议在板端直接编辑 `rdk_rppg_supervisor.sh`，将上述变量修改为与板端实际部署一致的路径后即可 运行。

Supervisor 会按顺序启动：
1. MIPI 摄像头 ROS2 节点
2. TTS 语音服务（端口 7878，加载模型约需 10–20 秒）
3. rPPG 推理服务器（端口 8080）

启动成功后，在浏览器访问 `http://<板卡IP>:8080/` 查看实时仪表盘。

#### 1.2 手动启动各组件

```bash
# 终端 1：启动 TTS 语音服务
cd rppg_runtime/board
python3 tts_service.py &

# 终端 2：启动 rPPG 服务器（摄像头模式）
python3 rppg_runtime/board/rdk_rppg_server.py \
    --source-mode camera \
    --topic /image_raw \
    --port 8080 \
    --signal-seconds 10
```

#### 1.3 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--source-mode` | `camera` | 数据源模式：`camera` |
| `--topic` | `/image_raw` | ROS2 摄像头话题名称（camera 模式） |
| `--port` | `8080` | HTTP 服务端口 |
| `--signal-seconds` | `10` | 信号分析窗口时长（秒，范围 8–60） |
| `--resize-width` | `640` | 画面缩放宽度 |
| `--jpeg-quality` | `78` | 仪表盘 JPEG 快照质量 |
| `--title` | — | Web 仪表盘标题 |

---

### 2. PC 端（Windows）

#### 2.1 一键启动（推荐）

双击或在终端执行：

```cmd
rppg_runtime\phone_bridge\windows\start_band10_full.cmd
```

此脚本会在两个独立的控制台窗口中分别启动中继服务和 BLE 桥接。

#### 2.2 分别启动

```cmd
:: 终端 1：启动中继服务（带板端转发）
python rppg_runtime\phone_bridge\windows\reference_hr_relay.py \
    --listen-host 0.0.0.0 \
    --listen-port 8090 \
    --board-url http://10.77.3.84:8080

:: 终端 2：启动 BLE 桥接
python rppg_runtime\phone_bridge\windows\mi_band_10_ble_to_pc.py \
    --relay-url http://127.0.0.1:8090
```

#### 2.3 命令行参数

**reference_hr_relay.py**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--listen-host` | `0.0.0.0` | HTTP 监听地址 |
| `--listen-port` | `8090` | HTTP 监听端口 |
| `--board-url` | `http://10.77.3.84:8080` | 开发板 rPPG 服务地址（为空则不转发） |
| `--timeout` | `1.0` | HTTP 请求超时（秒） |
| `--no-board-forward` | — | 禁用向开发板转发参考心率 |

**mi_band_10_ble_to_pc.py**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--relay-url` | `http://127.0.0.1:8090` | 中继服务地址 |
| `--repeat-seconds` | `2.0` | 重复发送间隔（秒） |
| `--scan-seconds` | `10.0` | BLE 扫描超时（秒） |
| `--source-name` | `mi_band_10_ble` | 心率数据来源标识 |
| `--scan-only` | — | 仅扫描附近 BLE 设备后退出 |
| `--check-only` | — | 仅检查能否连接手环后退出 |

#### 2.4 使用前准备

1. 确保 PC 蓝牙已开启
2. 小米手环 10 开启心率广播：「设置」→「心率广播」→ 开启
3. 手环与 PC 距离保持在 5 米以内

---

## rPPG 算法详解

本系统的核心算法实现在 [rppg_runtime/board/rdk_rppg_server.py](rppg_runtime/board/rdk_rppg_server.py) 中，处理流程如下：

### 阶段一：人脸检测与 ROI 提取

```
摄像头帧 (ROS2 /image_raw)
       │
       ▼
MediaPipe FaceMesh（468 个关键点，置信度阈值 0.5）
       │
       ├──▶ 468 个面部关键点坐标
       │
       ├──▶ 朝向检测：鼻尖在眼部边界框内的水平位置 → frontal_score
       │    （frontal_score < 0.35 时认为面部未正对摄像头，不计入 ROI）
       │
       └──▶ 三组 ROI 多边形区域（基于面部网格索引）：
            ├── 额头  (forehead):   索引 [9, 107, 66, 105, 104, 103, 67, 109, 10, 338, 297, 332, 333, 334, 296, 336]
            ├── 左脸颊 (left cheek):  索引 [132, 58, 172, 136, 150, 169, 210, 212, 202, 57]
            └── 右脸颊 (right cheek): 索引 [361, 288, 397, 365, 379, 394, 430, 432, 422, 287]
```

### 阶段二：RGB 信号提取

对每个 ROI 区域，使用 `cv2.fillPoly` + `cv2.mean` 计算区域均值，三区域取平均后得到帧级 `(R, G, B)` 原始信号。原始信号以 FIFO 队列（`samples`）缓存，窗口长度由 `signal_seconds` 参数控制（默认 10 秒）。

### 阶段三：rPPG 信号构建

原始 RGB 信号经过去均值归一化后，构建三种 rPPG 变体：

| 方法 | 公式 | 原理 |
|------|------|------|
| **Green** | `S_green = N_g` | 绿色通道信号，经典光电容积描记法 |
| **POS** | `X = N_g − N_b`<br>`Y = −2N_r + N_g + N_b`<br>`α = σ(X) / σ(Y)`<br>`S_pos = X + α·Y` | 肤色平面正交投影，抑制运动干扰 |
| **CHROM** | `X = 3N_r − 2N_g`<br>`Y = 1.5N_r + N_g − 1.5N_b`<br>`α = σ(X) / σ(Y)`<br>`S_chrom = X − α·Y` | 色度模型，抑制镜面反射分量 |

> 其中 `N_r, N_g, N_b` 为逐通道去均值归一化后的信号。

### 阶段四：信号处理

```
重采样（10 FPS 线性插值）
       │
       ▼
去趋势（移动平均基线移除，窗口 = 1.6s）
       │
       ▼
归一化（除以标准差）
       │
       ▼
二阶 Butterworth 带通滤波（0.7–3.0 Hz，对应 42–180 BPM）
  - 使用 scipy.signal.butter + filtfilt 实现零相位滤波
```

### 阶段五：频谱分析与峰值选择

```
Hanning 窗 FFT（N_FFT = 1024）
       │
       ▼
按 SNR（峰值功率 / 频带中位数功率）排序，取 Top-5 峰值
  - 相邻峰值间距 ≥ 6 BPM
       │
       ▼
峰值评分（逐峰计算）：
  ├── 子谐波惩罚：若 BPM < 70 且存在 2×BPM 处的强峰 → 分数 × 0.45
  ├── 边界惩罚：若 BPM 接近下限且存在高频强峰 → 分数 × 0.40
  └── 谐波支持：若 BPM ≥ 80 且存在 0.5×BPM 处的强峰 → 分数 × 1.18
```

### 阶段六：多方法融合与聚类

对 Green / POS / CHROM 三种方法的候选峰值：

1. 所有峰值按加权分数（SNR + 方法偏置）排序
2. 在 6 BPM 间距内进行聚类（加权平均 BPM）
3. 聚类分数 = Σ 成员分数 × (1 + 0.12 × (方法数 − 1))
   - 多方法一致的聚类获得 12% 额外加分
4. 选取最高分聚类作为 BPM 估计值

### 阶段七：轨迹跟踪与稳定

```
原始 BPM 估计值
       │
       ├── 硬上限过滤：> 110 BPM → 拒绝
       │
       ├── 剧烈跳变拦截：|ΔBPM| > 12 → 维持上一帧值
       │
       ├── 局部寻峰修正：在前一帧 BPM ± 18 范围内寻找支持峰值
       │
       ├── 跳变确认序列：小幅度跳变需连续 3 帧确认
       │
       └── 指数平滑：smoothing_factor = 0.2 × SNR 自适应
```

---

## HTTP API 参考

### 开发板服务（端口 8080）

| 端点 | 方法 | Content-Type | 说明 |
|------|------|-------------|------|
| `/` | GET | `text/html` | Web 实时仪表盘（摄像头画面、BPM 数值、信号波形） |
| `/status` | GET | `application/json` | 系统状态 JSON（含 BPM、人脸数、RGB 均值、参数、参考心率等） |
| `/params` | GET | `application/json` | 获取当前参数配置（`signal_seconds`, `min_bpm`, `max_bpm`） |
| `/params` | POST | `application/json` | 更新参数（请求体示例：`{"signal_seconds": 12, "min_bpm": 50}`） |
| `/signal` | GET | `application/json` | 最近 240 个 RGB 采样点序列（用于前端波形绘制） |
| `/snapshot.jpg` | GET | `image/jpeg` | 最新帧 JPEG 快照（含 FaceMesh 网格叠加） |
| `/reference_hr` | GET | `application/json` | 查询当前参考心率值 |
| `/reference_hr` | POST | `application/json` | 接收外部参考心率（请求体：`{"source": "...", "bpm": 72}`） |

**`/status` 响应字段说明：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `bpm` | number\|null | 当前估计心率（BPM） |
| `bpm_state` | string | 心率状态：`"tracking"` / `"rough"` / `"collecting"` / `"high_outlier"` |
| `face_count` | number | 当前检测到的人脸数量 |
| `frontal_score` | number | 面部朝向评分（0–1，越高越正） |
| `rgb` | object | 当前帧 ROI 区域的 RGB 均值 |
| `age_sec` | number | 当前 JPEG 快照距生成时刻的秒数 |
| `seq` | number | 帧序列号 |
| `params` | object | 当前参数配置 |
| `reference_hr` | object | 最新参考心率数据 |

---

### 中继服务（端口 8090）

| 端点 | 方法 | Content-Type | 说明 |
|------|------|-------------|------|
| `/` | GET | `text/html` | 心率状态页 |
| `/experiment` | GET | `text/html` | ★ 完整实验监控面板 |
| `/status` | GET | `application/json` | 最新参考心率 + 开发板状态（聚合） |
| `/events` | GET | `application/json` | 近期心率事件历史（最近 100 条） |
| `/reference_hr` | GET | `application/json` | 查询最新参考心率 |
| `/reference_hr` | POST | `application/json` | 接收 BLE 桥接的心率数据并转发至开发板 |
| `/board_params` | POST | `application/json` | 代理参数修改至开发板（透传 `/params`） |

**实验面板 (`/experiment`) 功能：**
- 开发板实时摄像头画面
- rPPG BPM 与参考 BPM 并排显示
- BPM 差值（rPPG − 参考）实时误差
- 信号窗口、BPM 范围在线调优
- 实验数据自动记录（CSV + JSONL）

---

## 配置参数

### 运行时参数（可通过 HTTP `/params` 动态调整）

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `signal_seconds` | `10` | 8–60 | 信号分析窗口时长（秒），越大越稳定但响应越慢 |
| `min_bpm` | `45` | 30–60 | 可检测的最低心率（BPM） |
| `max_bpm` | `110` | 80–180 | 可检测的最高心率（BPM） |
| `target_fps` | `10` | 固定 | 信号重采样帧率（Hz） |

### 算法常量（硬编码，修改需重启）

| 常量 | 值 | 说明 |
|------|------|------|
| `PEAK_MIN_SEPARATION_BPM` | 6.0 | 相邻峰值最小间隔（BPM） |
| `TOP_PEAK_COUNT` | 5 | 每个方法保留的候选峰值数 |
| `TRACK_NEAR_BPM` | 18.0 | 轨迹跟踪搜索范围（BPM） |
| `TRACK_MAX_JUMP_BPM` | 10.0 | 允许的单帧最大跳变（BPM） |
| `TRACK_SMOOTHING` | 0.2 | 指数平滑系数 |
| `TRACK_CONFIRM_FRAMES` | 3 | 跳变确认所需连续帧数 |
| `MIN_ANALYSIS_SECONDS` | 9.0 | 开始分析所需的最小数据时长（秒） |
| `n_fft` | 1024 | FFT 点数 |
| `BUTTER_ORDER` | 2 | Butterworth 滤波器阶数 |
| `BANDPASS_LOW` | 0.7 Hz | 带通滤波器下限（42 BPM） |
| `BANDPASS_HIGH` | 3.0 Hz | 带通滤波器上限（180 BPM） |

### Supervisor 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `PROJECT_DIR` | `$HOME/workspace_rdkx5/CV_Project` | 项目根目录 |
| `SOURCE_MODE` | `camera` | 数据源：`camera` |
| `SIGNAL_SECONDS` | `10` | 信号分析窗口时长 |

---

## 语音交互流程

rPPG 服务器内置一个语音状态机 (`VoiceManager`)，根据检测状态自动播放语音提示：

```
状态流转：

  无脸 / 脸未正对摄像头
    │  (每 6s 播放一次 "请对准摄像头")
    │
    ▼
  检测到人脸且正对 (frontal_score ≥ 0.35) 持续 2 秒
    │  播放 "请稍后"
    │
    ▼
  有效心率稳定输出 1.5 秒
    │  播放 "心率测量成功"
    │
    ▼
  周期性 TTS 播报 (每 8 秒)
    │  "您当前的心率为88。"
    │
    ▼
  人脸丢失 / 心率丢失超过 1 秒 → 回到初始状态
```

语音播放使用 `aplay -D <device>` 通过 USB 音频设备输出，各语音任务在独立线程中执行以避免阻塞主推理循环。TTS 合成通过 `sherpa-onnx` 在本地完成，无需网络连接。

---

## 实验数据记录

中继服务（`reference_hr_relay.py`）在实验模式下自动将数据记录到 `artifacts/experiment_logs/` 目录。

### 文件命名规则

```
artifacts/experiment_logs/
├── experiment_20260624_143052.csv     # CSV 格式实验记录
└── experiment_20260624_143052.jsonl   # JSONL 格式实验记录
```

> 文件名时间戳为实验开始时的时间。

### 记录字段

每条记录包含以下主要字段（完整列表见代码中 `ExperimentLogger.log_snapshot()`）：

| 字段 | 说明 |
|------|------|
| `timestamp_iso` / `timestamp_unix` | PC 端记录时间 |
| `board_bpm` | 开发板当前估计心率（BPM） |
| `board_bpm_state` | 心率状态（tracking / rough / collecting 等） |
| `board_face_count` | 检测到的人脸数量 |
| `board_r` / `board_g` / `board_b` | ROI 区域 RGB 均值 |
| `board_peak_bpm` / `board_peak_snr` | 主峰值 BPM 与 SNR |
| `board_peak1_bpm` ~ `board_peak3_snr` | Top-3 候选峰值 |
| `board_signal_method` | 选用的信号方法（green / pos / chrom） |
| `reference_bpm` | 手环参考心率 |
| `delta_bpm` | rPPG 与参考心率的差值 |
| `signal_seconds` / `min_bpm` / `max_bpm` | 当前参数配置 |
| `pc_seq` / `pc_interval_sec` | PC 端接收序列号与间隔 |
| `board_error` | 开发板连接错误信息（如有） |

---

## 模型来源与许可

### Matcha-TTS 中文女声模型

- **训练数据**：[Baker 中文标准女声数据集](https://en.data-baker.com/datasets/freeDatasets/)（约 12 小时，10000 句）
- **训练代码**：[icefall TTS](https://github.com/k2-fsa/icefall/tree/master/egs/baker_zh/TTS)
- **模型格式**：ONNX（通过 sherpa-onnx 推理）
- **许可类型**：**非商业用途**（Baker 数据集限制）

> ⚠️ **重要提示**：Baker 数据集的许可协议限制为非商业用途使用。如本项目涉及商业场景，请替换为商用许可的 TTS 模型。

---

