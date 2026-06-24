# rppg_runtime

> **Note**: Due to GitHub upload size limitations, this repository only contains the core code files. For the complete project (including virtual environment, pre-trained models, etc.), please download from Baidu Netdisk:
> - **Link**: [https://pan.baidu.com/s/1sGSuxOXjw0kl4_S2i-_v3A?pwd=hwm1]
> - **Password**: `hwm1`

Real-time contactless heart rate (rPPG) detection system based on camera video streams, running on the **RDK X5** embedded development board, with a companion PC-side BLE reference heart rate bridge. The system detects subtle color changes on facial skin caused by blood flow using Remote Photoplethysmography (rPPG), estimates heart rate (BPM) in real time, and supports Chinese text-to-speech (TTS) broadcast.

---

## Table of Contents

- [System Overview](#system-overview)
- [System Architecture](#system-architecture)
- [Directory Structure](#directory-structure)
- [Hardware & Software Requirements](#hardware--software-requirements)
- [Dependencies](#dependencies)
- [Quick Start](#quick-start)
- [rPPG Algorithm Details](#rppg-algorithm-details)
- [HTTP API Reference](#http-api-reference)
- [Configuration Parameters](#configuration-parameters)
- [Voice Interaction Flow](#voice-interaction-flow)
- [Experiment Data Logging](#experiment-data-logging)
- [Model Sources & License](#model-sources--license)

---

## System Overview

This system targets contactless physiological signal monitoring and consists of three cooperating nodes:

| Node | Platform | Role |
|------|----------|------|
| **rPPG Inference Server** | RDK X5 (Linux / ROS2 Humble) | Camera capture → Face detection → ROI extraction → Signal processing → Heart rate estimation → Web dashboard |
| **TTS Voice Service** | RDK X5 (Linux) | Chinese speech synthesis based on Matcha-TTS, heart rate broadcast via USB audio device |
| **Reference HR Bridge** | Windows PC | BLE connection to Xiaomi Band 10 for reference heart rate, experiment monitoring dashboard & data logging |

**Core Features:**

- **Fully Contactless Measurement**: No sensors touching the body — heart rate detected solely from camera images
- **Multi-Method Fusion**: Simultaneously uses Green, POS, and CHROM rPPG signal variants with cluster-based fusion for improved robustness
- **Real-Time Trajectory Tracking**: Adaptive smoothing, jump detection, and confirmation mechanisms to suppress transient fluctuations
- **Chinese Voice Broadcast**: Local ONNX model inference, no internet required, auto-detection of USB audio devices
- **Process Supervision**: Supervisor script continuously monitors all components, auto-restarts on crash, auto-recovery from stream stalls
- **Experiment Data Logging**: Automatically records rPPG estimates and reference heart rate as CSV/JSONL for offline analysis
- **Online Parameter Tuning**: Adjust signal window, BPM range, and other parameters in real time via the Web panel — no service restart required

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                  RDK X5 Development Board (Linux / ROS2 Humble)       │
│                                                                     │
│  ┌──────────────┐    ROS2 Topic     ┌──────────────────────────┐   │
│  │  MIPI Camera  │─── /image_raw ──▶│  rdk_rppg_server.py      │   │
│  │  (640×480)   │                  │  (Port 8080)             │   │
│  └──────────────┘                  │                          │   │
│                                    │  MediaPipe FaceMesh      │   │
│  ┌──────────────┐                  │  → ROI RGB Extraction     │   │
│  │  TTS Service  │ ◀── HTTP POST ──│  → Detrend + Bandpass     │   │
│  │  (Port 7878)  │                  │  → FFT + Peak Scoring     │   │
│  │  sherpa-onnx │                  │  → Multi-Method Clustering│   │
│  └──────┬───────┘                  │  → Trajectory Tracking    │   │
│         │                          │  → Web Dashboard          │   │
│         ▼                          └──────────┬───────────────┘   │
│    USB Audio Output                            │                    │
│                                               │ HTTP API            │
│  ┌────────────────────────┐                   │ /status /snapshot   │
│  │  rdk_rppg_supervisor.sh │──── Supervisor ──│ /params /signal     │
│  │  (Process Mgmt + Health) │                  │ /reference_hr       │
│  └────────────────────────┘                   └────────┬───────────┘
└───────────────────────────────────────────────────────┼────────────┘
                                                        │
                                           HTTP POST /reference_hr
                                           HTTP GET /status, /snapshot
                                                        │
┌───────────────────────────────────────────────────────┼────────────┐
│              Windows PC (Reference HR Bridge)              │            │
│                                                       │            │
│  ┌─────────────────────┐     BLE      ┌───────────────┴──────────┐ │
│  │  Xiaomi Band 10      │ ──────────▶ │  mi_band_10_ble_to_pc.py  │ │
│  │  (HR Broadcast Mode) │             │  (BLE → HTTP Bridge)      │ │
│  └─────────────────────┘             └──────────────┬───────────┘ │
│                                                     │              │
│                                        HTTP POST /reference_hr     │
│                                                     │              │
│                                          ┌──────────┴───────────┐  │
│                                          │  reference_hr_relay.py │  │
│                                          │  (Port 8090)           │  │
│                                          │                       │  │
│                                          │  /experiment Panel     │  │
│                                          │  /board_params Proxy   │  │
│                                          │  Experiment CSV+JSONL  │  │
│                                          └───────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

**Data Flow:**

1. MIPI Camera → ROS2 Topic `/image_raw` → rPPG Server
2. rPPG Server → Face Detection → RGB Signal → Heart Rate Estimation → Web Dashboard
3. Xiaomi Band 10 → BLE Broadcast → PC `mi_band_10_ble_to_pc.py` → HTTP → PC Relay Service
4. PC Relay Service → HTTP POST `/reference_hr` → Development Board (forwards reference heart rate as ground truth)
5. Development Board → TTS Client → HTTP POST → TTS Service → USB Audio Output
6. Supervisor script continuously monitors process health, auto-restarts on anomalies

---

## Directory Structure

```
rdkx5_port/
├── README.md                                 # Project documentation (this file)
├── .venv_rdkx5/                              # Python 3.10 virtual environment
│
└── rppg_runtime/                             # rPPG runtime code
    │
    ├── board/                                # RDK X5 board-side code
    │   ├── rdk_rppg_server.py                # ★ Core rPPG server
    │   │                                     #    - ROS2 image subscription
    │   │                                     #    - MediaPipe FaceMesh landmark detection
    │   │                                     #    - Multi-ROI RGB signal extraction
    │   │                                     #    - rPPG signal processing & HR estimation
    │   │                                     #    - HTTP server + Web dashboard
    │   │                                     #    - Reference HR interface & online tuning
    │   │                                     #    - Voice broadcast state machine
    │   │
    │   ├── rdk_rppg_supervisor.sh            # Process management & supervision script
    │   │                                     #    - Singleton mode (PID file lock)
    │   │                                     #    - Sequential startup: Camera → TTS → rPPG
    │   │                                     #    - Continuous monitoring, auto-restart on crash
    │   │                                     #    - Stream stall detection (age_sec > 5s)
    │   │
    │   ├── tts_service.py                    # TTS HTTP service (standalone, port 7878)
    │   │                                     #    - Loads Matcha-TTS ONNX model
    │   │                                     #    - Accepts POST text, returns synthesized audio
    │   │                                     #    - Auto-detects USB audio device
    │   │
    │   ├── tts_client.py                     # TTS client module (invoked by rPPG server)
    │   │                                     #    - tts_say(text) synchronous broadcast
    │   │                                     #    - tts_say_async(text) asynchronous broadcast
    │   │                                     #    - tts_check_service() health check
    │   │
    │   ├── audio_utils.py                    # USB audio device auto-detection utility
    │   │
    │   ├── ready.wav                         # Voice prompt: "Please face the camera"
    │   ├── wait.wav                          # Voice prompt: "Please wait"
    │   └── ok.wav                            # Voice prompt: "Measurement successful"
    │
    ├── model/                                # Pre-trained model resources
    │   └── matcha-icefall-zh-baker/          # Matcha-TTS Chinese female voice model
    │       ├── model-steps-3.onnx            #   Acoustic model (conditional flow matching)
    │       ├── vocos-22khz-univ.onnx         #   Vocoder (Vocos, 22kHz output)
    │       ├── tokens.txt                    #   Phoneme → ID mapping table
    │       ├── lexicon.txt                   #   Chinese word → phoneme dictionary
    │       ├── phone.fst                     #   Character/pronunciation rule FST
    │       ├── date.fst                      #   Date reading rule FST
    │       ├── number.fst                    #   Number reading rule FST
    │       ├── README.md                     #   Model source & license notes
    │       └── dict/                         #   CppJieba Chinese word segmentation dictionaries
    │           ├── jieba.dict.utf8           #    Max-probability segmentation dictionary
    │           ├── hmm_model.utf8            #    HMM segmentation model
    │           ├── idf.utf8                  #    TF-IDF keyword extraction
    │           ├── stop_words.utf8           #    Stop words list
    │           ├── user.dict.utf8            #    User custom dictionary
    │           ├── pos_dict/                 #    Part-of-speech tagging models
    │           └── README.md                 #    Dictionary notes
    │
    └── phone_bridge/                         # PC-side reference HR bridge
        └── windows/
            ├── mi_band_10_ble_to_pc.py       # BLE HR bridge
            │                                  #    - Auto-scans for and connects to Xiaomi Band 10
            │                                  #    - Subscribes to BLE HR measurement characteristic (0x2A37)
            │                                  #    - Parses standard HR measurement format
            │                                  #    - Deduplicates and POSTs to relay service
            │
            ├── reference_hr_relay.py         # HTTP relay service + experiment panel
            │                                  #    - Receives HR data from BLE bridge (port 8090)
            │                                  #    - Optionally forwards to board /reference_hr
            │                                  #    - /experiment full experiment monitoring panel
            │                                  #    - Automatic experiment data logging (CSV + JSONL)
            │
            ├── start_band10_full.cmd         # One-click launch (two windows: relay + BLE)
            ├── start_relay_with_board.cmd    # Standalone relay service (with board forwarding)
            └── start_mi_band_10_ble_to_pc.cmd # Standalone BLE bridge
```

---

## Hardware & Software Requirements

### RDK X5 Development Board

| Category | Requirement |
|----------|-------------|
| Hardware Platform | RDK X5 Development Board |
| Camera | MIPI CSI Camera (ROS2 `mipi_cam` driver, 640×480) |
| Audio Output | USB sound card / onboard audio (device must be recognized by `aplay`) |
| Operating System | Linux + TROS Humble |
| Python | Python 3.10 (`.venv_rdkx5` virtual environment, see below) |
| Network | Same LAN as the PC (reference HR forwarding relies on HTTP communication) |

### Windows PC

| Category | Requirement |
|----------|-------------|
| Operating System | Windows 10 / 11 |
| Python | Python 3 |
| Bluetooth | BLE (Bluetooth Low Energy) capable adapter |
| Network | Same LAN as the development board |

### Reference Heart Rate Device

| Device | Notes |
|--------|-------|
| Xiaomi Band 10 | Enable "Heart Rate Broadcast" on the band (Settings → Heart Rate Broadcast → On) |

### Board Virtual Environment (`.venv_rdkx5`)

`rdk_rppg_supervisor.sh` invokes Python via `$PORT_DIR/.venv_rdkx5/bin/python`. The virtual environment is located at the `rdkx5_port/` root, alongside `rppg_runtime/`:

```
rdkx5_port/
├── .venv_rdkx5/          # Python 3.10 virtual environment (include-system-site-packages = true)
│   └── lib/python3.10/site-packages/
│       ├── mediapipe/    # 0.10.9 (Apache 2.0)
│       ├── absl_py/      # 2.4.0
│       └── flatbuffers/  # 25.12.19
└── rppg_runtime/         # rPPG runtime code

```

The virtual environment is configured with `include-system-site-packages = true`, meaning it inherits system-global packages. The following dependencies come from the RDK X5 system environment: OpenCV, NumPy, SciPy, ROS2 (`rclpy`, `sensor_msgs`), `sherpa-onnx`, `requests`.

---

## Dependencies

### Board Side (RDK X5)

| Dependency | Purpose |
|------------|---------|
| `rclpy` | ROS2 Python client library, subscribes to MIPI camera topics |
| `sensor_msgs.msg.Image` | ROS2 image message type |
| `opencv-python` (`cv2`) | Image processing, JPEG encoding, polygon ROI extraction |
| `mediapipe` | FaceMesh facial landmark detection (468 points) |
| `numpy` | Numerical computation, signal processing |
| `scipy` | Butterworth filter design & zero-phase filtering (`butter`, `filtfilt`) |
| `sherpa-onnx` | Matcha-TTS ONNX model inference engine |
| `requests` | TTS client HTTP calls |
| `aplay` (ALSA) | Audio playback (system command) |

### PC Side (Windows)

| Dependency | Purpose |
|------------|---------|
| `bleak` | BLE async scanning, connection & data subscription |
| `asyncio` | Async event loop (Python standard library) |
| `http.server` | HTTP server (Python standard library) |

---

## Quick Start

### 1. Board Side (RDK X5)

#### 1.1 One-Click Launch via Supervisor (Recommended)

```bash
# Set project root directory (adjust to your actual path)
export PROJECT_DIR=$HOME/workspace_rdkx5/CV_Project

# Optional: adjust analysis window duration
export SIGNAL_SECONDS=10

# Launch (camera mode)
bash rppg_runtime/board/rdk_rppg_supervisor.sh
```

> ⚠️⚠️⚠️⚠️⚠️ **Note**: The directory structure of this open-source project differs slightly from the actual deployment layout on the RDK X5 board. `rdk_rppg_supervisor.sh` internally assembles component launch paths using variables such as `$PORT_DIR`. Before use, adjust the following configurations in the script to match the actual board directory layout:
>
> - `PORT_DIR` — Project root directory path (the script defaults to the board's actual path)
> - `PYTHON_BIN` — Python interpreter path (defaults to `.venv_rdkx5/bin/python`)
> - Component script/log paths
>
> It is recommended to edit `rdk_rppg_supervisor.sh` directly on the board, changing the above variables to paths consistent with the actual board deployment before running.

The Supervisor starts the following in sequence:
1. MIPI Camera ROS2 Node
2. TTS Voice Service (port 7878, model loading takes ~10–20 seconds)
3. rPPG Inference Server (port 8080)

After successful startup, visit `http://<board-IP>:8080/` in a browser to view the real-time dashboard.

#### 1.2 Manual Component Startup

```bash
# Terminal 1: Start TTS voice service
cd rppg_runtime/board
python3 tts_service.py &

# Terminal 2: Start rPPG server (camera mode)
python3 rppg_runtime/board/rdk_rppg_server.py \
    --source-mode camera \
    --topic /image_raw \
    --port 8080 \
    --signal-seconds 10
```

#### 1.3 Command-Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--source-mode` | `camera` | Data source mode: `camera` |
| `--topic` | `/image_raw` | ROS2 camera topic name (camera mode) |
| `--port` | `8080` | HTTP server port |
| `--signal-seconds` | `10` | Signal analysis window duration (seconds, range 8–60) |
| `--resize-width` | `640` | Frame resize width |
| `--jpeg-quality` | `78` | Dashboard JPEG snapshot quality |
| `--title` | — | Web dashboard title |

---

### 2. PC Side (Windows)

#### 2.1 One-Click Launch (Recommended)

Double-click or run in a terminal:

```cmd
rppg_runtime\phone_bridge\windows\start_band10_full.cmd
```

This script launches the relay service and BLE bridge in two separate console windows.

#### 2.2 Separate Launch

```cmd
:: Terminal 1: Start relay service (with board forwarding)
python rppg_runtime\phone_bridge\windows\reference_hr_relay.py \
    --listen-host 0.0.0.0 \
    --listen-port 8090 \
    --board-url http://10.77.3.84:8080

:: Terminal 2: Start BLE bridge
python rppg_runtime\phone_bridge\windows\mi_band_10_ble_to_pc.py \
    --relay-url http://127.0.0.1:8090
```

#### 2.3 Command-Line Arguments

**reference_hr_relay.py**

| Argument | Default | Description |
|----------|---------|-------------|
| `--listen-host` | `0.0.0.0` | HTTP listen address |
| `--listen-port` | `8090` | HTTP listen port |
| `--board-url` | `http://10.77.3.84:8080` | Board rPPG service address (leave empty to disable forwarding) |
| `--timeout` | `1.0` | HTTP request timeout (seconds) |
| `--no-board-forward` | — | Disable forwarding reference HR to the board |

**mi_band_10_ble_to_pc.py**

| Argument | Default | Description |
|----------|---------|-------------|
| `--relay-url` | `http://127.0.0.1:8090` | Relay service address |
| `--repeat-seconds` | `2.0` | Repeat send interval (seconds) |
| `--scan-seconds` | `10.0` | BLE scan timeout (seconds) |
| `--source-name` | `mi_band_10_ble` | Heart rate data source identifier |
| `--scan-only` | — | Only scan nearby BLE devices then exit |
| `--check-only` | — | Only check if band is connectable then exit |

#### 2.4 Pre-Use Preparation

1. Ensure PC Bluetooth is enabled
2. Enable Heart Rate Broadcast on Xiaomi Band 10: "Settings" → "Heart Rate Broadcast" → On
3. Keep the band within 5 meters of the PC

---

## rPPG Algorithm Details

The core algorithm is implemented in [rppg_runtime/board/rdk_rppg_server.py](rppg_runtime/board/rdk_rppg_server.py). The processing pipeline is as follows:

### Phase 1: Face Detection & ROI Extraction

```
Camera Frame (ROS2 /image_raw)
       │
       ▼
MediaPipe FaceMesh (468 landmarks, confidence threshold 0.5)
       │
       ├──▶ 468 facial landmark coordinates
       │
       ├──▶ Orientation Detection: horizontal position of nose tip within eye bounding box → frontal_score
       │    (frontal_score < 0.35 means face is not facing the camera, ROI is skipped)
       │
       └──▶ Three ROI polygon regions (based on facial mesh indices):
            ├── Forehead:    indices [9, 107, 66, 105, 104, 103, 67, 109, 10, 338, 297, 332, 333, 334, 296, 336]
            ├── Left Cheek:  indices [132, 58, 172, 136, 150, 169, 210, 212, 202, 57]
            └── Right Cheek: indices [361, 288, 397, 365, 379, 394, 430, 432, 422, 287]
```

### Phase 2: RGB Signal Extraction

For each ROI region, `cv2.fillPoly` + `cv2.mean` is used to compute the region mean. The three regions are averaged to obtain frame-level `(R, G, B)` raw signals. Raw signals are buffered in a FIFO queue (`samples`), with window length controlled by the `signal_seconds` parameter (default 10 seconds).

### Phase 3: rPPG Signal Construction

After per-channel mean subtraction and normalization, three rPPG variants are constructed:

| Method | Formula | Principle |
|--------|---------|------------|
| **Green** | `S_green = N_g` | Green channel signal, classic photoplethysmography |
| **POS** | `X = N_g − N_b`<br>`Y = −2N_r + N_g + N_b`<br>`α = σ(X) / σ(Y)`<br>`S_pos = X + α·Y` | Skin-tone plane orthogonal projection, motion artifact suppression |
| **CHROM** | `X = 3N_r − 2N_g`<br>`Y = 1.5N_r + N_g − 1.5N_b`<br>`α = σ(X) / σ(Y)`<br>`S_chrom = X − α·Y` | Chrominance model, specular reflection suppression |

> Where `N_r, N_g, N_b` are per-channel mean-subtracted and normalized signals.

### Phase 4: Signal Processing

```
Resampling (10 FPS linear interpolation)
       │
       ▼
Detrending (moving-average baseline removal, window = 1.6s)
       │
       ▼
Normalization (divide by standard deviation)
       │
       ▼
2nd-order Butterworth Bandpass Filter (0.7–3.0 Hz, corresponding to 42–180 BPM)
  - Implemented with scipy.signal.butter + filtfilt for zero-phase filtering
```

### Phase 5: Spectral Analysis & Peak Selection

```
Hanning Window FFT (N_FFT = 1024)
       │
       ▼
Sort by SNR (peak power / band median power), take Top-5 peaks
  - Adjacent peak separation ≥ 6 BPM
       │
       ▼
Peak Scoring (per peak):
  ├── Sub-harmonic Penalty: if BPM < 70 and strong peak exists at 2×BPM → score × 0.45
  ├── Boundary Penalty: if BPM near lower bound and high-frequency strong peak exists → score × 0.40
  └── Harmonic Support: if BPM ≥ 80 and strong peak exists at 0.5×BPM → score × 1.18
```

### Phase 6: Multi-Method Fusion & Clustering

For candidate peaks from Green / POS / CHROM:

1. All peaks sorted by weighted score (SNR + method bias)
2. Clustering within 6 BPM separation (weighted average BPM)
3. Cluster score = Σ member scores × (1 + 0.12 × (method count − 1))
   - Multi-method agreement clusters receive 12% bonus
4. Highest-scoring cluster selected as BPM estimate

### Phase 7: Trajectory Tracking & Stabilization

```
Raw BPM Estimate
       │
       ├── Hard Ceiling Filter: > 110 BPM → reject
       │
       ├── Large Jump Interception: |ΔBPM| > 12 → retain previous frame value
       │
       ├── Local Peak Correction: search for supporting peaks within ±18 BPM of previous frame
       │
       ├── Jump Confirmation Sequence: small jumps require 3 consecutive frames to confirm
       │
       └── Exponential Smoothing: smoothing_factor = 0.2 × SNR adaptive
```

---

## HTTP API Reference

### Board Service (Port 8080)

| Endpoint | Method | Content-Type | Description |
|----------|--------|-------------|-------------|
| `/` | GET | `text/html` | Web real-time dashboard (camera feed, BPM value, signal waveform) |
| `/status` | GET | `application/json` | System status JSON (BPM, face count, RGB means, params, reference HR, etc.) |
| `/params` | GET | `application/json` | Get current parameter configuration (`signal_seconds`, `min_bpm`, `max_bpm`) |
| `/params` | POST | `application/json` | Update parameters (request body: `{"signal_seconds": 12, "min_bpm": 50}`) |
| `/signal` | GET | `application/json` | Last 240 RGB sample points (for frontend waveform rendering) |
| `/snapshot.jpg` | GET | `image/jpeg` | Latest frame JPEG snapshot (with FaceMesh overlay) |
| `/reference_hr` | GET | `application/json` | Query current reference heart rate value |
| `/reference_hr` | POST | `application/json` | Receive external reference HR (request body: `{"source": "...", "bpm": 72}`) |

**`/status` Response Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `bpm` | number\|null | Current estimated heart rate (BPM) |
| `bpm_state` | string | Heart rate state: `"tracking"` / `"rough"` / `"collecting"` / `"high_outlier"` |
| `face_count` | number | Number of faces currently detected |
| `frontal_score` | number | Face orientation score (0–1, higher = more frontal) |
| `rgb` | object | RGB mean values of ROI region in current frame |
| `age_sec` | number | Seconds elapsed since the current JPEG snapshot was captured |
| `seq` | number | Frame sequence number |
| `params` | object | Current parameter configuration |
| `reference_hr` | object | Latest reference heart rate data |

---

### Relay Service (Port 8090)

| Endpoint | Method | Content-Type | Description |
|----------|--------|-------------|-------------|
| `/` | GET | `text/html` | Heart rate status page |
| `/experiment` | GET | `text/html` | ★ Full experiment monitoring panel |
| `/status` | GET | `application/json` | Latest reference HR + board status (aggregated) |
| `/events` | GET | `application/json` | Recent heart rate event history (last 100 entries) |
| `/reference_hr` | GET | `application/json` | Query latest reference HR |
| `/reference_hr` | POST | `application/json` | Receive BLE-bridged HR data and forward to board |
| `/board_params` | POST | `application/json` | Proxy parameter changes to board (passthrough `/params`) |

**Experiment Panel (`/experiment`) Features:**
- Real-time board camera feed
- rPPG BPM and reference BPM side-by-side display
- BPM difference (rPPG − reference) real-time error
- Signal window and BPM range online tuning
- Automatic experiment data logging (CSV + JSONL)

---

## Configuration Parameters

### Runtime Parameters (dynamically adjustable via HTTP `/params`)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `signal_seconds` | `10` | 8–60 | Signal analysis window duration (seconds). Larger = more stable but slower response |
| `min_bpm` | `45` | 30–60 | Minimum detectable heart rate (BPM) |
| `max_bpm` | `110` | 80–180 | Maximum detectable heart rate (BPM) |
| `target_fps` | `10` | fixed | Signal resampling frame rate (Hz) |

### Algorithm Constants (hardcoded, restart required to change)

| Constant | Value | Description |
|----------|-------|-------------|
| `PEAK_MIN_SEPARATION_BPM` | 6.0 | Minimum adjacent peak separation (BPM) |
| `TOP_PEAK_COUNT` | 5 | Number of candidate peaks retained per method |
| `TRACK_NEAR_BPM` | 18.0 | Trajectory tracking search range (BPM) |
| `TRACK_MAX_JUMP_BPM` | 10.0 | Maximum allowed single-frame jump (BPM) |
| `TRACK_SMOOTHING` | 0.2 | Exponential smoothing coefficient |
| `TRACK_CONFIRM_FRAMES` | 3 | Consecutive frames required for jump confirmation |
| `MIN_ANALYSIS_SECONDS` | 9.0 | Minimum data duration before analysis begins (seconds) |
| `n_fft` | 1024 | FFT points |
| `BUTTER_ORDER` | 2 | Butterworth filter order |
| `BANDPASS_LOW` | 0.7 Hz | Bandpass filter lower bound (42 BPM) |
| `BANDPASS_HIGH` | 3.0 Hz | Bandpass filter upper bound (180 BPM) |

### Supervisor Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `PROJECT_DIR` | `$HOME/workspace_rdkx5/CV_Project` | Project root directory |
| `SOURCE_MODE` | `camera` | Data source: `camera` |
| `SIGNAL_SECONDS` | `10` | Signal analysis window duration |

---

## Voice Interaction Flow

The rPPG server includes a built-in voice state machine (`VoiceManager`) that automatically plays voice prompts based on detection status:

```
State Flow:

  No face / face not frontal
    │  ("Please face the camera" every 6s)
    │
    ▼
  Face detected and frontal (frontal_score ≥ 0.35) for 2 seconds
    │  Plays "Please wait"
    │
    ▼
  Valid heart rate stable for 1.5 seconds
    │  Plays "Measurement successful"
    │
    ▼
  Periodic TTS broadcast (every 8 seconds)
    │  "Your current heart rate is 88."
    │
    ▼
  Face lost / HR lost for > 1 second → return to initial state
```

Voice playback uses `aplay -D <device>` to output through the USB audio device. Each voice task runs in a separate thread to avoid blocking the main inference loop. TTS synthesis is performed locally via `sherpa-onnx` — no internet connection is required.

---

## Experiment Data Logging

The relay service (`reference_hr_relay.py`) automatically logs data to the `artifacts/experiment_logs/` directory in experiment mode.

### File Naming Convention

```
artifacts/experiment_logs/
├── experiment_20260624_143052.csv     # CSV format experiment log
└── experiment_20260624_143052.jsonl   # JSONL format experiment log
```

> The filename timestamp corresponds to the experiment start time.

### Record Fields

Each record contains the following key fields (see `ExperimentLogger.log_snapshot()` in the code for the complete list):

| Field | Description |
|-------|-------------|
| `timestamp_iso` / `timestamp_unix` | PC-side recording time |
| `board_bpm` | Current estimated heart rate from the board (BPM) |
| `board_bpm_state` | Heart rate state (tracking / rough / collecting, etc.) |
| `board_face_count` | Number of detected faces |
| `board_r` / `board_g` / `board_b` | ROI region RGB mean values |
| `board_peak_bpm` / `board_peak_snr` | Primary peak BPM and SNR |
| `board_peak1_bpm` ~ `board_peak3_snr` | Top-3 candidate peaks |
| `board_signal_method` | Selected signal method (green / pos / chrom) |
| `reference_bpm` | Band reference heart rate |
| `delta_bpm` | Difference between rPPG and reference HR |
| `signal_seconds` / `min_bpm` / `max_bpm` | Current parameter configuration |
| `pc_seq` / `pc_interval_sec` | PC-side reception sequence number and interval |
| `board_error` | Board connection error message (if any) |

---

## Model Sources & License

### Matcha-TTS Chinese Female Voice Model

- **Training Data**: [Baker Chinese Standard Female Voice Dataset](https://en.data-baker.com/datasets/freeDatasets/) (~12 hours, 10,000 sentences)
- **Training Code**: [icefall TTS](https://github.com/k2-fsa/icefall/tree/master/egs/baker_zh/TTS)
- **Model Format**: ONNX (inference via sherpa-onnx)
- **License Type**: **Non-commercial use only** (per Baker dataset restrictions)

> ⚠️ **Important**: The Baker dataset license restricts usage to non-commercial purposes. If this project is used in a commercial scenario, please replace it with a commercially-licensed TTS model.

---
