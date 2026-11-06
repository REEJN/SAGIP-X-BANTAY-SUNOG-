# BANTAY-SUNOG: Raspberry Pi 3B Real-Time Fire Detection System

**Autonomous Indoor Firefighting Robot — Vision Subsystem**

BANTAY-SUNOG is an embedded computer vision application running entirely locally on a **Raspberry Pi 3 Model B (1 GB RAM)**. It captures live video from a standard USB webcam, detects visible flames using an on-device lightweight inference engine (or experimental HSV fallback), validates detections across multiple frames to eliminate false triggers, and transmits structured JSON telemetry to an **ESP32-WROOM** navigation controller over a wired 3.3V UART serial connection.

---

## 1. System Architecture

```
+-------------------------------------------------------------------+
|                         BANTAY-SUNOG ROBOT                        |
|                                                                   |
|   +----------------+                                              |
|   |   USB Webcam   |                                              |
|   +--------+-------+                                              |
|            | USB 2.0                                              |
|            v                                                      |
|   +-----------------------------------------------------------+   |
|   |                 Raspberry Pi 3 Model B                    |   |
|   |  - Video acquisition (OpenCV V4L2)                        |   |
|   |  - On-device detection (TFLite INT8 / HSV Fallback)       |   |
|   |  - 5-frame sliding window confirmation state machine      |   |
|   |  - Bounding box & normalized target center calculation   |   |
|   |  - Full UART TX terminal telemetry logging                |   |
|   |  - System resource & FPS performance monitoring           |   |
|   +----------------------------+------------------------------+   |
|                                |                                  |
|                                | Wired UART (3.3V TTL, 115200)    |
|                                v                                  |
|   +-----------------------------------------------------------+   |
|   |                    ESP32-WROOM Controller                 |   |
|   |  - Obstacle avoidance & autonomous navigation             |   |
|   |  - 4x DC drive motor PWM control                          |   |
|   |  - Aiming servo positioning                               |   |
|   |  - Fire extinguisher suppression solenoid activation      |   |
|   +-----------------------------------------------------------+   |
+-------------------------------------------------------------------+
```

### Strict Architectural Boundaries
- **No Wi-Fi or Cloud Dependency:** Operates 100% offline in disconnected environments.
- **Vision Subsystem Only:** The Raspberry Pi NEVER controls drive motors, steering servos, or fire-extinguishing solenoids. All actuator operations belong exclusively to the ESP32.
- **Image Coordinates Only:** Normalised target coordinates $(X, Y) \in [0.0, 1.0]$ represent relative position within the camera field of view, NOT physical distances.

---

## 2. Hardware Wiring & Pinout

### Raspberry Pi 3B (40-Pin Header) to ESP32-WROOM

| Raspberry Pi 3B Pin | Pi Function | ESP32-WROOM Pin | Description |
|---|---|---|---|
| **Pin 8** | `GPIO 14` (UART0 TXD) | **RX2** (GPIO 16) | Serial data from Pi to ESP32 |
| **Pin 10** | `GPIO 15` (UART0 RXD) | **TX2** (GPIO 17) | Serial data from ESP32 to Pi |
| **Pin 6** | `GND` | **GND** | Common Ground reference |

> [!CAUTION]
> **ELECTRICAL HAZARD — 3.3V LOGIC ONLY**:
> Raspberry Pi GPIO pins are strictly 3.3V logic tolerant and are **NOT 5V tolerant**. Connecting a 5V signal directly to GPIO 14/15 will permanently damage the BCM2837 SoC. The ESP32-WROOM operates at 3.3V logic, making direct wiring safe, but ensure common ground is connected.

---

## 3. Raspberry Pi OS UART Configuration

On the Raspberry Pi 3B, the onboard Bluetooth controller uses the primary hardware UART (`ttyAMA0`), leaving GPIO 14/15 routed to the clock-dependent Mini UART (`ttyS0`). To achieve stable, jitter-free 115200 baud communication, configure the Pi to assign the PL011 hardware UART to the GPIO header.

### Step 1: Disable Serial Linux Console
Disable the kernel boot console on UART so debug messages are not sent to the ESP32:
```bash
sudo raspi-config
# Navigate to: 3 Interface Options -> I6 Serial Port
# Would you like a login shell to be accessible over serial? -> NO
# Would you like the serial port hardware to be enabled?    -> YES
```

Alternatively, manually edit `/boot/firmware/cmdline.txt` (or `/boot/cmdline.txt`) and remove any occurrences of:
```
console=serial0,115200
```
Keep `console=tty1`.

### Step 2: Route Hardware UART to GPIO Pins
Edit `/boot/firmware/config.txt` (or `/boot/config.txt` on older OS):
```ini
[all]
enable_uart=1
dtoverlay=disable-bt
```
Then disable the Bluetooth system service:
```bash
sudo systemctl disable hciuart
sudo reboot
```

After rebooting, `/dev/serial0` will link directly to `/dev/ttyAMA0`.

---

## 4. Software Installation on Raspberry Pi

### Prerequisites (Raspberry Pi OS Lite 64-bit / Debian 12+)
```bash
sudo apt update && sudo apt install -y \
    python3 \
    python3-pip \
    python3-opencv \
    python3-serial \
    python3-psutil \
    v4l-utils
```

### Clone and Setup
```bash
cd ~
git clone https://github.com/your-repo/bantay_sunog.git
cd bantay_sunog
```

*(Optional: Install Python packages into a virtual environment if PEP 668 is enforced):*
```bash
python3 -m venv --system-site-packages ~/bantay_env
source ~/bantay_env/bin/activate
pip install -r requirements.txt
```

---

## 5. Detection Engines & Model Setup

The system supports three detection backends configured in `config.py` or via CLI `--detector`:

### 1. `tflite` (Recommended for Production)
Uses TensorFlow Lite with ARM NEON SIMD acceleration.
- Place your INT8 quantized SSD MobileNet model at `models/fire_detect.tflite`.
- Place class labels at `models/labels.txt` (e.g. `fire`).
- Refer to [`models/README.md`](models/README.md) for full Google Colab training and INT8 quantization instructions.

### 2. `hsv_fallback` (Built-in Heuristic)
Does not require any neural network model file. Uses:
- Dual-band HSV color masking for yellow/orange/red flame tones.
- OpenCV MOG2 background subtractor to detect dynamic flame flicker/motion.
- Contour area and aspect ratio filtering.
- *Status:* **EXPERIMENTAL**. Susceptible to false positives on orange clothing or bright light sources.

### 3. `simulation` (Headless Benchmarking)
Generates randomized synthetic flame targets without requiring a camera or model. Ideal for automated testing and validating the ESP32 UART parser.

---

## 6. Running the Application

### Headless Production Mode (Default)
Standard operation when running on the robot via systemd or SSH:
```bash
python3 -m bantay_sunog.main
```

### Simulation Mode (No Camera or Fire Required)
Generates test detections and transmits real JSON packets over UART:
```bash
python3 -m bantay_sunog.main --simulate
```

### Visual Preview Mode (With Graphical Display)
Opens an OpenCV window with bounding boxes, center crosshair, FPS, and state overlays:
```bash
python3 -m bantay_sunog.main --preview
```
*(Press `q` or `ESC` in the preview window to quit).*

### Live Web Browser Preview (View on Laptop)
Streams live video with bounding boxes, crosshairs, state badge, and real-time telemetry to your laptop browser over your local network:
```bash
python3 -m bantay_sunog.main --web
```
Open your laptop browser and navigate to:
```
http://192.168.137.200:5000/
```
*(Optional: Use `--port 8080` to change the listening port).*

### Debug Mode
Enables verbose per-frame detection coordinates:
```bash
python3 -m bantay_sunog.main --debug
```

---

## 7. UART Communication Protocol

All messages are JSON objects formatted on a single line and terminated with a newline character (`\n`).

### 1. Fire Confirmed (`FIRE_CONFIRMED`)
Transmitted immediately upon multi-frame confirmation, and periodically (default: every 1.0 s) while target remains visible.
```json
{
  "event": "FIRE_CONFIRMED",
  "confidence": 0.91,
  "x": 0.67,
  "y": 0.42,
  "bbox": [180, 90, 120, 160],
  "frame": [640, 480],
  "timestamp": 1730000000
}
```
- `x`, `y`: Normalized center coordinates where `(0.5, 0.5)` is the camera optical center.
- `bbox`: `[x, y, width, height]` in raw pixel coordinates.
- `frame`: `[frame_width, frame_height]`.

### 2. Fire Candidate (`FIRE_CANDIDATE`)
Transmitted when a flame is observed but has not yet met the temporal confirmation threshold (e.g., 1 or 2 positive frames).
```json
{
  "event": "FIRE_CANDIDATE",
  "confidence": 0.74,
  "x": 0.55,
  "y": 0.48,
  "timestamp": 1730000000
}
```

### 3. No Fire (`NO_FIRE`)
Transmitted once when all fire candidates clear.
```json
{
  "event": "NO_FIRE",
  "timestamp": 1730000000
}
```

### 4. Fire Target Lost (`FIRE_LOST`)
Transmitted when a previously confirmed fire target disappears for longer than `FIRE_LOSS_TIMEOUT` (default: 2.0 s).
```json
{
  "event": "FIRE_LOST",
  "reason": "detection_timeout",
  "timestamp": 1730000000
}
```

### 5. Error Alert (`ERROR`)
Transmitted upon camera, model, or hardware fault.
```json
{
  "event": "ERROR",
  "source": "CAMERA",
  "message": "Camera frame acquisition failed",
  "timestamp": 1730000000
}
```

### 6. ESP32 Return Status Telemetry (ESP32 -> Pi)
Transmitted periodically (default: every 200 ms) by the ESP32 controller over TX2 (`GPIO 17`) to Pi RXD (`GPIO 15`):
```json
{
  "robot_state": "APPROACHING",
  "dist_front": 32.5,
  "dist_left": 84.2,
  "dist_right": 71.0,
  "flame_detected": true,
  "tilt_angle": 88,
  "solenoid_active": false,
  "left_pwm": 160,
  "right_pwm": 160,
  "uptime": 14250
}
```
- Firmware implementation and full hardware wiring guide: [`../esp32_firmware/README.md`](../esp32_firmware/README.md).

---

## 8. Command-Line Terminal Logging

Every outgoing UART message is logged to the terminal in real-time with an indented JSON preview:

```text
[BANTAY-SUNOG] Initializing Raspberry Pi fire-detection subsystem...
[SYSTEM] Raspberry Pi 3B detected.
[CAMERA] Searching for USB webcam...
[CAMERA] USB webcam connected: device 0
[CAMERA] Actual: 640x480 @ 30 FPS
[MODEL] Loading fire-detection model...
[MODEL] Model loaded successfully.
[UART] Initializing serial communication on /dev/serial0 at 115200 baud...
[UART] Connected to ESP32-WROOM on /dev/serial0
[SYSTEM] Fire-detection system READY.

[08:30:01.100] [VISION] Fire candidate detected.
[08:30:01.140] [VISION] Fire confirmed (3/5 frames).
[08:30:01.150] [UART TX] {
  "event": "FIRE_CONFIRMED",
  "confidence": 0.91,
  "x": 0.67,
  "y": 0.42,
  "bbox": [180, 90, 120, 160],
  "frame": [640, 480],
  "timestamp": 1730000000
}
[08:30:01.160] [UART] Message written successfully to UART.
```

---

## 9. Performance Benchmark on Raspberry Pi 3B

| Metric | Target Specification | Measured on Pi 3B (ARM Cortex-A53) |
|---|---|---|
| **CPU Clock** | 1.2 GHz Quad-core | 1.2 GHz (100% online) |
| **RAM Usage** | < 1 GB | ~160 MB active system-wide |
| **Inference FPS (HSV Fallback)** | > 10 FPS | **14–18 FPS** (~55–70 ms) |
| **Inference FPS (TFLite INT8 MobileNet)** | ~3–6 FPS | **3.5–5.2 FPS** (~190–280 ms) |
| **UART Transmission Jitter** | < 5 ms | < 1.2 ms |

---

## 10. Automated Test Suite

Run the full automated test suite (mocking camera and serial):
```bash
python3 -m unittest discover -s bantay_sunog/tests -v
```

All 24 unit tests cover:
- Temporal confirmation 3-of-5 sliding window logic
- Target coordinate normalization and boundary clamping
- Duplicate suppression for NO_FIRE / repeated events
- Rate limiting and minimum transmission intervals
- Buffer draining to guarantee newest-frame processing
- Camera disconnection and recovery state transitions
