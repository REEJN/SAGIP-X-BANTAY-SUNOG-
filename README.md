# SAGIP-X: BANTAY-SUNOG (Autonomous Indoor Firefighting Robot)

[![C++](https://img.shields.io/badge/Firmware-ESP32%20C%2B%2B-blue.svg)](esp32_firmware/)
[![Python](https://img.shields.io/badge/Vision%20%26%20AI-Python%203.9%2B-brightgreen.svg)](bantay_sunog/)
[![Three.js](https://img.shields.io/badge/Digital%20Twin-Three.js%20WebGL-orange.svg)](simulation/frontend/)
[![TinyML](https://img.shields.io/badge/TinyML-818%20Params%20%7C%20%3C4KB-purple.svg)](simulation/models/)
[![Unit Tests](https://img.shields.io/badge/Tests-27%2F27%20Passing-success.svg)](simulation/tests/)

**SAGIP-X: BANTAY-SUNOG** is a dual-tier autonomous indoor firefighting robotic system engineered for rapid localization, obstacle evasion, visual fire detection, and targeted suppression in GPS-denied environments.

The system decouples high-level computer vision and low-level real-time actuation between a **Raspberry Pi 3 Model B** and an **ESP32-WROOM-32** microcontroller, verified end-to-end inside an interactive **3D Digital Twin Simulation & SLAM Platform**.

---

## 📑 Table of Contents
- [1. System Architecture](#1-system-architecture)
- [2. Key Capabilities & Engineering Highlights](#2-key-capabilities--engineering-highlights)
- [3. Hardware Specifications & Pinout](#3-hardware-specifications--pinout)
- [4. Repository Directory Structure](#4-repository-directory-structure)
- [5. Getting Started](#5-getting-started)
  - [A. 3D Digital Twin Simulation (Browser + SIL)](#a-3d-digital-twin-simulation-browser--sil)
  - [B. Running Automated Unit Tests](#b-running-automated-unit-tests)
  - [C. Retraining the TinyML Navigation Model](#c-retraining-the-tinyml-navigation-model)
  - [D. Flashing the ESP32 Firmware](#d-flashing-the-esp32-firmware)
  - [E. Deploying to Raspberry Pi 3B](#e-deploying-to-raspberry-pi-3b)
- [6. Testing & Validation Status](#6-testing--validation-status)
- [7. Authors & License](#7-authors--license)

---

## 1. System Architecture

```
+---------------------------------------------------------------------------------------+
|                                  BANTAY-SUNOG ROBOT                                   |
|                                                                                       |
|   +-----------------------+                                                           |
|   |   USB HD Webcam       |                                                           |
|   +-----------+-----------+                                                           |
|               | USB 2.0 Video Stream                                                  |
|               v                                                                       |
|   +-------------------------------------------------------------------------------+   |
|   |                        Raspberry Pi 3 Model B (Vision SBC)                    |   |
|   |  - Video acquisition (OpenCV V4L2 pipeline)                                   |   |
|   |  - Fire detection (TFLite INT8 Quantized / Experimental HSV Fallback)         |   |
|   |  - 5-frame sliding window confirmation state machine                          |   |
|   |  - Normalized target coordinate extraction (X, Y) in [0.0, 1.0]               |   |
|   |  - High-speed UART telemetry logger                                           |   |
|   +---------------------------------------+---------------------------------------+   |
|                                           |                                           |
|                                           | Wired 3.3V TTL UART (115200 Baud)         |
|                                           v                                           |
|   +-------------------------------------------------------------------------------+   |
|   |                        ESP32-WROOM-32 (Actuation & Nav)                       |   |
|   |  - TinyML Neural Local Navigation Policy (~818 params, < 4 KB footprint)      |   |
|   |  - Rule-Based SIL / Finite State Machine (SEARCH, TRACK, SUPPRESS, BACKTRACK) |   |
|   |  - 3x HC-SR04 Ultrasonic Distance Array (Front, Left, Right)                 |   |
|   |  - LM393 Optical IR Flame Sensor (Secondary Physical Safety Interlock)       |   |
|   |  - TB6612FNG Dual H-Bridge Motor Driver (4x DC Drive Motors with 20kHz PWM)  |   |
|   |  - Micro Servo Turret (Webcam / Extinguisher Nozzle Pitch Control)            |   |
|   |  - Extinguisher Solenoid Valve Control via IRLZ44N Logic-Level MOSFET         |   |
|   +-------------------------------------------------------------------------------+   |
+---------------------------------------------------------------------------------------+
```

### Architectural Principles
1. **100% Offline Autonomy**: Zero cloud or internet connectivity dependency. Operates completely in smoke-filled, offline, or communications-blackout indoor structures.
2. **Strict Subsystem Isolation**: The Raspberry Pi handles vision inference and feeds target coordinates over UART. Actuator safety, motor PWM, emergency stopping, and solenoid suppression belong exclusively to the ESP32.
3. **Multi-Modal Fire Confirmation**: Fire suppression triggers only when **both** the camera vision pipeline and the physical optical IR flame sensor confirm the fire at safe standoff distance ($\le 0.48\text{m}$).

---

## 2. Key Capabilities & Engineering Highlights

### 🧠 TinyML Local Navigation Policy
- **Ultra-Compact MLP**: 7 inputs $\rightarrow$ 32 ReLU $\rightarrow$ 16 ReLU $\rightarrow$ 2 continuous motor PWM outputs ($L_{\text{pwm}}, R_{\text{pwm}} \in [-255, +255]$).
- **Embedded Footprint**: Exactly **818 parameters** (~3.2 KB RAM), fitting easily within the ESP32's 320 KB SRAM alongside FreeRTOS tasks.
- **Multi-Platform Zero-Overhead Inference**:
  - Embedded C++ header: [`bantay_sunog_esp32/navigation_model.h`](bantay_sunog_esp32/navigation_model.h)
  - PlatformIO C++: [`esp32_firmware/include/navigation_model.h`](esp32_firmware/include/navigation_model.h)
  - PyTorch TorchScript: [`simulation/models/navigation_model.pt`](simulation/models/navigation_model.pt)
  - JavaScript WebGL: [`simulation/frontend/js/nav_model_runner.js`](simulation/frontend/js/nav_model_runner.js)

### 🗺️ 2D Ultrasonic SLAM & Topological Room Exploration
- **Persistent Occupancy Grid**: $45 \times 45$ grid ($0.20\text{m}$ resolution, $9\text{m} \times 9\text{m}$ bounds) consuming $< 2.5\text{ KB}$ RAM.
- **Hierarchical Frontier Classification**: Differentiates local frontiers within the current room (`dist <= 2.5m` + direct Line-of-Sight) from inter-room doorway frontiers.
- **Dead-End 180° Backtracking**: Detects room boundaries and cul-de-sacs. When a room is fully explored or a dead-end wall is reached, initiates an in-place 180° U-turn to navigate directly to open doorways into undiscovered rooms.
- **Bresenham Line-of-Sight Raycasting**: Penalizes occluded frontiers ($-900.0$ penalty) and prioritizes direct line-of-sight corridors ($+250.0$ bonus).
- **Waypoint Obstacle Inflation Pusher**: Pushes waypoints away from obstacle vertices to enforce a strict $\ge 0.28\text{m}$ ($11\text{ inches}$) chassis clearance margin.

### 🌐 3D Digital Twin Simulation Platform
- **WebGL 3D Twin**: Fully interactive Three.js simulator with realistic chassis kinematics, sensor raycasting, and flame particle systems.
- **Dynamic Layouts**:
  - `Arena Box`: Open perimeter testing.
  - `Multi-Room Facility`: Compartmentalized rooms, corridors, and doorways.
  - `Maze Labyrinth`: Narrow paths and sharp $90^\circ$ turns.
  - `Editable Sandbox Inspector`: Click and drag obstacles, change dimensions, and reorient walls in real time.
- **Layer 1 Camera Isolation**: Separates path prediction lines and debug gizmos from the virtual webcam feed, ensuring the computer vision detector receives clean, realistic camera frames without false triggers.

---

## 3. Hardware Specifications & Pinout

### ESP32-WROOM-32 Pin Assignment

| Peripheral | ESP32 Pin | GPIO | Direction | Connected To | Notes / Electrical Protection |
|---|---|---|---|---|---|
| **Pi Serial RX** | Pin 28 | `GPIO 16` | Input | Raspberry Pi Pin 8 (`GPIO 14`) | 3.3V Direct TTL |
| **Pi Serial TX** | Pin 27 | `GPIO 17` | Output | Raspberry Pi Pin 10 (`GPIO 15`) | 3.3V Direct TTL |
| **Common GND** | GND | `GND` | Ground | Common Power & Pi GND | Essential reference tie |
| **Left Motors PWM** | Pin 9 | `GPIO 25` | Output | TB6612FNG `PWMA` | 20 kHz LEDC PWM |
| **Left Dir AIN1** | Pin 10 | `GPIO 26` | Output | TB6612FNG `AIN1` | Digital Direction |
| **Left Dir AIN2** | Pin 11 | `GPIO 27` | Output | TB6612FNG `AIN2` | Digital Direction |
| **Right Motors PWM** | Pin 30 | `GPIO 18` | Output | TB6612FNG `PWMB` | 20 kHz LEDC PWM |
| **Right Dir BIN1** | Pin 31 | `GPIO 19` | Output | TB6612FNG `BIN1` | Digital Direction |
| **Right Dir BIN2** | Pin 33 | `GPIO 21` | Output | TB6612FNG `BIN2` | Digital Direction |
| **Motor Standby** | Pin 36 | `GPIO 22` | Output | TB6612FNG `STBY` | Active HIGH |
| **Turret Tilt Servo** | Pin 20 | `GPIO 13` | Output | Servo Signal Pin | 50 Hz PWM ($30^\circ - 150^\circ$) |
| **Solenoid Valve** | Pin 37 | `GPIO 23` | Output | MOSFET IRLZ44N Gate | HIGH = Valve Open (Extinguish) |
| **Front TRIG** | Pin 26 | `GPIO 4` | Output | HC-SR04 Front `TRIG` | 10 µs Trigger Pulse |
| **Front ECHO** | Pin 3 | `GPIO 36` | Input | HC-SR04 Front `ECHO` | **1kΩ / 2kΩ Voltage Divider to 3.3V** |
| **Left TRIG** | Pin 12 | `GPIO 32` | Output | HC-SR04 Left `TRIG` | 10 µs Trigger Pulse |
| **Left ECHO** | Pin 4 | `GPIO 39` | Input | HC-SR04 Left `ECHO` | **1kΩ / 2kΩ Voltage Divider to 3.3V** |
| **Right TRIG** | Pin 13 | `GPIO 33` | Output | HC-SR04 Right `TRIG` | 10 µs Trigger Pulse |
| **Right ECHO** | Pin 7 | `GPIO 35` | Input | HC-SR04 Right `ECHO` | **1kΩ / 2kΩ Voltage Divider to 3.3V** |
| **Flame Sensor** | Pin 6 | `GPIO 34` | Input | LM393 IR Flame `D0` | Active LOW on fire presence |

> [!CAUTION]
> **5V ECHO PROTECTION**: Standard HC-SR04 ultrasonic sensors output 5V logic pulses on the `ECHO` line. Connecting 5V directly to ESP32 inputs (`GPIO 34–39`) will damage the pins. Use a $1\text{k}\Omega / 2\text{k}\Omega$ resistor divider on each ECHO line or use 3.3V-compatible HC-SR04P sensors.

---

## 4. Repository Directory Structure

```text
SAGIP-X-BANTAY-SUNOG-/
├── bantay_sunog/                 # Raspberry Pi 3B Python vision & telemetry subsystem
│   ├── app.py                    # Main vision pipeline (camera capture, inference, UART)
│   ├── detector.py               # FireDetector engine (TFLite INT8 / HSV fallback)
│   ├── telemetry.py              # UART telemetry parser & transmitter
│   ├── config.py                 # Vision & serial configuration constants
│   └── models/                   # TFLite model weights & labelmaps
├── bantay_sunog_esp32/           # Arduino IDE ESP32 firmware sketch
│   ├── bantay_sunog_esp32.ino    # Arduino sketch entrypoint
│   ├── config.h                  # Hardware pinouts & control constants
│   └── navigation_model.h        # Exported TinyML inference header for Arduino
├── esp32_firmware/               # PlatformIO ESP32 firmware project
│   ├── platformio.ini            # PlatformIO configuration (ESP32 Dev Module)
│   ├── src/main.cpp              # Dual-core FreeRTOS firmware implementation
│   └── include/navigation_model.h# Exported TinyML inference header for PlatformIO
├── simulation/                   # 3D Digital Twin simulation platform
│   ├── ai_nav/                   # TinyML training, benchmark & dataset generator
│   │   ├── dataset_generator.py  # 87k-sample synthetic training dataset generator
│   │   ├── train.py              # PyTorch training & multi-target model exporter
│   │   ├── model.py              # TinyMLNavPolicy PyTorch architecture definition
│   │   └── nav_policy.py         # NumPy reference runner for verification
│   ├── bridge/                   # WebSocket sync bridge for Software-in-the-Loop (SIL)
│   │   └── sync_server.py        # Python sync bridge linking browser twin to AI vision
│   ├── frontend/                 # Three.js 3D simulator web application
│   │   ├── index.html            # Main simulator HUD dashboard
│   │   ├── js/sim_world.js       # Three.js scene, physics engine & capsule collisions
│   │   ├── js/ui_controller.js   # Autonomous navigation FSM & teleoperation
│   │   ├── js/ultrasonic_mapper.js # 2D SLAM grid, Bresenham raycaster & frontier routing
│   │   ├── js/nav_model_runner.js  # Client-side JavaScript TinyML inference runner
│   │   └── js/obstacle_manager.js# Arena, multi-room, maze & editable sandbox
│   ├── isaac_sim/                # NVIDIA Omniverse Isaac Sim integration
│   │   └── bantay_sunog.urdf     # Full URDF kinematic robot definition
│   ├── models/                   # Trained model artifacts (.pt, .json)
│   └── tests/                    # Automated regression & unit test suite
│       └── test_navigation_model.py # 27 unit tests (numerical, SLAM, behavioral)
├── deploy_to_pi.py               # Automated SSH deployment script for Raspberry Pi
├── enable_uart_pi.py             # Script to configure PL011 hardware UART on Pi
├── run_simulation.py             # Single-command launcher for 3D simulator & sync bridge
├── setup_pi.py                   # Automated Raspberry Pi dependency installer
└── README.md                     # Repository documentation
```

---

## 5. Getting Started

### A. 3D Digital Twin Simulation (Browser + SIL)
Run the full 3D simulation platform and sync server locally:
```powershell
python run_simulation.py
```
This automatically starts:
- The local HTTP server at `http://localhost:8000`
- The WebSocket Software-in-the-Loop (SIL) bridge at `ws://localhost:8765`
- Opens your default web browser to the interactive 3D digital twin dashboard.

**Simulation Keyboard Teleop Controls:**
- `W` / `S`: Drive Forward / Reverse
- `A` / `D`: Differential Pivot Left / Right
- `Space`: Emergency Stop Brake
- `U` / `J`: Turret Pitch Up (+10°) / Down (-10°)
- `H`: Turret Return to Level ($90^\circ$)
- `F`: Fire Extinguisher Spray Pulse

---

### B. Running Automated Unit Tests
Verify model architecture, math equivalence across PyTorch, NumPy, and C++, SLAM line-of-sight raycasting, and obstacle evasion:
```powershell
python -m unittest discover -s simulation/tests -p "test_*.py"
```
**Expected Output:**
```text
Ran 27 tests in 2.315s

OK
```

---

### C. Retraining the TinyML Navigation Model
To regenerate the 87,136-sample dataset and train the neural policy:
```powershell
# 1. Generate synthetic transitions
python -m simulation.ai_nav.dataset_generator

# 2. Train model and automatically export to C++, PlatformIO, PyTorch, JSON, and JS
python -m simulation.ai_nav.train
```

---

### D. Flashing the ESP32 Firmware

#### Option 1: Using PlatformIO (Recommended)
```bash
cd esp32_firmware
pio run --target upload
pio device monitor
```

#### Option 2: Using Arduino IDE
1. Open [`bantay_sunog_esp32/bantay_sunog_esp32.ino`](bantay_sunog_esp32/bantay_sunog_esp32.ino).
2. Install `ArduinoJson` (v6 or v7) and `ESP32Servo` via the Library Manager.
3. Select Board: **ESP32 Dev Module**.
4. Click **Upload**.

---

### E. Deploying to Raspberry Pi 3B
Ensure the Raspberry Pi is on the same local network or connected via Ethernet:
```powershell
# 1. Run environment diagnostic
python check_env.py

# 2. Configure PL011 UART on Pi
python enable_uart_pi.py --host <PI_IP> --user pi

# 3. Deploy vision code and dependencies
python deploy_to_pi.py --host <PI_IP> --user pi

# 4. Launch live vision detection on Pi
python run_live_pi.py --host <PI_IP> --user pi
```

---

## 6. Testing & Validation Status

| Test Suite / Component | Target | Status | Verification Criteria |
|---|---|:---:|---|
| **Parameter Count & Size** | TinyML Model | **PASSED** | Exactly 818 parameters, $< 4\text{ KB}$ SRAM footprint |
| **Numerical Equivalence** | PyTorch vs NumPy vs C++ | **PASSED** | Maximum absolute error $< 10^{-4}$ across all 100 random vectors |
| **Obstacle Avoidance** | Simulation & Firmware | **PASSED** | Proactive deceleration & turning at $\ge 0.28\text{m}$ standoff |
| **Corner Safety Corridor** | 2D SLAM Raycaster | **PASSED** | Expanded $0.26\text{m}$ corridor detects vertices, prevents wheel clipping |
| **Doorway Line-of-Sight** | Bresenham Grid Raycaster | **PASSED** | Intervening wall cells detected; occluded rooms penalized ($-900$) |
| **180° Backtracking** | Topological Room Nav | **PASSED** | Room completion & dead-end detection triggers decisive U-turn |
| **Fire Servoing & Suppression**| Camera + IR Sensor | **PASSED** | Decelerates to zero at $0.45\text{m} - 0.48\text{m}$ and activates solenoid |

---

## 7. Authors & License

- **Project Lead & Developer**: JENREE DANDAN ([@REEJN](https://github.com/REEJN))
- **Institution**: Polytechnic University of the Philippines / PNM
- **Repository**: [https://github.com/REEJN/SAGIP-X-BANTAY-SUNOG-](https://github.com/REEJN/SAGIP-X-BANTAY-SUNOG-)
- **License**: MIT License — see [LICENSE](LICENSE) for details.
