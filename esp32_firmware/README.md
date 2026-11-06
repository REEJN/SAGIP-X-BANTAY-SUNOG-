# BANTAY-SUNOG: ESP32-WROOM Actuation & Navigation Firmware

**Autonomous Firefighting Robot — Embedded Actuation Controller**

This directory contains the real-time C++ firmware running on the **ESP32-WROOM-32** controller. It handles chassis motion control, visual servoing, 3x ultrasonic obstacle avoidance, secondary optical flame verification, fire extinguisher solenoid suppression, and bidirectional UART telemetry communication with the Raspberry Pi 3B.

---

## 1. System Pinout & Wiring Table

| Peripheral | ESP32-WROOM Pin | GPIO | Direction | Connected To | Logic Level / Notes |
|---|---|---|---|---|---|
| **Pi Serial RX** | Pin 28 | `GPIO 16` (RX2) | Input | Raspberry Pi Pin 8 (`GPIO 14 TXD`) | 3.3V Direct |
| **Pi Serial TX** | Pin 27 | `GPIO 17` (TX2) | Output | Raspberry Pi Pin 10 (`GPIO 15 RXD`) | 3.3V Direct |
| **Common GND** | GND | `GND` | Ground | Raspberry Pi Pin 6 / Battery GND | Common Reference |
| **Motor PWMA** | Pin 9 | `GPIO 25` | Output | TB6612FNG `PWMA` | 20 kHz LEDC PWM (Left Motors) |
| **Motor AIN1** | Pin 10 | `GPIO 26` | Output | TB6612FNG `AIN1` | Digital Direction Left |
| **Motor AIN2** | Pin 11 | `GPIO 27` | Output | TB6612FNG `AIN2` | Digital Direction Left |
| **Motor PWMB** | Pin 30 | `GPIO 18` | Output | TB6612FNG `PWMB` | 20 kHz LEDC PWM (Right Motors) |
| **Motor BIN1** | Pin 31 | `GPIO 19` | Output | TB6612FNG `BIN1` | Digital Direction Right |
| **Motor BIN2** | Pin 33 | `GPIO 21` | Output | TB6612FNG `BIN2` | Digital Direction Right |
| **Motor STBY** | Pin 36 | `GPIO 22` | Output | TB6612FNG `STBY` | HIGH = Driver Active |
| **Tilt Servo** | Pin 20 | `GPIO 13` | Output | Micro Servo Signal Pin | 50 Hz PWM (Webcam/Nozzle/Sensor Pitch) |
| **Solenoid Valve** | Pin 37 | `GPIO 23` | Output | Gate of IRLZ44N MOSFET / Relay IN | HIGH = Spray Extinguisher |
| **Front TRIG** | Pin 26 | `GPIO 4` | Output | HC-SR04 Front `TRIG` | 10 µs Pulse |
| **Front ECHO** | Pin 3 | `GPIO 36` (VP) | Input | HC-SR04 Front `ECHO` | **MUST BE 3.3V SAFE** (Use 1k/2k Divider) |
| **Left TRIG** | Pin 12 | `GPIO 32` | Output | HC-SR04 Left `TRIG` | 10 µs Pulse |
| **Left ECHO** | Pin 4 | `GPIO 39` (VN) | Input | HC-SR04 Left `ECHO` | **MUST BE 3.3V SAFE** (Use 1k/2k Divider) |
| **Right TRIG** | Pin 13 | `GPIO 33` | Output | HC-SR04 Right `TRIG` | 10 µs Pulse |
| **Right ECHO** | Pin 7 | `GPIO 35` | Input | HC-SR04 Right `ECHO` | **MUST BE 3.3V SAFE** (Use 1k/2k Divider) |
| **Flame Sensor** | Pin 6 | `GPIO 34` | Input | LM393 Optical Flame Module `D0` | Active LOW (Flame present) |

> [!CAUTION]
> **HC-SR04 5V ECHO HAZARD**:
> Standard HC-SR04 ultrasonic sensors output 5V logic on their `ECHO` pins. The ESP32 input pins (`GPIO 34, 35, 36, 39`) are strictly **3.3V tolerant**.
> Always insert a voltage divider between the HC-SR04 `ECHO` pin and the ESP32 pin:
> ```
> HC-SR04 ECHO (5V) ────[ 1kΩ ]────┬────► ESP32 ECHO PIN (3.3V)
>                                  │
>                                [ 2kΩ ]
>                                  │
>                                 GND
> ```
> Alternatively, use native 3.3V sensors such as **HC-SR04P** or **RCWL-1601**.

---

## 2. Power Architecture & Isolation

- **Motors & Solenoid Power**: Powered by external battery pack (e.g. 2S 7.4V or 3S 11.1V LiPo / 12V Pack) connected to `VM` on the TB6612FNG and the Solenoid Valve power rail.
- **Logic Power (ESP32)**: Powered via 5V buck converter into ESP32 `VIN` pin, or via micro-USB.
- **Common Ground**: All GND pins (Battery negative, ESP32 GND, Raspberry Pi Pin 6 GND, TB6612FNG GND) **MUST** be tied together.

---

## 3. How to Build & Flash

### Option A: Using PlatformIO (Recommended)
1. Install [PlatformIO Core](https://platformio.org/) or the PlatformIO IDE extension in VS Code.
2. Connect your ESP32 to your PC via USB cable.
3. Open terminal in `esp32_firmware/` and build:
   ```bash
   pio run
   ```
4. Flash firmware:
   ```bash
   pio run --target upload
   ```
5. View serial monitor:
   ```bash
   pio device monitor
   ```

### Option B: Using Arduino IDE
1. In Arduino IDE **Boards Manager**, install `esp32` by Espressif Systems.
2. In **Library Manager**, install:
   - `ArduinoJson` (v7.x or v6.x)
   - `ESP32Servo` by Kevin Harrington
3. Select Board: **ESP32 Dev Module**.
4. Set Upload Speed: `921600`, Flash Frequency: `80MHz`.
5. Open `src/main.cpp` (or combine/symlink into an `.ino` sketch) and click **Upload**.

---

## 4. Finite State Machine Summary

```
[INIT] ────────► [SEARCHING] ◄─────────────────────────────────────────────┐
                     │                                                     │
                     │ (FIRE_CONFIRMED)                                    │
                     ▼                                                     │
               [ALIGNING] ◄────────┐ (Drift off-center)                    │
                     │             │                                       │
                     │ (Aligned)   │                                       │
                     ▼             │                                       │
              [APPROACHING] ───────┘                                       │
                     │                                                     │
                     │ (Front Dist <= 30cm or Bounding Box Area)           │
                     ▼                                                     │
             [EXTINGUISHING] (Physical Flame Verification + Solenoid Valve)│
                     │                                                     │
                     │ (Spray Duration Done: 3.0s)                         │
                     ▼                                                     │
                [COOLDOWN] ────────────────────────────────────────────────┘
                               (Vapor cleared & Fire Extinguished)
```

---

## 5. Telemetry Protocol Specifications

### Incoming from Raspberry Pi (115200 baud, UART2 GPIO 16 RX):
```json
{"event":"FIRE_CONFIRMED","confidence":0.91,"x":0.67,"y":0.42,"bbox":[180,90,120,160],"frame":[640,480],"timestamp":1730000000}
```

### Outgoing to Raspberry Pi (115200 baud, UART2 GPIO 17 TX):
```json
{"robot_state":"APPROACHING","dist_front":32.5,"dist_left":84.2,"dist_right":71.0,"flame_detected":true,"tilt_angle":88,"solenoid_active":false,"left_pwm":160,"right_pwm":160,"uptime":14250}
```
