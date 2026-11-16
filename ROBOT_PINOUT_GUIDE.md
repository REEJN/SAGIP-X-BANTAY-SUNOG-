# 🤖 BANTAY-SUNOG (SAGIP-X): Master Hardware Pinout & Wiring Guide

**Project**: SAGIP-X Autonomous Firefighting Mobile Robot  
**Microcontroller 1**: ESP32-WROOM-32 (38-Pin / 30-Pin DevKit) — Real-Time Low-Level Actuation & Sensor Hub  
**Microcontroller 2**: Raspberry Pi 3 Model B — High-Level AI Vision, TinyML Inference & Web Sync Server  
**Revision**: 2.2 (Updated with Dual Servo Actuation & MPU-6050 IMU)

---

## 📌 1. Master ESP32 Pinout Quick-Reference Table

| Peripheral Subsystem | ESP32 GPIO | ESP32 Pin Name | Pin Type | Direction | Target Connected Pin / Device | Logic Level & Voltage | Key Notes & Parameters |
|---|---|---|---|---|---|---|---|
| **UART Communication** | `GPIO 16` | RX2 | Digital | **Input** | Raspberry Pi Pin 8 (`GPIO 14 TXD`) | 3.3V TTL | Inter-board telemetry receiver (115200 baud) |
| **UART Communication** | `GPIO 17` | TX2 | Digital | **Output** | Raspberry Pi Pin 10 (`GPIO 15 RXD`) | 3.3V TTL | Inter-board telemetry transmitter (115200 baud) |
| **Left Motors PWM** | `GPIO 25` | D25 | PWM | **Output** | TB6612FNG `PWMA` | 3.3V (20 kHz PWM) | LEDC Ch 0, 8-bit resolution (0–255) |
| **Left Motors Direction 1** | `GPIO 26` | D26 | Digital | **Output** | TB6612FNG `AIN1` | 3.3V Logic | HIGH = Forward, LOW = Reverse |
| **Left Motors Direction 2** | `GPIO 27` | D27 | Digital | **Output** | TB6612FNG `AIN2` | 3.3V Logic | LOW = Forward, HIGH = Reverse |
| **Right Motors PWM** | `GPIO 18` | D18 | PWM | **Output** | TB6612FNG `PWMB` | 3.3V (20 kHz PWM) | LEDC Ch 1, 8-bit resolution (0–255) |
| **Right Motors Direction 1** | `GPIO 19` | D19 | Digital | **Output** | TB6612FNG `BIN1` | 3.3V Logic | Direction logic (inverted in firmware for right-side polarity) |
| **Right Motors Direction 2** | `GPIO 21` | D21 | Digital | **Output** | TB6612FNG `BIN2` | 3.3V Logic | Direction logic |
| **Motor Driver Standby** | `GPIO 22` | D22 | Digital | **Output** | TB6612FNG `STBY` | 3.3V Logic | **HIGH** = Enable H-Bridge; **LOW** = Safe Coast/Disable |
| **Turret Tilt Servo** | `GPIO 13` | D13 | PWM | **Output** | MG90S Micro Servo Signal | 3.3V PWM (50 Hz) | 45° (Down) to 135° (Up); 90° = Level Horizon |
| **Extinguisher Trigger Servo** | `GPIO 23` | D23 | PWM | **Output** | MG996R High-Torque Servo Signal | 3.3V PWM (50 Hz) | 0° = Neutral; 90°–120° = Depress Canister Valve Lever |
| **IMU I2C Data** | `GPIO 14` | D14 | I2C | **In / Out** | MPU-6050 `SDA` | 3.3V Logic | Gyroscope angular rate & accelerometer pitch/roll |
| **IMU I2C Clock** | `GPIO 15` | D15 | I2C | **Output** | MPU-6050 `SCL` | 3.3V Logic | I2C Bus Clock (400 kHz Fast-Mode) |
| **Front Ultrasonic TRIG** | `GPIO 4` | D4 | Digital | **Output** | HC-SR04 Front `TRIG` | 3.3V TTL | 10 µs clean trigger pulse |
| **Front Ultrasonic ECHO** | `GPIO 36` | SENSOR_VP | Digital | **Input Only** | HC-SR04 Front `ECHO` | **3.3V via 1k/2k Divider** | Input-Only GPI pin. **DO NOT APPLY DIRECT 5V!** |
| **Left Ultrasonic TRIG** | `GPIO 32` | D32 | Digital | **Output** | HC-SR04 Left `TRIG` | 3.3V TTL | 10 µs clean trigger pulse |
| **Left Ultrasonic ECHO** | `GPIO 39` | SENSOR_VN | Digital | **Input Only** | HC-SR04 Left `ECHO` | **3.3V via 1k/2k Divider** | Input-Only GPI pin. **DO NOT APPLY DIRECT 5V!** |
| **Right Ultrasonic TRIG** | `GPIO 33` | D33 | Digital | **Output** | HC-SR04 Right `TRIG` | 3.3V TTL | 10 µs clean trigger pulse |
| **Right Ultrasonic ECHO** | `GPIO 35` | D35 | Digital | **Input Only** | HC-SR04 Right `ECHO` | **3.3V via 1k/2k Divider** | Input-Only GPI pin. **DO NOT APPLY DIRECT 5V!** |
| **Secondary Flame Sensor** | `GPIO 34` | D34 | Digital | **Input Only** | LM393 Optical Flame Module `D0` | 3.3V Logic | Active **LOW** on 760nm–1100nm infrared detection |
| **System Ground** | `GND` | GND | Power | **Ground** | Star Ground Reference | 0V | Unified ground tied to Battery, Pi, and Buck converters |
| **Logic Power Input** | `VIN` | 5V / VIN | Power | **Power In** | Buck Converter 2 (`+5V_AUX`) | 5.0V – 6.0V DC | Feeds onboard 3.3V LDO regulator on ESP32 DevKit |

---

## 🔌 2. Subsystem Wiring Breakdown

### 2.1 Inter-Board UART Bus (ESP32 ⇄ Raspberry Pi 3B)
Establishes the bidirectional telemetry link between the high-level Raspberry Pi 3B navigation/vision process and the low-level ESP32 actuator controller at **115,200 baud**.

```
    Raspberry Pi 3B (40-Pin Header)               ESP32-WROOM-32
  ┌─────────────────────────────────┐           ┌─────────────────┐
  │ Pin 8  (GPIO 14 / UART0 TXD)    ├──────────►│ GPIO 16 (RX2)   │
  │ Pin 10 (GPIO 15 / UART0 RXD)    │◄──────────┤ GPIO 17 (TX2)   │
  │ Pin 6  (GND / Ground)           ├───────────┤ GND             │
  └─────────────────────────────────┘           └─────────────────┘
```
> [!NOTE]
> No level shifter is needed between Raspberry Pi and ESP32 UART lines because both boards operate at **3.3V TTL logic levels**.

---

### 2.2 TB6612FNG Dual H-Bridge Motor Driver (4 DC Gearmotors)
The TB6612FNG controls 4 DC motors configured in parallel pairs (Left Front + Left Rear on Channel A; Right Front + Right Rear on Channel B).

```
          ESP32                         TB6612FNG Driver Board
  ┌───────────────────┐               ┌───────────────────────────┐
  │ GPIO 25           ├──────────────►│ PWMA (Left PWM Speed)     │
  │ GPIO 26           ├──────────────►│ AIN1 (Left Direction 1)   │
  │ GPIO 27           ├──────────────►│ AIN2 (Left Direction 2)   │
  │ GPIO 18           ├──────────────►│ PWMB (Right PWM Speed)    │
  │ GPIO 19           ├──────────────►│ BIN1 (Right Direction 1)  │
  │ GPIO 21           ├──────────────►│ BIN2 (Right Direction 2)  │
  │ GPIO 22           ├──────────────►│ STBY (Standby, HIGH=Run)  │
  │ 3.3V (or 5V Aux)  ├──────────────►│ VCC (Logic Power)         │
  │ GND               ├──────────────►│ GND (Logic Ground)        │
  └───────────────────┘               │                           │
                                      │ VM (Motor Power) ◄────────┼── +11.1V LiPo High-Current Bus
                                      │ GND (Power Ground) ◄──────┼── Battery GND Bus
                                      │                           │
                                      │ AO1 / AO2 ───────────────►│ Left Motors (M1 & M2 parallel)
                                      │ BO1 / BO2 ───────────────►│ Right Motors (M3 & M4 parallel)
                                      └───────────────────────────┘
```

#### Motor Direction Truth Table
| STBY | IN1 | IN2 | PWM | Motor State |
|:---:|:---:|:---:|:---:|:---|
| **HIGH** | HIGH | LOW | 1–255 | **Forward Rotation** |
| **HIGH** | LOW | HIGH | 1–255 | **Reverse Rotation** |
| **HIGH** | HIGH | HIGH | Any | **Brake / Active Halt** |
| **HIGH** | LOW | LOW | Any | **Coasting / Idle Stop** |
| **LOW** | X | X | X | **Disabled / Standby** |

> [!IMPORTANT]
> **Right-Side Motor Polarity**: Because physical motors face opposite directions across the chassis, the firmware automatically sets `INVERT_RIGHT_MOTOR = true` in [`config.h`](file:///C:/Users/danda/.gemini/antigravity/worktrees/proud-babbage/v2_navigation_model/bantay_sunog_esp32/config.h) so that a forward drive command moves all 4 wheels forward in sync.

---

### 2.3 Aiming Turret & Extinguisher Trigger Actuators (Dual Servos)
The robot utilizes two servo motors:
1. **Turret Pitch Tilt Servo (`MG90S` Micro Metal Gear on GPIO 13)**: Tilts the webcam, optical flame sensor, and suppression aim up/down between 45° and 135° (90° = level horizon).
2. **Extinguisher Mechanical Trigger Servo (`MG996R` High-Torque on GPIO 23)**: Mechanically compresses the commercial fire extinguisher canister handle to spray extinguishing agent.

```
       ESP32                                     Servos
  ┌──────────────┐                       ┌───────────────────────────────┐
  │ GPIO 13      ├──────────────────────►│ MG90S Tilt Servo (Signal)     │
  │ GPIO 23      ├──────────────────────►│ MG996R Trigger Servo (Signal) │
  └──────────────┘                       │                               │
  +5V_AUX (LM2596 Buck OUT, 5.0–6.0V) ──►│ Red Wires (VCC both servos)   │
  Common Ground Bus ────────────────────►│ Brown/Black (GND both servos) │
                                         └───────────────────────────────┘
```

> [!CAUTION]
> **DO NOT POWER SERVOS FROM ESP32 3.3V PIN!**
> The MG996R high-torque servo draws stall currents exceeding **1.5A** when compressing the canister lever. Servos **must** be connected directly to the `+5V_AUX` rail from the LM2596 buck converter with an external buffer capacitor (470 µF – 1000 µF) to prevent brownouts.

---

### 2.4 Ultrasonic Obstacle Avoidance Array (3x HC-SR04)
Three HC-SR04 ultrasonic rangefinders are mounted in a tri-directional array:
- **Front Center**: Detects head-on collisions and maintains the 30 cm fire standoff distance.
- **Left (45° angle)**: Monitors side clearance for corridor following and wall avoidance.
- **Right (45° angle)**: Monitors right-side clearance.

```
          ESP32                                HC-SR04 Ultrasonic Sensors
  ┌───────────────────┐                      ┌────────────────────────────┐
  │ GPIO 4 (Output)   ├─────────────────────►│ Front HC-SR04 TRIG        │
  │ GPIO 32 (Output)  ├─────────────────────►│ Left HC-SR04 TRIG         │
  │ GPIO 33 (Output)  ├─────────────────────►│ Right HC-SR04 TRIG        │
  │                   │                      │                            │
  │ GPIO 36 (Input)   │◄── 3.3V Divider ─────┤ Front HC-SR04 ECHO (5V)    │
  │ GPIO 39 (Input)   │◄── 3.3V Divider ─────┤ Left HC-SR04 ECHO (5V)     │
  │ GPIO 35 (Input)   │◄── 3.3V Divider ─────┤ Right HC-SR04 ECHO (5V)    │
  └───────────────────┘                      └────────────────────────────┘
```

#### ⚠️ Mandatory 5V → 3.3V Resistor Voltage Divider Circuit
Standard HC-SR04 sensors output a **5V logic pulse** on their `ECHO` pin. ESP32 GPIOs 34–39 are strictly rated for **3.3V maximum**! You must wire a resistor divider for each ECHO pin:

```
  HC-SR04 ECHO (5V) ────[ 1.0 kΩ ]────┬────► ESP32 ECHO PIN (GPIO 36, 39, or 35)
                                      │
                                  [ 2.0 kΩ ]
                                      │
                                     GND
```
$$\text{Calculated Output Voltage: } V_{out} = 5.0\text{ V} \times \frac{2\text{ k}\Omega}{1\text{ k}\Omega + 2\text{ k}\Omega} = 3.33\text{ V}$$

---

### 2.5 Secondary Optical Flame Sensor (LM393 Infrared Module)
Mounted co-axially on the MG90S tilt turret alongside the webcam lens.

```
       ESP32                               LM393 IR Flame Sensor
  ┌──────────────┐                       ┌───────────────────────────────┐
  │ GPIO 34      │◄──────────────────────┤ D0 (Digital Comparator Out)   │
  │ 3.3V (or 5V) ├──────────────────────►│ VCC                           │
  │ GND          ├──────────────────────►│ GND                           │
  └──────────────┘                       └───────────────────────────────┘
```
- **Detection Spectrum**: 760 nm to 1100 nm (infrared fire flame signature).
- **Trigger Logic**: Active **LOW** (`0` = Flame present, `1` = Ambient/No flame).
- **Potentiometer**: Tuned so the module only triggers when open fire is within **1.0 meter**.

---

### 2.6 Inertial Measurement Unit (GY-521 / MPU-6050 6-Axis IMU)
Provides pitch/roll tilt verification and yaw angular rate tracking for precise in-place pivot turns.

```
       ESP32                                 MPU-6050 Breakout (GY-521)
  ┌──────────────┐                       ┌───────────────────────────────┐
  │ GPIO 14      ├◄─────────────────────►│ SDA (I2C Data)                │
  │ GPIO 15      ├──────────────────────►│ SCL (I2C Clock)               │
  │ 5V (or 3.3V) ├──────────────────────►│ VCC (Power)                   │
  │ GND          ├──────────────────────►│ GND                           │
  │ GND          ├──────────────────────►│ AD0 (Address Select -> 0x68)  │
  └──────────────┘                       └───────────────────────────────┘
```
- **VCC Power Note**: GY-521 breakout modules include an onboard 3.3V LDO regulator (marked `662K`). Connect `VCC` to **5V** (or ESP32 `VIN`) for optimal regulation, or to `3.3V` if your module does not have an onboard LDO.
- **I2C Address**: The firmware auto-probes both `0x68` (AD0 tied to GND) and `0x69` (AD0 tied to 3.3V / floating). Connecting AD0 to GND is recommended.
- **INT Pin**: Leave unconnected (polled over I2C at 50 Hz).

---

## 🍓 3. Raspberry Pi 3 Model B Pinout Map

| Pi Physical Pin | Pi BCM Name | Direction | Connected To | Description |
|:---:|:---:|:---:|---|---|
| **Pin 2** | `5V` | **Power In** | XL4015 Buck Converter OUT+ (`+5V_PI`) | Clean 5.15V high-amperage input rail |
| **Pin 4** | `5V` | Power | *Unused / Test point* | Tied to Pin 2 rail |
| **Pin 6** | `GND` | **Ground** | Common Star Ground | Common ground reference with ESP32 & battery |
| **Pin 8** | `GPIO 14 (TXD)` | **Output** | ESP32 `GPIO 16 (RX2)` | Transmits target coordinates & nav commands |
| **Pin 10** | `GPIO 15 (RXD)` | **Input** | ESP32 `GPIO 17 (TX2)` | Receives live sensor telemetry & motor states |
| **Pin 9 / 14 / 20 / 25** | `GND` | Ground | Ground Bus | Secondary ground tie points |
| **USB 2.0 Port 1** | USB Bus | In / Out | USB HD Webcam | Captures 640×480 @ 30 FPS video feed for HSV/TinyML detector |

---

## ⚡ 4. Power Distribution & Regulation Architecture

The robot employs an isolated **Dual-Buck Power Architecture** driven by a single high-discharge **3S LiPo battery** (11.1V nominal, 12.6V fully charged):

```
                       ┌──────────────────────────┐
                       │   3S LiPo Battery Pack   │
                       │   (11.1V - 12.6V, 25C)   │
                       └────────────┬─────────────┘
                                    │
                                [SPST Switch]
                                    │
                              [5A Inline Fuse]
                                    │
            ┌───────────────────────┴────────────────────────┐
            │ +11.1V High-Current Unregulated Power Bus     │
            └───────┬───────────────────┬────────────────┬───┘
                    │                   │                │
                    ▼                   ▼                ▼
          ┌──────────────────┐ ┌──────────────────┐ ┌───────────────────────────┐
          │ XL4015 Step-Down │ │ LM2596 Step-Down │ │ TB6612FNG VM Pin          │
          │ Buck Converter 1 │ │ Buck Converter 2 │ │ (Direct 11.1V Motor Power)│
          └─────────┬────────┘ └────────┬─────────┘ └─────────────┬─────────────┘
                    │                   │                         │
                    ▼                   ▼                         ▼
             +5.15V DC Rail       +5.5V DC Rail            4x DC Drive Motors
             (Dedicated Pi)      (ESP32 & Servos)           (High-Current)
                    │                   │
         ┌──────────┴──────┐ ┌──────────┴───────────────┐
         │ Raspberry Pi 3B │ │ • ESP32 DevKit VIN       │
         │ (Pin 2 & Pin 6) │ │ • MG996R Trigger Servo   │
         │ USB Webcam      │ │ • MG90S Tilt Servo       │
         └─────────────────┘ │ • 3x HC-SR04 Ultrasonic  │
                             └──────────────────────────┘
```

### Safety & Isolation Rules
1. **Common Ground**: All GND pins (Battery negative terminal, Buck 1 OUT-, Buck 2 OUT-, ESP32 GND, Raspberry Pi Pin 6 GND, TB6612FNG GND) **MUST be tied together in a single star-ground configuration**.
2. **Dedicated Pi Rail**: The Raspberry Pi 3B is powered exclusively by the XL4015 buck converter tuned to **5.15V**. This prevents the Pi's under-voltage bolt warning (lightning bolt icon) during heavy CPU inference.
3. **Dedicated Servo Rail**: The high-torque MG996R servo and MG90S servo share Buck 2 with the ESP32. A **470 µF to 1000 µF electrolytic capacitor** is wired across the `+5V_AUX` and `GND` rails near the servos to absorb inductive current spikes when the canister lever is depressed.

---

## ⚠️ 5. Critical ESP32 Hardware Constraints & Warnings

> [!WARNING]
> ### 1. Input-Only Pins (GPI)
> `GPIO 34`, `GPIO 35`, `GPIO 36 (VP)`, and `GPIO 39 (VN)` on the ESP32 are **Input-Only (GPI)**:
> - They **cannot** be configured as `OUTPUT`.
> - They **do not have internal pull-up or pull-down resistors**.
> - In this robot, they are strictly assigned as inputs:
>   - `GPIO 36` = Front Echo
>   - `GPIO 39` = Left Echo
>   - `GPIO 35` = Right Echo
>   - `GPIO 34` = Flame Sensor Digital Out

> [!CAUTION]
> ### 2. 5V Ultrasonic Echo Hazard
> Connecting 5V directly to ESP32 pins will permanently damage the microcontroller GPIO inputs. Ensure all 3 HC-SR04 sensors have a 1kΩ / 2kΩ voltage divider installed on their `ECHO` lines before powering on.

> [!TIP]
> ### 3. Strapping Pin Awareness
> `GPIO 12`, `GPIO 0`, and `GPIO 2` are ESP32 strapping pins that determine boot mode (flashing vs SPI flash boot). None of these pins are used in this pinout, ensuring the ESP32 boots reliably every time without getting stuck in bootloader mode.

---

## 🛠️ 6. Quick Continuity & Bench Testing Checklist

Before placing the robot on the floor:
- [ ] **GND Continuity**: Verify $0\ \Omega$ resistance between Pi Pin 6, ESP32 GND, TB6612FNG GND, and Battery (-).
- [ ] **Buck 1 Voltage**: Measure between XL4015 OUT+ and GND with multimeter: **5.15V ± 0.05V**.
- [ ] **Buck 2 Voltage**: Measure between LM2596 OUT+ and GND with multimeter: **5.50V ± 0.10V**.
- [ ] **Motor Rotation Test**: Open Serial Monitor at 115200 baud, type `test`, and verify both left and right wheels rotate forward.
- [ ] **Extinguisher Trigger Test**: Type `trigger` in Serial Monitor to verify the MG996R servo actuates 90° for 1.0 second and returns to 0°.
- [ ] **Ultrasonic Distance Check**: Type `sensors` in Serial Monitor and wave hand in front of Front, Left, and Right sensors to confirm accurate cm readings.
