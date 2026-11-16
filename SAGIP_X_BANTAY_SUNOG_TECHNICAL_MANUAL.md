# SAGIP-X: BANTAY-SUNOG (Autonomous Indoor Firefighting Robot)
## Comprehensive Technical System Dossier & Engineering Manual

**Project Lead & Developer:** Jenree Dandan ([@REEJN](https://github.com/REEJN))  
**Institution:** Computer Studies / Colegio De Montalban  
**Repository:** [https://github.com/REEJN/SAGIP-X-BANTAY-SUNOG-](https://github.com/REEJN/SAGIP-X-BANTAY-SUNOG-)  
**License:** MIT License  
**System Classification:** Dual-Tier Edge-AI Autonomous Indoor Firefighting Robot (Skid-Steer Mobile Base, Embedded TinyML Navigation, Multi-Modal Visual & Optical Flame Confirmation, Servo-Actuated Suppression)

---

## 1. Materials Needed for This Project (Bill of Materials - BOM)

### 1.1 Compute & Processing Units
| Item | Component Name | Exact Model / Part Number | Key Specifications & Electrical Ratings | Qty | Functional Role in System |
|---|---|---|---|:---:|---|
| **C1** | High-Level Vision SBC | **Raspberry Pi 3 Model B** | Broadcom BCM2837 Quad-Core ARM Cortex-A53 @ 1.2 GHz, 1 GB LPDDR2 RAM, 4x USB 2.0, PL011 Hardware UART (`/dev/serial0`), 5V / 2.5A input | 1 | Executes OpenCV V4L2 video ingestion, TFLite INT8 / HSV fire detection pipeline, temporal sliding window state machine, and serial telemetry transmission. |
| **C2** | Low-Level Real-Time Microcontroller | **ESP32-WROOM-32** (38-pin Dev Module) | Dual-Core Tensilica Xtensa 32-bit LX6 @ 240 MHz, 520 KB SRAM, 4 MB Flash, Hardware LEDC 20 kHz PWM, Hardware UART2 (`GPIO 16/17`), 3.3V logic | 1 | Real-time motion control, 20 kHz motor PWM, ultrasonic round-robin pinging, MPU-6050 I2C sampling, turret tilt servo aiming, extinguisher trigger servo actuation, and TinyML local policy inference. |
| **C3** | Operating System Storage | **SanDisk Ultra MicroSD Card 32GB** | Class 10, UHS-I, A1 Application Performance Class, >= 98 MB/s read speed | 1 | Hosts Raspberry Pi OS Lite (64-bit / 32-bit), Python 3.9+ runtime, OpenCV, TensorFlow Lite runtime, and application logs. |

### 1.2 Perception & Sensory System
| Item | Component Name | Exact Model / Part Number | Key Specifications & Electrical Ratings | Qty | Functional Role in System |
|---|---|---|---|:---:|---|
| **S1** | Optical Vision Sensor | **USB HD UVC Webcam** (Logitech C270 / Generic 720p/1080p) | 1280x720 / 640x360 @ 30 FPS, MJPG / YUYV pixel format, UVC compliant, USB 2.0 interface, 60° - 75° Field of View (FOV) | 1 | Captures real-time forward video frames for neural object detection and visual servoing. Mounted on the articulated tilt turret. |
| **S2** | Front Ultrasonic Distance Sensor | **HC-SR04** (Front 0°) | 5V VCC, 15 mA operating current, 2 cm to 400 cm range, 15° measuring angle, 10 µs TTL trigger pulse, 5V TTL echo output | 1 | Primary forward obstacle detection, wall proximity sensing, and fire extinguishing standoff distance measurement (target: 30.0 cm). |
| **S3** | Left Ultrasonic Distance Sensor | **HC-SR04** (Left +45°) | 5V VCC, 15 mA operating current, 2 cm to 400 cm range, 15° measuring angle, 10 µs TTL trigger pulse, 5V TTL echo output | 1 | Flank obstacle detection, corridor wall following, doorway edge detection, and dead-end cul-de-sac mapping. |
| **S4** | Right Ultrasonic Distance Sensor | **HC-SR04** (Right -45°) | 5V VCC, 15 mA operating current, 2 cm to 400 cm range, 15° measuring angle, 10 µs TTL trigger pulse, 5V TTL echo output | 1 | Flank obstacle detection, corridor wall following, doorway edge detection, and dead-end cul-de-sac mapping. |
| **S5** | Optical Infrared Flame Sensor | **LM393 IR Flame Sensor Module** | Detection spectrum 760 nm – 1100 nm, 60° detection angle, 3.3V–5V VCC, Digital Output (D0) Active LOW on flame presence, on-board sensitivity potentiometer | 1 | Secondary physical safety interlock. Mounted co-axially with webcam on the tilt gimbal. Both camera and flame sensor must confirm fire before spraying. |
| **S6** | 6-Axis Inertial Measurement Unit (IMU) | **MPU-6050 (GY-521 Module)** | 3-axis accelerometer (+/-2g to +/-16g) + 3-axis gyroscope (+/-250 to +/-2000 deg/s), I2C interface (SDA: GPIO 14, SCL: GPIO 15), 3.3V-5V VCC | 1 | Real-time angular heading rate (yaw velocity in rad/s) and chassis pitch/roll tilt detection feeding Input #7 into the TinyML neural navigation policy. |

### 1.3 Actuation & Drive Subsystem
| Item | Component Name | Exact Model / Part Number | Key Specifications & Electrical Ratings | Qty | Functional Role in System |
|---|---|---|---|:---:|---|
| **A1** | Dual H-Bridge Motor Driver | **TB6612FNG** Dual Motor Driver Carrier | Dual MOSFET H-Bridge, VM = 4.5V - 15V, VCC = 2.7V - 5.5V, Output current 1.2A continuous (3.2A peak) per channel, up to 100 kHz PWM, Standby (STBY) pin | 1 | High-efficiency motor driver controlling left and right drive wheel motor pairs. Low internal resistance produces minimal heat and eliminates the large 2V drop of older L298N drivers. |
| **A2** | Drive Motors | **Geared DC TT Motors / 12V Metal Gearmotors** | 1:48 or 1:90 gear ratio, operating voltage 6V–12V DC, no-load speed ~150–250 RPM, stall torque ~1.5–2.5 kg·cm | 4 | Powers the 4WD skid-steer differential chassis for agile zero-radius turns and precise positioning. |
| **A3** | Drive Wheels | **High-Traction Rubber Grip Wheels** | Outer diameter 65 mm to 84 mm, width 26 mm to 30 mm, silicone/rubber tread with hex or D-shaft brass coupler | 4 | Provides strong traction across indoor tile, concrete, and wood flooring while maintaining precise odometry and zero-radius turns. |
| **A4** | Articulated Gimbal Tilt Servo | **TowerPro MG90S** (Metal Gear Micro Servo) | Operating voltage 4.8V–6.0V, torque 2.2 kg·cm @ 6.0V, operating speed 0.08 s / 60°, metal gears, 50 Hz PWM input | 1 | Directly tilts the co-axial sensor turret (webcam, nozzle, flame sensor) between 45° (floor targets) and 135° (elevated flames). |
| **A5** | Extinguisher Trigger Actuator | **TowerPro MG996R** (High-Torque Metal Gear Servo) | Operating voltage 4.8V–6.6V, stall torque 9.4–11 kg·cm @ 6.0V, metal gear train, 50 Hz PWM input on `GPIO 23` | 1 | Mechanically pulls/depresses the discharge trigger lever of the onboard fire extinguisher cylinder or pressurized spray canister. |

### 1.4 Fire Suppression Hardware
| Item | Component Name | Exact Model / Part Number | Key Specifications & Electrical Ratings | Qty | Functional Role in System |
|---|---|---|---|:---:|---|
| **F1** | Fire Extinguisher / Canister | **Portable Fire Extinguisher / Pressurized Aerosol Canister** | 500g – 1000g capacity, equipped with mechanical depressable squeeze trigger lever | 1 | Onboard fire extinguishing medium. Trigger lever is physically engaged by the MG996R high-torque servo. |
| **F2** | Mechanical Trigger Bracket | **3D-Printed / Aluminum Clamp Assembly** | Custom lever arm linkage coupling the MG996R servo horn directly to the extinguisher discharge lever | 1 | Converts rotational servo torque into linear compression force to depress the extinguisher valve. |
| **F3** | Flexible Hose & Discharge Tip | **Reinforced Discharge Tubing & Nozzle** | Directs extinguishing agent from the cylinder nozzle toward the targeted flame core | 1 | Routes the extinguishing agent forward toward the fire base. |

### 1.5 Power Supply & Voltage Regulation Subsystem
| Item | Component Name | Exact Model / Part Number | Key Specifications & Electrical Ratings | Qty | Functional Role in System |
|---|---|---|---|:---:|---|
| **P1** | Primary Energy Storage | **3S LiPo Battery Pack** (or 3x 18650 Li-ion with BMS) | 11.1V nominal (12.6V fully charged), 2200 mAh – 3300 mAh capacity, 25C continuous discharge rate, XT60 connector | 1 | Main system power source providing high-current output for drive motors, servos, and digital compute rails. |
| **P2** | Dedicated SBC Buck Converter | **XL4015 or LM2596 DC-DC Step-Down Converter** | Input 8V–36V, Output tuned to exactly 5.15V DC, 3.0A continuous (5.0A peak), > 92% efficiency | 1 | Supplies isolated, regulated 5.15V to Raspberry Pi 3B (preventing undervoltage lightning bolt icon and CPU throttling). |
| **P3** | Auxiliary Actuator Buck Converter | **LM2596 DC-DC Step-Down Converter** | Input 8V–36V, Output tuned to 5.0V–6.0V DC, 3.0A peak | 1 | Dedicated high-current power rail for both servos (MG996R Trigger Servo & MG90S Tilt Servo), MPU-6050, and 3x HC-SR04 sensors. |
| **P4** | Master Power Switch & Fuse | **SPST Heavy-Duty Rocker Switch + Inline 5A Fuse** | Switch rated 10A @ 125V AC / 12V DC, inline mini automotive fuse holder with 5A fast-blow blade fuse | 1 | Primary system emergency cut-off and overcurrent short-circuit fire prevention. |

---

## 2. Full System Architecture & Wiring Diagram

### 2.1 Complete System Circuit Schematic (Servo Mechanical Trigger + MPU-6050)

![SAGIP-X BANTAY-SUNOG System Circuit Schematic with Servo Trigger](file:///C:/Users/danda/.gemini/antigravity/brain/99337e95-8055-4d32-8156-4ebdbd5ed89c/servo_trigger_circuit_1794277167208.jpg)

### 2.2 System Interconnection Architecture
```
=======================================================================================================================
                                       SAGIP-X: BANTAY-SUNOG SYSTEM ARCHITECTURE
=======================================================================================================================

   +---------------------------------------------------------------------------------------------------------------+
   |                                          POWER DISTRIBUTION SUBSYSTEM                                         |
   |                                                                                                               |
   |   [ 3S LiPo Battery 11.1V - 12.6V ] ===(Master Switch & 5A Fuse)===> [ 12V High-Current Power Bus ]           |
   |                                                                                 |                             |
   |           +---------------------------------+-----------------------------------+                             |
   |           |                                 |                                   |                             |
   |           v                                 v                                   v                             |
   |   [ Buck Converter 1 ]             [ Buck Converter 2 ]               [ 12V Direct Motor Rail ]               |
   |   (XL4015: 12V -> 5.15V @ 3A)      (LM2596: 12V -> 5.0V/6.0V @ 3A)    - TB6612FNG VM Pin                      |
   |           |                                 |                                                                 |
   |           v                                 +-------------------------------------------------------------+   |
   |   [ Raspberry Pi 3B (5V Pin) ]              |                                                             |   |
   |                                             v                                                             v   |
   |                                    - MG996R Trigger Servo VCC                                - MG90S Tilt VCC |
   |                                    - MPU-6050 IMU VCC                                        - 3x HC-SR04 VCC |
   |                                    - ESP32 5V Vin Pin                                                         |
   +---------------------------------------------------------------------------------------------------------------+
                                                       | Common System Star Ground (GND)
                                                       v
   +---------------------------------------------------------------------------------------------------------------+
   |                                          HIGH-LEVEL VISION SBC (Raspberry Pi 3B)                              |
   |                                                                                                               |
   |   +-----------------------+                                                                                   |
   |   |   USB HD Webcam       | ===(USB 2.0 Video Stream @ 30 FPS)===> [ OpenCV V4L2 Threaded Capture ]           |
   |   +-----------------------+                                                        |                          |
   |                                                                                    v                          |
   |                                                                     [ TFLite SSD-MobileNet INT8 ]             |
   |                                                                     [  / HSV Fallback Detection ]             |
   |                                                                                    |                          |
   |                                                                                    v                          |
   |                                                                     [ 5-Frame Temporal Validator ]            |
   |                                                                     (3 of 5 frames = FIRE_CONFIRMED)          |
   |                                                                                    |                          |
   |                                                                                    v                          |
   |                                                                     [ UART Serial Telemetry Engine ]          |
   |                                                                     - Formats: {"event":"FIRE_CONFIRMED",     |
   |                                                                                 "x":0.52,"y":0.48}            |
   +------------------------------------------------------------------------------------+--------------------------+
                                                                                        |
                                                              3.3V TTL UART (115200 Baud)|
                                                              Pi TX (GPIO 14) -> ESP32 RX (GPIO 16)
                                                              Pi RX (GPIO 15) <- ESP32 TX (GPIO 17)
                                                              Common Ground
                                                                                        |
                                                                                        v
   +---------------------------------------------------------------------------------------------------------------+
   |                                    LOW-LEVEL REAL-TIME CONTROLLER (ESP32-WROOM-32)                            |
   |                                                                                                               |
   |   [ UART2 Protocol Parser ] <-----------------------------------------------------+                           |
   |               |                                                                   |                           |
   |               v                                                                   v                           |
   |   [ Sensor Manager: 60ms Round-Robin ]                            [ TinyML Local Navigation Policy ]          |
   |   - Front HC-SR04 (Trig: 4, Echo: 36 via Divider)                 - 818 Parameters, < 4 KB SRAM               |
   |   - Left  HC-SR04 (Trig: 32, Echo: 39 via Divider)                - 7 Inputs -> Dense(32) -> Dense(16) -> 2   |
   |   - Right HC-SR04 (Trig: 33, Echo: 35 via Divider)                - Input 7: Heading Rate from MPU-6050 Gyro! |
   |   - LM393 IR Flame Sensor (D0: GPIO 34)                           - Outputs continuous Left/Right Motor PWM   |
   |   - MPU-6050 6-Axis IMU (SDA: 14, SCL: 15)                                       |                           |
   |               |                                                                   |                           |
   |               +-----------------------------------+-------------------------------+                           |
   |                                                   |                                                           |
   |                                                   v                                                           |
   |                                   [ Finite State Machine (FSM) ]                                              |
   |                         (SEARCHING -> ALIGNING -> APPROACHING -> EXTINGUISHING)                               |
   |                                       |                           |                                           |
   |                   +-------------------+                           +--------------------+                      |
   |                   |                                                                    |                      |
   |                   v                                                                    v                      |
   |   [ TB6612FNG Motor Controller ]                                       [ Suppression & Gimbal Control ]       |
   |   - Left PWM (GPIO 25, LEDC Ch0 @ 20 kHz)                              - MG90S Tilt Servo (GPIO 13 @ 50 Hz)   |
   |   - Left Dir (GPIO 26 AIN1, GPIO 27 AIN2)                              - MG996R Trigger Servo (GPIO 23 @ 50Hz)|
   |   - Right PWM (GPIO 18, LEDC Ch1 @ 20 kHz)                             - 3000ms Suppression Watchdog Timer    |
   |   - Right Dir (GPIO 19 BIN1, GPIO 21 BIN2)                                                                    |
   |   - Standby (GPIO 22 STBY -> HIGH)                                                                            |
   +-------------------+--------------------------------------------------------------------+----------------------+
                       |                                                                    |
                       v                                                                    v
       +-------------------------------+                                    +-------------------------------+
       |   4WD Differential Motors     |                                    |   Extinguisher Actuators      |
       | - Left Pair (Front & Rear)    |                                    | - MG90S Tilt Aiming Turret    |
       | - Right Pair (Front & Rear)   |                                    | - MG996R Mechanical Trigger   |
       | - Skid-steer zero-radius turn |                                    | - Squeezes Fire Extinguisher  |
       | - 20 kHz silent drive         |                                    |   Handle / Lever Directly     |
       +-------------------------------+                                    +-------------------------------+
=======================================================================================================================
```

---

## 3. Hardware Configurations & Pinout Map

### 3.1 Complete ESP32 Pin Assignment
| Peripheral | ESP32 Pin | GPIO | Direction | Connected To | Notes / Circuit Details |
|---|---|---|---|---|---|
| **Pi Serial RX** | Pin 28 | `GPIO 16` | Input | Raspberry Pi Pin 8 (`GPIO 14 TXD`) | Direct 3.3V TTL connection. |
| **Pi Serial TX** | Pin 27 | `GPIO 17` | Output | Raspberry Pi Pin 10 (`GPIO 15 RXD`) | Direct 3.3V TTL connection. |
| **Common GND** | GND | `GND` | Ground | Common Ground Bus | Essential common reference tie for all components. |
| **Left Motors PWM** | Pin 9 | `GPIO 25` | Output | TB6612FNG `PWMA` | LEDC Channel 0, 20 kHz frequency, 8-bit resolution. |
| **Left Dir AIN1** | Pin 10 | `GPIO 26` | Output | TB6612FNG `AIN1` | Digital Direction Logic: HIGH/LOW = Forward, LOW/HIGH = Reverse. |
| **Left Dir AIN2** | Pin 11 | `GPIO 27` | Output | TB6612FNG `AIN2` | Digital Direction Logic. |
| **Right Motors PWM** | Pin 30 | `GPIO 18` | Output | TB6612FNG `PWMB` | LEDC Channel 1, 20 kHz frequency, 8-bit resolution. |
| **Right Dir BIN1** | Pin 31 | `GPIO 19` | Output | TB6612FNG `BIN1` | Digital Direction Logic: HIGH/LOW = Forward, LOW/HIGH = Reverse. |
| **Right Dir BIN2** | Pin 33 | `GPIO 21` | Output | TB6612FNG `BIN2` | Digital Direction Logic. |
| **Motor Standby** | Pin 36 | `GPIO 22` | Output | TB6612FNG `STBY` | Must be driven HIGH to enable motor output bridges. |
| **Turret Tilt Servo**| Pin 20 | `GPIO 13` | Output | MG90S Signal Pin | 50 Hz PWM control (0.5 ms to 2.5 ms, 45° to 135° pitch). |
| **Extinguisher Trigger Servo**| Pin 37| `GPIO 23` | Output | MG996R Signal Pin | 50 Hz PWM (0° = Neutral / Lever released; 90° = Squeeze lever). |
| **MPU-6050 SDA** | Pin 14 | `GPIO 14` | In/Out | MPU-6050 `SDA` | I2C Data line for gyro angular rate & accelerometer tilt. |
| **MPU-6050 SCL** | Pin 15 | `GPIO 15` | Output | MPU-6050 `SCL` | I2C Clock line for gyro angular rate & accelerometer tilt. |
| **Front TRIG** | Pin 26 | `GPIO 4` | Output | HC-SR04 Front `TRIG` | 10 µs clean TTL trigger pulse. |
| **Front ECHO** | Pin 3 | `GPIO 36` | Input | HC-SR04 Front `ECHO` | **Input Only pin (SENSOR_VP). 1k/2k resistor divider to 3.3V!** |
| **Left TRIG** | Pin 12 | `GPIO 32` | Output | HC-SR04 Left `TRIG` | 10 µs clean TTL trigger pulse. |
| **Left ECHO** | Pin 4 | `GPIO 39` | Input | HC-SR04 Left `ECHO` | **Input Only pin (SENSOR_VN). 1k/2k resistor divider to 3.3V!** |
| **Right TRIG** | Pin 13 | `GPIO 33` | Output | HC-SR04 Right `TRIG` | 10 µs clean TTL trigger pulse. |
| **Right ECHO** | Pin 7 | `GPIO 35` | Input | HC-SR04 Right `ECHO` | **Input Only pin. 1k/2k resistor divider to 3.3V!** |
| **Flame Sensor** | Pin 6 | `GPIO 34` | Input | LM393 Module `D0` | **Input Only pin.** Active LOW on optical flame detection. |

### 3.2 Extinguisher Servo Mechanical Trigger Operation
Instead of an electrical solenoid, the fire extinguisher is mechanically triggered:
- **Idle State (`0^\circ`):** The MG996R servo horn rests in a neutral position with zero tension on the extinguisher handle.
- **Suppression State (`90^\circ` - `120^\circ`):** Upon multi-modal fire confirmation at $\le 30\text{ cm}$, the ESP32 drives `GPIO 23` to rotate the MG996R servo horn. The attached mechanical cam/linkage physically depresses the extinguisher valve handle for 3.0 seconds.
- **Release & Reset (`0^\circ`):** After 3.0 seconds, the servo returns to $0^\circ$, releasing the spring-loaded extinguisher handle and halting discharge.

---

## 4. Cirkit AI Prompt (with Servo Trigger & MPU-6050)

Copy and paste this prompt directly into [Cirkit Designer AI](https://www.cirkitdesigner.com/):

```text
Create a complete, modular, and professional circuit schematic and wiring diagram for an autonomous mobile robot named "SAGIP-X: BANTAY-SUNOG" with a dual-tier compute architecture, dual buck power distribution, multi-sensor array, MPU-6050 IMU, motor driver, and dual servo actuators (tilt aiming and extinguisher mechanical trigger).

### 1. POWER DISTRIBUTION & REGULATION
- Power Source: 3S LiPo Battery (11.1V nominal, 12.6V max) connected to an SPST toggle switch and a 5A fast-blow inline fuse leading to a +12V High-Current Bus.
- Buck Converter 1 (XL4015 DC-DC Step-Down):
  * IN+ connects to +12V Bus, IN- connects to Common GND.
  * OUT+ tuned to 5.15V DC (labeled +5V_PI) dedicated to Raspberry Pi 3B Pin 2 (5V).
  * OUT- connects to Common GND.
- Buck Converter 2 (LM2596 DC-DC Step-Down):
  * IN+ connects to +12V Bus, IN- connects to Common GND.
  * OUT+ tuned to 5.0V-6.0V DC (labeled +5V_AUX) with a 470uF electrolytic capacitor to GND.
  * Supplies: ESP32 Vin pin, 3x HC-SR04 VCC pins, MPU-6050 VCC pin, MG90S Tilt Servo VCC, and MG996R Trigger Servo VCC.
  * OUT- connects to Common GND.
- Common Ground: Establish a single unified Star Ground net (GND) across all subsystems.

### 2. COMPUTE BOARDS & INTER-BOARD UART BUS
- Microcontroller 1: Raspberry Pi 3 Model B
  * Pin 2 (5V) -> +5V_PI
  * Pin 6 (GND) -> Common GND
  * Pin 8 (GPIO 14 / TXD) -> ESP32 GPIO 16 (UART2 RX)
  * Pin 10 (GPIO 15 / RXD) <- ESP32 GPIO 17 (UART2 TX)
  * USB Port connects to USB HD Webcam.
- Microcontroller 2: ESP32-WROOM-32 (38-pin Dev Module)
  * Vin pin -> +5V_AUX
  * GND pin -> Common GND
  * GPIO 16 -> Pi Pin 8 (TXD)
  * GPIO 17 -> Pi Pin 10 (RXD)

### 3. INERTIAL MEASUREMENT UNIT (MPU-6050 IMU)
- Module: MPU-6050 6-Axis Accelerometer & Gyroscope (GY-521 Breakout)
  * VCC -> +5V_AUX (or ESP32 3.3V pin)
  * GND -> Common GND
  * SDA (I2C Data) -> ESP32 GPIO 14
  * SCL (I2C Clock) -> ESP32 GPIO 15
  * AD0 -> Common GND (Sets I2C address to 0x68)

### 4. ULTRASONIC SENSOR ARRAY & 5V-TO-3.3V VOLTAGE DIVIDERS
Connect three HC-SR04 ultrasonic modules. All sensor VCC pins connect to +5V_AUX, all GND pins connect to Common GND.
Add a two-resistor voltage divider (1kΩ high-side, 2kΩ low-side to GND) to each ECHO pin:
- Front HC-SR04 (0°):
  * TRIG pin -> ESP32 GPIO 4
  * ECHO pin -> 1.0kΩ resistor -> ESP32 GPIO 36 (SENSOR_VP)
  * 2.0kΩ resistor from ESP32 GPIO 36 to GND.
- Left HC-SR04 (+45°):
  * TRIG pin -> ESP32 GPIO 32
  * ECHO pin -> 1.0kΩ resistor -> ESP32 GPIO 39 (SENSOR_VN)
  * 2.0kΩ resistor from ESP32 GPIO 39 to GND.
- Right HC-SR04 (-45°):
  * TRIG pin -> ESP32 GPIO 33
  * ECHO pin -> 1.0kΩ resistor -> ESP32 GPIO 35
  * 2.0kΩ resistor from ESP32 GPIO 35 to GND.

### 5. OPTICAL IR FLAME SENSOR
- Module: LM393 Optical IR Flame Sensor Module
  * VCC -> ESP32 3.3V pin (or +5V_AUX)
  * GND -> Common GND
  * D0 (Digital Output) -> ESP32 GPIO 34 (Input-only, Active LOW)

### 6. MOTOR DRIVER & 4WD DC DRIVE MOTORS
- Motor Driver: TB6612FNG Dual H-Bridge Driver
  * VM pin -> +12V Bus (with a 1000uF 25V decoupling capacitor placed across VM and GND)
  * VCC pin -> +5V_AUX (Logic supply)
  * GND pins -> Common GND
  * PWMA pin -> ESP32 GPIO 25 (LEDC PWM Channel 0 @ 20 kHz)
  * AIN1 pin -> ESP32 GPIO 26
  * AIN2 pin -> ESP32 GPIO 27
  * PWMB pin -> ESP32 GPIO 18 (LEDC PWM Channel 1 @ 20 kHz)
  * BIN1 pin -> ESP32 GPIO 19
  * BIN2 pin -> ESP32 GPIO 21
  * STBY pin -> ESP32 GPIO 22 (Active HIGH enable)
  * Output AO1 & AO2 -> Connect to two Left DC Geared Motors wired in parallel.
  * Output BO1 & BO2 -> Connect to two Right DC Geared Motors wired in parallel.

### 7. TURRET TILT SERVO
- Actuator: TowerPro MG90S Micro Servo (Gimbal Pitch Control)
  * VCC (Red) -> +5V_AUX
  * GND (Brown/Black) -> Common GND
  * Signal (Orange/Yellow) -> ESP32 GPIO 13

### 8. EXTINGUISHER MECHANICAL TRIGGER SERVO MOTOR
- Actuator: TowerPro MG996R High-Torque Metal Gear Servo (Extinguisher Trigger Puller)
  * VCC (Red) -> +5V_AUX (6.0V high current rail)
  * GND (Brown/Black) -> Common GND
  * Signal (Orange/Yellow) -> ESP32 GPIO 23 (50 Hz PWM control)
  * Function: Mechanically pulls/depresses the fire extinguisher handle lever to release extinguishing agent. No MOSFET or solenoid valve used.

### 9. SCHEMATIC LAYOUT & VISUAL PREFERENCES
- Arrange power regulation on the lower left, Raspberry Pi and ESP32 in the center, MPU-6050 and ultrasonic sensors on the upper/middle left, motor driver and motors on the right, and the two servos (Tilt & Trigger) clearly labeled on the lower right.
- Color code the nets: Red for +12V, Orange for +5V_PI, Yellow for +5V_AUX, Black for GND, Blue for PWM/Control, Green for Sensor Echo/Signals.
```
