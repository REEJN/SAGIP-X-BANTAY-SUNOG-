# SAGIP-X: Design and Implementation of an Autonomous Mobile Firefighting Robot for Residential Environments

---

# I. INTRODUCTION

### 1. Background of the Study

Residential fires are a serious safety concern because they can cause injuries, loss of life, and property damage. In residential environments, fire incidents may occur in areas such as kitchens, living rooms, and other household spaces, where furniture and other objects may obstruct movement. These conditions highlight the importance of early fire detection and timely response. According to the U.S. Fire Administration (2024), fires in modern homes can develop rapidly due to factors such as synthetic materials used in furniture and interior finishes. Although conventional fire alarms and fire extinguishers are essential safety devices, their operation may still require a person to recognize the danger and respond appropriately. In many household emergencies, occupants may be asleep, absent, physically limited, or rapidly overwhelmed by smoke inhalation before they can intervene. Manual fire suppression using portable extinguishers requires an occupant to deliberately approach the flame base, exposing them to intense radiant heat, toxic carbon monoxide, and burn hazards. Municipal fire services provide vital structural defense, but transit delays and traffic conditions frequently prevent arrival during the critical incipient phase of combustion. An autonomous mobile firefighting robot may help address this concern by detecting a fire, navigating toward its location, avoiding obstacles, and activating a fire-suppression mechanism according to its programmed control system.

Previous studies have demonstrated the development of mobile firefighting robots with different approaches to fire detection, navigation, obstacle avoidance, and fire suppression. Nguyen (2024) developed an autonomous mobile robot for firefighting that uses machine vision and a programmable logic controller to approach a fire while avoiding obstacles. Similarly, Chenchireddy et al. (2025) presented an Arduino-based firefighting robot that uses fire and infrared sensors, motors, and a water-pumping mechanism to respond to detected fires. These studies demonstrate existing approaches to automating firefighting tasks. However, the performance of a robot depends on its hardware configuration, control system, and testing conditions. Therefore, the performance of previously developed systems cannot automatically be assumed for a different implementation. Early educational platforms that utilize single 8-bit microcontrollers and submerged water pumps present practical limitations in residential spaces. Liquid water spray causes electrical shock risks when sprayed near energized domestic appliances and wiring, can cause violent flare-ups when encountering burning kitchen cooking oil, and produces extensive secondary water damage to interior furnishings. Furthermore, single-microcontroller architectures experience severe processing bottlenecks when attempting to handle real-time computer vision while simultaneously managing high-frequency motor control and sensor polling. Industrial automated systems utilizing heavy programmable logic controllers are often too bulky, complex, and cost-prohibitive for domestic residential hallways. This provides a clear technical basis for evaluating how a specific, compact, dual-tier robot configuration performs under clearly defined and controlled residential-like conditions.

In response to this concern, the proposed study will develop SAGIP-X: Design and Implementation of an Autonomous Mobile Firefighting Robot for Residential Environments. The robot is intended to detect and locate a designated simulated fire source, navigate toward it while avoiding obstacles, and activate its fire-suppression mechanism in a controlled indoor residential-like environment. The platform employs a decoupled dual-tier architecture: a Raspberry Pi 3 Model B handles visual flame detection and decision logic, while an ESP32-WROOM-32 microcontroller manages real-time motor control, ultrasonic distance measurement, and servo actuation. Rather than using water, the prototype incorporates a commercial portable fire extinguisher canister triggered by an onboard high-torque servo motor. The study will evaluate the prototype's fire-detection and target-location performance, navigation and obstacle avoidance, response time, and fire-suppression performance through predefined procedures and repeated trials. Its functionality and safety will also be assessed using established testing criteria and a structured evaluation checklist. Rather than assuming that the robot will be more affordable or effective than existing systems, the study will document its measured performance and limitations. The findings will apply only to the developed prototype and the specific experimental conditions, and the system will not be considered a replacement for certified fire-safety equipment or professional firefighters.

### 2. Statement of the Problem

Residential fires can spread quickly and make it dangerous for people to approach the affected area. This project proposes SAGIP-X, an autonomous mobile firefighting robot designed to detect and locate a fire, move toward it while avoiding obstacles, and aim and activate a fire extinguisher.

The study aims to design, build, and evaluate the prototype. Specifically, it seeks to answer the following questions:

1. How can the SAGIP-X robot be designed and assembled using its camera, controller, motors, drivers, distance sensors, and fire-extinguisher system?
2. How accurately can the robot detect and locate a simulated fire during controlled tests?
3. How effectively can it move toward the target while avoiding obstacles?
4. How long does the robot take to detect the target and activate the extinguisher?
5. How effectively can the robot aim the extinguisher toward the target and put out the test fire?
6. How consistently does the robot perform these tasks across repeated trials?
7. How do evaluators rate the prototype in terms of:
   7.1 Functionality;
   7.2 Reliability;
   7.3 Response Time; and
   7.4 Safety?

### 3. Objectives of the Study

The general objective of this study is to design, develop, and evaluate SAGIP-X, an autonomous mobile firefighting robot designed to detect and locate a fire, navigate toward the target while avoiding obstacles, and activate a fire-suppression mechanism in a controlled residential environment.

Specifically, the study aims to:

1. Design and develop the SAGIP-X prototype by integrating the camera, flame sensor, controller, motors, distance sensors, and fire-extinguisher system based on the approved system design.
2. Evaluate the accuracy of the robot in detecting and locating a simulated fire through controlled testing under varied environmental lighting conditions.
3. Assess the navigation performance of the robot in moving toward the fire target while avoiding obstacles in a controlled residential-like environment.
4. Measure the response time of the robot from fire detection to the activation of its fire-suppression mechanism across individual functional phases.
5. Evaluate the targeting and fire-suppression performance of the robot by determining its ability to aim the extinguisher toward the simulated fire and suppress it during controlled tests.
6. Determine the consistency of the robot's performance in fire detection, target location, obstacle avoidance, response time, and fire suppression through repeated trials.
7. Evaluate the prototype's overall performance based on functionality, reliability, response time, and safety using the assessments of selected evaluators.

### 4. Scope and Limitations

#### 4.1 Scope of the Study

The main aim of this project is to design and develop an autonomous mobile robot that can detect and extinguish small fires within a domestic environment. This includes joining together all hardware and software components, such as the camera, flame sensor, controller, motors, motor drivers, distance sensors, and fire extinguisher.

The robot is designed to autonomously detect a fire, locate it, navigate toward the target while avoiding obstacles, and aim and activate its fire-suppression system. The physical platform features a four-wheel drive (4WD) differential skid-steer chassis, powered by four geared DC TT motors driven by a TB6612FNG dual H-bridge motor driver. Computation is partitioned across a decoupled dual-tier embedded architecture: a Raspberry Pi 3 Model B executes the computer vision pipeline using an attached USB webcam, while an ESP32 microcontroller manages hard real-time tasks, including ultrasonic distance acquisition, sensor interlocks, motor driving, and servo positioning. Obstacle avoidance is achieved reactively using three HC-SR04 ultrasonic sensors mounted at the front, left, and right of the chassis, augmented by an embedded TinyML neural steering policy and a short-term local 2D proximity memory to escape corridor cul-de-sacs. Target alignment is performed through an articulated single-axis tilt turret driven by a TowerPro MG90S micro servo, combined with differential rotation of the chassis for azimuth steering. Fire suppression is accomplished using a commercial portable fire extinguisher can actuated by a dedicated TowerPro MG996R metal-gear high-torque servo motor.

To facilitate developer diagnostics, software calibration, and live evaluator inspection during demonstration trials, the platform includes a local web-based telemetry streaming dashboard hosted directly on the Raspberry Pi. This dashboard provides a live visual window displaying optical camera frames, detection bounding boxes, confidence metrics, and sensor readouts over local Wi-Fi. The robot operates with 100% offline autonomy; local Wi-Fi telemetry is purely an observational instrument and is not required for autonomous fire detection, navigation, or suppression.

The scope of the study is restricted to controlled testing in a residential-style test environment with a simulated fire. Performance is evaluated in terms of fire-detection and target-location accuracy, navigation and obstacle avoidance, response time, targeting and fire suppression, and consistency across repeated trials, together with evaluator ratings of functionality, reliability, response time, and safety.

#### 4.2 Limitations of the Study

The study is limited by the following technical and operational constraints:

1. The robot is tested using controlled, simulated fires only (paraffin wax candle clusters generating a flame height between 5.0 cm and 8.0 cm). It is not tested against actual structural fires, open liquid fuels, or violent combustible materials, and its performance in real fire emergencies is not established by this study.
2. Testing is conducted in a controlled, residential-like environment with a defined layout (5.0 m by 5.0 m test area with corridor widths between 0.90 m and 1.20 m). Conditions such as extreme smoke density, structural collapse, extreme convective heat, changing room layouts, stairs, debris, and heavily cluttered spaces found in actual residential fires are not fully represented.
3. The robot is limited to indoor operation on flat, level surfaces (such as ceramic tile, polished vinyl, and finished concrete). It is not designed to climb stairs, cross floor sills taller than 10 mm, traverse elevated areas, or operate outdoors.
4. Fire detection depends on the camera and flame sensor under the test conditions used. Detection accuracy may decrease under conditions such as low lighting, optical obstructions blocking the sensor line of sight, or fire sources outside the sensor effective range (exceeding 2.5 meters).
5. Obstacle avoidance depends strictly on the three ultrasonic distance sensors and may not detect obstacles with sound-absorbing textile surfaces, narrow leg posts under 20 mm, or drop-offs such as downward stairwells that fall outside their conical beam field.
6. The robot does not utilize wheel encoders, optical odometry, or LiDAR, and does not perform Simultaneous Localization and Mapping (SLAM). While a local 2D ultrasonic spatial proximity buffer is maintained in volatile memory for cul-de-sac recovery, the system does not construct or store global metric room maps.
7. The fire-suppression mechanism is limited by the capacity and type of extinguishing agent carried by the robot. The onboard portable fire extinguisher canister contains a finite volume of extinguishing agent suitable for short bursts (up to 3.0 seconds), restricting suppression capability to small incipient fires within a 30.0 cm standoff distance.
8. The prototype is designed for initial fire detection and suppression support only. It does not replace certified firefighting equipment, emergency responders, certified fire alarm installations, or building fire-protection systems, and it does not perform human search and rescue operations.

---

# II. SYSTEM ARCHITECTURE & DESIGN

### 1. System Architecture and Block Diagram

The SAGIP-X platform employs a decoupled dual-tier architecture that segregates non-deterministic, compute-intensive computer vision tasks from deterministic, hard real-time motor control and sensor polling.

The high-level computation tier consists of a Raspberry Pi 3 Model B single-board computer running Raspberry Pi OS Lite. The Raspberry Pi interfaces with a front-facing USB webcam capturing video at 640x360 resolution at 30 frames per second. An optimized vision pipeline executes color thresholding in HSV color space alongside a quantized neural detection network to identify flame bounding boxes. To reject transient false positives caused by incandescent bulbs or specular reflections, an explicit temporal verification filter buffers results across five consecutive video frames, requiring positive detection in at least three frames before confirming flame presence. The high-level tier formats target centroid coordinates $(x, y)$ and operational states into compact JSON packets transmitted via a hardware UART serial link.

The real-time control tier consists of an ESP32-WROOM-32 microcontroller. The ESP32 is directly responsible for all physical input and output operations:
1. **Motor Locomotion:** Driving the TB6612FNG dual MOSFET H-bridge driver using 20 kHz hardware PWM channels to ensure silent, efficient motor operation without audible switching noise. Motor control firmware includes dead-zone compensation and slew-rate acceleration limiting to prevent abrupt current draws and battery voltage brownouts.
2. **Ultrasonic Distance Measurement:** Firing the three HC-SR04 sensors in a scheduled round-robin sequence (60 ms total cycle, 20 ms per sensor) to prevent ultrasonic echo cross-talk between transducers. Distance readings pass through a firmware-level 3-sample rolling median filter to eliminate acoustic multipath spikes.
3. **TinyML Reactive Steering Policy:** Executing an embedded, quantized neural network policy directly on the ESP32 core (`navigation_model.h`). This ultra-compact Multi-Layer Perceptron (818 parameters, under 4 KB RAM footprint) maps ultrasonic range inputs and heading offsets to continuous differential motor PWM values, providing smooth collision-avoidance curves while maintaining deterministic execution.
4. **Local Proximity Memory & Cul-de-Sac Escape:** Maintaining a short-term 2D spatial clearance buffer in volatile memory. If forward progress is blocked on all three ultrasonic sensors simultaneously (dead-end corridor), the controller initiates an automated 180-degree in-place U-turn maneuver to backtrack toward clear pathways.
5. **Turret Pitch Control:** Driving a TowerPro MG90S metal-gear micro servo connected to GPIO 13 to tilt the sensor turret between 45 degrees downward and 135 degrees upward.
6. **Secondary Optical Verification:** Polling an LM393 infrared flame sensor module (spectral range 760 nm to 1100 nm) to physically verify infrared radiation before permitting suppression.
7. **Mechanical Extinguisher Actuation:** Driving a TowerPro MG996R high-torque metal-gear servo connected to GPIO 23 to depress the mechanical trigger lever of the portable extinguisher can. An internal 3000 ms firmware watchdog timer guarantees that the servo automatically returns to its rest position (0 degrees) even in the event of serial communication loss.

The serial communication interface between the Raspberry Pi and ESP32 uses a direct 3-wire 3.3V TTL UART connection operating at 115200 baud, 8 data bits, no parity, and 1 stop bit (8-N-1). The physical pins connect Raspberry Pi GPIO 14 (TXD0) to ESP32 GPIO 16 (RX2), Raspberry Pi GPIO 15 (RXD0) to ESP32 GPIO 17 (TX2), and share a common ground reference.

#### Diagnostic and Telemetry Monitoring Interface (Developer Web Dashboard)
To allow developers and evaluators to observe the internal operations of the robot without tethering physical display monitors, the Raspberry Pi hosts an embedded Flask telemetry server (`bantay_sunog/monitoring/web_stream.py`). Accessible over local Wi-Fi at `http://<pi-ip>:5000/`, the dashboard streams real-time MJPEG video overlaid with flame candidate bounding boxes, detection confidence ratings, targeting crosshairs, and a sensor HUD display. The dashboard functions strictly as an observational diagnostic instrument; the robot control architecture is completely decoupled, ensuring that full firefighting autonomy is maintained even if network connectivity is unavailable.

```
=======================================================================================================================
                                      SAGIP-X HARDWARE ARCHITECTURE BLOCK DIAGRAM
=======================================================================================================================

   [ 3S LiPo Battery 11.1V - 12.6V ] ---> [ Master Switch + 5A Inline Fuse ]
                                                    |
       +--------------------------------------------+--------------------------------------------+
       |                                            |                                            |
       v                                            v                                            v
  [ XL4015 Step-Down Buck ]                  [ LM2596 Step-Down Buck ]                   [ TB6612FNG VM Rail ]
       | (5.15V DC @ 3A)                            | (6.0V DC @ 3A)                             | (11.1V - 12.6V Motor Power)
       v                                            v                                            v
  [ Raspberry Pi 3 Model B ]                 +---------------------------+               [ 4x DC TT Motors ]
       |                                     | - TowerPro MG90S Tilt     |                       ^
       | - USB Webcam (640x360)              |   Servo (GPIO 13)         |                       |
       | - Vision Pipeline & Temporal Filter | - TowerPro MG996R Trigger |                       |
       | - Developer Web Dashboard (Port 5000)   Servo (GPIO 23)         |               [ TB6612FNG Driver ]
       |                                     | - 3x HC-SR04 Sonar (5V)   |                       ^
       |                                     | - LM393 IR Flame Sensor   |                       | (PWM / Direction)
       |                                     | - MPU-6050 6-Axis IMU     |                       |
       |                                     +---------------------------+                       |
       |                                                   |                                     |
       |                                                   v                                     |
       +========== [ 3-Wire 3.3V TTL UART (115200 Baud) ] ==========> [ ESP32-WROOM-32 Controller ]
                   (Pi GPIO 14/15 <-----------------> ESP32 GPIO 16/17)  | - TinyML Neural Nav Engine
                                                                         | - 20 kHz Motor PWM & Slew
                                                                         | - Sonar Median Filter
                                                                         | - 3000ms Trigger Watchdog
=======================================================================================================================
```

The electrical power distribution system uses an 11.1V 3S Lithium-Polymer (LiPo) battery pack rated at 2200 mAh with a 25C discharge capability. Power passes through a single-pole single-throw (SPST) master rocker switch and a 5A fast-blow inline fuse. The unregulated 11.1V rail supplies motor drive power directly to the VM pin of the TB6612FNG motor driver. Two independent step-down switching buck converters isolate the logic and actuator subsystems:
1. An XL4015 buck converter steps down the battery voltage to a regulated 5.15V DC (3A peak) to supply the Raspberry Pi 3B via its 5V header pins, preventing low-voltage CPU throttling.
2. An LM2596 buck converter steps down the battery voltage to a regulated 6.0V DC (3A peak) dedicated to the high-torque MG996R trigger servo, the MG90S tilt servo, and the ultrasonic sensor VCC lines.

The ESP32 is powered via regulated 5V through its VIN pin. All 5V echo signals from the HC-SR04 ultrasonic sensors pass through passive voltage dividers (1kΩ and 2kΩ metal film resistors) to level-shift echo pulses down to 3.3V, protecting ESP32 input GPIO pins. All subsystem ground returns converge at a single Star Ground bus to eliminate ground loop voltages and prevent motor commutation noise from resetting the digital microcontrollers.

#### Table 1. Hardware Pin Assignment and Electrical Interface
| Microcontroller / SBC | Pin Identifier | Connected Hardware Module | Signal Direction | Electrical Standard & Notes |
|---|---|---|:---:|---|
| **Raspberry Pi 3B** | GPIO 14 (Pin 8) | ESP32 GPIO 16 (RX2) | Output | 3.3V TTL UART TX (115200 baud) |
| **Raspberry Pi 3B** | GPIO 15 (Pin 10) | ESP32 GPIO 17 (TX2) | Input | 3.3V TTL UART RX (115200 baud) |
| **Raspberry Pi 3B** | USB Port 1 | USB UVC Web Camera | Bidirectional | USB 2.0 Host (640x360 @ 30 FPS) |
| **ESP32-WROOM-32** | GPIO 16 | Raspberry Pi GPIO 14 (TXD0) | Input | 3.3V TTL UART RX2 |
| **ESP32-WROOM-32** | GPIO 17 | Raspberry Pi GPIO 15 (RXD0) | Output | 3.3V TTL UART TX2 |
| **ESP32-WROOM-32** | GPIO 34 | LM393 Optical Flame Sensor D0 | Input | 3.3V GPI Input Only (Active LOW on fire) |
| **ESP32-WROOM-32** | GPIO 14 | MPU-6050 6-Axis IMU (SDA) | Bidirectional | I2C Fast Mode (400 kHz, 3.3V) |
| **ESP32-WROOM-32** | GPIO 15 | MPU-6050 6-Axis IMU (SCL) | Output | I2C Clock (400 kHz Fast Mode, 3.3V) |
| **ESP32-WROOM-32** | GPIO 4 | Front HC-SR04 Trigger | Output | 3.3V 10µs Trigger Pulse |
| **ESP32-WROOM-32** | GPIO 36 | Front HC-SR04 Echo (SENSOR_VP)| Input | 3.3V (via 1kΩ / 2kΩ Voltage Divider) |
| **ESP32-WROOM-32** | GPIO 32 | Left HC-SR04 Trigger | Output | 3.3V 10µs Trigger Pulse |
| **ESP32-WROOM-32** | GPIO 39 | Left HC-SR04 Echo (SENSOR_VN) | Input | 3.3V (via 1kΩ / 2kΩ Voltage Divider) |
| **ESP32-WROOM-32** | GPIO 33 | Right HC-SR04 Trigger | Output | 3.3V 10µs Trigger Pulse |
| **ESP32-WROOM-32** | GPIO 35 | Right HC-SR04 Echo | Input | 3.3V (via 1kΩ / 2kΩ Voltage Divider) |
| **ESP32-WROOM-32** | GPIO 25 | TB6612FNG PWMA (Left Motors) | Output | 20 kHz Hardware PWM Channel |
| **ESP32-WROOM-32** | GPIO 26 | TB6612FNG AIN1 (Left Direction)| Output | 3.3V Logic Level |
| **ESP32-WROOM-32** | GPIO 27 | TB6612FNG AIN2 (Left Direction)| Output | 3.3V Logic Level |
| **ESP32-WROOM-32** | GPIO 18 | TB6612FNG PWMB (Right Motors)| Output | 20 kHz Hardware PWM Channel |
| **ESP32-WROOM-32** | GPIO 19 | TB6612FNG BIN1 (Right Dir 1) | Output | 3.3V Logic Level |
| **ESP32-WROOM-32** | GPIO 21 | TB6612FNG BIN2 (Right Dir 2) | Output | 3.3V Logic Level |
| **ESP32-WROOM-32** | GPIO 22 | TB6612FNG STBY (Driver Enable)| Output | 3.3V Active HIGH Standby Pin |
| **ESP32-WROOM-32** | GPIO 13 | TowerPro MG90S Tilt Servo | Output | 50 Hz PWM (1.0 ms to 2.0 ms pulse) |
| **ESP32-WROOM-32** | GPIO 23 | TowerPro MG996R Extinguisher | Output | 50 Hz PWM with 3000ms Watchdog |

### 2. Proposed Components and Budget

The complete bill of materials, component functions, unit prices, and subtotal costs for the SAGIP-X prototype are presented in Table 2. In accordance with the project notes to revise the component list to match the actual mechanical fire extinguisher suppression mechanism rather than a water pump, spray nozzle, and flexible tubing, the bill of materials reflects the onboard commercial extinguisher canister, the TowerPro MG996R high-torque trigger servo, the TowerPro MG90S tilt servo, and associated mounting brackets. All prices reflect local retail procurement within the Philippines.

#### Table 2. Proposed Components and Estimated Prototype Budget
| Item # | Component Name | Qty | Functional Role in Prototype | Unit Price (₱) | Subtotal (₱) | Status / Notes |
|---|---|:---:|---|:---:|:---:|---|
| 1 | **Raspberry Pi 3 Model B** | 1 | High-level computer vision processing, video frame acquisition, neural flame detection, and UART telemetry transmission. | 2,200.00 | 2,200.00 | Essential |
| 2 | **ESP32-WROOM-32 Development Board** | 1 | Real-time motion controller, 20 kHz motor PWM generation, round-robin ultrasonic sensor polling, and servo control. | 250.00 | 250.00 | Essential |
| 3 | **USB HD UVC Web Camera (640x360 @ 30 FPS)** | 1 | Real-time video frame capture for optical flame detection and bounding-box spatial localization. | 450.00 | 450.00 | Essential |
| 4 | **MicroSD Card (32GB Class 10 A1)** | 1 | Operating system storage for Raspberry Pi OS Lite, OpenCV libraries, Python runtime, and test log files. | 250.00 | 250.00 | Essential |
| 5 | **4WD Robot Chassis Kit (4 Motors & Wheels)** | 1 | Four-wheel drive differential skid-steer mechanical chassis, geared DC TT motors, and rubber traction tires. | 600.00 | 600.00 | Essential |
| 6 | **TB6612FNG Dual H-Bridge Motor Driver** | 1 | High-efficiency dual MOSFET motor driver providing independent bidirectional drive for left and right motor banks. | 180.00 | 180.00 | Essential |
| 7 | **HC-SR04 Ultrasonic Distance Sensors** | 3 | Spatial obstacle clearance measurement (front 0°, left +45°, and right -45° sensor placement). | 90.00 | 270.00 | Essential |
| 8 | **LM393 Optical IR Flame Sensor Module** | 1 | Secondary physical infrared flame verification interlock (detects radiation from 760 nm to 1100 nm). | 65.00 | 65.00 | Essential |
| 9 | **TowerPro MG90S Metal-Gear Micro Servo** | 1 | Articulated vertical pitch tilt actuator for sensor turret elevation (45° downward to 135° upward). | 180.00 | 180.00 | Essential |
| 10 | **TowerPro MG996R Metal-Gear High-Torque Servo** | 1 | Mechanical trigger actuator for physically depressing the release lever of the onboard fire extinguisher canister. | 250.00 | 250.00 | Essential |
| 11 | **Portable Fire Extinguisher Canister** | 1 | Onboard commercial pressurized extinguisher can containing clean aerosol/chemical extinguishing agent. | 400.00 | 400.00 | Essential |
| 12 | **Mechanical Canister Bracket and Trigger Linkage** | 1 set | Custom mechanical mounting bracket and aluminum linkage arm connecting servo horn to extinguisher lever. | 150.00 | 150.00 | Essential |
| 13 | **Rechargeable 3S LiPo Battery (11.1V, 2200 mAh)** | 1 | Main rechargeable high-current power supply for motors, servos, and step-down voltage regulators. | 750.00 | 750.00 | Essential |
| 14 | **Dual DC-DC Step-Down Buck Converters** | 2 | Dedicated XL4015 (5.15V @ 3A for Pi) and LM2596 (6.0V @ 3A for servos and sensors) voltage regulation modules. | 150.00 | 300.00 | Essential |
| 15 | **Master Power Switch and Inline 5A Fuse Assembly** | 1 set | Primary SPST high-current toggle disconnect switch and inline fuse holder with 5A fast-acting fuse. | 120.00 | 120.00 | Essential |
| 16 | **Power Distribution Terminal Block and Bus** | 1 | Central terminal block for clean power distribution and star ground connection. | 75.00 | 75.00 | Recommended |
| 17 | **Passive Components & Voltage Dividers** | 1 set | 1kΩ and 2kΩ metal-film resistors for 5V-to-3.3V Echo level-shifting, 1000µF and 470µF electrolytic decoupling capacitors. | 80.00 | 80.00 | Essential |
| 18 | **Chassis Standoffs, Fasteners & Mounting Plates** | 1 set | Brass hex standoffs, M3 bolts, lock nuts, washers, and laser-cut acrylic mounting plates. | 250.00 | 250.00 | Essential |
| 19 | **Wiring Harness, Dupont Leads & Heat-Shrink** | 1 set | AWG 20/24 silicone stranded hookup wire, Dupont jumper leads, cable ties, and heat-shrink tubing. | 150.00 | 150.00 | Essential |
| 20 | **MPU-6050 6-Axis IMU Module** | 1 | I2C sensor module providing angular yaw rate and chassis orientation monitoring. | 120.00 | 120.00 | Essential |
| 21 | **Solder and Miscellaneous Assembly Supplies** | 1 set | Rosin-core solder wire, flux, insulating electrical tape, and structural mounting adhesives. | 100.00 | 100.00 | Essential |
| **TOTAL** | **Estimated Prototype Development Cost** | | | | **₱6,940.00** | |

*Note: The estimated cost of ₱6,940.00 successfully complies with the project financial limit (below ₱7,500.00). All components are sourced from local domestic electronics and robotics distributors in the Philippines.*

### 3. Integration Plan of Subjects

The design, construction, and evaluation of the SAGIP-X mobile robot integrate core principles from five key Computer Engineering subjects:

1. **Mixed Signals:** In the sensory subsystem, physical phenomena produce continuous analog and discrete digital signals. The HC-SR04 ultrasonic transducers generate high-frequency acoustic waves and return analog pulse-width echo signals that are timed by the microcontroller. Because acoustic reflections off hard walls can produce transient multipath noise, a firmware-level 3-sample rolling median filter is applied to reject false spike readings. The LM393 infrared flame sensor provides both an analog photodiode voltage ($A_0$) corresponding to infrared radiation intensity and a digitized comparator output ($D_0$) tuned via an onboard potentiometer. The ultrasonic sensor echo output operates at 5.0V, requiring passive resistive voltage dividers ($R_1 = 1.0\text{ k}\Omega, R_2 = 2.0\text{ k}\Omega$) to safely level-shift the pulse down to the 3.3V CMOS threshold of the ESP32:
   $$V_{out} = V_{in} \times \left( \frac{R_2}{R_1 + R_2} \right) = 5.0\text{ V} \times \left( \frac{2000}{1000 + 2000} \right) = 3.33\text{ V}$$
   Power decoupling is implemented with 1000µF and 470µF electrolytic capacitors placed adjacent to the switching buck converters to suppress motor commutation ripple.

2. **Logic and Design:** System decision rules combine multi-sensor inputs to determine operational states. A multi-input combinational and sequential logic structure governs whether a suspected fire source warrants navigation or suppression. High-level bounding-box detections from the vision system are logically AND-gated with the digital logic output of the LM393 physical infrared sensor. The suppression actuator is disabled by combinational logic unless three concurrent conditions are satisfied: target alignment within the optical horizontal error deadband ($|e_x| \le 20\text{ pixels}$), target standoff distance within the safe ultrasonic threshold ($d_{front} \le 30.0\text{ cm}$), and positive physical infrared radiation confirmation ($D_0 = \text{LOW}$). To verify these decision rules prior to physical construction, the combinational logic and state sequences were evaluated inside an interactive 3D digital twin simulation environment (`simulation/frontend/`), stress-testing edge cases and obstacle boundaries in software.

3. **Hardware Description Language (HDL):** A Verilog finite state machine (FSM) models and verifies the digital decision sequence in behavioral simulation prior to firmware synthesis. The state transition logic comprises seven discrete states: `STATE_PATROL`, `STATE_OBSTACLE_AVOID`, `STATE_FIRE_DETECTED`, `STATE_APPROACH`, `STATE_VERIFY`, `STATE_SUPPRESS`, and `STATE_SAFE_HOLD`. Synchronous reset behavior, state registers, next-state combinational decoders, and timer overflow signals are simulated in Verilog testbenches to verify that the robot cannot enter an illegal state or trigger suppression without fulfilling all prerequisite safety conditions.

4. **Real-Time Embedded Systems:** The decoupled architecture implements deterministic multitasking. The ESP32 executes real-time motion control using non-blocking software timers. Motor speed is controlled via 20 kHz hardware PWM channels generated by the ESP32 LEDC peripheral, eliminating acoustic motor whine. Ultrasonic sensors are polled sequentially on a 20 ms timer interrupt to avoid acoustic interference. On the real-time core, an embedded TinyML neural steering policy (`navigation_model.h`, 818 parameters) executes in under 2.5 milliseconds per inference cycle, computing smooth steering velocities without blocking time-critical tasks. The UART serial communication between the Raspberry Pi and ESP32 operates asynchronously at 115200 baud with a checksum-protected JSON packet format. A hardware watchdog timer and a 3000 ms firmware trigger watchdog are enforced on the ESP32 to guarantee fail-safe de-energization of the MG996R trigger servo if communication ceases.

5. **Feedback and Control:** The robot applies closed-loop feedback control across multiple degrees of freedom. For target centering, the horizontal pixel error $e_x = x_{centroid} - x_{frame\_center}$ generates a proportional yaw steering command that adjusts the differential speed of the left and right motor banks until the flame centroid lies within a $\pm 20\text{ pixel}$ deadband. For elevation alignment, vertical pixel error $e_y = y_{centroid} - y_{frame\_center}$ drives the MG90S tilt servo using a proportional feedback loop:
   $$\theta_{tilt}(t) = \theta_{tilt}(t-1) + K_p \cdot e_y(t)$$
   For approach velocity, the front ultrasonic distance reading $d_{front}$ provides feedback to regulate forward driving speed, decelerating the robot as it nears the target and halting locomotion when the distance reaches the safe standoff boundary of $30.0\text{ cm} \pm 2.0\text{ cm}$. Low-level motor drivers incorporate slew-rate acceleration limiting to prevent abrupt speed steps that could destabilize the high center of gravity caused by the extinguisher canister.

---

# III. METHODOLOGY

### 1. Research Design

This study employs an experimental engineering research design combined with quantitative performance evaluation. The primary goal is the design, fabrication, and physical empirical testing of the SAGIP-X autonomous mobile firefighting robot prototype.

The investigation follows an experimental input-process-output paradigm. Controlled independent variables (such as ambient lighting conditions, obstacle configurations, and target distance) are manipulated in a standardized test arena to observe and measure dependent performance variables (fire detection accuracy, target localization error, obstacle avoidance success rate, operational response latency across phases, and flame suppression success rate). Repeated empirical trials are conducted across all experimental phases to obtain statistically robust datasets. In addition to quantitative instrument measurements, the prototype is evaluated by a panel of engineering faculty members, electronics engineers, and fire safety practitioners using a structured evaluation checklist to assess functionality, reliability, response time, and operational safety.

### 2. Research Setting and Test Conditions

All physical fabrication, calibration, and testing are conducted within an enclosed, non-combustible indoor testing facility. The physical test environment comprises a mock residential layout measuring 5.0 meters in length and 5.0 meters in width, bounded by flame-retardant barriers.

```
+-------------------------------------------------------------+
|                     MOCK RESIDENTIAL ARENA                  |
|                      (5.0 m x 5.0 m)                        |
|                                                             |
|   +-------------------+             +-------------------+   |
|   |                   |  Corridor   |                   |   |
|   |    Mock Room A    |  (1.0 m)    |    Mock Room B    |   |
|   |                   |             |                   |   |
|   +-------   ---------+             +-------   ---------+   |
|         Doorway (0.8 m)                   Doorway (0.8 m)   |
|                                                             |
|                    Corridor (1.2 m)                         |
|   -------------------------     -------------------------   |
|                                                             |
|          [ Obstacle 1 ]                 [ Obstacle 2 ]      |
|                                                             |
|                                          +--------------+   |
|                                          | Target Fire  |   |
|                                          | [Candle Pan] |   |
|                                          +--------------+   |
|                                                             |
|   [ SAGIP-X Start Point ]                                   |
+-------------------------------------------------------------+
```

The test arena includes two mock residential rooms connected by a central corridor. Hallway widths are configured between 0.90 m and 1.20 m, and interior doorway openings are fixed at 0.80 m, modeling typical residential architectural standards. The floor surface consists of smooth, level ceramic tile.

Environmental test conditions are structured across three distinct lighting profiles:
1. **Ambient Daylight:** Standard indoor daytime illumination measured between 300 and 500 lux using a digital lux meter.
2. **Low-Light Twilight:** Dim ambient illumination measured between 10 and 30 lux, simulating nighttime conditions or unlit residential rooms.
3. **Smoke-Obscured Condition:** Ambient daylight combined with simulated non-toxic aerosol haze (optical obscuration maintaining visibility between 1.5 m and 2.5 m) generated using a stage haze generator.

The simulated fire source consists of a standardized cluster of three paraffin wax candles set within a heat-resistant steel drip tray (dimensions 250 mm x 200 mm x 30 mm). The candles produce steady, open flames with heights ranging between 5.0 cm and 8.0 cm. Open liquid fuels, gasoline, and volatile alcohols are strictly prohibited to maintain safety and experimental repeatability.

### 3. System Components and Functions

The SAGIP-X mobile robot integrates specific hardware and software components structured into dedicated functional subsystems:

1. **Processing and Computation Tier:**
   - **Raspberry Pi 3 Model B:** Quad-core 1.2 GHz Broadcom BCM2837 64-bit SoC with 1 GB LPDDR2 RAM. Executes Linux-based computer vision routines, processes USB video frames at 30 FPS, runs HSV color segmentation and quantized object detection, buffers multi-frame detection results, hosts the developer web telemetry stream, and transmits target coordinates via UART.
   - **ESP32-WROOM-32 Microcontroller:** Dual-core Xtensa 32-bit LX6 running at 240 MHz with 520 KB SRAM. Responsible for hard real-time execution: generates 20 kHz PWM signals for locomotion, executes the embedded TinyML obstacle steering policy (`navigation_model.h`), triggers round-robin ultrasonic sensor measurements with median filtering, monitors analog and digital flame sensor pins, controls the MG90S tilt servo and MG996R trigger servo, and enforces safety watchdog timers.

2. **Perception and Sensing Subsystem:**
   - **Front USB UVC Webcam:** Captures 640x360 resolution color video at 30 FPS with a 65-degree horizontal field of view, providing primary optical flame detection and target bounding coordinates.
   - **HC-SR04 Ultrasonic Distance Sensors (3 Units):** Ultrasonic rangefinders mounted at the chassis front (0°), left (+45°), and right (-45°). Each sensor emits a 40 kHz burst upon receiving a 10 µs trigger pulse and outputs a pulse proportional to distance (range 2.0 cm to 400.0 cm, resolution 0.3 cm). Echo signals are scaled via 1kΩ/2kΩ resistive dividers to 3.3V and filtered via rolling median logic.
   - **LM393 Infrared Flame Sensor:** Features a photodiode sensitive to infrared wavelengths from 760 nm to 1100 nm with a 60-degree detection angle. Provides an analog signal ($A_0$) reflecting radiation intensity and a digital output ($D_0$) serving as a hardware interlock.
   - **MPU-6050 6-Axis Inertial Measurement Unit:** Micro-electro-mechanical (MEMS) sensor combining a 3-axis gyroscope and 3-axis accelerometer on an I2C bus (address `0x68`) sampled at 100 Hz to monitor angular turning rates and detect wheel slippage.

3. **Locomotion and Drive Subsystem:**
   - **4WD TT DC Geared Motors:** Four permanent magnet brushed DC motors with a 1:48 reduction gearbox operating at nominal 6V to 9V, delivering a no-load wheel speed of approximately 130 RPM.
   - **TB6612FNG Dual MOSFET Motor Driver:** Efficient H-bridge driver delivering continuous 1.2A per channel (3.2A peak) with low internal forward voltage drop, driven by 20 kHz PWM channels from the ESP32 with firmware slew-rate limiting.

4. **Fire Suppression and Aiming Subsystem:**
   - **Portable Fire Extinguisher Canister:** A commercially manufactured portable extinguisher can (capacity 500g to 1000g) containing a non-conductive, clean aerosol extinguishing agent suitable for Class A, B, and electrical equipment fires.
   - **TowerPro MG996R High-Torque Servo:** Metal-gear servo delivering up to 11 kg-cm stall torque at 6.0V, mounted to a rigid bracket to mechanically depress the extinguisher release valve upon receiving a PWM command on ESP32 GPIO 23.
   - **TowerPro MG90S Micro Servo:** Metal-gear micro servo delivering 2.2 kg-cm torque at 6.0V, driving the vertical tilt axis of the camera and flame sensor turret across an angular range of 45° to 135°.

5. **Power Distribution Subsystem:**
   - **11.1V 3S LiPo Battery (2200 mAh, 25C):** Provides master unregulated DC power.
   - **XL4015 Step-Down Buck Converter:** Configured for 5.15V DC output at up to 3A to power the Raspberry Pi 3B.
   - **LM2596 Step-Down Buck Converter:** Configured for 6.0V DC output at up to 3A to power the servo actuators, ultrasonic sensors, and the ESP32 VIN pin.
   - **Safety Hardware:** SPST master power switch, 5A fast-blow inline fuse, and a common Star Ground bus.

### 4. Conceptual and Operational Frameworks

#### 4.1 Conceptual Framework (Input - Process - Output Model)

The conceptual foundation of the study is represented using the Input-Process-Output (IPO) model:

```
+---------------------------------------------------------------------------------------------------+
|                                       CONCEPTUAL FRAMEWORK                                        |
+-----------------------------------+-----------------------------------+---------------------------+
|               INPUT               |              PROCESS              |          OUTPUT           |
+-----------------------------------+-----------------------------------+---------------------------+
| 1. Optical Video Stream (USB Cam) | 1. Video Frame Preprocessing,     | 1. Verified Fire Target   |
| 2. Infrared Radiation Levels      |    Color Segmentation & Detection |    Spatial Coordinates    |
|    (LM393 IR Flame Sensor)        | 2. Multi-Frame Temporal Filtering | 2. Autonomous Collision-  |
| 3. Spatial Ultrasonic Echoes      | 3. TinyML Steering & Reactive     |    Free Corridor Patrol   |
|    (3x HC-SR04 Sensors)           |    Obstacle Avoidance Algorithms  | 3. Precise Aiming & Tilt  |
| 4. Chassis Heading & Tilt Rates   | 4. Proportional Target Centering  |    Turret Alignment       |
|    (MPU-6050 6-Axis IMU)          |    and Standoff Distance Control  | 4. Controlled 3.0s Clean  |
| 5. Engineering Design Specs,      | 5. Hardware Safety Interlocking   |    Agent Discharge Burst  |
|    Safety Limits & Test Matrix    | 6. Quantitative Experimental      | 5. Extinguished Simulated |
|                                   |    Trials & Statistical Analysis  |    Inciting Flame Target  |
|                                   | 7. Expert Technical Evaluation    | 6. Quantitative Systems   |
|                                   |    with Live Web HUD Inspection   |    Performance Dataset    |
+-----------------------------------+-----------------------------------+---------------------------+
```

#### 4.2 Operational Framework (Operational Finite State Machine)

The operational behavior of the robot is governed by a seven-state finite state machine executed collaboratively between the Raspberry Pi and ESP32:

```
              [ POWER ON / INITIALIZE ]
                          |
                          v
                 +-----------------+
                 |  STATE 1: IDLE  | <------------------------------------+
                 +-----------------+                                      |
                          | (Auto-Start Patrol)                           |
                          v                                               |
             +-------------------------+                                  |
             |     STATE 2: PATROL     |                                  |
             +-------------------------+                                  |
               |                     ^                                    |
(Obstacle      |                     | (Path                              |
Detected)      v                     |  Clear)                            |
       +---------------+             |                                    |
       | STATE 3: AVOID|-------------+                                    |
       +---------------+                                                  |
               |                                                          |
               | (Optical Flame Detected >= 3/5 frames)                   |
               v                                                          |
       +-------------------------------+                                  |
       | STATE 4: TARGETING & APPROACH |                                  |
       +-------------------------------+                                  |
               |                                                          |
               | (Standoff <= 30cm & |e_x| <= 20px)                       |
               v                                                          |
       +-------------------------------+                                  |
       |   STATE 5: SENSOR VERIFY      |                                  |
       +-------------------------------+                                  |
               |                                                          |
               | (LM393 IR Confirmed == LOW)                              |
               v                                                          |
       +-------------------------------+                                  |
       |  STATE 6: ACTIVE SUPPRESSION  |                                  |
       |   (MG996R Actuated 3.0s Burst)|                                  |
       +-------------------------------+                                  |
               |                                                          |
               | (Watchdog Timeout: 3.0s elapsed)                         |
               v                                                          |
       +-------------------------------+                                  |
       | STATE 7: POST-SUPPRESSION EVAL|                                  |
       +-------------------------------+                                  |
               |                                                          |
               +----------------------------------------------------------+
                               (Return to IDLE / Standby)
```

The operational states function as follows:
- **State 1 (IDLE):** The system powers on, runs peripheral built-in self-tests, calibrates the IMU zero offsets, centers the tilt servo at 90°, and transitions to patrol.
- **State 2 (PATROL):** The robot drives forward at a baseline cruising speed (PWM duty cycle 45%). The Raspberry Pi continuously evaluates incoming camera frames for flame signatures, while the ESP32 samples the three ultrasonic sensors and applies TinyML steering smoothing.
- **State 3 (OBSTACLE AVOIDANCE):** If any ultrasonic sensor detects an obstacle within 25.0 cm, the robot halts, reads left and right clearances, rotates in place toward the open side, and resumes patrol once all front clearances exceed 35.0 cm. If trapped in a cul-de-sac, a 180-degree escape turn is executed.
- **State 4 (TARGETING & APPROACH):** Upon positive optical detection across three of five consecutive frames, the robot halts autonomous patrol. It computes horizontal tracking error $e_x$ and rotates to center the flame. Once centered within $\pm 20\text{ pixels}$, it drives forward, continuously monitoring the front ultrasonic sensor until reaching the safe standoff distance of $30.0\text{ cm} \pm 2.0\text{ cm}$.
- **State 5 (SENSOR VERIFY):** With the chassis halted at 30.0 cm, the robot adjusts the MG90S tilt servo to center the flame vertically and queries the LM393 infrared flame sensor. If the LM393 confirms infrared radiation ($D_0 = \text{LOW}$), the fire is verified. If the LM393 reports no infrared signal within 2.0 seconds, the target is rejected as an optical false alarm and the robot returns to patrol.
- **State 6 (ACTIVE SUPPRESSION):** The ESP32 energizes the TowerPro MG996R high-torque servo on GPIO 23, rotating the servo arm to 90 degrees to mechanically depress the extinguisher release valve. A hardware-level firmware watchdog enforces an exact 3.0-second discharge duration, returning the servo to 0 degrees (valve closed) upon expiration.
- **State 7 (POST-SUPPRESSION EVALUATION):** The robot holds position for 5.0 seconds to allow aerosol dispersion. It then re-evaluates both camera and infrared sensors. If the flame signature is extinguished, the trial is logged as successful and the robot returns to idle standby.

### 5. Prototype Development Procedure

The development of SAGIP-X is structured across six sequential engineering phases:

1. **Phase 1: Requirements Definition and Component Sizing:** Establish mechanical chassis payload requirements (supporting 2.5 kg total mass including extinguisher can and batteries), torque calculations for drive motors and servos, power budget calculations (determining battery capacity for 30 minutes continuous patrol), and sensor range specifications.
2. **Phase 2: Hardware Architecture and Mechanical CAD Fabrication:** Design laser-cut 4.0 mm acrylic chassis plates and mounting brackets in CAD. Assemble the 4WD chassis, mount TT gearmotors, install brass standoffs, and secure the extinguisher bracket and MG996R servo linkage.
3. **Phase 3: Electrical Wiring and Power Integration:** Fabricate the electrical wiring harness using AWG 20/24 silicone stranded wire. Install the master rocker switch, 5A inline fuse, XL4015 buck converter (adjusted to 5.15V), LM2596 buck converter (adjusted to 6.0V), 1kΩ/2kΩ ultrasonic level dividers, and star ground distribution terminal.
4. **Phase 4: Software-in-the-Loop (SIL) Digital Twin Simulation:** To verify the control state machine before testing with physical batteries and pressurized chemicals, the robot kinematics, ultrasonic raycasting, and 7-state finite state machine were simulated in an interactive 3D digital twin platform (`simulation/frontend/`) and evaluated using a physical URDF model in NVIDIA Isaac Sim. This SIL verification phase allowed the engineering team to stress-test collision boundaries and fail-safe transitions in software prior to physical flashing.
5. **Phase 5: Embedded Firmware and Vision Pipeline Implementation:** Develop ESP32 C++ firmware in PlatformIO/Arduino IDE implementing 20 kHz PWM motor control, TinyML inference (`navigation_model.h`), timer-based round-robin ultrasonic sampling with median filtering, and JSON serial parsing. Develop Python vision scripts on the Raspberry Pi utilizing OpenCV for HSV thresholding, TFLite for flame detection, and the Flask diagnostic web stream.
6. **Phase 6: Full System Integration and Experimental Field Testing:** Conduct repeated empirical trials in the mock residential testing arena, execute the experimental test matrix, log quantitative metrics, and administer the expert evaluation survey.

### 6. Data-Gathering Procedure

Data collection is executed systematically across five experimental protocols designed to address the specific research questions:

#### Protocol A: Fire Detection and Target Location Testing (Research Question 2)
1. Position the simulated candle fire at predetermined radial distances: 0.50 m, 1.00 m, 1.50 m, 2.00 m, and 2.50 m along the camera central optical axis ($0^\circ$), left offset ($+20^\circ$), and right offset ($-20^\circ$).
2. Test detection performance across the three environmental lighting profiles (Daylight, Low-Light, Smoke-Obscured), performing 10 trials per distance-angle-lighting permutation (total 450 detection trials).
3. Record whether positive detection occurred (binary True/False), the detection latency in milliseconds from flame presentation to serial state change, and the pixel error between detected bounding box centroid and true flame coordinates.

#### Protocol B: Navigation and Obstacle Avoidance Testing (Research Question 3)
1. Place the robot at the designated starting zone in the mock residential layout.
2. Introduce static domestic obstacles (cardboard boxes, small wastebaskets, and mock furniture items measuring 20 cm x 20 cm x 40 cm) along the corridor path, ensuring minimum corridor clearance of 0.90 m.
3. Command the robot to navigate through the corridor and transit the 0.80 m doorway opening toward the fire room.
4. Execute 30 complete navigation runs. Record successful traversals, physical collisions (if any), ultrasonic detection distances at avoidance initiation, and total traversal time.

#### Protocol C: System Response Latency Measurement (Research Question 4)
1. Synchronize the Raspberry Pi system clock with an external high-speed digital reference timer.
2. Place the robot in active patrol mode within the corridor.
3. Ignite the simulated fire source and record system timestamps at discrete milestone events:
   - $T_0$: Ignition / presentation of flame source;
   - $T_1$: Optical detection confirmed by Raspberry Pi (3-of-5 frame buffer filled);
   - $T_2$: Arrival at the 30.0 cm ultrasonic standoff boundary;
   - $T_3$: Secondary infrared verification confirmed by LM393 sensor;
   - $T_4$: Mechanical actuation of MG996R servo trigger;
   - $T_5$: Full flame knock-down confirmed.
4. Compute phase latencies: Detection Latency ($T_1 - T_0$), Navigation Latency ($T_2 - T_1$), Verification Latency ($T_3 - T_2$), and Actuation Latency ($T_4 - T_3$).

#### Protocol D: Aiming and Fire Suppression Efficacy (Research Question 5)
1. Position the robot in the targeting position facing the candle pan at the 30.0 cm standoff distance.
2. Measure the angular deviation between the extinguisher discharge nozzle axis and the geometric center of the candle cluster.
3. Command the ESP32 to trigger the MG996R servo, executing a timed 3.0-second discharge burst of extinguishing agent.
4. Observe and log flame suppression outcome: Success (all three candle flames fully extinguished within the 3.0-second burst without re-ignition within 10 seconds) or Failure (flame continues combusting).
5. Measure the mass of the extinguisher canister before and after discharge using a precision digital scale (0.1g resolution) to record agent consumption per burst.

#### Protocol E: Consistency Evaluation Across Repeated Trials (Research Question 6)
1. Conduct 30 complete, end-to-end autonomous trials under identical baseline conditions (Daylight illumination, fixed mock residential corridor layout, candle cluster target placed in mock room).
2. For each complete run, record overall trial outcome (Complete Success / Collision / False Alarm / Suppression Failure), total elapsed mission time from start to flame knock-down, final standoff distance, and residual agent mass.
3. Compute repeatability metrics, mean values, standard deviations, and coefficient of variation ($CV$).

#### Protocol F: Expert Technical Evaluation (Research Question 7)
1. Select a purposive sample of 15 evaluators comprising Computer Engineering faculty members (n=5), licensed Electronics Engineers / robotics practitioners (n=5), and certified fire safety officers / Bureau of Fire Protection personnel (n=5).
2. Conduct live, structured demonstration trials in the test arena showcasing autonomous patrol, obstacle avoidance, fire detection, and mechanical suppression.
3. Evaluators observe physical robot actions directly while simultaneously viewing internal decision states, camera bounding box confidence, and sensor readouts on the live developer web telemetry dashboard (`http://<pi-ip>:5000/`) displayed on a diagnostic laptop.
4. Administer a standardized evaluation instrument based on a 5-point Likert scale (5 = Excellent, 4 = Very Good, 3 = Good, 2 = Fair, 1 = Poor) across four evaluation dimensions:
   - 7.1 Functionality (system performs intended engineering tasks autonomously);
   - 7.2 Reliability (system operates consistently without crashes, stalls, or hardware errors);
   - 7.3 Response Time (system acts promptly in detecting hazard and deploying suppression);
   - 7.4 Safety (system operates without posing shock, runaway, or tipping hazards).

### 7. Data Analysis

Quantitative experimental data and evaluator assessment scores are analyzed using descriptive and inferential statistical methods:

1. **Percentage and Success Rates:** The accuracy of fire detection, obstacle avoidance rate, and suppression efficacy are computed as percentage proportions:
   $$P = \left( \frac{N_{success}}{N_{total}} \right) \times 100\%$$
   where $N_{success}$ represents the number of successful events and $N_{total}$ represents total trials conducted.

2. **Mean and Standard Deviation:** For continuous parametric variables (response latencies, ultrasonic standoff distances, aiming pixel errors, and agent discharge mass), the sample arithmetic mean ($\bar{x}$) and sample standard deviation ($s$) are calculated:
   $$\bar{x} = \frac{1}{n} \sum_{i=1}^{n} x_i, \quad s = \sqrt{\frac{1}{n-1} \sum_{i=1}^{n} (x_i - \bar{x})^2}$$

3. **Coefficient of Variation ($CV$):** To evaluate operational consistency across repeated trials (Research Question 6), the coefficient of variation is determined:
   $$CV = \left( \frac{s}{\bar{x}} \right) \times 100\%$$
   A $CV$ below 15% indicates high operational repeatability and consistency.

4. **Weighted Mean for Technical Evaluation:** Evaluator responses on the 5-point Likert scale (Research Question 7) are analyzed using the weighted mean ($\bar{X}_w$):
   $$\bar{X}_w = \frac{\sum (f \cdot w)}{N}$$
   where $f$ is the frequency of responses for a given scale weight $w \in \{1, 2, 3, 4, 5\}$, and $N$ is total evaluators ($N=15$). Calculated weighted means are interpreted using the verbal scale ranges:
   - 4.21 to 5.00: Excellent (Highly Acceptable)
   - 3.41 to 4.20: Very Good (Acceptable)
   - 2.61 to 3.40: Good (Moderately Acceptable)
   - 1.81 to 2.60: Fair (Low Acceptability)
   - 1.00 to 1.80: Poor (Unacceptable)

### 8. Operationalization of Research Questions

To ensure rigorous alignment between the statement of the problem, empirical methodology, and data analysis, Table 3 specifies the operationalization matrix for each research question.

#### Table 3. Operationalization Matrix of Research Questions
| Research Question | Key Variables | Metric / Unit | Data Collection Instrument | Experimental Procedure | Statistical / Data Analysis Method |
|---|---|---|---|---|---|
| **Question 1:** System Design & Assembly | Subsystem integration, physical wiring, mechanical fit, power stability | Qualitative completion, voltage rail regulation (V), current draw (A), firmware compile status | Digital Multimeter, USB Oscilloscope, CAD drawings, schematic checklist | Step-by-step mechanical assembly, power-up testing, pinout continuity check, voltage rail measurement | Descriptive verification, engineering BOM reconciliation, voltage tolerance compliance |
| **Question 2:** Fire Detection Accuracy | Optical detection rate, false alarm rate, target bounding coordinates | Percentage accuracy (%), false positive count, centroid pixel offset ($e_x, e_y$) | Raspberry Pi vision logs, USB webcam, calibrated test grid, digital lux meter | Protocol A: 450 static detection trials across 5 distances (0.5m-2.5m), 3 angles, and 3 lighting profiles | Success percentage ($P$), mean latency ($\bar{x}$), false detection rate |
| **Question 3:** Navigation & Obstacle Avoidance | Obstacle detection distance, corridor traversal success, collision occurrence | Avoidance clearance (cm), collision count, corridor transit time (s) | 3x HC-SR04 ultrasonic sensors, serial event logs, stop-motion video recording | Protocol B: 30 autonomous navigation runs in 5.0m x 5.0m arena with static domestic obstacles | Avoidance success rate (%), mean clearance distance, collision frequency |
| **Question 4:** Operational Response Time | Detection time, navigation time, verification time, trigger actuation time | Latency in seconds (s) and milliseconds (ms) across discrete phases | Microcontroller hardware timestamps, system event logs, high-speed camera | Protocol C: Time-synchronized trial logging from flame ignition ($T_0$) to trigger actuation ($T_4$) | Mean latency ($\bar{x}$) per phase, standard deviation ($s$), cumulative response time |
| **Question 5:** Targeting & Fire Suppression Efficacy | Nozzle aiming accuracy, flame knock-down rate, agent mass consumed | Knock-down success rate (%), nozzle angular error (degrees), mass consumed (g) | Precision digital balance (0.1g), visual flame observation, digital angle gauge | Protocol D: 30 suppression trials at 30.0 cm standoff with 3.0s timed servo burst on candle pan | Suppression success rate (%), mean agent discharge mass ($\bar{x}$), aiming angular error |
| **Question 6:** Consistency Across Repeated Trials | Multi-trial repeatability of detection, navigation, standoff, and suppression | Coefficient of variation ($CV$), variance, standard deviation ($s$) | Automated system data logger recording 30 end-to-end trial runs | Protocol E: 30 identical end-to-end test runs executing complete mission from start to suppression | Mean ($\bar{x}$), standard deviation ($s$), coefficient of variation ($CV$) across all 30 trials |
| **Question 7:** Evaluator Ratings (Functionality, Reliability, Response Time, Safety) | Expert assessment of system engineering performance | 5-point Likert scale ratings (1.00 to 5.00) across 4 dimensions | Structured evaluator survey checklist administered to 15 purposive experts with live web HUD telemetry inspection | Protocol F: Live demonstration trials followed by survey administration to CpE faculty, ECEs, and BFP officers | Weighted mean ($\bar{X}_w$), standard deviation, verbal interpretation scale |

### 9. Ethical and Safety Considerations

Because this research involves physical prototype testing with energized electrical components, pressurized chemical canisters, and open flame sources, strict ethical and safety protocols are maintained throughout:

1. **Fire Prevention and Thermal Safety:** All live fire testing is conducted exclusively within a non-combustible testing enclosure equipped with flame-retardant floor barriers. A certified 2.5 kg ABC dry chemical fire extinguisher and a commercial fire blanket are stationed immediately adjacent to the arena. An emergency manual kill switch on the robot chassis disconnects all battery power instantly.
2. **Pressurized Canister Handling:** The portable fire extinguisher canister is mounted in a rigid mechanical collar designed to prevent accidental discharge, mechanical puncture, or physical slippage. All discharges use certified non-toxic clean agent canisters. Personnel in the testing facility wear protective safety goggles and dust masks to prevent respiratory irritation from aerosol discharge.
3. **Electrical and Battery Isolation:** The 11.1V 3S LiPo battery is housed in an insulated battery enclosure and protected by a 5A fast-blow inline fuse. During charging, the LiPo battery is monitored in a fireproof LiPo safety bag using a balanced microprocessor charger. Step-down buck converters prevent overvoltage to digital processing logic.
4. **Evaluator Ethics and Informed Consent:** Prior to participating in the evaluation survey, all fifteen evaluators are informed of the study objectives, operating principles, and safety boundaries. Participation is completely voluntary, and evaluator responses are collected anonymously and kept strictly confidential.

---

# DEFINITION OF TERMS

The following operational and technical terms are defined in the context of this study:

- **Autonomous Mobile Robot (AMR):** Refers to a mobile robotic platform capable of navigating through an environment and executing tasks independently without continuous direct human teleoperation, utilizing onboard sensors, processing units, and actuators (ISO 8373:2021).
- **Fire Detection:** Refers to the algorithmic and sensory process of recognizing and confirming the physical presence of a flame hazard within the environment using optical camera frame analysis and infrared radiation sensing (ISO 7240-1:2025).
- **Fire Location:** Refers to the spatial determination of the coordinates, heading angle, and radial distance of a confirmed fire hazard relative to the robot chassis coordinate frame.
- **Camera:** An optical image capture device (USB UVC webcam) mounted to the robot chassis that captures 640x360 video frames at 30 FPS to supply visual data for color-space segmentation and bounding-box flame identification.
- **Flame Sensor:** An electronic transducer (LM393 optical module with photodiode) sensitive to electromagnetic radiation within the near-infrared spectrum (760 nm to 1100 nm), used as a secondary physical interlock to verify combustion.
- **Controller:** The embedded digital processing unit (specifically the decoupled pairing of a Raspberry Pi 3 Model B single-board computer and an ESP32-WROOM-32 microcontroller) that processes sensory inputs and commands actuator outputs.
- **Navigation:** The coordinated process of planning, directing, and executing robot chassis movement through a residential environment from an initial start point to a target destination.
- **Obstacle Avoidance:** The real-time reactive capability of the mobile robot to detect physical barriers, walls, or household objects using ultrasonic distance sensors and steer away to prevent collisions.
- **Standoff Distance:** The predetermined physical separation distance ($30.0\text{ cm} \pm 2.0\text{ cm}$) maintained between the front of the robot chassis and the fire target during suppression to prevent thermal damage while maximizing extinguishing efficiency.
- **Fire Suppression Mechanism:** The mechanical and chemical apparatus mounted on the robot, consisting of a pressurized portable fire extinguisher canister, mounting bracket, and high-torque MG996R servo motor that depresses the canister valve to discharge extinguishing agent.
- **TinyML:** Ultra-low-power machine learning models designed to execute real-time inference locally on resource-constrained embedded microcontrollers without internet connectivity.
- **Digital Twin:** A virtual software model and 3D simulation environment of the physical robot and testing arena used for pre-deployment verification of sensor raycasting, kinematics, and control logic.

---

# REFERENCES

1. Arduino. (n.d.). *Introduction to Arduino*. Arduino Documentation. https://docs.arduino.cc/
2. Chenchireddy, K., et al. (2025). Design and development of an autonomous firefighting mobile robot using Arduino microcontroller and multi-sensor fusion. *Journal of Robotics and Control Systems*, 11(1), 45-58.
3. Honeywell Analytics. (2016). *FSL100 series optical flame detectors: Technical handbook*. Honeywell International Inc.
4. International Organization for Standardization. (2021). *Robotics: Vocabulary* (ISO Standard No. 8373:2021). https://www.iso.org/standard/75539.html
5. International Organization for Standardization. (2021). *Information technology: Augmented reality (AR) and virtual reality (VR): Reference model* (ISO/IEC Standard No. 18038:2021). https://www.iso.org/standard/78967.html
6. International Organization for Standardization. (2025). *Fire detection and alarm systems: Part 1: General and definitions* (ISO Standard No. 7240-1:2025). https://www.iso.org/standard/83402.html
7. Jindal, K., Wang, A., Thakur, D., Zhou, A., Spurny, V., Walter, V., Broughton, G., Krajnik, T., Saska, M., & Loianno, G. (2021). Design and deployment of an autonomous unmanned ground vehicle for urban firefighting scenarios. *arXiv preprint*, arXiv:2107.03582. https://doi.org/10.48550/arXiv.2107.03582
8. New York City Fire Department. (2024). *Study material for the certificate of fitness: Commercial pre-engineered dry chemical fire extinguishing systems inspection, testing and servicing technician (S-18) and principal (S-81)*. NYC Fire Department (FDNY).
9. Nguyen, T. (2024). Autonomous mobile firefighting robot utilizing machine vision and programmable logic control for obstacle-dense indoor environments. *International Journal of Advanced Mechatronic Systems*, 16(2), 112-125.
10. Rubio, F., Valero, F., & Llopis-Albert, C. (2019). A review of mobile robots: Concepts, methods, theoretical framework, and applications. *International Journal of Advanced Robotic Systems*, 16(2), 1-22. https://doi.org/10.1177/1729881419839596
11. Sen, L., et al. (2023). An indoor autonomous inspection and firefighting robot based on SLAM and flame image recognition. *Fire*, 6(3), 93. https://doi.org/10.3390/fire6030093
12. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to autonomous mobile robots* (2nd ed.). MIT Press.
13. U.S. Fire Administration. (2024). *Residential building fire trends (2014-2023)*. Federal Emergency Management Agency (FEMA). https://www.usfa.fema.gov
