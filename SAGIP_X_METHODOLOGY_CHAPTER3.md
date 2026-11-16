# CHAPTER 3: METHODOLOGY

## 1. Research Design

This study employs an experimental engineering research design combined with quantitative performance evaluation. The primary goal is the design, fabrication, and physical empirical testing of the SAGIP-X autonomous mobile firefighting robot prototype.

The investigation follows an experimental input-process-output paradigm. Controlled independent variables (such as ambient lighting conditions, obstacle configurations, and target distance) are manipulated in a standardized test arena to observe and measure dependent performance variables (fire detection accuracy, target localization error, obstacle avoidance success rate, operational response latency across phases, and flame suppression success rate). Repeated empirical trials are conducted across all experimental phases to obtain statistically robust datasets. In addition to quantitative instrument measurements, the prototype is evaluated by a panel of engineering faculty members, electronics engineers, and fire safety practitioners using a structured evaluation checklist to assess functionality, reliability, response time, and operational safety.

## 2. Research Setting and Test Conditions

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

## 3. System Components and Functions

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

## 4. Conceptual and Operational Frameworks

### 4.1 Conceptual Framework (Input - Process - Output Model)

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

### 4.2 Operational Framework (Operational Finite State Machine)

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

## 5. Prototype Development Procedure

The development of SAGIP-X is structured across six sequential engineering phases:

1. **Phase 1: Requirements Definition and Component Sizing:** Establish mechanical chassis payload requirements (supporting 2.5 kg total mass including extinguisher can and batteries), torque calculations for drive motors and servos, power budget calculations (determining battery capacity for 30 minutes continuous patrol), and sensor range specifications.
2. **Phase 2: Hardware Architecture and Mechanical CAD Fabrication:** Design laser-cut 4.0 mm acrylic chassis plates and mounting brackets in CAD. Assemble the 4WD chassis, mount TT gearmotors, install brass standoffs, and secure the extinguisher bracket and MG996R servo linkage.
3. **Phase 3: Electrical Wiring and Power Integration:** Fabricate the electrical wiring harness using AWG 20/24 silicone stranded wire. Install the master rocker switch, 5A inline fuse, XL4015 buck converter (adjusted to 5.15V), LM2596 buck converter (adjusted to 6.0V), 1kΩ/2kΩ ultrasonic level dividers, and star ground distribution terminal.
4. **Phase 4: Software-in-the-Loop (SIL) Digital Twin Simulation:** To verify the control state machine before testing with physical batteries and pressurized chemicals, the robot kinematics, ultrasonic raycasting, and 7-state finite state machine were simulated in an interactive 3D digital twin platform (`simulation/frontend/`) and evaluated using a physical URDF model in NVIDIA Isaac Sim. This SIL verification phase allowed the engineering team to stress-test collision boundaries and fail-safe transitions in software prior to physical flashing.
5. **Phase 5: Embedded Firmware and Vision Pipeline Implementation:** Develop ESP32 C++ firmware in PlatformIO/Arduino IDE implementing 20 kHz PWM motor control, TinyML inference (`navigation_model.h`), timer-based round-robin ultrasonic sampling with median filtering, and JSON serial parsing. Develop Python vision scripts on the Raspberry Pi utilizing OpenCV for HSV thresholding, TFLite for flame detection, and the Flask diagnostic web stream.
6. **Phase 6: Full System Integration and Experimental Field Testing:** Conduct repeated empirical trials in the mock residential testing arena, execute the experimental test matrix, log quantitative metrics, and administer the expert evaluation survey.

## 6. Data-Gathering Procedure

Data collection is executed systematically across five experimental protocols designed to address the specific research questions:

### Protocol A: Fire Detection and Target Location Testing (Research Question 2)
1. Position the simulated candle fire at predetermined radial distances: 0.50 m, 1.00 m, 1.50 m, 2.00 m, and 2.50 m along the camera central optical axis ($0^\circ$), left offset ($+20^\circ$), and right offset ($-20^\circ$).
2. Test detection performance across the three environmental lighting profiles (Daylight, Low-Light, Smoke-Obscured), performing 10 trials per distance-angle-lighting permutation (total 450 detection trials).
3. Record whether positive detection occurred (binary True/False), the detection latency in milliseconds from flame presentation to serial state change, and the pixel error between detected bounding box centroid and true flame coordinates.

### Protocol B: Navigation and Obstacle Avoidance Testing (Research Question 3)
1. Place the robot at the designated starting zone in the mock residential layout.
2. Introduce static domestic obstacles (cardboard boxes, small wastebaskets, and mock furniture items measuring 20 cm x 20 cm x 40 cm) along the corridor path, ensuring minimum corridor clearance of 0.90 m.
3. Command the robot to navigate through the corridor and transit the 0.80 m doorway opening toward the fire room.
4. Execute 30 complete navigation runs. Record successful traversals, physical collisions (if any), ultrasonic detection distances at avoidance initiation, and total traversal time.

### Protocol C: System Response Latency Measurement (Research Question 4)
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

### Protocol D: Aiming and Fire Suppression Efficacy (Research Question 5)
1. Position the robot in the targeting position facing the candle pan at the 30.0 cm standoff distance.
2. Measure the angular deviation between the extinguisher discharge nozzle axis and the geometric center of the candle cluster.
3. Command the ESP32 to trigger the MG996R servo, executing a timed 3.0-second discharge burst of extinguishing agent.
4. Observe and log flame suppression outcome: Success (all three candle flames fully extinguished within the 3.0-second burst without re-ignition within 10 seconds) or Failure (flame continues combusting).
5. Measure the mass of the extinguisher canister before and after discharge using a precision digital scale (0.1g resolution) to record agent consumption per burst.

### Protocol E: Consistency Evaluation Across Repeated Trials (Research Question 6)
1. Conduct 30 complete, end-to-end autonomous trials under identical baseline conditions (Daylight illumination, fixed mock residential corridor layout, candle cluster target placed in mock room).
2. For each complete run, record overall trial outcome (Complete Success / Collision / False Alarm / Suppression Failure), total elapsed mission time from start to flame knock-down, final standoff distance, and residual agent mass.
3. Compute repeatability metrics, mean values, standard deviations, and coefficient of variation ($CV$).

### Protocol F: Expert Technical Evaluation (Research Question 7)
1. Select a purposive sample of 15 evaluators comprising Computer Engineering faculty members (n=5), licensed Electronics Engineers / robotics practitioners (n=5), and certified fire safety officers / Bureau of Fire Protection personnel (n=5).
2. Conduct live, structured demonstration trials in the test arena showcasing autonomous patrol, obstacle avoidance, fire detection, and mechanical suppression.
3. Evaluators observe physical robot actions directly while simultaneously viewing internal decision states, camera bounding box confidence, and sensor readouts on the live developer web telemetry dashboard (`http://<pi-ip>:5000/`) displayed on a diagnostic laptop.
4. Administer a standardized evaluation instrument based on a 5-point Likert scale (5 = Excellent, 4 = Very Good, 3 = Good, 2 = Fair, 1 = Poor) across four evaluation dimensions:
   - 7.1 Functionality (system performs intended engineering tasks autonomously);
   - 7.2 Reliability (system operates consistently without crashes, stalls, or hardware errors);
   - 7.3 Response Time (system acts promptly in detecting hazard and deploying suppression);
   - 7.4 Safety (system operates without posing shock, runaway, or tipping hazards).

## 7. Data Analysis

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

## 8. Operationalization of Research Questions

To ensure rigorous alignment between the statement of the problem, empirical methodology, and data analysis, Table 1 specifies the operationalization matrix for each research question.

### Table 1. Operationalization Matrix of Research Questions
| Research Question | Key Variables | Metric / Unit | Data Collection Instrument | Experimental Procedure | Statistical / Data Analysis Method |
|---|---|---|---|---|---|
| **Question 1:** System Design & Assembly | Subsystem integration, physical wiring, mechanical fit, power stability | Qualitative completion, voltage rail regulation (V), current draw (A), firmware compile status | Digital Multimeter, USB Oscilloscope, CAD drawings, schematic checklist | Step-by-step mechanical assembly, power-up testing, pinout continuity check, voltage rail measurement | Descriptive verification, engineering BOM reconciliation, voltage tolerance compliance |
| **Question 2:** Fire Detection Accuracy | Optical detection rate, false alarm rate, target bounding coordinates | Percentage accuracy (%), false positive count, centroid pixel offset ($e_x, e_y$) | Raspberry Pi vision logs, USB webcam, calibrated test grid, digital lux meter | Protocol A: 450 static detection trials across 5 distances (0.5m-2.5m), 3 angles, and 3 lighting profiles | Success percentage ($P$), mean latency ($\bar{x}$), false detection rate |
| **Question 3:** Navigation & Obstacle Avoidance | Obstacle detection distance, corridor traversal success, collision occurrence | Avoidance clearance (cm), collision count, corridor transit time (s) | 3x HC-SR04 ultrasonic sensors, serial event logs, stop-motion video recording | Protocol B: 30 autonomous navigation runs in 5.0m x 5.0m arena with static domestic obstacles | Avoidance success rate (%), mean clearance distance, collision frequency |
| **Question 4:** Operational Response Time | Detection time, navigation time, verification time, trigger actuation time | Latency in seconds (s) and milliseconds (ms) across discrete phases | Microcontroller hardware timestamps, system event logs, high-speed camera | Protocol C: Time-synchronized trial logging from flame ignition ($T_0$) to trigger actuation ($T_4$) | Mean latency ($\bar{x}$) per phase, standard deviation ($s$), cumulative response time |
| **Question 5:** Targeting & Fire Suppression Efficacy | Nozzle aiming accuracy, flame knock-down rate, agent mass consumed | Knock-down success rate (%), nozzle angular error (degrees), mass consumed (g) | Precision digital balance (0.1g), visual flame observation, digital angle gauge | Protocol D: 30 suppression trials at 30.0 cm standoff with 3.0s timed servo burst on candle pan | Suppression success rate (%), mean agent discharge mass ($\bar{x}$), aiming angular error |
| **Question 6:** Consistency Across Repeated Trials | Multi-trial repeatability of detection, navigation, standoff, and suppression | Coefficient of variation ($CV$), variance, standard deviation ($s$) | Automated system data logger recording 30 end-to-end trial runs | Protocol E: 30 identical end-to-end test runs executing complete mission from start to suppression | Mean ($\bar{x}$), standard deviation ($s$), coefficient of variation ($CV$) across all 30 trials |
| **Question 7:** Evaluator Ratings (Functionality, Reliability, Response Time, Safety) | Expert assessment of system engineering performance | 5-point Likert scale ratings (1.00 to 5.00) across 4 dimensions | Structured evaluator survey checklist administered to 15 purposive experts with live web HUD telemetry inspection | Protocol F: Live demonstration trials followed by survey administration to CpE faculty, ECEs, and BFP officers | Weighted mean ($\bar{X}_w$), standard deviation, verbal interpretation scale |

## 9. Ethical and Safety Considerations

Because this research involves physical prototype testing with energized electrical components, pressurized chemical canisters, and open flame sources, strict ethical and safety protocols are maintained throughout:

1. **Fire Prevention and Thermal Safety:** All live fire testing is conducted exclusively within a non-combustible testing enclosure equipped with flame-retardant floor barriers. A certified 2.5 kg ABC dry chemical fire extinguisher and a commercial fire blanket are stationed immediately adjacent to the arena. An emergency manual kill switch on the robot chassis disconnects all battery power instantly.
2. **Pressurized Canister Handling:** The portable fire extinguisher canister is mounted in a rigid mechanical collar designed to prevent accidental discharge, mechanical puncture, or physical slippage. All discharges use certified non-toxic clean agent canisters. Personnel in the testing facility wear protective safety goggles and dust masks to prevent respiratory irritation from aerosol discharge.
3. **Electrical and Battery Isolation:** The 11.1V 3S LiPo battery is housed in an insulated battery enclosure and protected by a 5A fast-blow inline fuse. During charging, the LiPo battery is monitored in a fireproof LiPo safety bag using a balanced microprocessor charger. Step-down buck converters prevent overvoltage to digital processing logic.
4. **Evaluator Ethics and Informed Consent:** Prior to participating in the evaluation survey, all fifteen evaluators are informed of the study objectives, operating principles, and safety boundaries. Participation is completely voluntary, and evaluator responses are collected anonymously and kept strictly confidential.
