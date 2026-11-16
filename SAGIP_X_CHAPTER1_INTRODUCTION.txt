# CHAPTER 1: INTRODUCTION

## 1. Background of the Study

Residential fires are a serious safety concern because they can cause injuries, loss of life, and property damage. In residential environments, fire incidents may occur in areas such as kitchens, living rooms, and other household spaces, where furniture and other objects may obstruct movement. These conditions highlight the importance of early fire detection and timely response. According to the U.S. Fire Administration (2024), fires in modern homes can develop rapidly due to factors such as synthetic materials used in furniture and interior finishes. Although conventional fire alarms and fire extinguishers are essential safety devices, their operation may still require a person to recognize the danger and respond appropriately. In many household emergencies, occupants may be asleep, absent, physically limited, or rapidly overwhelmed by smoke inhalation before they can intervene. Manual fire suppression using portable extinguishers requires an occupant to deliberately approach the flame base, exposing them to intense radiant heat, toxic carbon monoxide, and burn hazards. Municipal fire services provide vital structural defense, but transit delays and traffic conditions frequently prevent arrival during the critical incipient phase of combustion. An autonomous mobile firefighting robot may help address this concern by detecting a fire, navigating toward its location, avoiding obstacles, and activating a fire-suppression mechanism according to its programmed control system.

Previous studies have demonstrated the development of mobile firefighting robots with different approaches to fire detection, navigation, obstacle avoidance, and fire suppression. Nguyen (2024) developed an autonomous mobile robot for firefighting that uses machine vision and a programmable logic controller to approach a fire while avoiding obstacles. Similarly, Chenchireddy et al. (2025) presented an Arduino-based firefighting robot that uses fire and infrared sensors, motors, and a water-pumping mechanism to respond to detected fires. These studies demonstrate existing approaches to automating firefighting tasks. However, the performance of a robot depends on its hardware configuration, control system, and testing conditions. Therefore, the performance of previously developed systems cannot automatically be assumed for a different implementation. Early educational platforms that utilize single 8-bit microcontrollers and submerged water pumps present practical limitations in residential spaces. Liquid water spray causes electrical shock risks when sprayed near energized domestic appliances and wiring, can cause violent flare-ups when encountering burning kitchen cooking oil, and produces extensive secondary water damage to interior furnishings. Furthermore, single-microcontroller architectures experience severe processing bottlenecks when attempting to handle real-time computer vision while simultaneously managing high-frequency motor control and sensor polling. Industrial automated systems utilizing heavy programmable logic controllers are often too bulky, complex, and cost-prohibitive for domestic residential hallways. This provides a clear technical basis for evaluating how a specific, compact, dual-tier robot configuration performs under clearly defined and controlled residential-like conditions.

In response to this concern, the proposed study will develop SAGIP-X: Design and Implementation of an Autonomous Mobile Firefighting Robot for Residential Environments. The robot is intended to detect and locate a designated simulated fire source, navigate toward it while avoiding obstacles, and activate its fire-suppression mechanism in a controlled indoor residential-like environment. The platform employs a decoupled dual-tier architecture: a Raspberry Pi 3 Model B handles visual flame detection and decision logic, while an ESP32-WROOM-32 microcontroller manages real-time motor control, ultrasonic distance measurement, and servo actuation. Rather than using water, the prototype incorporates a commercial portable fire extinguisher canister triggered by an onboard high-torque servo motor. The study will evaluate the prototype's fire-detection and target-location performance, navigation and obstacle avoidance, response time, and fire-suppression performance through predefined procedures and repeated trials. Its functionality and safety will also be assessed using established testing criteria and a structured evaluation checklist. Rather than assuming that the robot will be more affordable or effective than existing systems, the study will document its measured performance and limitations. The findings will apply only to the developed prototype and the specific experimental conditions, and the system will not be considered a replacement for certified fire-safety equipment or professional firefighters.

## 2. Statement of the Problem

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

## 3. Objectives of the Study

### 3.1 General Objective
The general objective of this study is to design, develop, and evaluate SAGIP-X, an autonomous mobile firefighting robot designed to detect and locate a fire, navigate toward the target while avoiding obstacles, and activate a fire-suppression mechanism in a controlled residential environment.

### 3.2 Specific Objectives
Specifically, the study aims to:

1. Design and develop the SAGIP-X prototype by integrating the camera, flame sensor, controller, motors, distance sensors, and fire-extinguisher system based on the approved system design.
2. Evaluate the accuracy of the robot in detecting and locating a simulated fire through controlled testing under varied environmental lighting conditions.
3. Assess the navigation performance of the robot in moving toward the fire target while avoiding obstacles in a controlled residential-like environment.
4. Measure the response time of the robot from fire detection to the activation of its fire-suppression mechanism across individual functional phases.
5. Evaluate the targeting and fire-suppression performance of the robot by determining its ability to aim the extinguisher toward the simulated fire and suppress it during controlled tests.
6. Determine the consistency of the robot's performance in fire detection, target location, obstacle avoidance, response time, and fire suppression through repeated trials.
7. Evaluate the prototype's overall performance based on functionality, reliability, response time, and safety using the assessments of selected evaluators.

## 4. Scope and Limitations

### 4.1 Scope of the Study
The main aim of this project is to design and develop an autonomous mobile robot that can detect and extinguish small fires within a domestic environment. This includes joining together all hardware and software components, such as the camera, flame sensor, controller, motors, motor drivers, distance sensors, and fire extinguisher.

The robot is designed to autonomously detect a fire, locate it, navigate toward the target while avoiding obstacles, and aim and activate its fire-suppression system. The physical platform features a four-wheel drive (4WD) differential skid-steer chassis, powered by four geared DC TT motors driven by a TB6612FNG dual H-bridge motor driver. Computation is partitioned across a decoupled dual-tier embedded architecture: a Raspberry Pi 3 Model B executes the computer vision pipeline using an attached USB webcam, while an ESP32 microcontroller manages hard real-time tasks, including ultrasonic distance acquisition, sensor interlocks, motor driving, and servo positioning. Obstacle avoidance is achieved reactively using three HC-SR04 ultrasonic sensors mounted at the front, left, and right of the chassis, augmented by an embedded TinyML neural steering policy and a short-term local 2D proximity memory to escape corridor cul-de-sacs. Target alignment is performed through an articulated single-axis tilt turret driven by a TowerPro MG90S micro servo, combined with differential rotation of the chassis for azimuth steering. Fire suppression is accomplished using a commercial portable fire extinguisher can actuated by a dedicated TowerPro MG996R metal-gear high-torque servo motor.

To facilitate developer diagnostics, software calibration, and live evaluator inspection during demonstration trials, the platform includes a local web-based telemetry streaming dashboard hosted directly on the Raspberry Pi. This dashboard provides a live visual window displaying optical camera frames, detection bounding boxes, confidence metrics, and sensor readouts over local Wi-Fi. The robot operates with 100% offline autonomy; local Wi-Fi telemetry is purely an observational instrument and is not required for autonomous fire detection, navigation, or suppression.

The scope of the study is restricted to controlled testing in a residential-style test environment with a simulated fire. Performance is evaluated in terms of fire-detection and target-location accuracy, navigation and obstacle avoidance, response time, targeting and fire suppression, and consistency across repeated trials, together with evaluator ratings of functionality, reliability, response time, and safety.

### 4.2 Limitations of the Study
The study is limited by the following technical and operational constraints:

1. The robot is tested using controlled, simulated fires only (paraffin wax candle clusters generating a flame height between 5.0 cm and 8.0 cm). It is not tested against actual structural fires, open liquid fuels, or violent combustible materials, and its performance in real fire emergencies is not established by this study.
2. Testing is conducted in a controlled, residential-like environment with a defined layout (5.0 m by 5.0 m test area with corridor widths between 0.90 m and 1.20 m). Conditions such as extreme smoke density, structural collapse, extreme convective heat, changing room layouts, stairs, debris, and heavily cluttered spaces found in actual residential fires are not fully represented.
3. The robot is limited to indoor operation on flat, level surfaces (such as ceramic tile, polished vinyl, and finished concrete). It is not designed to climb stairs, cross floor sills taller than 10 mm, traverse elevated areas, or operate outdoors.
4. Fire detection depends on the camera and flame sensor under the test conditions used. Detection accuracy may decrease under conditions such as low lighting, optical obstructions blocking the sensor line of sight, or fire sources outside the sensor effective range (exceeding 2.5 meters).
5. Obstacle avoidance depends strictly on the three ultrasonic distance sensors and may not detect obstacles with sound-absorbing textile surfaces, narrow leg posts under 20 mm, or drop-offs such as downward stairwells that fall outside their conical beam field.
6. The robot does not utilize wheel encoders, optical odometry, or LiDAR, and does not perform Simultaneous Localization and Mapping (SLAM). While a local 2D ultrasonic spatial proximity buffer is maintained in volatile memory for cul-de-sac recovery, the system does not construct or store global metric room maps.
7. The fire-suppression mechanism is limited by the capacity and type of extinguishing agent carried by the robot. The onboard portable fire extinguisher canister contains a finite volume of extinguishing agent suitable for short bursts (up to 3.0 seconds), restricting suppression capability to small incipient fires within a 30.0 cm standoff distance.
8. The prototype is designed for initial fire detection and suppression support only. It does not replace certified firefighting equipment, emergency responders, certified fire alarm installations, or building fire-protection systems, and it does not perform human search and rescue operations.
