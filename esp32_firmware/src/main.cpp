#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "motor_controller.h"
#include "servo_controller.h"
#include "sensor_manager.h"
#include "telemetry_parser.h"
#include "telemetry_reporter.h"

// Subsystem instances
static MotorController motorCtrl;
static ServoController servoCtrl;
static SensorManager sensorMgr;
static TelemetryParser parser;
static TelemetryReporter reporter;

// State machine tracker
static RobotState currentState = STATE_INIT;
static uint32_t stateEnterMs = 0;
static uint32_t lastRoamTurnMs = 0;
static bool roamingTurnLeft = true;

void transitionTo(RobotState newState) {
    if (currentState != newState) {
        Serial.printf("[FSM] State Change: %s -> %s\n",
                      robotStateToString(currentState),
                      robotStateToString(newState));
        currentState = newState;
        stateEnterMs = millis();
    }
}

// -----------------------------------------------------------------------------
// Interactive Serial Monitor Test & Diagnostics Engine
// -----------------------------------------------------------------------------
static bool streamSensorsActive = false;
static uint32_t lastSensorStreamMs = 0;
static String serialInputBuffer = "";

void printTestMenu() {
    Serial.println("\n=======================================================");
    Serial.println("         BANTAY-SUNOG SERIAL TEST & DIAGNOSTICS        ");
    Serial.println("=======================================================");
    Serial.println(" [MOTORS & DRIVER CHECKS]");
    Serial.println("   check  : Run FULL TB6612FNG Driver Diagnostic Check (AO1/AO2 & BO1/BO2)");
    Serial.println("   pins   : Test raw ESP32 GPIO blink (PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY)");
    Serial.println("   w      : Drive Forward (1.0 sec test)");
    Serial.println("   s      : Drive Reverse (1.0 sec test)");
    Serial.println("   a      : Pivot Left (0.8 sec test)");
    Serial.println("   d      : Pivot Right (0.8 sec test)");
    Serial.println("   x      : Emergency STOP motors");
    Serial.println(" [TILT SERVO]");
    Serial.println("   u      : Tilt UP (+10 degrees)");
    Serial.println("   j      : Tilt DOWN (-10 degrees)");
    Serial.println("   h      : Tilt Level / HOME (90 degrees)");
    Serial.println("   sweep  : Test smooth servo sweep (45 to 135 deg)");
    Serial.println(" [SUPPRESSION]");
    Serial.println("   f      : Fire Solenoid Pulse (1.0 sec spray)");
    Serial.println(" [SENSORS]");
    Serial.println("   r      : Snapshot read of all sensors");
    Serial.println("   t      : Toggle continuous live sensor stream (250ms)");
    Serial.println(" [SIMULATE VISION PACKETS (BENCH TEST)]");
    Serial.println("   sim    : Inject FIRE_CONFIRMED centered (x=0.50, y=0.50)");
    Serial.println("   simL   : Inject FIRE_CONFIRMED on LEFT  (x=0.20, y=0.50)");
    Serial.println("   simR   : Inject FIRE_CONFIRMED on RIGHT (x=0.80, y=0.50)");
    Serial.println("   simU   : Inject FIRE_CONFIRMED ELEVATED (x=0.50, y=0.20)");
    Serial.println("   simD   : Inject FIRE_CONFIRMED LOW      (x=0.50, y=0.80)");
    Serial.println("   clear  : Clear vision target (inject NO_FIRE)");
    Serial.println(" [OPERATING MODE]");
    Serial.println("   test   : Enter Manual Bench Test Mode (pauses autonomous roam)");
    Serial.println("   auto   : Resume Autonomous Firefighting Operation");
    Serial.println("   help/m : Show this diagnostic menu");
    Serial.println("=======================================================\n");
}

void handleSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            serialInputBuffer.trim();
            if (serialInputBuffer.length() > 0) {
                String cmd = serialInputBuffer;
                serialInputBuffer = "";

                if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("m") || cmd.equals("?")) {
                    printTestMenu();
                } else if (cmd.equalsIgnoreCase("check") || cmd.equalsIgnoreCase("c")) {
                    transitionTo(STATE_MANUAL_TEST);
                    motorCtrl.runDriverCheck();
                } else if (cmd.equalsIgnoreCase("pins") || cmd.equalsIgnoreCase("p")) {
                    transitionTo(STATE_MANUAL_TEST);
                    motorCtrl.testDirectGpios();
                } else if (cmd.equalsIgnoreCase("w")) {
                    transitionTo(STATE_MANUAL_TEST);
                    Serial.println("[TEST] Driving FORWARD for 1 second...");
                    motorCtrl.forward(SPEED_APPROACH_BASE);
                    delay(1000);
                    motorCtrl.stop();
                    Serial.println("[TEST] Motors stopped.");
                } else if (cmd.equalsIgnoreCase("s")) {
                    transitionTo(STATE_MANUAL_TEST);
                    Serial.println("[TEST] Driving REVERSE for 1 second...");
                    motorCtrl.reverse(SPEED_APPROACH_BASE);
                    delay(1000);
                    motorCtrl.stop();
                    Serial.println("[TEST] Motors stopped.");
                } else if (cmd.equalsIgnoreCase("a")) {
                    transitionTo(STATE_MANUAL_TEST);
                    Serial.println("[TEST] Pivoting LEFT for 0.8 second...");
                    motorCtrl.pivotLeft(SPEED_PIVOT_ALIGN);
                    delay(800);
                    motorCtrl.stop();
                    Serial.println("[TEST] Motors stopped.");
                } else if (cmd.equalsIgnoreCase("d")) {
                    transitionTo(STATE_MANUAL_TEST);
                    Serial.println("[TEST] Pivoting RIGHT for 0.8 second...");
                    motorCtrl.pivotRight(SPEED_PIVOT_ALIGN);
                    delay(800);
                    motorCtrl.stop();
                    Serial.println("[TEST] Motors stopped.");
                } else if (cmd.equalsIgnoreCase("x")) {
                    transitionTo(STATE_MANUAL_TEST);
                    motorCtrl.stop();
                    sensorMgr.deactivateSolenoid();
                    Serial.println("[TEST] EMERGENCY STOP: All actuators halted.");
                } else if (cmd.equalsIgnoreCase("u")) {
                    int newAngle = servoCtrl.getAngle() + 10;
                    servoCtrl.setAngle(newAngle);
                    Serial.printf("[TEST] Tilt UP: %d deg\n", servoCtrl.getAngle());
                } else if (cmd.equalsIgnoreCase("j")) {
                    int newAngle = servoCtrl.getAngle() - 10;
                    servoCtrl.setAngle(newAngle);
                    Serial.printf("[TEST] Tilt DOWN: %d deg\n", servoCtrl.getAngle());
                } else if (cmd.equalsIgnoreCase("h")) {
                    servoCtrl.home();
                    Serial.printf("[TEST] Tilt HOME: %d deg (level)\n", servoCtrl.getAngle());
                } else if (cmd.equalsIgnoreCase("sweep")) {
                    Serial.println("[TEST] Executing full servo sweep...");
                    for (int a = 90; a >= SERVO_TILT_MIN; a -= 2) { servoCtrl.setAngle(a); delay(20); }
                    for (int a = SERVO_TILT_MIN; a <= SERVO_TILT_MAX; a += 2) { servoCtrl.setAngle(a); delay(20); }
                    for (int a = SERVO_TILT_MAX; a >= SERVO_TILT_HOME; a -= 2) { servoCtrl.setAngle(a); delay(20); }
                    Serial.println("[TEST] Sweep complete. Restored to 90 deg.");
                } else if (cmd.equalsIgnoreCase("f")) {
                    transitionTo(STATE_MANUAL_TEST);
                    Serial.println("[TEST] Activating Extinguisher Solenoid for 1.0 second...");
                    sensorMgr.activateSolenoid();
                    delay(1000);
                    sensorMgr.deactivateSolenoid();
                    Serial.println("[TEST] Solenoid deactivated.");
                } else if (cmd.equalsIgnoreCase("r")) {
                    Serial.printf("[SENSORS] Front: %.1f cm | Left: %.1f cm | Right: %.1f cm | Flame Sensor: [%s]\n",
                                  sensorMgr.getFrontDistance(),
                                  sensorMgr.getLeftDistance(),
                                  sensorMgr.getRightDistance(),
                                  sensorMgr.isFlameDetected() ? "FIRE DETECTED!" : "CLEAR");
                } else if (cmd.equalsIgnoreCase("t")) {
                    streamSensorsActive = !streamSensorsActive;
                    Serial.printf("[TEST] Continuous sensor stream: %s\n",
                                  streamSensorsActive ? "ENABLED (Type 't' to stop)" : "DISABLED");
                } else if (cmd.equalsIgnoreCase("sim")) {
                    parser.injectTarget(EVENT_FIRE_CONFIRMED, 0.95f, 0.50f, 0.50f);
                    Serial.println("[SIM] Injected FIRE_CONFIRMED centered (x=0.50, y=0.50)");
                    if (currentState == STATE_MANUAL_TEST) {
                        Serial.println("[SIM] Note: Type 'auto' to enable autonomous tracking!");
                    }
                } else if (cmd.equalsIgnoreCase("simL")) {
                    parser.injectTarget(EVENT_FIRE_CONFIRMED, 0.92f, 0.20f, 0.50f);
                    Serial.println("[SIM] Injected FIRE_CONFIRMED on LEFT (x=0.20, y=0.50)");
                    if (currentState == STATE_MANUAL_TEST) {
                        Serial.println("[SIM] Note: Type 'auto' to enable autonomous tracking!");
                    }
                } else if (cmd.equalsIgnoreCase("simR")) {
                    parser.injectTarget(EVENT_FIRE_CONFIRMED, 0.92f, 0.80f, 0.50f);
                    Serial.println("[SIM] Injected FIRE_CONFIRMED on RIGHT (x=0.80, y=0.50)");
                    if (currentState == STATE_MANUAL_TEST) {
                        Serial.println("[SIM] Note: Type 'auto' to enable autonomous tracking!");
                    }
                } else if (cmd.equalsIgnoreCase("simU")) {
                    parser.injectTarget(EVENT_FIRE_CONFIRMED, 0.90f, 0.50f, 0.20f);
                    Serial.println("[SIM] Injected FIRE_CONFIRMED ELEVATED (x=0.50, y=0.20)");
                } else if (cmd.equalsIgnoreCase("simD")) {
                    parser.injectTarget(EVENT_FIRE_CONFIRMED, 0.90f, 0.50f, 0.80f);
                    Serial.println("[SIM] Injected FIRE_CONFIRMED LOW (x=0.50, y=0.80)");
                } else if (cmd.equalsIgnoreCase("clear")) {
                    parser.injectTarget(EVENT_NO_FIRE, 0.0f, 0.5f, 0.5f);
                    Serial.println("[SIM] Injected NO_FIRE target cleared.");
                } else if (cmd.equalsIgnoreCase("test")) {
                    transitionTo(STATE_MANUAL_TEST);
                    motorCtrl.stop();
                    sensorMgr.deactivateSolenoid();
                    Serial.println("[MODE] Entered MANUAL_TEST mode. Autonomous navigation paused.");
                    Serial.println("       Type 'help' or 'm' to see all test commands.");
                } else if (cmd.equalsIgnoreCase("auto")) {
                    transitionTo(STATE_SEARCHING);
                    Serial.println("[MODE] Resumed AUTONOMOUS firefighting operation.");
                } else {
                    Serial.printf("Unknown command '%s'. Type 'help' or 'm' for menu.\n", cmd.c_str());
                }
            }
        } else {
            serialInputBuffer += c;
        }
    }

    if (streamSensorsActive) {
        uint32_t now = millis();
        if (now - lastSensorStreamMs >= 250) {
            lastSensorStreamMs = now;
            Serial.printf("[STREAM] Front: %5.1f cm | Left: %5.1f cm | Right: %5.1f cm | Flame: [%s]\n",
                          sensorMgr.getFrontDistance(),
                          sensorMgr.getLeftDistance(),
                          sensorMgr.getRightDistance(),
                          sensorMgr.isFlameDetected() ? "FIRE DETECTED!" : "CLEAR");
        }
    }
}

void setup() {
    // 1. Hardware debug serial over USB
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==============================================");
    Serial.println("   BANTAY-SUNOG: ESP32 Actuation Controller   ");
    Serial.println("==============================================");

    // 2. Initialize subsystems
    Serial.println("[INIT] Initializing UART2 communication with Pi...");
    parser.begin();

    Serial.println("[INIT] Initializing TB6612FNG motor driver...");
    motorCtrl.begin();

    Serial.println("[INIT] Initializing tilt servo turret...");
    servoCtrl.begin();

    Serial.println("[INIT] Initializing ultrasonic & flame sensors...");
    sensorMgr.begin();

    Serial.println("[SYSTEM] ESP32 Firefighting Controller READY.");
    Serial.println("[INFO] >> Type 'help' or 'm' in Serial Monitor for interactive diagnostics! <<");
    transitionTo(STATE_SEARCHING);
}

void loop() {
    uint32_t now = millis();

    // Handle interactive test commands from Serial Monitor
    handleSerialCommands();

    // 1. Process incoming messages from Raspberry Pi
    parser.update();

    // 2. Poll sensors (ultrasonic round-robin, flame sensor, solenoid watchdog)
    sensorMgr.update();

    const VisionTarget& target = parser.getTarget();
    bool targetFresh = parser.isTargetFresh();

    // -------------------------------------------------------------------------
    // 3. Finite State Machine
    // -------------------------------------------------------------------------
    switch (currentState) {

        // =====================================================================
        // STATE_SEARCHING: Autonomous Patrol & Room Sweep with Obstacle Avoidance
        // =====================================================================
        case STATE_SEARCHING: {
            // Check for confirmed fire target from vision subsystem
            if (targetFresh && target.event == EVENT_FIRE_CONFIRMED) {
                motorCtrl.stop();
                transitionTo(STATE_ALIGNING);
                break;
            } else if (targetFresh && target.event == EVENT_FIRE_CANDIDATE) {
                transitionTo(STATE_TRACKING_CANDIDATE);
                break;
            }

            float dF = sensorMgr.getFrontDistance();
            float dL = sensorMgr.getLeftDistance();
            float dR = sensorMgr.getRightDistance();

            // Geometric Classification of Obstruction:
            // Continuous Wall (both flanks constrained or symmetric close corridor) vs Discrete Obstacle (open flank)
            bool isWall = (dL < 60.0f && dR < 60.0f) || (fabsf(dL - dR) < 18.0f && dF < 45.0f);
            bool isFlankClearRight = (dR >= dL);

            // Obstacle Avoidance Navigation
            if (sensorMgr.isFrontBlocked()) {
                if (isWall) {
                    // Continuous Wall Encountered: Execute decisive 90-degree Lane Turn away from wall
                    if (now - lastRoamTurnMs > 600) {
                        roamingTurnLeft = !isFlankClearRight;
                        lastRoamTurnMs = now;
                    }
                    if (roamingTurnLeft) {
                        motorCtrl.pivotLeft(SPEED_OBSTACLE_TURN);
                    } else {
                        motorCtrl.pivotRight(SPEED_OBSTACLE_TURN);
                    }
                } else {
                    // Discrete Obstacle (Crate / Barrel): Perform smooth bypass curve without losing lane heading
                    if (isFlankClearRight) {
                        motorCtrl.driveSteer(SPEED_SEARCH_ROAM, 75); // curve right around crate
                    } else {
                        motorCtrl.driveSteer(SPEED_SEARCH_ROAM, -75); // curve left around crate
                    }
                }
            } else if (sensorMgr.isLeftBlocked()) {
                // Steer slightly away from left wall
                motorCtrl.driveSteer(SPEED_SEARCH_ROAM, 40);
            } else if (sensorMgr.isRightBlocked()) {
                // Steer slightly away from right wall
                motorCtrl.driveSteer(SPEED_SEARCH_ROAM, -40);
            } else {
                // Path clear - proceed forward and sweep tilt servo to look for flames
                motorCtrl.forward(SPEED_SEARCH_ROAM);
                servoCtrl.updateSearchSweep();
            }
            break;
        }

        // =====================================================================
        // STATE_TRACKING_CANDIDATE: Decelerate and Observe
        // =====================================================================
        case STATE_TRACKING_CANDIDATE: {
            if (!targetFresh) {
                transitionTo(STATE_SEARCHING);
                break;
            }
            if (target.event == EVENT_FIRE_CONFIRMED) {
                motorCtrl.stop();
                transitionTo(STATE_ALIGNING);
                break;
            }

            // Slow down and steer tentatively toward candidate
            int steer = round((target.x - OPTICAL_CENTER_X) * (KP_CHASSIS_YAW * 0.7f));
            motorCtrl.driveSteer(SPEED_SEARCH_ROAM - 20, steer);
            servoCtrl.adjustPitch(target.y);
            break;
        }

        // =====================================================================
        // STATE_ALIGNING: In-Place Yaw (Chassis) & Pitch (Servo) Optical Centering
        // =====================================================================
        case STATE_ALIGNING: {
            if (!targetFresh) {
                transitionTo(STATE_SEARCHING);
                break;
            }

            float xError = target.x - OPTICAL_CENTER_X;
            bool xAligned = fabs(xError) <= X_ALIGN_TOLERANCE;
            bool yAligned = servoCtrl.adjustPitch(target.y);

            if (!xAligned) {
                if (xError < 0) {
                    motorCtrl.pivotLeft(SPEED_PIVOT_ALIGN);
                } else {
                    motorCtrl.pivotRight(SPEED_PIVOT_ALIGN);
                }
            } else {
                motorCtrl.stop();
            }

            // Once both X (body) and Y (servo) are centered on target
            if (xAligned && yAligned) {
                motorCtrl.stop();
                transitionTo(STATE_APPROACHING);
            }
            break;
        }

        // =====================================================================
        // STATE_APPROACHING: Drive Toward Flame While Maintaining Visual Center
        // =====================================================================
        case STATE_APPROACHING: {
            if (!targetFresh) {
                motorCtrl.stop();
                transitionTo(STATE_SEARCHING);
                break;
            }

            float frontDist = sensorMgr.getFrontDistance();

            // Check if we have arrived at extinguishing standoff distance
            if ((frontDist > 0.0f && frontDist <= EXTINGUISH_DISTANCE_CM) ||
                (target.bbox[2] > 200)) // Bounding box width > 200px also indicates close proximity
            {
                motorCtrl.stop();
                transitionTo(STATE_EXTINGUISHING);
                break;
            }

            // Emergency stop if dangerously close
            if (frontDist > 0.0f && frontDist <= MIN_APPROACH_DISTANCE_CM) {
                motorCtrl.stop();
                transitionTo(STATE_EXTINGUISHING);
                break;
            }

            // Check if flame has drifted significantly off-center
            if (fabs(target.x - OPTICAL_CENTER_X) > (X_ALIGN_TOLERANCE * 2.2f)) {
                motorCtrl.stop();
                transitionTo(STATE_ALIGNING);
                break;
            }

            // Proportional visual steering while advancing
            int steer = round((target.x - OPTICAL_CENTER_X) * KP_CHASSIS_YAW);
            motorCtrl.driveSteer(SPEED_APPROACH_BASE, steer);
            servoCtrl.adjustPitch(target.y);
            break;
        }

        // =====================================================================
        // STATE_EXTINGUISHING: Flame Verification & Solenoid Valve Suppression
        // =====================================================================
        case STATE_EXTINGUISHING: {
            motorCtrl.stop();

            // Secondary verification via physical IR flame sensor
            bool flameVerified = sensorMgr.isFlameDetected();

#if REQUIRE_HARDWARE_FLAME_VERIFICATION
            if (!flameVerified && !sensorMgr.isSolenoidActive()) {
                // If flame sensor hasn't triggered yet, allow up to 600ms for fine adjustment
                if (now - stateEnterMs > 600) {
                    Serial.println("[EXTINGUISH] Hardware flame sensor did not verify flame. Aborting spray.");
                    transitionTo(STATE_COOLDOWN);
                    break;
                }
                break;
            }
#endif

            // Trigger fire extinguishing solenoid valve / pump
            if (!sensorMgr.isSolenoidActive()) {
                Serial.println("[EXTINGUISH] Fire verified! Activating suppression solenoid...");
                sensorMgr.activateSolenoid();
            }

            // Check if spray duration has concluded
            if (sensorMgr.getSolenoidElapsedMs() >= SOLENOID_MAX_DURATION_MS) {
                sensorMgr.deactivateSolenoid();
                Serial.println("[EXTINGUISH] Spray cycle completed.");
                transitionTo(STATE_COOLDOWN);
            }
            break;
        }

        // =====================================================================
        // STATE_COOLDOWN: Settle Vapor/Smoke & Evaluate Extinguishment Success
        // =====================================================================
        case STATE_COOLDOWN: {
            motorCtrl.stop();
            sensorMgr.deactivateSolenoid();

            // Wait for extinguisher vapor to clear
            if (now - stateEnterMs >= COOLDOWN_DURATION_MS) {
                if (!targetFresh || target.event == EVENT_NO_FIRE || target.event == EVENT_FIRE_LOST) {
                    Serial.println("[COOLDOWN] Target extinguished successfully! Resuming patrol.");
                    parser.invalidateTarget();
                    transitionTo(STATE_SEARCHING);
                } else if (targetFresh && target.event == EVENT_FIRE_CONFIRMED) {
                    Serial.println("[COOLDOWN] Flame still persistent. Re-aligning for second pass.");
                    transitionTo(STATE_ALIGNING);
                } else {
                    transitionTo(STATE_SEARCHING);
                }
            }
            break;
        }

        // =====================================================================
        // STATE_MANUAL_TEST: Interactive bench test mode (motors idle unless commanded)
        // =====================================================================
        case STATE_MANUAL_TEST: {
            // Autonomous state machine paused; manual test commands active
            break;
        }

        default:
            transitionTo(STATE_SEARCHING);
            break;
    }

    // -------------------------------------------------------------------------
    // 4. Bidirectional Telemetry: Transmit Robot Status Back to Raspberry Pi
    // -------------------------------------------------------------------------
    RobotTelemetry telemetry;
    telemetry.state = currentState;
    telemetry.dist_front_cm = sensorMgr.getFrontDistance();
    telemetry.dist_left_cm = sensorMgr.getLeftDistance();
    telemetry.dist_right_cm = sensorMgr.getRightDistance();
    telemetry.flame_sensor_triggered = sensorMgr.isFlameDetected();
    telemetry.tilt_angle_deg = servoCtrl.getAngle();
    telemetry.solenoid_active = sensorMgr.isSolenoidActive();
    telemetry.left_motor_pwm = motorCtrl.getLeftPwm();
    telemetry.right_motor_pwm = motorCtrl.getRightPwm();
    telemetry.uptime_ms = now;

    reporter.sendTelemetry(telemetry);

    delay(5); // Cooperative yield
}
