#pragma once

#include <Arduino.h>

// =============================================================================
// BANTAY-SUNOG ESP32 CONFIGURATION & PINOUT
// Hardware: ESP32-WROOM-32 (38-pin / 30-pin)
// =============================================================================

// -----------------------------------------------------------------------------
// 1. UART Communication (UART2 with Raspberry Pi 3B)
// -----------------------------------------------------------------------------
#define UART_PI_BAUD                115200
#define PIN_UART2_RX                16       // Connected to Pi GPIO 14 (TXD)
#define PIN_UART2_TX                17       // Connected to Pi GPIO 15 (RXD)

// -----------------------------------------------------------------------------
// 2. TB6612FNG Dual H-Bridge Motor Driver
// Controls 4 DC Drive Motors (Left pair on Chan A, Right pair on Chan B)
// -----------------------------------------------------------------------------
#define PIN_MOTOR_PWMA              25       // Left motors speed (PWM)
#define PIN_MOTOR_AIN1              26       // Left motors direction 1
#define PIN_MOTOR_AIN2              27       // Left motors direction 2

#define PIN_MOTOR_PWMB              18       // Right motors speed (PWM)
#define PIN_MOTOR_BIN1              19       // Right motors direction 1
#define PIN_MOTOR_BIN2              21       // Right motors direction 2

#define PIN_MOTOR_STBY              22       // Standby pin (HIGH = enabled)

// Motor PWM Parameters (LEDC Peripheral)
#define MOTOR_PWM_FREQ_HZ           20000    // 20 kHz (silent, above audible range)
#define MOTOR_PWM_RESOLUTION_BITS   8        // 8-bit resolution (0 - 255)
#define MOTOR_PWM_CHAN_LEFT         0        // LEDC channel for Left
#define MOTOR_PWM_CHAN_RIGHT        1        // LEDC channel for Right

// Motor Speeds (0 - 255)
#define SPEED_SEARCH_ROAM           140      // Gentle speed while patrolling
#define SPEED_APPROACH_BASE         160      // Baseline forward speed towards fire
#define SPEED_PIVOT_ALIGN           130      // In-place pivot rotation speed
#define SPEED_OBSTACLE_TURN         150      // Obstacle avoidance pivot speed
#define SPEED_MIN_THRESHOLD         70       // Minimum duty cycle to overcome friction

// -----------------------------------------------------------------------------
// 3. Aiming Turret & Suppression Actuators
// -----------------------------------------------------------------------------
#define PIN_SERVO_TILT              13       // Tilt servo for Webcam + Nozzle + Flame sensor
#define PIN_SOLENOID                23       // Extinguisher valve trigger (via MOSFET/Relay)

// Servo Angles (Degrees)
#define SERVO_TILT_MIN              45       // Look downward toward close ground
#define SERVO_TILT_MAX              135      // Look upward toward elevated targets
#define SERVO_TILT_HOME             90       // Level horizon (neutral)
#define SERVO_TILT_SMOOTH_DELAY_MS  15       // Milliseconds per degree step

// Solenoid / Extinguisher Parameters
#define SOLENOID_ACTIVE_HIGH        true     // true: HIGH=Spray, false: LOW=Spray
#define SOLENOID_MAX_DURATION_MS    3000     // Maximum continuous spray burst (3.0s)
#define COOLDOWN_DURATION_MS        2000     // Wait for smoke/vapor clearing (2.0s)

// -----------------------------------------------------------------------------
// 4. Ultrasonic Sensors (HC-SR04)
// Note: Echo outputs must be 3.3V safe (via 1k/2k resistor divider or 3.3V sensors)
// Pins 34, 35, 36, 39 are input-only GPI pins without internal pullup/pulldown.
// -----------------------------------------------------------------------------
#define PIN_TRIG_FRONT              4
#define PIN_ECHO_FRONT              36       // SENSOR_VP (Input Only)

#define PIN_TRIG_LEFT               32
#define PIN_ECHO_LEFT               39       // SENSOR_VN (Input Only)

#define PIN_TRIG_RIGHT              33
#define PIN_ECHO_RIGHT              35       // Input Only

// Ultrasonic Distance Limits (Centimeters)
#define ULTRASONIC_MAX_DIST_CM      300.0f
#define ULTRASONIC_TIMEOUT_US       25000    // ~400 cm round-trip timeout

#define OBSTACLE_THRESHOLD_FRONT_CM 25.0f    // Stop and turn if front obstacle < 25 cm
#define OBSTACLE_THRESHOLD_SIDE_CM  18.0f    // Steer away if side obstacle < 18 cm

// -----------------------------------------------------------------------------
// 5. Hardware Flame Sensor (Secondary Physical Validation)
// Mounted on the tilt assembly co-aligned with webcam optical axis
// -----------------------------------------------------------------------------
#define PIN_FLAME_DO                34       // Input Only (LM393 D0 comparator out)
#define FLAME_SENSOR_ACTIVE_LOW     true     // Most IR flame modules output LOW on fire

// If true, solenoid will ONLY trigger if both vision confirmed AND flame sensor triggers.
// Set false during bench testing if hardware flame module is disconnected.
#define REQUIRE_HARDWARE_FLAME_VERIFICATION true

// -----------------------------------------------------------------------------
// 6. Navigation, Visual Servoing & Approach Parameters
// -----------------------------------------------------------------------------
#define EXTINGUISH_DISTANCE_CM      30.0f    // Target distance to stop and spray (cm)
#define MIN_APPROACH_DISTANCE_CM    20.0f    // Absolute minimum safety standoff distance (cm)

// Optical Center Align Tolerances (Normalized Coordinates 0.0 - 1.0)
#define OPTICAL_CENTER_X            0.50f
#define OPTICAL_CENTER_Y            0.50f
#define X_ALIGN_TOLERANCE           0.07f    // Center band: 0.43 to 0.57
#define Y_ALIGN_TOLERANCE           0.08f    // Center band: 0.42 to 0.58

// Visual Servoing Gains (Proportional steering)
#define KP_CHASSIS_YAW              180.0f   // Multiplier for X-offset to differential speed
#define KP_SERVO_PITCH              25.0f    // Multiplier for Y-offset to servo degrees

// Timeouts (Milliseconds)
#define TARGET_LOSS_TIMEOUT_MS      2500     // Revert to SEARCHING if no message for 2.5s
#define TELEMETRY_SEND_INTERVAL_MS  200      // Transmit robot status back to Pi every 200ms
#define SENSOR_READ_INTERVAL_MS     60       // Read ultrasonic sensors round-robin every 60ms
