#pragma once

#include <Arduino.h>

// =============================================================================
// BANTAY-SUNOG DATA TYPES & STATE ENUMERATIONS
// =============================================================================

/**
 * High-level operational states for the ESP32 mobile firefighting robot.
 */
enum RobotState {
    STATE_INIT = 0,
    STATE_SEARCHING,             // Autonomous patrol & room sweep with obstacle avoidance
    STATE_TRACKING_CANDIDATE,    // Decelerating and centering on a tentative fire candidate
    STATE_ALIGNING,              // In-place rotation (yaw) and servo tilt (pitch) to center flame
    STATE_APPROACHING,           // Driving forward toward flame while maintaining optical center
    STATE_EXTINGUISHING,         // Stopped at standoff distance, physical flame verification & spraying
    STATE_COOLDOWN,              // Post-spray pause for smoke/vapor clearing & verification
    STATE_OBSTACLE_AVOID,        // Temporary maneuver around obstacle encountered during approach
    STATE_MANUAL_TEST,           // Interactive serial monitor diagnostic/bench test mode
    STATE_EMERGENCY_STOP         // Fault state or manual safety halt
};

/**
 * Returns a human-readable string representation of RobotState for telemetry.
 */
inline const char* robotStateToString(RobotState state) {
    switch (state) {
        case STATE_INIT:               return "INIT";
        case STATE_SEARCHING:          return "SEARCHING";
        case STATE_TRACKING_CANDIDATE: return "TRACKING_CANDIDATE";
        case STATE_ALIGNING:           return "ALIGNING";
        case STATE_APPROACHING:        return "APPROACHING";
        case STATE_EXTINGUISHING:      return "EXTINGUISHING";
        case STATE_COOLDOWN:           return "COOLDOWN";
        case STATE_OBSTACLE_AVOID:     return "OBSTACLE_AVOID";
        case STATE_MANUAL_TEST:        return "MANUAL_TEST";
        case STATE_EMERGENCY_STOP:     return "EMERGENCY_STOP";
        default:                       return "UNKNOWN";
    }
}

/**
 * Incoming vision events parsed from the Raspberry Pi UART stream.
 */
enum VisionEvent {
    EVENT_NONE = 0,
    EVENT_FIRE_CONFIRMED,
    EVENT_FIRE_CANDIDATE,
    EVENT_NO_FIRE,
    EVENT_FIRE_LOST,
    EVENT_ERROR
};

/**
 * Cached target telemetry parsed from Raspberry Pi JSON messages.
 */
struct VisionTarget {
    VisionEvent event;
    float confidence;
    float x;                 // Normalized horizontal center (0.0 to 1.0)
    float y;                 // Normalized vertical center (0.0 to 1.0)
    int bbox[4];             // [x, y, w, h] pixel coordinates
    int frame_dims[2];       // [width, height]
    uint32_t last_update_ms; // Monotonic millisecond timestamp of receipt
    bool valid;

    void reset() {
        event = EVENT_NONE;
        confidence = 0.0f;
        x = 0.5f;
        y = 0.5f;
        bbox[0] = bbox[1] = bbox[2] = bbox[3] = 0;
        frame_dims[0] = 640;
        frame_dims[1] = 480;
        last_update_ms = 0;
        valid = false;
    }
};

/**
 * Outgoing telemetry status payload sent back to Raspberry Pi over UART2.
 */
struct RobotTelemetry {
    RobotState state;
    float dist_front_cm;
    float dist_left_cm;
    float dist_right_cm;
    bool flame_sensor_triggered;
    int tilt_angle_deg;
    bool solenoid_active;
    int left_motor_pwm;
    int right_motor_pwm;
    uint32_t uptime_ms;
};
