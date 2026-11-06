#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

/**
 * Tilt Servo Controller for the Y-Axis assembly.
 * Directly tilts the USB Webcam, Extinguisher Nozzle, and IR Flame Sensor.
 */
class ServoController {
public:
    ServoController();

    /**
     * Attach servo to GPIO pin and drive to neutral horizon position.
     */
    void begin();

    /**
     * Set target angle directly in degrees (clamped to [SERVO_TILT_MIN, SERVO_TILT_MAX]).
     */
    void setAngle(int angleDeg);

    /**
     * Adjust pitch based on normalized optical target coordinate Y (0.0 to 1.0).
     * @param targetY Normalized vertical coordinate (0.5 = optical center).
     * @return True if target is within Y_ALIGN_TOLERANCE, False otherwise.
     */
    bool adjustPitch(float targetY);

    /**
     * Gentle vertical sweep to scan the room when in search mode.
     */
    void updateSearchSweep();

    /**
     * Reset servo to home position (90 degrees).
     */
    void home();

    int getAngle() const { return _currentAngle; }

private:
    Servo _servo;
    int _currentAngle;
    int _sweepDirection;
    uint32_t _lastSweepMs;
};
