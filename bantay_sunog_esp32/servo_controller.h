#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

/**
 * Dual Servo Controller:
 * 1. Tilt Servo (TowerPro MG90S on GPIO 13): Tilts webcam, co-axial turret, and IR flame sensor.
 * 2. Extinguisher Trigger Servo (TowerPro MG996R on GPIO 23): Mechanically depresses fire extinguisher can lever.
 */
class ServoController {
public:
    ServoController();

    /**
     * Attach servos to GPIO pins and initialize to neutral/idle positions.
     */
    void begin();

    /**
     * Periodic watchdog check (auto-releases trigger servo if max burst exceeded).
     */
    void update();

    // -------------------------------------------------------------------------
    // Tilt Servo (Pitch Gimbal)
    // -------------------------------------------------------------------------
    void setAngle(int angleDeg);
    bool adjustPitch(float targetY);
    void updateSearchSweep();
    void home();
    int getAngle() const { return _currentAngle; }

    // -------------------------------------------------------------------------
    // Extinguisher Canister Mechanical Trigger Servo (MG996R on GPIO 23)
    // -------------------------------------------------------------------------
    void triggerExtinguisher();
    void releaseExtinguisher();
    bool isExtinguisherActive() const { return _triggerActive; }
    uint32_t getExtinguisherElapsedMs() const;

private:
    Servo _tiltServo;
    Servo _triggerServo;

    int _currentAngle;
    int _sweepDirection;
    uint32_t _lastSweepMs;

    bool _triggerActive;
    uint32_t _triggerStartTimeMs;
};
