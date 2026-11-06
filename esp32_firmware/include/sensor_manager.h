#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * Sensor and Actuator Manager.
 * Orchestrates Front/Left/Right Ultrasonic distance sensors,
 * physical IR flame sensor validation, and suppression solenoid valve timing.
 */
class SensorManager {
public:
    SensorManager();

    /**
     * Configure sensor and solenoid GPIOs.
     */
    void begin();

    /**
     * Periodic sensor polling and safety watchdog.
     * Call inside main loop().
     */
    void update();

    // Ultrasonic Distance Getters (Centimeters)
    float getFrontDistance() const { return _distFront; }
    float getLeftDistance() const { return _distLeft; }
    float getRightDistance() const { return _distRight; }

    bool isFrontBlocked() const { return _distFront > 0.0f && _distFront <= OBSTACLE_THRESHOLD_FRONT_CM; }
    bool isLeftBlocked() const { return _distLeft > 0.0f && _distLeft <= OBSTACLE_THRESHOLD_SIDE_CM; }
    bool isRightBlocked() const { return _distRight > 0.0f && _distRight <= OBSTACLE_THRESHOLD_SIDE_CM; }

    // Physical Flame Sensor
    bool isFlameDetected() const;

    // Fire Extinguisher Solenoid Valve Control
    void activateSolenoid();
    void deactivateSolenoid();
    bool isSolenoidActive() const { return _solenoidActive; }
    uint32_t getSolenoidElapsedMs() const;

private:
    float _measureDistanceCm(uint8_t trigPin, uint8_t echoPin);

    float _distFront;
    float _distLeft;
    float _distRight;

    uint8_t _activeSensorIndex;
    uint32_t _lastSensorPollMs;

    bool _solenoidActive;
    uint32_t _solenoidStartTimeMs;
};
