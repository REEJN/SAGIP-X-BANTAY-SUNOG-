#include "sensor_manager.h"

SensorManager::SensorManager()
    : _distFront(100.0f), _distLeft(100.0f), _distRight(100.0f),
      _activeSensorIndex(0), _lastSensorPollMs(0),
      _solenoidActive(false), _solenoidStartTimeMs(0) {}

void SensorManager::begin() {
    // Ultrasonic Trigger pins (Outputs)
    pinMode(PIN_TRIG_FRONT, OUTPUT);
    pinMode(PIN_TRIG_LEFT, OUTPUT);
    pinMode(PIN_TRIG_RIGHT, OUTPUT);

    digitalWrite(PIN_TRIG_FRONT, LOW);
    digitalWrite(PIN_TRIG_LEFT, LOW);
    digitalWrite(PIN_TRIG_RIGHT, LOW);

    // Ultrasonic Echo pins (Inputs)
    pinMode(PIN_ECHO_FRONT, INPUT);
    pinMode(PIN_ECHO_LEFT, INPUT);
    pinMode(PIN_ECHO_RIGHT, INPUT);

    // Hardware IR Flame Sensor (Input)
    pinMode(PIN_FLAME_DO, INPUT);

    // Solenoid Valve Trigger (Output)
    pinMode(PIN_SOLENOID, OUTPUT);
    deactivateSolenoid();
}

void SensorManager::update() {
    uint32_t now = millis();

    // 1. Solenoid automatic safety timeout watchdog
    if (_solenoidActive) {
        if ((now - _solenoidStartTimeMs) >= SOLENOID_MAX_DURATION_MS) {
            deactivateSolenoid();
        }
    }

    // 2. Ultrasonic round-robin sampling (prevents echo crosstalk between sensors)
    if (now - _lastSensorPollMs >= SENSOR_READ_INTERVAL_MS) {
        _lastSensorPollMs = now;

        float reading = 0.0f;
        switch (_activeSensorIndex) {
            case 0:
                reading = _measureDistanceCm(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
                if (reading > 0.0f) _distFront = (_distFront * 0.4f) + (reading * 0.6f);
                _activeSensorIndex = 1;
                break;
            case 1:
                reading = _measureDistanceCm(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
                if (reading > 0.0f) _distLeft = (_distLeft * 0.4f) + (reading * 0.6f);
                _activeSensorIndex = 2;
                break;
            case 2:
                reading = _measureDistanceCm(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
                if (reading > 0.0f) _distRight = (_distRight * 0.4f) + (reading * 0.6f);
                _activeSensorIndex = 0;
                break;
            default:
                _activeSensorIndex = 0;
                break;
        }
    }
}

float SensorManager::_measureDistanceCm(uint8_t trigPin, uint8_t echoPin) {
    // Generate clean 10us trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo duration
    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
    if (duration == 0) {
        return -1.0f; // Timeout / out of range
    }

    // Sound speed = 343 m/s = 29.1 us/cm. Round trip: duration / 58.2
    float distanceCm = (float)duration / 58.2f;
    if (distanceCm > ULTRASONIC_MAX_DIST_CM) {
        return ULTRASONIC_MAX_DIST_CM;
    }
    return distanceCm;
}

bool SensorManager::isFlameDetected() const {
    int rawVal = digitalRead(PIN_FLAME_DO);
    if (FLAME_SENSOR_ACTIVE_LOW) {
        return rawVal == LOW;
    } else {
        return rawVal == HIGH;
    }
}

void SensorManager::activateSolenoid() {
    _solenoidActive = true;
    _solenoidStartTimeMs = millis();
    digitalWrite(PIN_SOLENOID, SOLENOID_ACTIVE_HIGH ? HIGH : LOW);
}

void SensorManager::deactivateSolenoid() {
    _solenoidActive = false;
    _solenoidStartTimeMs = 0;
    digitalWrite(PIN_SOLENOID, SOLENOID_ACTIVE_HIGH ? LOW : HIGH);
}

uint32_t SensorManager::getSolenoidElapsedMs() const {
    if (!_solenoidActive) return 0;
    return millis() - _solenoidStartTimeMs;
}
