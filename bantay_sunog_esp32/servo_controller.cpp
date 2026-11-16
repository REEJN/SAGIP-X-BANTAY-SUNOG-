#include "servo_controller.h"

ServoController::ServoController()
    : _currentAngle(SERVO_TILT_HOME), _sweepDirection(1), _lastSweepMs(0),
      _triggerActive(false), _triggerStartTimeMs(0) {}

void ServoController::begin() {
    // Allocate timers for ESP32Servo
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // 1. Tilt Servo (TowerPro MG90S on GPIO 13)
    _tiltServo.setPeriodHertz(50);
    _tiltServo.attach(PIN_SERVO_TILT, 500, 2400);
    home();

    // 2. Extinguisher Trigger Servo (TowerPro MG996R on GPIO 23)
    _triggerServo.setPeriodHertz(50);
    _triggerServo.attach(PIN_SERVO_TRIGGER, 500, 2400);
    releaseExtinguisher();
}

void ServoController::update() {
    // Unconditional fail-safe watchdog: release trigger if burst exceeds maximum duration
    if (_triggerActive) {
        if (millis() - _triggerStartTimeMs >= EXTINGUISHER_MAX_BURST_MS) {
            releaseExtinguisher();
        }
    }
}

void ServoController::setAngle(int angleDeg) {
    _currentAngle = constrain(angleDeg, SERVO_TILT_MIN, SERVO_TILT_MAX);
    _tiltServo.write(_currentAngle);
}

void ServoController::home() {
    setAngle(SERVO_TILT_HOME);
}

bool ServoController::adjustPitch(float targetY) {
    float yError = targetY - OPTICAL_CENTER_Y;

    if (fabs(yError) <= Y_ALIGN_TOLERANCE) {
        return true;
    }

    int angleDelta = round(-yError * KP_SERVO_PITCH);
    if (angleDelta == 0) {
        angleDelta = (yError < 0) ? 1 : -1;
    }

    setAngle(_currentAngle + angleDelta);
    return false;
}

void ServoController::updateSearchSweep() {
    uint32_t now = millis();
    if (now - _lastSweepMs < 80) {
        return;
    }
    _lastSweepMs = now;

    int newAngle = _currentAngle + (_sweepDirection * 1);
    if (newAngle >= (SERVO_TILT_HOME + 20)) {
        newAngle = SERVO_TILT_HOME + 20;
        _sweepDirection = -1;
    } else if (newAngle <= (SERVO_TILT_HOME - 15)) {
        newAngle = SERVO_TILT_HOME - 15;
        _sweepDirection = 1;
    }

    setAngle(newAngle);
}

void ServoController::triggerExtinguisher() {
    _triggerActive = true;
    _triggerStartTimeMs = millis();
    _triggerServo.write(SERVO_TRIGGER_ENGAGED);
}

void ServoController::releaseExtinguisher() {
    _triggerActive = false;
    _triggerStartTimeMs = 0;
    _triggerServo.write(SERVO_TRIGGER_RELEASED);
}

uint32_t ServoController::getExtinguisherElapsedMs() const {
    if (!_triggerActive) return 0;
    return millis() - _triggerStartTimeMs;
}
