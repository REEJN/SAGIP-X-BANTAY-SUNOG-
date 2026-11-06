#include "servo_controller.h"

ServoController::ServoController()
    : _currentAngle(SERVO_TILT_HOME), _sweepDirection(1), _lastSweepMs(0) {}

void ServoController::begin() {
    // Allocate timers for ESP32Servo
    ESP32PWM::allocateTimer(2);
    _servo.setPeriodHertz(50); // Standard 50Hz servo
    _servo.attach(PIN_SERVO_TILT, 500, 2400); // 500us to 2400us pulse width

    home();
}

void ServoController::setAngle(int angleDeg) {
    _currentAngle = constrain(angleDeg, SERVO_TILT_MIN, SERVO_TILT_MAX);
    _servo.write(_currentAngle);
}

void ServoController::home() {
    setAngle(SERVO_TILT_HOME);
}

bool ServoController::adjustPitch(float targetY) {
    float yError = targetY - OPTICAL_CENTER_Y;

    // Check if target is already vertically centered within deadband
    if (fabs(yError) <= Y_ALIGN_TOLERANCE) {
        return true;
    }

    // In computer vision coordinates, y=0.0 is top of image, y=1.0 is bottom.
    // If flame is near the top (y < 0.5), we need to pitch the camera UP (+angle or -angle depending on linkage).
    // Standard convention: Increasing angle pitches upward.
    // Therefore: error < 0 -> target is up -> increase angle.
    //            error > 0 -> target is down -> decrease angle.
    int angleDelta = round(-yError * KP_SERVO_PITCH);
    if (angleDelta == 0) {
        angleDelta = (yError < 0) ? 1 : -1;
    }

    setAngle(_currentAngle + angleDelta);
    return false;
}

void ServoController::updateSearchSweep() {
    uint32_t now = millis();
    // Step sweep every 80 ms for a gentle head scan
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
