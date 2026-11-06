#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * TB6612FNG Dual H-Bridge Motor Driver Controller.
 * Manages 4 DC drive motors with skid-steer differential driving.
 */
class MotorController {
public:
    MotorController();

    /**
     * Initialize GPIO pins, configure LEDC PWM channels, and enable standby.
     */
    void begin();

    /**
     * Drive forward with specified base speed (0 - 255).
     */
    void forward(uint8_t speed);

    /**
     * Drive in reverse with specified base speed (0 - 255).
     */
    void reverse(uint8_t speed);

    /**
     * Pivot/Spin in place to the left (Left wheels reverse, Right wheels forward).
     */
    void pivotLeft(uint8_t speed);

    /**
     * Pivot/Spin in place to the right (Left wheels forward, Right wheels reverse).
     */
    void pivotRight(uint8_t speed);

    /**
     * Differential drive with steering offset.
     * @param baseSpeed Base forward speed (0 - 255).
     * @param steerOffset Positive steers right (speeds up left, slows right);
     *                    Negative steers left (speeds up right, slows left).
     */
    void driveSteer(int baseSpeed, int steerOffset);

    /**
     * Direct individual control of left and right motor speeds.
     * @param leftSpeed -255 to 255 (negative = reverse)
     * @param rightSpeed -255 to 255 (negative = reverse)
     */
    void setRawMotors(int leftSpeed, int rightSpeed);

    /**
     * Stop all motors immediately and enter coast or brake.
     */
    void stop();

    /**
     * Enable or disable the driver via STBY pin.
     */
    void setStandby(bool enable);

    /**
     * Run full TB6612FNG driver and motor diagnostic test with countdown.
     * Ideal for testing with motors or LEDs connected to AO1/AO2 and BO1/BO2.
     */
    void runDriverCheck();

    /**
     * Direct ESP32 GPIO blink test (bypasses PWM).
     * Tests ESP32 pins 25, 26, 27, 18, 19, 21, 22 directly with an LED to GND.
     */
    void testDirectGpios();

    int getLeftPwm() const { return _currentLeftPwm; }
    int getRightPwm() const { return _currentRightPwm; }

private:
    void _setLeftMotor(int speed);
    void _setRightMotor(int speed);
    void _writeLeftPwm(uint8_t duty);
    void _writeRightPwm(uint8_t duty);

    int _currentLeftPwm;
    int _currentRightPwm;
    bool _isStandby;
};
