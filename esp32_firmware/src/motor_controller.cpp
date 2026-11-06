#include "motor_controller.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#if (defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3)
#define USE_ESP32_CORE_V3 1
#else
// Default to v3 on newer Arduino ESP32 core
#define USE_ESP32_CORE_V3 1
#endif

MotorController::MotorController()
    : _currentLeftPwm(0), _currentRightPwm(0), _isStandby(false) {}

void MotorController::begin() {
    // Direction & standby pins
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_BIN1, OUTPUT);
    pinMode(PIN_MOTOR_BIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Initial safe state: motors stopped, standby active
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_BIN2, LOW);

    // Setup LEDC PWM for ESP32
#if USE_ESP32_CORE_V3
    ledcAttach(PIN_MOTOR_PWMA, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_PWMB, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
#else
    ledcSetup(MOTOR_PWM_CHAN_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_PWMA, MOTOR_PWM_CHAN_LEFT);

    ledcSetup(MOTOR_PWM_CHAN_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_PWMB, MOTOR_PWM_CHAN_RIGHT);
#endif

    _writeLeftPwm(0);
    _writeRightPwm(0);

    // Wake driver out of standby
    setStandby(true);
}

void MotorController::setStandby(bool enable) {
    _isStandby = !enable;
    digitalWrite(PIN_MOTOR_STBY, enable ? HIGH : LOW);
}

void MotorController::forward(uint8_t speed) {
    setRawMotors(speed, speed);
}

void MotorController::reverse(uint8_t speed) {
    setRawMotors(-speed, -speed);
}

void MotorController::pivotLeft(uint8_t speed) {
    setRawMotors(-speed, speed);
}

void MotorController::pivotRight(uint8_t speed) {
    setRawMotors(speed, -speed);
}

void MotorController::driveSteer(int baseSpeed, int steerOffset) {
    int leftSpeed = baseSpeed + steerOffset;
    int rightSpeed = baseSpeed - steerOffset;

    // Clamp within 0 - 255
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    setRawMotors(leftSpeed, rightSpeed);
}

void MotorController::setRawMotors(int leftSpeed, int rightSpeed) {
    _setLeftMotor(leftSpeed);
    _setRightMotor(rightSpeed);
}

void MotorController::stop() {
    _setLeftMotor(0);
    _setRightMotor(0);
}

void MotorController::_setLeftMotor(int speed) {
    _currentLeftPwm = speed;
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        _writeLeftPwm((uint8_t)constrain(speed, 0, 255));
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, HIGH);
        _writeLeftPwm((uint8_t)constrain(-speed, 0, 255));
    } else {
        // Coast / Stop
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        _writeLeftPwm(0);
    }
}

void MotorController::_setRightMotor(int speed) {
    _currentRightPwm = speed;
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_BIN1, HIGH);
        digitalWrite(PIN_MOTOR_BIN2, LOW);
        _writeRightPwm((uint8_t)constrain(speed, 0, 255));
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_BIN1, LOW);
        digitalWrite(PIN_MOTOR_BIN2, HIGH);
        _writeRightPwm((uint8_t)constrain(-speed, 0, 255));
    } else {
        // Coast / Stop
        digitalWrite(PIN_MOTOR_BIN1, LOW);
        digitalWrite(PIN_MOTOR_BIN2, LOW);
        _writeRightPwm(0);
    }
}

void MotorController::_writeLeftPwm(uint8_t duty) {
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_MOTOR_PWMA, duty);
#else
    ledcWrite(MOTOR_PWM_CHAN_LEFT, duty);
#endif
}

void MotorController::_writeRightPwm(uint8_t duty) {
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_MOTOR_PWMB, duty);
#else
    ledcWrite(MOTOR_PWM_CHAN_RIGHT, duty);
#endif
}

void MotorController::runDriverCheck() {
    Serial.println("\n=======================================================");
    Serial.println("         TB6612FNG MOTOR DRIVER DIAGNOSTIC CHECK        ");
    Serial.println("=======================================================");
    Serial.println("This test verifies all motor control signals step-by-step.");
    Serial.println("If using LEDs as motor indicators:");
    Serial.println("  * Connect LED Anode (+) to AO1/BO1 and Cathode (-) to AO2/BO2");
    Serial.println("  * Ensure TB6612FNG VM pin has power (5V-12V battery)");
    Serial.println("  * Ensure TB6612FNG VCC has 3.3V/5V and GND is connected!");
    Serial.println("-------------------------------------------------------");

    // Phase 1: Standby Pin Check
    Serial.println("[CHECK 1/5] Enabling STBY pin (GPIO 22 HIGH)...");
    digitalWrite(PIN_MOTOR_STBY, HIGH);
    delay(500);
    Serial.println(" -> STBY is active HIGH.");

    // Phase 2: Left Motor (Channel A) - Forward
    Serial.println("\n[CHECK 2/5] Testing LEFT MOTOR (Channel A) FORWARD...");
    Serial.printf(" -> Setting AIN1 (GPIO %d)=HIGH, AIN2 (GPIO %d)=LOW, PWMA (GPIO %d)=255\n",
                  PIN_MOTOR_AIN1, PIN_MOTOR_AIN2, PIN_MOTOR_PWMA);
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    _writeLeftPwm(255);
    Serial.println(" -> LED on AO1(+) & AO2(-) should be ON now!");
    for (int i = 3; i > 0; i--) {
        Serial.printf("    Holding for %d seconds...\n", i);
        delay(1000);
    }
    stop();
    Serial.println(" -> Left motor STOP.");
    delay(500);

    // Phase 3: Left Motor (Channel A) - Reverse
    Serial.println("\n[CHECK 3/5] Testing LEFT MOTOR (Channel A) REVERSE...");
    Serial.printf(" -> Setting AIN1 (GPIO %d)=LOW, AIN2 (GPIO %d)=HIGH, PWMA (GPIO %d)=255\n",
                  PIN_MOTOR_AIN1, PIN_MOTOR_AIN2, PIN_MOTOR_PWMA);
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    _writeLeftPwm(255);
    Serial.println(" -> LED on AO2(+) & AO1(-) should be ON now (reversed polarity)!");
    for (int i = 3; i > 0; i--) {
        Serial.printf("    Holding for %d seconds...\n", i);
        delay(1000);
    }
    stop();
    Serial.println(" -> Left motor STOP.");
    delay(500);

    // Phase 4: Right Motor (Channel B) - Forward
    Serial.println("\n[CHECK 4/5] Testing RIGHT MOTOR (Channel B) FORWARD...");
    Serial.printf(" -> Setting BIN1 (GPIO %d)=HIGH, BIN2 (GPIO %d)=LOW, PWMB (GPIO %d)=255\n",
                  PIN_MOTOR_BIN1, PIN_MOTOR_BIN2, PIN_MOTOR_PWMB);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_BIN2, LOW);
    _writeRightPwm(255);
    Serial.println(" -> LED on BO1(+) & BO2(-) should be ON now!");
    for (int i = 3; i > 0; i--) {
        Serial.printf("    Holding for %d seconds...\n", i);
        delay(1000);
    }
    stop();
    Serial.println(" -> Right motor STOP.");
    delay(500);

    // Phase 5: Right Motor (Channel B) - Reverse
    Serial.println("\n[CHECK 5/5] Testing RIGHT MOTOR (Channel B) REVERSE...");
    Serial.printf(" -> Setting BIN1 (GPIO %d)=LOW, BIN2 (GPIO %d)=HIGH, PWMB (GPIO %d)=255\n",
                  PIN_MOTOR_BIN1, PIN_MOTOR_BIN2, PIN_MOTOR_PWMB);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_BIN2, HIGH);
    _writeRightPwm(255);
    Serial.println(" -> LED on BO2(+) & BO1(-) should be ON now (reversed polarity)!");
    for (int i = 3; i > 0; i--) {
        Serial.printf("    Holding for %d seconds...\n", i);
        delay(1000);
    }
    stop();
    Serial.println(" -> Right motor STOP.");
    delay(500);

    // Phase 6: PWM Speed Fade Test
    Serial.println("\n[PWM CHECK] Testing PWM Dimming Ramp (0% -> 100% -> 0%)...");
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_BIN2, LOW);
    for (int duty = 0; duty <= 255; duty += 15) {
        _writeLeftPwm(duty);
        _writeRightPwm(duty);
        Serial.printf(" -> Duty: %3d / 255 (%d%%)\n", duty, (duty * 100) / 255);
        delay(80);
    }
    for (int duty = 255; duty >= 0; duty -= 15) {
        _writeLeftPwm(duty);
        _writeRightPwm(duty);
        delay(40);
    }
    stop();
    Serial.println(" -> PWM Fade Test Complete.");

    Serial.println("\n-------------------------------------------------------");
    Serial.println("               DIAGNOSTIC TROUBLESHOOTING              ");
    Serial.println("-------------------------------------------------------");
    Serial.println("If your LED did NOT turn on during this test:");
    Serial.println(" 1. TB6612FNG VM PIN: VM powers the motor outputs!");
    Serial.println("    You MUST connect a battery or 5V power to the 'VM' pin.");
    Serial.println("    VCC (3.3V/5V) only powers the internal logic chip, NOT the motors/LEDs!");
    Serial.println(" 2. COMMON GROUND: Connect battery GND, TB6612FNG GND, and ESP32 GND together.");
    Serial.println(" 3. STBY PIN: TB6612FNG STBY must receive 3.3V (GPIO 22).");
    Serial.println(" 4. LED POLARITY: An LED is a diode. In Forward mode, connect Anode(+) to AO1/BO1");
    Serial.println("    and Cathode(-) to AO2/BO2.");
    Serial.println(" 5. DIRECT GPIO TEST: Type 'pins' in Serial Monitor to test raw ESP32 GPIOs directly.");
    Serial.println("=======================================================\n");
}

void MotorController::testDirectGpios() {
    Serial.println("\n=======================================================");
    Serial.println("       DIRECT ESP32 GPIO PIN BLINK TEST (NO DRIVER)    ");
    Serial.println("=======================================================");
    Serial.println("This test detaches PWM and drives each motor GPIO pin HIGH (3.3V)");
    Serial.println("one-by-one so you can test directly with an LED + resistor on the ESP32!");
    Serial.println("Wiring: LED Anode (+) to GPIO Pin, Cathode (-) with 220-470 ohm to GND");
    Serial.println("-------------------------------------------------------");

    const int pins[] = {
        PIN_MOTOR_PWMA, // 25
        PIN_MOTOR_AIN1, // 26
        PIN_MOTOR_AIN2, // 27
        PIN_MOTOR_PWMB, // 18
        PIN_MOTOR_BIN1, // 19
        PIN_MOTOR_BIN2, // 21
        PIN_MOTOR_STBY  // 22
    };
    const char* names[] = {
        "GPIO 25 (PWMA - Left Speed)",
        "GPIO 26 (AIN1 - Left Dir 1)",
        "GPIO 27 (AIN2 - Left Dir 2)",
        "GPIO 18 (PWMB - Right Speed)",
        "GPIO 19 (BIN1 - Right Dir 1)",
        "GPIO 21 (BIN2 - Right Dir 2)",
        "GPIO 22 (STBY - Standby Enable)"
    };

    for (int i = 0; i < 7; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
        Serial.printf("[PIN TEST %d/7] >> %s is now HIGH (3.3V) <<\n", i + 1, names[i]);
        Serial.println("                LED should be ON now. Waiting 2.5s...");
        delay(2500);
        digitalWrite(pins[i], LOW);
        Serial.printf("                %s is now LOW (0V). OFF.\n\n", names[i]);
        delay(400);
    }

    Serial.println("Direct GPIO test complete. Re-initializing normal PWM driver...");
    begin();
    Serial.println("Driver re-initialized.\n=======================================================\n");
}
