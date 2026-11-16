#include "sensor_manager.h"
#include <Wire.h>
#include <math.h>

SensorManager::SensorManager()
    : _distFront(100.0f), _distLeft(100.0f), _distRight(100.0f),
      _activeSensorIndex(0), _lastSensorPollMs(0),
      _mpuConnected(false), _mpuAddr(MPU6050_I2C_ADDR),
      _accelX(0.0f), _accelY(0.0f), _accelZ(1.0f),
      _gyroX(0.0f), _gyroY(0.0f), _gyroZ(0.0f),
      _pitchDeg(0.0f), _rollDeg(0.0f), _yawDeg(0.0f), _tempC(25.0f),
      _lastMpuPollMs(0) {}

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

    // Initialize MPU-6050 6-Axis IMU
    _initMpu6050();
}

void SensorManager::update() {
    uint32_t now = millis();

    // Ultrasonic round-robin sampling (prevents echo crosstalk between sensors)
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

    // MPU-6050 IMU Periodic Polling (50 Hz / 20ms)
    if (_mpuConnected) {
        if (now - _lastMpuPollMs >= MPU6050_POLL_INTERVAL_MS) {
            float dt = (_lastMpuPollMs > 0) ? ((float)(now - _lastMpuPollMs) / 1000.0f) : 0.02f;
            _lastMpuPollMs = now;
            _pollMpu6050(dt);
        }
    } else {
        // Retry connection every 5 seconds if disconnected
        if (now - _lastMpuPollMs >= 5000) {
            _lastMpuPollMs = now;
            _initMpu6050();
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

bool SensorManager::_initMpu6050() {
#if defined(ENABLE_MPU6050) && ENABLE_MPU6050
    // Enable internal pull-ups on I2C pins before starting Wire
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, MPU6050_I2C_FREQ_HZ);

    // Auto-probe candidate addresses: 0x68 (AD0=GND) and 0x69 (AD0=VCC/floating)
    uint8_t addrs[2] = {0x68, 0x69};
    bool found = false;

    for (int i = 0; i < 2; i++) {
        uint8_t testAddr = addrs[i];
        Wire.beginTransmission(testAddr);
        Wire.write(0x6B); // PWR_MGMT_1
        Wire.write(0x00); // Wake up MPU-6050
        byte err = Wire.endTransmission();
        if (err == 0) {
            _mpuAddr = testAddr;
            found = true;
            break;
        }
    }

    if (!found) {
        _mpuConnected = false;
        return false;
    }

    // Configure DLPF (~44Hz low-pass filter to reject chassis vibration)
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x1A); // CONFIG
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro config: ±500 deg/s (65.5 LSB / deg/s)
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x08);
    Wire.endTransmission();

    // Accel config: ±2g (16384 LSB / g)
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(0x00);
    Wire.endTransmission();

    _mpuConnected = true;
    Serial.printf("[MPU6050] Connected & initialized on I2C (SDA=%d, SCL=%d, Addr=0x%02X)\n",
                  PIN_I2C_SDA, PIN_I2C_SCL, _mpuAddr);
    return true;
#else
    _mpuConnected = false;
    return false;
#endif
}

void SensorManager::_pollMpu6050(float dt) {
    if (!_mpuConnected) return;

    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x3B); // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        _mpuConnected = false;
        return;
    }

    // Request 14 bytes (Accel 6B + Temp 2B + Gyro 6B)
    if (Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)14) == 14) {
        int16_t rawAx = (Wire.read() << 8) | Wire.read();
        int16_t rawAy = (Wire.read() << 8) | Wire.read();
        int16_t rawAz = (Wire.read() << 8) | Wire.read();
        int16_t rawTemp = (Wire.read() << 8) | Wire.read();
        int16_t rawGx = (Wire.read() << 8) | Wire.read();
        int16_t rawGy = (Wire.read() << 8) | Wire.read();
        int16_t rawGz = (Wire.read() << 8) | Wire.read();

        // Convert to engineering units (±2g -> 16384 LSB/g, ±500 dps -> 65.5 LSB/dps)
        _accelX = (float)rawAx / 16384.0f;
        _accelY = (float)rawAy / 16384.0f;
        _accelZ = (float)rawAz / 16384.0f;
        _tempC = ((float)rawTemp / 340.0f) + 36.53f;
        _gyroX = (float)rawGx / 65.5f;
        _gyroY = (float)rawGy / 65.5f;
        _gyroZ = (float)rawGz / 65.5f;

        // Compute Pitch and Roll tilt from accelerometer (degrees)
        float pitchRad = atan2(-_accelX, sqrt(_accelY * _accelY + _accelZ * _accelZ));
        float rollRad = atan2(_accelY, _accelZ);
        _pitchDeg = pitchRad * (180.0f / 3.14159265f);
        _rollDeg = rollRad * (180.0f / 3.14159265f);

        // Deadband gyro Z noise (< 0.5 deg/s) to prevent stationary drift
        float gzRate = (fabs(_gyroZ) > 0.5f) ? _gyroZ : 0.0f;
        if (dt > 0.0f && dt < 1.0f) {
            _yawDeg += gzRate * dt;
            // Wrap to 0.0 - 360.0
            while (_yawDeg >= 360.0f) _yawDeg -= 360.0f;
            while (_yawDeg < 0.0f) _yawDeg += 360.0f;
        }
    }
}

