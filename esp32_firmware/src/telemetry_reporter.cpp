#include "telemetry_reporter.h"

TelemetryReporter::TelemetryReporter() : _lastSendMs(0) {}

void TelemetryReporter::sendTelemetry(const RobotTelemetry& telemetry, bool force) {
    uint32_t now = millis();
    if (!force && (now - _lastSendMs < TELEMETRY_SEND_INTERVAL_MS)) {
        return;
    }
    _lastSendMs = now;

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<512> doc;
#endif

    doc["robot_state"] = robotStateToString(telemetry.state);
    doc["dist_front"] = round(telemetry.dist_front_cm * 10.0f) / 10.0f;
    doc["dist_left"] = round(telemetry.dist_left_cm * 10.0f) / 10.0f;
    doc["dist_right"] = round(telemetry.dist_right_cm * 10.0f) / 10.0f;
    doc["flame_detected"] = telemetry.flame_sensor_triggered;
    doc["tilt_angle"] = telemetry.tilt_angle_deg;
    doc["solenoid_active"] = telemetry.solenoid_active;
    doc["left_pwm"] = telemetry.left_motor_pwm;
    doc["right_pwm"] = telemetry.right_motor_pwm;
    doc["uptime"] = telemetry.uptime_ms;

    // MPU-6050 6-Axis Motion & Attitude
    doc["mpu_connected"] = telemetry.mpu_connected;
    if (telemetry.mpu_connected) {
        doc["accel_x"] = round(telemetry.accel_x * 100.0f) / 100.0f;
        doc["accel_y"] = round(telemetry.accel_y * 100.0f) / 100.0f;
        doc["accel_z"] = round(telemetry.accel_z * 100.0f) / 100.0f;
        doc["gyro_x"] = round(telemetry.gyro_x * 10.0f) / 10.0f;
        doc["gyro_y"] = round(telemetry.gyro_y * 10.0f) / 10.0f;
        doc["gyro_z"] = round(telemetry.gyro_z * 10.0f) / 10.0f;
        doc["pitch"] = round(telemetry.pitch_deg * 10.0f) / 10.0f;
        doc["roll"] = round(telemetry.roll_deg * 10.0f) / 10.0f;
        doc["yaw"] = round(telemetry.yaw_deg * 10.0f) / 10.0f;
        doc["temp_c"] = round(telemetry.temp_c * 10.0f) / 10.0f;
    }

    // Serialize to Serial2 (Raspberry Pi UART) and Serial (USB debug port)
    serializeJson(doc, Serial2);
    Serial2.print('\n');
    serializeJson(doc, Serial);
    Serial.print('\n');
}
