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
    StaticJsonDocument<256> doc;
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

    // Serialize to Serial2 (Raspberry Pi UART) and Serial (USB debug port)
    serializeJson(doc, Serial2);
    Serial2.print('\n');
    serializeJson(doc, Serial);
    Serial.print('\n');
}
