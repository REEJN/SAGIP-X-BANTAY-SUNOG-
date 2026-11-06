#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "types.h"

/**
 * Serializes and transmits periodic robot status telemetry back to the Raspberry Pi.
 */
class TelemetryReporter {
public:
    TelemetryReporter();

    /**
     * Transmit current telemetry if the reporting interval has elapsed.
     */
    void sendTelemetry(const RobotTelemetry& telemetry, bool force = false);

private:
    uint32_t _lastSendMs;
};
