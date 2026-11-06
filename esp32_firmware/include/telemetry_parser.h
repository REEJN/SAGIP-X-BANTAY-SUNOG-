#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "types.h"

/**
 * Non-blocking UART2 receiver and JSON parser for Raspberry Pi vision messages.
 */
class TelemetryParser {
public:
    TelemetryParser();

    /**
     * Initialize HardwareSerial UART2 on assigned RX/TX pins.
     */
    void begin();

    /**
     * Poll incoming serial stream, assemble newline-delimited lines, and parse JSON.
     * @return True if a new valid message was parsed this cycle, False otherwise.
     */
    bool update();

    const VisionTarget& getTarget() const { return _currentTarget; }
    bool isTargetFresh() const;
    void invalidateTarget();
    void injectTarget(VisionEvent evt, float conf, float x, float y);

private:
    void _parseJsonLine(const char* line);

    VisionTarget _currentTarget;
    char _rxBuffer[384];
    size_t _rxIndex;
    bool _newMsgParsed;
};
