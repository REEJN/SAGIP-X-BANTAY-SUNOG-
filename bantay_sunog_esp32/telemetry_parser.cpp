#include "telemetry_parser.h"

TelemetryParser::TelemetryParser() : _rxIndex(0), _newMsgParsed(false), _cmdCallback(nullptr) {
    _currentTarget.reset();
}

void TelemetryParser::begin() {
    // Configure UART2 with Pi baud rate and GPIO pins
    Serial2.begin(UART_PI_BAUD, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);
}

bool TelemetryParser::update() {
    _newMsgParsed = false;

    while (Serial2.available() > 0) {
        char c = (char)Serial2.read();

        if (c == '\n' || c == '\r') {
            if (_rxIndex > 0) {
                _rxBuffer[_rxIndex] = '\0';
                _parseJsonLine(_rxBuffer);
                _rxIndex = 0;
            }
        } else {
            if (_rxIndex < sizeof(_rxBuffer) - 1) {
                _rxBuffer[_rxIndex++] = c;
            } else {
                // Buffer overflow protection: reset buffer
                _rxIndex = 0;
            }
        }
    }

    return _newMsgParsed;
}

void TelemetryParser::_parseJsonLine(const char* line) {
    if (!line || strlen(line) == 0) return;

    // Skip leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    // Check for raw mission control commands
    if (strcasecmp(line, "START_PATROL") == 0 || strcasecmp(line, "START") == 0 || strcasecmp(line, "patrol") == 0) {
        _currentTarget.event = EVENT_START_PATROL;
        _currentTarget.last_update_ms = millis();
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("start");
        return;
    }
    if (strcasecmp(line, "PAUSE") == 0 || strcasecmp(line, "IDLE") == 0 || strcasecmp(line, "STANDBY") == 0 || strcasecmp(line, "stop") == 0) {
        _currentTarget.event = EVENT_PAUSE_STANDBY;
        _currentTarget.last_update_ms = millis();
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("pause");
        return;
    }
    if (strcasecmp(line, "PI_READY") == 0 || strcasecmp(line, "ready") == 0 || strcasecmp(line, "handshake") == 0) {
        _currentTarget.event = EVENT_PI_READY;
        _currentTarget.last_update_ms = millis();
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("handshake");
        return;
    }

    // Raw non-JSON bench and teleop commands ('w', 's', 'a', 'd', 'x', 'f', 'u', 'j', 'h', 'sweep', 'tilt ...', etc.)
    if (line[0] != '{') {
        if (_cmdCallback) {
            _cmdCallback(line);
        }
        return;
    }

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<384> doc;
#endif

    DeserializationError error = deserializeJson(doc, line);
    if (error) {
        // If not valid JSON, treat as raw command if possible
        if (_cmdCallback) _cmdCallback(line);
        return;
    }

    uint32_t now = millis();

    // Check type or cmd fields for handshake/mission control
    const char* typeStr = doc["type"] | doc["cmd"] | "";
    if (strcasecmp(typeStr, "PI_READY") == 0) {
        _currentTarget.event = EVENT_PI_READY;
        _currentTarget.last_update_ms = now;
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("handshake");
        return;
    }
    if (strcasecmp(typeStr, "START_PATROL") == 0 || strcasecmp(typeStr, "START") == 0) {
        _currentTarget.event = EVENT_START_PATROL;
        _currentTarget.last_update_ms = now;
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("start");
        return;
    }
    if (strcasecmp(typeStr, "PAUSE") == 0 || strcasecmp(typeStr, "IDLE") == 0 || strcasecmp(typeStr, "STANDBY") == 0) {
        _currentTarget.event = EVENT_PAUSE_STANDBY;
        _currentTarget.last_update_ms = now;
        _newMsgParsed = true;
        if (_cmdCallback) _cmdCallback("pause");
        return;
    }

    // Direct JSON command parameter: {"cmd":"w"} or {"type":"BENCH_COMMAND","cmd":"f"}
    if (doc.containsKey("cmd")) {
        const char* c = doc["cmd"];
        if (c && _cmdCallback) {
            _cmdCallback(c);
            return;
        }
    }

    const char* eventStr = doc["event"];
    if (!eventStr) {
        return;
    }

    if (strcmp(eventStr, "FIRE_CONFIRMED") == 0) {
        _currentTarget.event = EVENT_FIRE_CONFIRMED;
        _currentTarget.confidence = doc["confidence"] | 0.0f;
        _currentTarget.x = doc["x"] | 0.5f;
        _currentTarget.y = doc["y"] | 0.5f;

        JsonArray bbox = doc["bbox"];
        if (bbox && bbox.size() == 4) {
            _currentTarget.bbox[0] = bbox[0];
            _currentTarget.bbox[1] = bbox[1];
            _currentTarget.bbox[2] = bbox[2];
            _currentTarget.bbox[3] = bbox[3];
        }

        JsonArray frame = doc["frame"];
        if (frame && frame.size() == 2) {
            _currentTarget.frame_dims[0] = frame[0];
            _currentTarget.frame_dims[1] = frame[1];
        }

        _currentTarget.last_update_ms = now;
        _currentTarget.valid = true;
        _newMsgParsed = true;

    } else if (strcmp(eventStr, "FIRE_CANDIDATE") == 0) {
        _currentTarget.event = EVENT_FIRE_CANDIDATE;
        _currentTarget.confidence = doc["confidence"] | 0.0f;
        _currentTarget.x = doc["x"] | 0.5f;
        _currentTarget.y = doc["y"] | 0.5f;
        _currentTarget.last_update_ms = now;
        _currentTarget.valid = true;
        _newMsgParsed = true;

    } else if (strcmp(eventStr, "NO_FIRE") == 0) {
        _currentTarget.event = EVENT_NO_FIRE;
        _currentTarget.valid = false;
        _newMsgParsed = true;

    } else if (strcmp(eventStr, "FIRE_LOST") == 0) {
        _currentTarget.event = EVENT_FIRE_LOST;
        _currentTarget.valid = false;
        _newMsgParsed = true;

    } else if (strcmp(eventStr, "ERROR") == 0) {
        _currentTarget.event = EVENT_ERROR;
        _currentTarget.valid = false;
        _newMsgParsed = true;
    }
}

bool TelemetryParser::isTargetFresh() const {
    if (!_currentTarget.valid) {
        return false;
    }
    return (millis() - _currentTarget.last_update_ms) <= TARGET_LOSS_TIMEOUT_MS;
}

void TelemetryParser::invalidateTarget() {
    _currentTarget.valid = false;
}

void TelemetryParser::injectTarget(VisionEvent evt, float conf, float x, float y) {
    _currentTarget.event = evt;
    _currentTarget.confidence = conf;
    _currentTarget.x = x;
    _currentTarget.y = y;
    _currentTarget.bbox[0] = 160;
    _currentTarget.bbox[1] = 120;
    _currentTarget.bbox[2] = 120;
    _currentTarget.bbox[3] = 160;
    _currentTarget.frame_dims[0] = 640;
    _currentTarget.frame_dims[1] = 480;
    _currentTarget.last_update_ms = millis();
    _currentTarget.valid = (evt == EVENT_FIRE_CONFIRMED || evt == EVENT_FIRE_CANDIDATE);
    _newMsgParsed = true;
}
