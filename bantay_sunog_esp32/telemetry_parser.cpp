#include "telemetry_parser.h"

TelemetryParser::TelemetryParser() : _rxIndex(0), _newMsgParsed(false) {
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
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<384> doc;
#endif

    DeserializationError error = deserializeJson(doc, line);
    if (error) {
        // Malformed line or partial packet - ignore
        return;
    }

    const char* eventStr = doc["event"];
    if (!eventStr) {
        return;
    }

    uint32_t now = millis();

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
