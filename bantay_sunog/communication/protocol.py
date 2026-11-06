"""
BANTAY-SUNOG Communication Protocol Module

Standardized JSON message formatting and validation for wired UART
communication between Raspberry Pi 3B and ESP32-WROOM.

Message Specifications:
-----------------------
1. FIRE_CONFIRMED:
   {
     "event": "FIRE_CONFIRMED",
     "confidence": 0.91,
     "x": 0.67,
     "y": 0.42,
     "bbox": [180, 90, 120, 160],
     "frame": [640, 480],
     "timestamp": 1730000000
   }

2. FIRE_CANDIDATE:
   {
     "event": "FIRE_CANDIDATE",
     "confidence": 0.74,
     "x": 0.55,
     "y": 0.48,
     "timestamp": 1730000000
   }

3. NO_FIRE:
   {
     "event": "NO_FIRE",
     "timestamp": 1730000000
   }

4. FIRE_LOST:
   {
     "event": "FIRE_LOST",
     "reason": "detection_timeout",
     "timestamp": 1730000000
   }

5. ERROR:
   {
     "event": "ERROR",
     "source": "CAMERA" | "MODEL" | "UART",
     "message": "Description of error",
     "timestamp": 1730000000
   }
"""

import json
import time
from typing import Any, Dict, List, Optional


def get_timestamp() -> int:
    """Return current Unix timestamp in seconds."""
    return int(time.time())


def build_fire_confirmed(
    confidence: float,
    x: float,
    y: float,
    bbox: List[int],
    frame_dims: List[int],
    timestamp: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Build FIRE_CONFIRMED message payload.

    Args:
        confidence: Detection confidence (0.0 to 1.0).
        x: Normalized target horizontal center (0.0 to 1.0).
        y: Normalized target vertical center (0.0 to 1.0).
        bbox: [x, y, width, height] in pixel coordinates.
        frame_dims: [width, height] frame dimensions.
        timestamp: Optional unix timestamp integer.
    """
    return {
        "event": "FIRE_CONFIRMED",
        "confidence": round(float(confidence), 2),
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "bbox": [int(v) for v in bbox],
        "frame": [int(v) for v in frame_dims],
        "timestamp": timestamp if timestamp is not None else get_timestamp(),
    }


def build_fire_candidate(
    confidence: float,
    x: float,
    y: float,
    timestamp: Optional[int] = None,
) -> Dict[str, Any]:
    """Build FIRE_CANDIDATE message payload."""
    return {
        "event": "FIRE_CANDIDATE",
        "confidence": round(float(confidence), 2),
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "timestamp": timestamp if timestamp is not None else get_timestamp(),
    }


def build_no_fire(timestamp: Optional[int] = None) -> Dict[str, Any]:
    """Build NO_FIRE message payload."""
    return {
        "event": "NO_FIRE",
        "timestamp": timestamp if timestamp is not None else get_timestamp(),
    }


def build_fire_lost(
    reason: str = "detection_timeout",
    timestamp: Optional[int] = None,
) -> Dict[str, Any]:
    """Build FIRE_LOST message payload."""
    return {
        "event": "FIRE_LOST",
        "reason": str(reason),
        "timestamp": timestamp if timestamp is not None else get_timestamp(),
    }


def build_error(
    source: str,
    message: str,
    timestamp: Optional[int] = None,
) -> Dict[str, Any]:
    """Build ERROR message payload."""
    return {
        "event": "ERROR",
        "source": str(source).upper(),
        "message": str(message),
        "timestamp": timestamp if timestamp is not None else get_timestamp(),
    }


def serialize(message: Dict[str, Any]) -> bytes:
    """
    Serialize message dict into compact JSON bytes with a newline delimiter.
    
    Compact serialization avoids unnecessary whitespace bytes over the 115200 UART link.
    """
    compact_json = json.dumps(message, separators=(",", ":"), ensure_ascii=True)
    return (compact_json + "\n").encode("utf-8")


def serialize_pretty(message: Dict[str, Any]) -> str:
    """Serialize message into indented JSON string for clear console logging."""
    return json.dumps(message, indent=2)


def validate_message(message: Any) -> bool:
    """
    Validate that message complies with protocol requirements.

    Returns True if valid, False otherwise.
    """
    if not isinstance(message, dict):
        return False

    event = message.get("event")
    if not event or not isinstance(event, str):
        return False

    valid_events = {"FIRE_CONFIRMED", "FIRE_CANDIDATE", "NO_FIRE", "FIRE_LOST", "ERROR"}
    if event not in valid_events:
        return False

    if event == "FIRE_CONFIRMED":
        if "confidence" not in message or "x" not in message or "y" not in message:
            return False
        if not (0.0 <= message["confidence"] <= 1.0):
            return False
        if not (0.0 <= message["x"] <= 1.0) or not (0.0 <= message["y"] <= 1.0):
            return False
        if "bbox" not in message or len(message["bbox"]) != 4:
            return False
        if "frame" not in message or len(message["frame"]) != 2:
            return False

    elif event == "FIRE_CANDIDATE":
        if "confidence" not in message or "x" not in message or "y" not in message:
            return False
        if not (0.0 <= message["confidence"] <= 1.0):
            return False
        if not (0.0 <= message["x"] <= 1.0) or not (0.0 <= message["y"] <= 1.0):
            return False

    elif event == "FIRE_LOST":
        if "reason" not in message:
            return False

    elif event == "ERROR":
        if "source" not in message or "message" not in message:
            return False

    return True
