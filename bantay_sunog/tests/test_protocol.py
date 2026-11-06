"""
Unit tests for communication.protocol module.
"""

import json
import unittest

from bantay_sunog.communication import protocol


class TestProtocol(unittest.TestCase):
    """Test JSON message construction, serialization, and validation."""

    def test_build_fire_confirmed(self):
        msg = protocol.build_fire_confirmed(
            confidence=0.9123,
            x=0.6666,
            y=0.4218,
            bbox=[180, 90, 120, 160],
            frame_dims=[640, 480],
            timestamp=1730000000,
        )
        self.assertEqual(msg["event"], "FIRE_CONFIRMED")
        self.assertEqual(msg["confidence"], 0.91)
        self.assertEqual(msg["x"], 0.667)
        self.assertEqual(msg["y"], 0.422)
        self.assertEqual(msg["bbox"], [180, 90, 120, 160])
        self.assertEqual(msg["frame"], [640, 480])
        self.assertEqual(msg["timestamp"], 1730000000)
        self.assertTrue(protocol.validate_message(msg))

    def test_build_fire_candidate(self):
        msg = protocol.build_fire_candidate(
            confidence=0.741,
            x=0.5512,
            y=0.4891,
            timestamp=1730000001,
        )
        self.assertEqual(msg["event"], "FIRE_CANDIDATE")
        self.assertEqual(msg["confidence"], 0.74)
        self.assertEqual(msg["x"], 0.551)
        self.assertEqual(msg["y"], 0.489)
        self.assertEqual(msg["timestamp"], 1730000001)
        self.assertTrue(protocol.validate_message(msg))

    def test_build_no_fire(self):
        msg = protocol.build_no_fire(timestamp=1730000002)
        self.assertEqual(msg["event"], "NO_FIRE")
        self.assertEqual(msg["timestamp"], 1730000002)
        self.assertTrue(protocol.validate_message(msg))

    def test_build_fire_lost(self):
        msg = protocol.build_fire_lost(reason="detection_timeout", timestamp=1730000003)
        self.assertEqual(msg["event"], "FIRE_LOST")
        self.assertEqual(msg["reason"], "detection_timeout")
        self.assertTrue(protocol.validate_message(msg))

    def test_build_error(self):
        msg = protocol.build_error("CAMERA", "Frame acquisition failed", timestamp=1730000004)
        self.assertEqual(msg["event"], "ERROR")
        self.assertEqual(msg["source"], "CAMERA")
        self.assertEqual(msg["message"], "Frame acquisition failed")
        self.assertTrue(protocol.validate_message(msg))

    def test_serialize(self):
        msg = protocol.build_no_fire(timestamp=1730000000)
        serialized = protocol.serialize(msg)
        self.assertTrue(serialized.endswith(b"\n"))
        # Parse back
        parsed = json.loads(serialized.decode("utf-8").strip())
        self.assertEqual(parsed["event"], "NO_FIRE")

    def test_validate_message_invalid_inputs(self):
        self.assertFalse(protocol.validate_message(None))
        self.assertFalse(protocol.validate_message("invalid"))
        self.assertFalse(protocol.validate_message({}))
        self.assertFalse(protocol.validate_message({"event": "UNKNOWN_EVENT"}))
        # Coordinates out of bounds
        self.assertFalse(
            protocol.validate_message({
                "event": "FIRE_CONFIRMED",
                "confidence": 1.5,
                "x": 0.5,
                "y": 0.5,
                "bbox": [0, 0, 10, 10],
                "frame": [640, 480],
            })
        )


if __name__ == "__main__":
    unittest.main()
