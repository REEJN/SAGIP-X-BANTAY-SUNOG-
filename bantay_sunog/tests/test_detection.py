"""
Unit tests for detection.fire_detector and model_loader.
"""

import unittest
import numpy as np

from bantay_sunog import config
from bantay_sunog.detection.fire_detector import Detection, FireDetector
from bantay_sunog.detection.model_loader import load_model


class TestDetection(unittest.TestCase):
    """Test Detection dataclass and detector backends."""

    def test_detection_dataclass_coordinates(self):
        # Frame 640x480, box at x=100, y=100, w=200, h=100
        # Center = (200, 150)
        # Normalized: 200/640 = 0.3125, 150/480 = 0.3125
        det = Detection(
            class_name="fire",
            confidence=0.88,
            x=100,
            y=100,
            width=200,
            height=100,
            frame_width=640,
            frame_height=480,
        )
        self.assertEqual(det.center_x, 200.0)
        self.assertEqual(det.center_y, 150.0)
        self.assertAlmostEqual(det.normalized_x, 0.3125, places=4)
        self.assertAlmostEqual(det.normalized_y, 0.3125, places=4)

    def test_detection_coordinate_clamping(self):
        # Box extending out of frame
        det = Detection(
            class_name="fire",
            confidence=0.9,
            x=600,
            y=450,
            width=200,
            height=200,
            frame_width=640,
            frame_height=480,
        )
        self.assertLessEqual(det.normalized_x, 1.0)
        self.assertLessEqual(det.normalized_y, 1.0)
        self.assertGreaterEqual(det.normalized_x, 0.0)
        self.assertGreaterEqual(det.normalized_y, 0.0)

    def test_simulation_detector(self):
        info = load_model("simulation")
        detector = FireDetector(info)
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Run several times; with probability > 0 it should generate detections
        config.SIMULATION_FIRE_PROBABILITY = 1.0
        detections = detector.detect(dummy_frame)
        self.assertGreater(len(detections), 0)
        det = detections[0]
        self.assertEqual(det.class_name, "fire")
        self.assertTrue(0.0 <= det.normalized_x <= 1.0)
        self.assertTrue(0.0 <= det.normalized_y <= 1.0)

    def test_hsv_fallback_detector_clean_frame(self):
        info = load_model("hsv_fallback")
        detector = FireDetector(info)
        # Completely black image should yield no detections
        black_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = detector.detect(black_frame)
        self.assertEqual(len(detections), 0)


if __name__ == "__main__":
    unittest.main()
