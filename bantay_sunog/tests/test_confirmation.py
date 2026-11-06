"""
Unit tests for detection.confirmation module.
"""

import time
import unittest

from bantay_sunog import config
from bantay_sunog.detection.confirmation import ConfirmationTracker
from bantay_sunog.detection.fire_detector import Detection


def make_dummy_detection(confidence: float = 0.85) -> Detection:
    """Helper to generate a dummy detection."""
    return Detection(
        class_name="fire",
        confidence=confidence,
        x=100,
        y=100,
        width=50,
        height=80,
        frame_width=640,
        frame_height=480,
    )


class TestConfirmation(unittest.TestCase):
    """Test temporal confirmation state machine and sliding window."""

    def setUp(self):
        # Configure test thresholds
        config.CONFIRMATION_WINDOW_SIZE = 5
        config.CONFIRMATION_REQUIRED_POSITIVES = 3
        config.FIRE_LOSS_TIMEOUT = 0.2  # Fast timeout for test
        self.tracker = ConfirmationTracker()

    def test_initial_state(self):
        self.assertEqual(self.tracker.get_state(), config.DetectionState.IDLE)

    def test_transition_to_no_fire(self):
        state, changed_from, det = self.tracker.update([])
        self.assertEqual(state, config.DetectionState.NO_FIRE)
        self.assertEqual(changed_from, config.DetectionState.IDLE)
        self.assertIsNone(det)

    def test_candidate_transition(self):
        # Initialize
        self.tracker.update([])

        # First positive detection
        d = make_dummy_detection(0.75)
        state, changed_from, det = self.tracker.update([d])
        self.assertEqual(state, config.DetectionState.FIRE_CANDIDATE)
        self.assertEqual(changed_from, config.DetectionState.NO_FIRE)
        self.assertIsNotNone(det)
        self.assertEqual(det.confidence, 0.75)

    def test_confirmation_requires_three_of_five(self):
        self.tracker.update([])
        d = make_dummy_detection(0.80)

        # Frame 1: positive -> CANDIDATE
        state, _, _ = self.tracker.update([d])
        self.assertEqual(state, config.DetectionState.FIRE_CANDIDATE)

        # Frame 2: positive -> CANDIDATE (only 2 positives)
        state, _, _ = self.tracker.update([d])
        self.assertEqual(state, config.DetectionState.FIRE_CANDIDATE)

        # Frame 3: positive -> CONFIRMED (3 positives reached!)
        state, changed_from, det = self.tracker.update([d])
        self.assertEqual(state, config.DetectionState.FIRE_CONFIRMED)
        self.assertEqual(changed_from, config.DetectionState.FIRE_CANDIDATE)
        self.assertIsNotNone(det)

    def test_fire_lost_on_timeout(self):
        self.tracker.update([])
        d = make_dummy_detection(0.85)

        # Drive into CONFIRMED
        self.tracker.update([d])
        self.tracker.update([d])
        self.tracker.update([d])
        self.assertEqual(self.tracker.get_state(), config.DetectionState.FIRE_CONFIRMED)

        # Wait longer than loss timeout
        time.sleep(0.25)

        # Process empty frame
        state, changed_from, _ = self.tracker.update([])
        self.assertEqual(state, config.DetectionState.FIRE_LOST)
        self.assertEqual(changed_from, config.DetectionState.FIRE_CONFIRMED)

        # Next update returns to NO_FIRE
        next_state, next_changed, _ = self.tracker.update([])
        self.assertEqual(next_state, config.DetectionState.NO_FIRE)
        self.assertEqual(next_changed, config.DetectionState.FIRE_LOST)

    def test_error_state_handling(self):
        prev = self.tracker.set_error_state(config.DetectionState.CAMERA_ERROR)
        self.assertEqual(self.tracker.get_state(), config.DetectionState.CAMERA_ERROR)
        self.assertEqual(prev, config.DetectionState.IDLE)

        # Clear error
        self.tracker.clear_error()
        self.assertEqual(self.tracker.get_state(), config.DetectionState.NO_FIRE)


if __name__ == "__main__":
    unittest.main()
