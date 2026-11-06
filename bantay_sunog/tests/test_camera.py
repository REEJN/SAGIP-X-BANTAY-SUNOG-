"""
Unit tests for camera.usb_camera module.
"""

import unittest
from unittest.mock import MagicMock, patch
import numpy as np

from bantay_sunog import config
from bantay_sunog.camera.usb_camera import USBCamera


class TestUSBCamera(unittest.TestCase):
    """Test camera initialization, buffer draining, and failure tracking."""

    @patch("cv2.VideoCapture")
    def test_camera_open_success(self, mock_videocapture):
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        dummy_frame = np.ones((480, 640, 3), dtype=np.uint8)
        mock_cap.read.return_value = (True, dummy_frame)
        mock_cap.get.side_effect = lambda prop: 640.0 if prop == 3 else (480.0 if prop == 4 else 30.0)
        mock_videocapture.return_value = mock_cap

        cam = USBCamera(threaded=False)
        success = cam.open()
        self.assertTrue(success)
        self.assertTrue(cam.is_opened())
        self.assertEqual(cam.get_resolution(), (640, 480))
        self.assertEqual(cam.get_resolution_str(), "640x480")
        cam.release()

    @patch("cv2.VideoCapture")
    def test_camera_buffer_drain(self, mock_videocapture):
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True

        # Return dummy frame on read
        dummy_frame = np.ones((480, 640, 3), dtype=np.uint8)
        mock_cap.read.return_value = (True, dummy_frame)
        mock_videocapture.return_value = mock_cap

        cam = USBCamera(threaded=False)
        cam.open()

        success, frame = cam.read()
        self.assertTrue(success)
        self.assertIsNotNone(frame)
        self.assertEqual(mock_cap.read.call_count, config.CAMERA_BUFFER_DRAIN_COUNT)
        cam.release()

    @patch("cv2.VideoCapture")
    def test_camera_threaded_acquisition(self, mock_videocapture):
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True

        dummy_frame = np.full((360, 640, 3), 42, dtype=np.uint8)
        mock_cap.read.return_value = (True, dummy_frame)
        mock_cap.get.side_effect = lambda prop: 640.0 if prop == 3 else (360.0 if prop == 4 else 30.0)
        mock_videocapture.return_value = mock_cap

        cam = USBCamera(threaded=True)
        self.assertTrue(cam.open())

        success, frame = cam.read()
        self.assertTrue(success)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape, (360, 640, 3))
        cam.release()

    @patch("cv2.VideoCapture")
    def test_camera_failure_tracking(self, mock_videocapture):
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        # Always fail read
        mock_cap.read.return_value = (False, None)
        mock_videocapture.return_value = mock_cap

        cam = USBCamera(threaded=False)
        cam.open()

        for _ in range(config.CAMERA_MAX_FAILURES):
            success, frame = cam.read()
            self.assertFalse(success)

        self.assertTrue(cam.has_critical_failure())
        cam.release()


if __name__ == "__main__":
    unittest.main()
