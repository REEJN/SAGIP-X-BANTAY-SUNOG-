"""
Unit tests for communication.uart_sender module.
"""

import time
import unittest
from unittest.mock import MagicMock, patch

from bantay_sunog import config
from bantay_sunog.communication import protocol
from bantay_sunog.communication.uart_sender import UARTSender


class TestUARTSender(unittest.TestCase):
    """Test UART serial communication, duplicate suppression, and stats."""

    def setUp(self):
        config.MIN_TRANSMISSION_INTERVAL = 0.05
        self.sender = UARTSender()

    def _setup_mock_serial(self, mock_serial):
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_serial.Serial.return_value = mock_instance
        mock_serial.EIGHTBITS = 8
        mock_serial.PARITY_NONE = "N"
        mock_serial.STOPBITS_ONE = 1
        return mock_instance

    @patch("bantay_sunog.communication.uart_sender.serial")
    def test_connect_success(self, mock_serial):
        mock_instance = self._setup_mock_serial(mock_serial)
        success = self.sender.connect()
        self.assertTrue(success)
        self.assertTrue(self.sender.is_connected())

    @patch("bantay_sunog.communication.uart_sender.serial")
    def test_send_message_success(self, mock_serial):
        mock_instance = self._setup_mock_serial(mock_serial)
        self.sender.connect()

        msg = protocol.build_no_fire(timestamp=1730000000)
        sent = self.sender.send(msg, force=True)
        self.assertTrue(sent)
        self.assertEqual(self.sender.total_sent, 1)
        mock_instance.write.assert_called_once()

    @patch("bantay_sunog.communication.uart_sender.serial")
    def test_duplicate_suppression_no_fire(self, mock_serial):
        mock_instance = self._setup_mock_serial(mock_serial)
        self.sender.connect()

        msg = protocol.build_no_fire(timestamp=1730000000)

        # First send should succeed
        sent1 = self.sender.send(msg, force=False)
        self.assertTrue(sent1)

        # Sleep past rate limit
        time.sleep(0.06)

        # Second send of NO_FIRE should be suppressed
        sent2 = self.sender.send(msg, force=False)
        self.assertFalse(sent2)
        self.assertEqual(self.sender.total_sent, 1)

    @patch("bantay_sunog.communication.uart_sender.serial")
    def test_confirmed_never_suppressed_if_forced_or_changed(self, mock_serial):
        mock_instance = self._setup_mock_serial(mock_serial)
        self.sender.connect()

        msg1 = protocol.build_fire_confirmed(0.9, 0.5, 0.5, [10, 10, 20, 20], [640, 480])
        sent1 = self.sender.send(msg1, force=True)
        self.assertTrue(sent1)

        time.sleep(0.06)
        msg2 = protocol.build_fire_confirmed(0.92, 0.51, 0.5, [12, 10, 20, 20], [640, 480])
        sent2 = self.sender.send(msg2, force=True)
        self.assertTrue(sent2)
        self.assertEqual(self.sender.total_sent, 2)

    @patch("bantay_sunog.communication.uart_sender.serial")
    def test_read_esp32_telemetry(self, mock_serial):
        mock_instance = self._setup_mock_serial(mock_serial)
        self.sender.connect()

        # Simulate incoming ESP32 packet
        mock_instance.in_waiting = 80
        mock_instance.readline.return_value = (
            b'{"robot_state":"APPROACHING","dist_front":32.5,"dist_left":84.2,'
            b'"dist_right":71.0,"flame_detected":true,"tilt_angle":88,"solenoid_active":false}\n'
        )

        data = self.sender.read_telemetry()
        self.assertIsNotNone(data)
        self.assertEqual(data["robot_state"], "APPROACHING")
        self.assertEqual(data["dist_front"], 32.5)
        self.assertTrue(data["flame_detected"])
        self.assertEqual(self.sender.get_last_esp32_telemetry(), data)


if __name__ == "__main__":
    unittest.main()
