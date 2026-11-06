"""
BANTAY-SUNOG Serial Connector Bridge
Handles wired/USB serial UART communication with the ESP32-WROOM controller.
Includes automatic COM port scanning, telemetry ingestion, and mock fallback.
"""

import json
import logging
import threading
import time
from typing import Callable, List, Optional

logger = logging.getLogger("SerialConnector")

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    logger.warning("pyserial is not installed; running in Mock Loopback mode only.")


class SerialConnector:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, mock_mode: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.mock_mode = mock_mode or not SERIAL_AVAILABLE
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.read_thread: Optional[threading.Thread] = None

        self.on_telemetry_received: Optional[Callable[[dict], None]] = None
        self.last_telemetry: dict = {}

    @staticmethod
    def list_ports() -> List[str]:
        """List available serial/COM ports."""
        if not SERIAL_AVAILABLE:
            return ["MOCK_PORT_COM_SIM"]
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port: Optional[str] = None) -> bool:
        if port:
            self.port = port

        if self.mock_mode or not self.port or self.port.startswith("MOCK"):
            logger.info("[Serial] Connecting in MOCK / Virtual loopback mode.")
            self.running = True
            self.read_thread = threading.Thread(target=self._mock_worker, daemon=True)
            self.read_thread.start()
            return True

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_worker, daemon=True)
            self.read_thread.start()
            logger.info(f"[Serial] Connected to physical ESP32 on {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            logger.error(f"[Serial] Failed to connect to {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        logger.info("[Serial] Disconnected.")

    def send_line(self, data_str: str) -> bool:
        """Send raw string line with newline delimiter to ESP32."""
        if self.mock_mode or not self.ser:
            logger.debug(f"[Serial MOCK TX] -> {data_str}")
            return True

        try:
            payload = (data_str.strip() + "\n").encode("utf-8")
            self.ser.write(payload)
            self.ser.flush()
            return True
        except Exception as e:
            logger.error(f"[Serial TX Error] {e}")
            return False

    def send_json(self, payload: dict) -> bool:
        """Serialize and send structured JSON message."""
        return self.send_line(json.dumps(payload))

    def _read_worker(self):
        buffer = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Check if line is valid JSON telemetry
                if line.startswith("{") and line.endswith("}"):
                    try:
                        data = json.loads(line)
                        self.last_telemetry = data
                        if self.on_telemetry_received:
                            self.on_telemetry_received(data)
                    except json.JSONDecodeError:
                        logger.debug(f"[Serial Non-JSON] {line}")
                else:
                    logger.debug(f"[ESP32 Debug] {line}")
            except Exception as e:
                logger.error(f"[Serial Read Error] {e}")
                time.sleep(0.5)

    def _mock_worker(self):
        """Simulates periodic ESP32 telemetry in mock mode."""
        uptime = 0
        while self.running:
            mock_data = {
                "robot_state": "SEARCHING",
                "dist_front": 120.5,
                "dist_left": 95.0,
                "dist_right": 88.2,
                "flame_detected": False,
                "tilt_angle": 90,
                "solenoid_active": False,
                "left_pwm": 140,
                "right_pwm": 140,
                "uptime": uptime,
                "_is_mock": True
            }
            self.last_telemetry = mock_data
            if self.on_telemetry_received:
                self.on_telemetry_received(mock_data)
            uptime += 500
            time.sleep(0.5)
