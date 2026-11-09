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
        self.on_log_received: Optional[Callable[[dict], None]] = None
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
            if self.on_log_received:
                self.on_log_received({
                    "source": "ESP32",
                    "level": "INFO",
                    "message": f"[UART TX] Dispatched command: {data_str.strip()}",
                    "timestamp": time.time()
                })
            return True

        try:
            payload = (data_str.strip() + "\n").encode("utf-8")
            self.ser.write(payload)
            self.ser.flush()
            if self.on_log_received:
                self.on_log_received({
                    "source": "ESP32",
                    "level": "INFO",
                    "message": f"[UART TX] -> {data_str.strip()}",
                    "timestamp": time.time()
                })
            return True
        except Exception as e:
            logger.error(f"[Serial TX Error] {e}")
            return False

    def send_json(self, payload: dict) -> bool:
        """Serialize and send structured JSON message."""
        return self.send_line(json.dumps(payload))

    def _read_worker(self):
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
                        if self.on_log_received:
                            self.on_log_received({
                                "source": "ESP32",
                                "level": "DEBUG",
                                "message": f"[TELEMETRY RX] {line}",
                                "timestamp": time.time()
                            })
                    except json.JSONDecodeError:
                        logger.debug(f"[Serial Non-JSON] {line}")
                        if self.on_log_received:
                            self.on_log_received({
                                "source": "ESP32",
                                "level": "WARN",
                                "message": f"[CORRUPT] {line}",
                                "timestamp": time.time()
                            })
                else:
                    logger.debug(f"[ESP32 Debug] {line}")
                    if self.on_log_received:
                        level = "WARN" if "ERR" in line or "FAIL" in line else "INFO"
                        self.on_log_received({
                            "source": "ESP32",
                            "level": level,
                            "message": line,
                            "timestamp": time.time()
                        })
            except Exception as e:
                logger.error(f"[Serial Read Error] {e}")
                if self.on_log_received:
                    self.on_log_received({
                        "source": "ESP32",
                        "level": "ERROR",
                        "message": f"[SERIAL ERROR] {e}",
                        "timestamp": time.time()
                    })
                time.sleep(0.5)

    def _mock_worker(self):
        """Simulates periodic ESP32 telemetry & diagnostic logs in mock mode."""
        uptime = 0
        step = 0
        mock_log_templates = [
            "[FSM] State: SEARCHING | Obstacle clearances: F:120cm L:95cm R:88cm",
            "[MOTORS] PWM output updated: PWMA:140 PWMB:140 | STBY: HIGH",
            "[SENSORS] HC-SR04 ping active (3-beam sweep: 40Hz, 10us TRIG pulse)",
            "[UART] RX Heartbeat OK from Pi (/dev/ttyAMA0 @ 115200 8-N-1)",
            "[TURRET] Pitch servo holding level at 90.0° (no flame deflection)",
            "[SOLENOID] Valve safety interlock ENGAGED | State: OFF",
            "[TINYML] Local inference cycle: 0.12ms (ESP32 240MHz Xtensa Dual-Core)",
            "[OPTICAL] LM393 IR Flame Sensor digital read: HIGH (Clear, no IR trigger)"
        ]

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

            # Emit periodic mock log for dashboard console terminal
            if self.on_log_received and step % 2 == 0:
                log_idx = (step // 2) % len(mock_log_templates)
                self.on_log_received({
                    "source": "ESP32",
                    "level": "INFO",
                    "message": mock_log_templates[log_idx],
                    "timestamp": time.time()
                })

            uptime += 500
            step += 1
            time.sleep(0.5)

