"""
BANTAY-SUNOG Serial Connector Bridge
Handles wired/USB serial UART communication with the ESP32-WROOM controller.
Includes automatic COM port scanning, telemetry ingestion, and mock fallback.
"""

import json
import logging
import os
import socket
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
        self.is_physical = False

        # Wi-Fi Bridge to Raspberry Pi 3B (/dev/serial0)
        self.wifi_host = os.environ.get("BANTAY_PI_HOST", "192.168.137.200")
        self.wifi_port = int(os.environ.get("BANTAY_PI_PORT", "8888"))
        self.sock: Optional[socket.socket] = None
        self.is_wifi_bridge = False
        self.cached_pi_metrics: dict = {}

        self.on_telemetry_received: Optional[Callable[[dict], None]] = None
        self.on_log_received: Optional[Callable[[dict], None]] = None
        self.on_hardware_status_received: Optional[Callable[[dict], None]] = None
        self.on_pi_log_received: Optional[Callable[[dict], None]] = None
        self.last_telemetry: dict = {}

    @staticmethod
    def list_ports() -> List[str]:
        """List available serial/COM ports."""
        if not SERIAL_AVAILABLE:
            return ["MOCK_PORT_COM_SIM"]
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def is_physical_connected(self) -> bool:
        """Returns True when either a physical COM port or Pi Wi-Fi bridge is active."""
        if self.is_wifi_bridge and self.sock and self.is_physical:
            return True
        return bool(self.ser and self.ser.is_open and self.is_physical)

    def connect_physical(self, port: str) -> bool:
        """Explicitly connect to a physical serial port."""
        if not SERIAL_AVAILABLE:
            return False
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(port, self.baudrate, timeout=1.0)
            self.port = port
            self.is_physical = True
            self.mock_mode = False
            self.running = True
            self.read_thread = threading.Thread(target=self._read_worker, daemon=True)
            self.read_thread.start()
            logger.info(f"[Serial] Connected to PHYSICAL ESP32 on {port} at {self.baudrate} baud.")
            if self.on_log_received:
                self.on_log_received({
                    "source": "ESP32",
                    "level": "INFO",
                    "message": f"[HARDWARE LINK] Connected to physical ESP32 on {port} at {self.baudrate} baud",
                    "timestamp": time.time(),
                    "is_physical": True
                })
            return True
        except Exception as e:
            logger.warning(f"[Serial] Could not connect to physical port {port}: {e}")
            self.ser = None
            self.is_physical = False
            return False

    def connect_wifi_bridge(self, host: Optional[str] = None, port: Optional[int] = None) -> bool:
        """Connect to Raspberry Pi 3B UART Wi-Fi Bridge stream."""
        target_host = host or self.wifi_host
        target_port = port or self.wifi_port
        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.5)
            s.connect((target_host, target_port))
            s.settimeout(None)
            self.sock = s
            self.is_wifi_bridge = True
            self.is_physical = True
            self.mock_mode = False
            self.running = True
            self.port = f"Raspberry Pi 3B ({target_host} -> /dev/serial0)"
            
            self.read_thread = threading.Thread(target=self._wifi_read_worker, daemon=True)
            self.read_thread.start()
            logger.info(f"[Serial/Wi-Fi] Connected to Raspberry Pi 3B Wi-Fi Bridge at {target_host}:{target_port}")
            if self.on_log_received:
                self.on_log_received({
                    "source": "ESP32",
                    "level": "INFO",
                    "message": f"[WI-FI BRIDGE LINK] Connected to ESP32 via Raspberry Pi 3B ({target_host} -> /dev/serial0)",
                    "timestamp": time.time(),
                    "is_physical": True
                })
            return True
        except Exception as e:
            logger.debug(f"[Serial/Wi-Fi] Could not connect to Pi Wi-Fi Bridge at {target_host}:{target_port}: {e}")
            self.sock = None
            self.is_wifi_bridge = False
            return False

    def auto_scan_and_connect(self) -> bool:
        """Scan available connections: first probe Pi Wi-Fi bridge, then local USB COM ports."""
        if self.is_physical_connected():
            return True

        # 1. Probe Raspberry Pi 3B Wi-Fi bridge first (primary link without ESP32 ports on laptop)
        if self.connect_wifi_bridge():
            return True

        # 2. Probe local USB serial ports
        if SERIAL_AVAILABLE:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                dev = p.device
                desc = p.description or ""
                logger.info(f"[Serial Auto-Scan] Probing local port {dev} ({desc})...")
                if self.connect_physical(dev):
                    return True
        return False

    def connect(self, port: Optional[str] = None) -> bool:
        if port:
            self.port = port

        # Check for Wi-Fi bridge specified in port parameter
        if self.port and (":" in self.port or "192.168" in self.port or "wifi" in self.port.lower()):
            clean = self.port.replace("http://", "").replace("ws://", "")
            parts = clean.split(":")
            h = parts[0]
            p = int(parts[1]) if len(parts) > 1 else self.wifi_port
            if self.connect_wifi_bridge(h, p):
                return True

        # If a real local COM port was provided, attempt physical connection
        if self.port and not self.port.startswith("MOCK"):
            if self.connect_physical(self.port):
                return True

        # Auto probe Wi-Fi bridge first before falling back to mock
        if self.auto_scan_and_connect():
            return True

        if self.mock_mode or not self.port or self.port.startswith("MOCK"):
            logger.info("[Serial] Connecting in MOCK / Virtual loopback mode.")
            self.running = True
            self.is_physical = False
            self.read_thread = threading.Thread(target=self._mock_worker, daemon=True)
            self.read_thread.start()
            return True

        return False

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self.is_wifi_bridge = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        logger.info("[Serial] Disconnected.")

    def send_line(self, data_str: str) -> bool:
        """Send raw string line to ESP32 via Wi-Fi bridge or local serial."""
        if self.is_wifi_bridge and self.sock:
            try:
                payload = (data_str.strip() + "\n").encode("utf-8")
                self.sock.sendall(payload)
                if self.on_log_received:
                    self.on_log_received({
                        "source": "ESP32",
                        "level": "INFO",
                        "message": f"[WI-FI TX -> ESP32] {data_str.strip()}",
                        "timestamp": time.time(),
                        "is_physical": True
                    })
                return True
            except Exception as e:
                logger.error(f"[Wi-Fi Bridge TX Error] {e}")
                return False

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

    def _wifi_read_worker(self):
        """Worker thread to read live telemetry & Pi metrics over Wi-Fi bridge TCP socket."""
        try:
            f = self.sock.makefile('r', encoding='utf-8', errors='replace')
        except Exception as e:
            logger.error(f"[Wi-Fi Bridge] Cannot makefile on socket: {e}")
            return

        while self.running and self.sock:
            try:
                line = f.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue

                if line.startswith("{") and line.endswith("}"):
                    try:
                        data = json.loads(line)
                        msg_type = data.get("type", "")

                        if msg_type == "HARDWARE_STATUS":
                            if "pi_metrics" in data:
                                self.cached_pi_metrics = data["pi_metrics"]
                            if self.on_hardware_status_received:
                                self.on_hardware_status_received(data)
                            continue

                        if msg_type == "PI_LOG" and "data" in data:
                            if self.on_pi_log_received:
                                self.on_pi_log_received(data["data"])
                            continue

                        # Raw ESP32 telemetry packet
                        if "robot_state" in data or "dist_front" in data:
                            data["is_physical"] = True
                            data["_is_mock"] = False
                            data["port"] = self.port
                            self.last_telemetry = data
                            if self.on_telemetry_received:
                                self.on_telemetry_received(data)
                            if self.on_log_received:
                                self.on_log_received({
                                    "source": "ESP32",
                                    "level": "DEBUG",
                                    "message": f"[TELEMETRY RX] {line}",
                                    "timestamp": time.time(),
                                    "is_physical": True
                                })
                    except json.JSONDecodeError:
                        pass
                else:
                    if self.on_log_received:
                        self.on_log_received({
                            "source": "ESP32",
                            "level": "INFO",
                            "message": line,
                            "timestamp": time.time(),
                            "is_physical": True
                        })
            except Exception as e:
                logger.warning(f"[Wi-Fi Bridge Read Error] {e}")
                break

        logger.info("[Wi-Fi Bridge] Connection closed.")
        self.is_physical = False
        self.is_wifi_bridge = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

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
                        data["is_physical"] = True
                        data["_is_mock"] = False
                        self.last_telemetry = data
                        if self.on_telemetry_received:
                            self.on_telemetry_received(data)
                        if self.on_log_received:
                            self.on_log_received({
                                "source": "ESP32",
                                "level": "DEBUG",
                                "message": f"[TELEMETRY RX] {line}",
                                "timestamp": time.time(),
                                "is_physical": True
                            })
                    except json.JSONDecodeError:
                        logger.debug(f"[Serial Non-JSON] {line}")
                        if self.on_log_received:
                            self.on_log_received({
                                "source": "ESP32",
                                "level": "WARN",
                                "message": f"[CORRUPT] {line}",
                                "timestamp": time.time(),
                                "is_physical": True
                            })
                else:
                    logger.debug(f"[ESP32 Debug] {line}")
                    if self.on_log_received:
                        level = "WARN" if "ERR" in line or "FAIL" in line else "INFO"
                        self.on_log_received({
                            "source": "ESP32",
                            "level": level,
                            "message": line,
                            "timestamp": time.time(),
                            "is_physical": True
                        })
            except Exception as e:
                logger.error(f"[Serial Read Error / Disconnected] {e}")
                self.is_physical = False
                if self.on_log_received:
                    self.on_log_received({
                        "source": "ESP32",
                        "level": "ERROR",
                        "message": f"[SERIAL DISCONNECTED] {e}",
                        "timestamp": time.time(),
                        "is_physical": False
                    })
                time.sleep(0.5)

    def _mock_worker(self):
        """Simulates periodic ESP32 telemetry in mock mode for testing without hardware."""
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
                "_is_mock": True,
                "is_physical": False
            }
            self.last_telemetry = mock_data
            if self.on_telemetry_received:
                self.on_telemetry_received(mock_data)

            # NOTE: NO fake logs are emitted here. Real logs ONLY come from physical serial RX in _read_worker.
            uptime += 500
            time.sleep(0.5)



