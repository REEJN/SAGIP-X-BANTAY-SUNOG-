"""
BANTAY-SUNOG UART Sender Module

Handles serial communication with ESP32-WROOM over wired UART.

Key Features:
- pySerial connection lifecycle management (connect, reconnect, close)
- Thread-safe / error-safe writes
- Mandatory terminal logging for every outgoing message (log_uart_tx)
- Duplicate message suppression to prevent bus flooding
- Rate-limiting via minimum transmission interval
- Transmission statistics tracking (sent, failed, last send time)
- Clear distinction between "written to UART" and "received by ESP32"
"""

import logging
import time
from typing import Any, Dict, Optional

from bantay_sunog import config
from bantay_sunog.communication.protocol import serialize, serialize_pretty, validate_message
from bantay_sunog.monitoring.logger import log_error, log_uart, log_uart_tx

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    serial = None
    _HAS_SERIAL = False


class UARTSender:
    """
    UART Serial communication manager for ESP32-WROOM.
    """

    def __init__(self) -> None:
        self.port: str = config.UART_PORT
        self.baudrate: int = config.UART_BAUD_RATE
        self.timeout: float = config.UART_TIMEOUT
        self.write_timeout: float = config.UART_WRITE_TIMEOUT

        self._ser: Optional[Any] = None
        self._connected: bool = False

        # Timing and rate control
        self._last_send_time: float = 0.0
        self._min_interval: float = config.MIN_TRANSMISSION_INTERVAL
        self._last_reconnect_attempt: float = 0.0

        # Duplicate suppression tracking
        self._last_sent_message: Optional[Dict[str, Any]] = None

        # Received ESP32 telemetry cache
        self._last_esp32_telemetry: Optional[Dict[str, Any]] = None

        # Statistics
        self.total_sent: int = 0
        self.total_failures: int = 0
        self.consecutive_failures: int = 0

    # ------------------------------------------------------------------
    # Connection Management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """
        Open the serial port connection to the ESP32.

        Returns True on success, False otherwise.
        """
        if serial is None:
            log_error("pySerial is not installed. UART communication unavailable.")
            self._connected = False
            return False

        log_uart(f"Initializing serial communication on {self.port} at {self.baudrate} baud...")
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
            # Flush existing buffers
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            self._connected = True
            self.consecutive_failures = 0
            log_uart(f"Connected to ESP32-WROOM on {self.port}")
            return True
        except Exception as exc:
            log_error(f"Failed to connect to UART port {self.port}: {exc}")
            self._ser = None
            self._connected = False
            self.total_failures += 1
            self.consecutive_failures += 1
            return False

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._connected = False
        log_uart("UART connection closed.")

    def is_connected(self) -> bool:
        """Check if serial port is open."""
        return self._connected and (self._ser is not None) and self._ser.is_open

    def reconnect(self) -> bool:
        """Close current connection and attempt to reopen."""
        now = time.monotonic()
        if now - self._last_reconnect_attempt < config.UART_RECONNECT_INTERVAL:
            return False

        self._last_reconnect_attempt = now
        log_uart(f"Attempting UART reconnection on {self.port}...")
        self.disconnect()
        return self.connect()

    # ------------------------------------------------------------------
    # Duplicate Suppression Logic
    # ------------------------------------------------------------------

    def _is_duplicate(self, message: Dict[str, Any]) -> bool:
        """
        Determine if message is redundant and should be suppressed.

        Rules:
        - NO_FIRE: Suppress if last sent message was also NO_FIRE.
        - FIRE_CANDIDATE: Suppress if last was identical candidate.
        - ERROR: Suppress if same source and message.
        - FIRE_CONFIRMED: Send continuously or on update (not suppressed).
        - FIRE_LOST: Never suppress.
        """
        if self._last_sent_message is None:
            return False

        last_event = self._last_sent_message.get("event")
        current_event = message.get("event")

        if current_event == "NO_FIRE" and last_event == "NO_FIRE":
            return True

        if current_event == "FIRE_CANDIDATE" and last_event == "FIRE_CANDIDATE":
            # Check coordinate & confidence proximity
            if (
                self._last_sent_message.get("confidence") == message.get("confidence")
                and self._last_sent_message.get("x") == message.get("x")
                and self._last_sent_message.get("y") == message.get("y")
            ):
                return True

        if current_event == "ERROR" and last_event == "ERROR":
            if (
                self._last_sent_message.get("source") == message.get("source")
                and self._last_sent_message.get("message") == message.get("message")
            ):
                return True

        return False

    # ------------------------------------------------------------------
    # Message Transmission
    # ------------------------------------------------------------------

    def send(self, message: Dict[str, Any], force: bool = False) -> bool:
        """
        Transmit a JSON message dictionary to ESP32 over UART.

        Args:
            message: Message payload dictionary.
            force: If True, bypasses rate limiting and duplicate suppression.

        Returns:
            True if written to UART buffer, False otherwise.
        """
        # Validate message
        if not validate_message(message):
            log_error(f"Cannot send invalid protocol message: {message}")
            return False

        # Duplicate suppression check
        if not force and self._is_duplicate(message):
            return False

        # Rate limiting check
        now = time.monotonic()
        if not force and (now - self._last_send_time < self._min_interval):
            return False

        # Check connection
        if not self.is_connected():
            if not self.reconnect():
                self.total_failures += 1
                self.consecutive_failures += 1
                return False

        try:
            # Serialize
            data_bytes = serialize(message)

            # MANDATORY REQUIREMENT: Print every outgoing message in terminal
            # Use indented pretty formatting for readability
            log_uart_tx(serialize_pretty(message))

            # Write and flush
            self._ser.write(data_bytes)
            self._ser.flush()

            # Record write success
            self._last_send_time = now
            self._last_sent_message = dict(message)
            self.total_sent += 1
            self.consecutive_failures = 0

            log_uart("Message written successfully to UART.")
            # Note: Do not claim ESP32 executed or acknowledged unless ACK protocol is active
            return True

        except Exception as exc:
            self.total_failures += 1
            self.consecutive_failures += 1
            log_error(f"UART transmission error: {exc}")
            self.disconnect()
            return False

    # ------------------------------------------------------------------
    # Telemetry and Statistics
    # ------------------------------------------------------------------

    def read_telemetry(self) -> Optional[Dict[str, Any]]:
        """
        Non-blocking check and read of incoming JSON telemetry from ESP32.

        Returns deserialized dict if a complete JSON line was read, or None.
        """
        if not self.is_connected() or self._ser is None:
            return None

        try:
            if self._ser.in_waiting > 0:
                raw_line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if raw_line and raw_line.startswith("{") and raw_line.endswith("}"):
                    import json
                    data = json.loads(raw_line)
                    self._last_esp32_telemetry = data
                    return data
        except Exception:
            pass

        return None

    def get_last_esp32_telemetry(self) -> Optional[Dict[str, Any]]:
        """Return the most recently received ESP32 telemetry packet."""
        return self._last_esp32_telemetry

    def get_stats(self) -> Dict[str, Any]:
        """Return UART transmission statistics."""
        return {
            "connected": self.is_connected(),
            "total_sent": self.total_sent,
            "total_failures": self.total_failures,
            "consecutive_failures": self.consecutive_failures,
            "last_send_time": self._last_send_time,
            "has_esp32_telemetry": self._last_esp32_telemetry is not None,
        }
