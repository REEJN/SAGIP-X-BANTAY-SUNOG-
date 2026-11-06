"""
BANTAY-SUNOG Performance Monitor

Tracks FPS, inference latency, CPU/RAM usage, and detection statistics.
Periodically emits a summary report to the logger.
"""

import time
from collections import deque
from typing import Optional

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_perf

# Try importing psutil — gracefully degrade if unavailable
try:
    import psutil
    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False


class PerformanceMonitor:
    """
    Lightweight performance tracker for Raspberry Pi 3B.

    Tracks:
    - Frames per second (rolling window average)
    - Per-frame inference time (ms)
    - CPU and memory usage (via psutil)
    - Detection and transmission statistics
    """

    def __init__(self) -> None:
        # FPS tracking
        self._frame_times: deque = deque(maxlen=config.PERF_FPS_WINDOW)
        self._last_frame_time: float = 0.0

        # Inference timing
        self._inference_times: deque = deque(maxlen=config.PERF_FPS_WINDOW)
        self._inference_start: float = 0.0

        # Statistics
        self.total_frames: int = 0
        self.total_detections: int = 0
        self.total_confirmed: int = 0
        self.total_uart_sent: int = 0
        self.total_uart_failures: int = 0

        # Periodic report
        self._last_report_time: float = time.monotonic()

        # Current state cache (set externally)
        self.current_state: str = config.DetectionState.IDLE
        self.camera_resolution: str = "N/A"
        self.serial_status: str = "N/A"

        # Initialise psutil CPU tracking
        if _HAS_PSUTIL:
            psutil.cpu_percent(interval=None)  # Prime the counter

    # ----- FPS tracking -----

    def frame_start(self) -> None:
        """Mark the start of a new frame processing cycle."""
        now = time.monotonic()
        if self._last_frame_time > 0:
            self._frame_times.append(now - self._last_frame_time)
        self._last_frame_time = now
        self.total_frames += 1

    def get_fps(self) -> float:
        """Return the rolling average FPS."""
        if len(self._frame_times) < 2:
            return 0.0
        avg_interval = sum(self._frame_times) / len(self._frame_times)
        if avg_interval <= 0:
            return 0.0
        return 1.0 / avg_interval

    # ----- Inference timing -----

    def inference_start(self) -> None:
        """Mark the start of model inference."""
        self._inference_start = time.monotonic()

    def inference_end(self) -> None:
        """Mark the end of model inference and record the duration."""
        if self._inference_start > 0:
            elapsed_ms = (time.monotonic() - self._inference_start) * 1000.0
            self._inference_times.append(elapsed_ms)
            self._inference_start = 0.0

    def get_avg_inference_ms(self) -> float:
        """Return the rolling average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)

    def get_last_inference_ms(self) -> float:
        """Return the most recent inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return self._inference_times[-1]

    # ----- Statistics -----

    def record_detection(self) -> None:
        """Increment the detection counter (any fire candidate)."""
        self.total_detections += 1

    def record_confirmed(self) -> None:
        """Increment the confirmed fire counter."""
        self.total_confirmed += 1

    def record_uart_sent(self) -> None:
        """Increment the UART message-sent counter."""
        self.total_uart_sent += 1

    def record_uart_failure(self) -> None:
        """Increment the UART transmission-failure counter."""
        self.total_uart_failures += 1

    # ----- System resources -----

    def get_cpu_percent(self) -> Optional[float]:
        """Return CPU usage percentage, or None if psutil is unavailable."""
        if not _HAS_PSUTIL:
            return None
        try:
            return psutil.cpu_percent(interval=config.PERF_CPU_INTERVAL)
        except Exception:
            return None

    def get_memory_usage(self) -> Optional[dict]:
        """
        Return memory usage information, or None if psutil is unavailable.

        Returns dict with keys: used_mb, total_mb, percent
        """
        if not _HAS_PSUTIL:
            return None
        try:
            mem = psutil.virtual_memory()
            return {
                "used_mb": round(mem.used / (1024 * 1024), 1),
                "total_mb": round(mem.total / (1024 * 1024), 1),
                "percent": mem.percent,
            }
        except Exception:
            return None

    def get_cpu_temperature(self) -> Optional[float]:
        """Return CPU temperature in Celsius, or None if unavailable."""
        if not _HAS_PSUTIL:
            return None
        try:
            temps = psutil.sensors_temperatures()
            if "cpu_thermal" in temps and temps["cpu_thermal"]:
                return round(temps["cpu_thermal"][0].current, 1)
        except Exception:
            pass
        # Fallback: read sysfs directly
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return round(float(f.read().strip()) / 1000.0, 1)
        except Exception:
            return None

    # ----- Periodic report -----

    def should_report(self) -> bool:
        """Return True if enough time has passed for a periodic report."""
        return (time.monotonic() - self._last_report_time) >= config.PERF_REPORT_INTERVAL

    def print_report(self) -> None:
        """Print a performance summary to the logger."""
        self._last_report_time = time.monotonic()

        fps = self.get_fps()
        inf_ms = self.get_avg_inference_ms()
        cpu = self.get_cpu_percent()
        mem = self.get_memory_usage()
        temp = self.get_cpu_temperature()

        log_perf(f"--- Performance Report ---")
        log_perf(f"FPS: {fps:.1f}  |  Avg inference: {inf_ms:.1f} ms")
        log_perf(f"Camera: {self.camera_resolution}  |  State: {self.current_state}")
        log_perf(f"Serial: {self.serial_status}")
        log_perf(
            f"Detections: {self.total_detections}  |  "
            f"Confirmed: {self.total_confirmed}  |  "
            f"UART sent: {self.total_uart_sent}  |  "
            f"UART fails: {self.total_uart_failures}"
        )

        if cpu is not None:
            cpu_str = f"CPU: {cpu:.1f}%"
        else:
            cpu_str = "CPU: N/A (psutil not available)"

        if mem is not None:
            mem_str = f"RAM: {mem['used_mb']}/{mem['total_mb']} MB ({mem['percent']}%)"
        else:
            mem_str = "RAM: N/A"

        if temp is not None:
            temp_str = f"Temp: {temp}°C"
        else:
            temp_str = "Temp: N/A"

        log_perf(f"{cpu_str}  |  {mem_str}  |  {temp_str}")
        log_perf(f"Total frames processed: {self.total_frames}")
        log_perf(f"--------------------------")
