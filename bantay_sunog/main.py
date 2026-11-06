"""
BANTAY-SUNOG Main Application

Raspberry Pi 3B Real-Time Fire Detection Subsystem.

Coordinates video acquisition, local flame detection, multi-frame
confirmation, wired UART communication with ESP32-WROOM, command-line
telemetry logging, and performance monitoring.
"""

import argparse
import os
import signal
import sys
import time
from typing import Optional

# Ensure parent directory is in sys.path so 'import bantay_sunog' works
# regardless of current working directory or execution method
_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_PARENT_DIR = os.path.dirname(_APP_DIR)
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)
if _APP_DIR not in sys.path:
    sys.path.insert(0, _APP_DIR)

import cv2
import numpy as np

from bantay_sunog import config
from bantay_sunog.camera.usb_camera import USBCamera
from bantay_sunog.communication import protocol
from bantay_sunog.communication.uart_sender import UARTSender
from bantay_sunog.detection.confirmation import ConfirmationTracker
from bantay_sunog.detection.fire_detector import Detection, FireDetector
from bantay_sunog.detection.model_loader import load_model
from bantay_sunog.monitoring.logger import (
    get_logger,
    log_banner,
    log_camera,
    log_error,
    log_model,
    log_system,
    log_uart,
    log_vision,
    set_startup_mode,
)
from bantay_sunog.monitoring.performance import PerformanceMonitor
from bantay_sunog.monitoring.web_stream import WebStreamServer


class BantaySunogApp:
    """Main application orchestrator."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self._running = False

        # Apply CLI overrides to config
        if self.args.debug:
            config.LOG_LEVEL = "DEBUG"
        if self.args.preview:
            config.PREVIEW_ENABLED = True
        if self.args.simulate:
            config.DETECTION_BACKEND = "simulation"
        elif self.args.detector:
            config.DETECTION_BACKEND = self.args.detector

        # Core subsystems
        self.camera: Optional[USBCamera] = None
        self.detector: Optional[FireDetector] = None
        self.tracker: Optional[ConfirmationTracker] = None
        self.uart: Optional[UARTSender] = None
        self.perf: Optional[PerformanceMonitor] = None
        self.web_server: Optional[WebStreamServer] = None

        if getattr(self.args, "web", False):
            config.WEB_STREAM_ENABLED = True
        if getattr(self.args, "port", None):
            config.WEB_STREAM_PORT = self.args.port

        # Confirmed target telemetry timer
        self._last_confirmed_tx_time: float = 0.0

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(self) -> bool:
        """
        Initialize all subsystems and print startup telemetry.
        
        Returns True if system is ready to operate.
        """
        # 1. Start logging & banner
        get_logger()
        log_banner()

        # Check Pi hardware environment
        is_arm = os.uname().machine.startswith(("arm", "aarch64")) if hasattr(os, "uname") else False
        if is_arm:
            log_system("Raspberry Pi architecture detected.")
        else:
            log_system("Host environment detected (Simulation/Development mode).")

        # 2. Camera Initialization
        self.camera = USBCamera(device=getattr(self.args, "camera", None))
        if not self.args.simulate:
            if not self.camera.open():
                log_error("Could not initialize USB webcam. Retrying in background...")
        else:
            log_camera("Simulation mode active: synthetic video frames enabled.")

        # 3. Model & Detector Initialization
        log_model(f"Loading fire-detection backend: {config.DETECTION_BACKEND}...")
        try:
            model_info = load_model(config.DETECTION_BACKEND)
            self.detector = FireDetector(model_info)
        except Exception as exc:
            log_error(f"Failed to load detection backend: {exc}")
            return False

        # 4. Confirmation Tracker Initialization
        self.tracker = ConfirmationTracker()

        # 5. UART Communication Initialization
        self.uart = UARTSender()
        self.uart.connect()

        # 6. Performance Monitor
        self.perf = PerformanceMonitor()
        self.perf.camera_resolution = (
            self.camera.get_resolution_str() if self.camera.is_opened() else "640x480"
        )
        self.perf.serial_status = "Connected" if self.uart.is_connected() else "Disconnected"

        # 7. Web Stream Preview Server (for viewing on laptop browser)
        if config.WEB_STREAM_ENABLED:
            self.web_server = WebStreamServer(
                host=config.WEB_STREAM_HOST,
                port=config.WEB_STREAM_PORT,
            )
            self.web_server.start()

        # End of startup sequence
        set_startup_mode(False)
        log_system("Fire-detection system READY.")
        return True

    # ------------------------------------------------------------------
    # Main Execution Loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Run the continuous fire-detection loop."""
        self._running = True
        frames_processed = 0

        while self._running:
            if getattr(self.args, "max_frames", 0) and frames_processed >= self.args.max_frames:
                log_system(f"Reached maximum frame limit ({self.args.max_frames}). Stopping.")
                if self.perf:
                    self.perf.print_report()
                break
            frames_processed += 1

            self.perf.frame_start()

            # 1. Acquire Frame
            frame = self._acquire_frame()
            if frame is None:
                self._handle_camera_drop()
                time.sleep(0.1)
                continue

            # In simulation mode, add small delay to simulate ~20 FPS acquisition
            if self.args.simulate:
                time.sleep(0.05)

            frame_h, frame_w = frame.shape[:2]

            # 2. Detect Flames
            self.perf.inference_start()
            try:
                detections = self.detector.detect(frame)
            except Exception as exc:
                log_error(f"Inference error: {exc}")
                detections = []
                self.tracker.set_error_state(config.DetectionState.MODEL_ERROR)
                self.uart.send(protocol.build_error("MODEL", str(exc)))
            self.perf.inference_end()

            # 3. Temporal Confirmation Validation
            state, state_changed_from, best_detection = self.tracker.update(detections)
            self.perf.current_state = state
            self.perf.serial_status = "Connected" if self.uart.is_connected() else "Disconnected"

            if best_detection is not None:
                self.perf.record_detection()
                log_vision(
                    f"Fire target at ({best_detection.normalized_x:.3f}, {best_detection.normalized_y:.3f}) "
                    f"conf={best_detection.confidence:.2f} bbox=[{best_detection.x},{best_detection.y},"
                    f"{best_detection.width},{best_detection.height}]",
                    level=10,  # DEBUG
                )

            # 4. Handle State-Driven UART Messaging
            self._process_uart_events(state, state_changed_from, best_detection, frame_w, frame_h)

            # Check for incoming bidirectional telemetry from ESP32
            if self.uart is not None:
                self.uart.read_telemetry()

            # 5. Visual Preview & Web Streaming (Laptop)
            if config.PREVIEW_ENABLED or config.WEB_STREAM_ENABLED:
                annotated = self._annotate_frame(frame, detections, state)

                # Local GUI display (if display connected)
                if config.PREVIEW_ENABLED:
                    cv2.imshow(config.PREVIEW_WINDOW_NAME, annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        log_system("Exit requested from preview window.")
                        self.stop()
                        break

                # Web streaming to laptop browser
                if config.WEB_STREAM_ENABLED and self.web_server:
                    mem = self.perf.get_memory_usage() or {"used_mb": 0.0, "total_mb": 0.0}
                    telemetry = {
                        "state": state,
                        "fps": self.perf.get_fps(),
                        "target_x": best_detection.normalized_x if best_detection else None,
                        "target_y": best_detection.normalized_y if best_detection else None,
                        "confidence": best_detection.confidence if best_detection else 0.0,
                        "cpu": self.perf.get_cpu_percent() or 0.0,
                        "ram_used": mem.get("used_mb", 0.0),
                        "ram_total": mem.get("total_mb", 0.0),
                        "temp": self.perf.get_cpu_temperature() or 0.0,
                        "serial_status": "Connected" if self.uart.is_connected() else "Disconnected",
                        "uart_sent": self.perf.total_uart_sent,
                        "last_uart_tx": getattr(self.uart, "_last_sent_message", None),
                        "esp32_telemetry": self.uart.get_last_esp32_telemetry(),
                    }
                    self.web_server.update_frame(annotated, telemetry)

            # 6. Periodic Performance Telemetry
            if self.perf.should_report():
                self.perf.print_report()

    # ------------------------------------------------------------------
    # Frame Acquisition Helpers
    # ------------------------------------------------------------------

    def _acquire_frame(self) -> Optional[np.ndarray]:
        """Obtain real or simulated video frame."""
        if self.args.simulate:
            return self._create_synthetic_frame()

        if not self.camera.is_opened():
            self.camera.reconnect()
            if not self.camera.is_opened():
                return None

        success, frame = self.camera.read()
        if not success:
            return None
        return frame

    def _create_synthetic_frame(self) -> np.ndarray:
        """
        Generate an animated simulated indoor room frame with floor perspective,
        ambient lighting, and reticle so the user sees an active visual feed
        even before a physical USB webcam is plugged in.
        """
        h, w = config.CAMERA_HEIGHT, config.CAMERA_WIDTH
        frame = np.full((h, w, 3), (25, 30, 38), dtype=np.uint8)

        # Draw indoor room perspective (floor and ceiling lines)
        cv2.line(frame, (0, int(h * 0.7)), (w, int(h * 0.7)), (45, 55, 70), 2)
        for x_step in range(0, w, 80):
            cv2.line(frame, (x_step, int(h * 0.7)), (int((x_step - w / 2) * 1.8 + w / 2), h), (35, 45, 58), 1)

        # Draw simulated room door/window outline
        cv2.rectangle(frame, (int(w * 0.75), int(h * 0.25)), (int(w * 0.92), int(h * 0.7)), (50, 60, 75), 2)

        # Draw optical center target reticle
        cx, cy = w // 2, h // 2
        cv2.drawMarker(frame, (cx, cy), (70, 85, 105), cv2.MARKER_CROSS, 24, 1)
        cv2.circle(frame, (cx, cy), 35, (70, 85, 105), 1)

        # Simulation watermark banner
        cv2.putText(
            frame,
            "[SIMULATION MODE] No USB Webcam Detected",
            (int(w * 0.05), int(h * 0.88)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 200, 255),
            2,
        )
        cv2.putText(
            frame,
            "Plug in USB webcam to /dev/video0 for live physical video",
            (int(w * 0.05), int(h * 0.94)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (160, 175, 195),
            1,
        )
        return frame

    def _handle_camera_drop(self) -> None:
        """Handle webcam disconnection or frame drops."""
        if self.camera.has_critical_failure():
            prev = self.tracker.set_error_state(config.DetectionState.CAMERA_ERROR)
            if prev != config.DetectionState.CAMERA_ERROR:
                err_msg = protocol.build_error("CAMERA", "Camera frame acquisition failed")
                self.uart.send(err_msg, force=True)
            self.camera.reconnect()

    # ------------------------------------------------------------------
    # UART Event Processing
    # ------------------------------------------------------------------

    def _process_uart_events(
        self,
        state: str,
        state_changed_from: Optional[str],
        best_detection: Optional[Detection],
        frame_w: int,
        frame_h: int,
    ) -> None:
        """Transmit appropriate messages based on state changes and timers."""
        now = time.monotonic()

        # State transition events
        if state_changed_from is not None:
            if state == config.DetectionState.FIRE_CONFIRMED and best_detection:
                self.perf.record_confirmed()
                msg = protocol.build_fire_confirmed(
                    confidence=best_detection.confidence,
                    x=best_detection.normalized_x,
                    y=best_detection.normalized_y,
                    bbox=[best_detection.x, best_detection.y, best_detection.width, best_detection.height],
                    frame_dims=[frame_w, frame_h],
                )
                if self.uart.send(msg, force=True):
                    self.perf.record_uart_sent()
                    self._last_confirmed_tx_time = now
                else:
                    self.perf.record_uart_failure()

            elif state == config.DetectionState.FIRE_CANDIDATE and best_detection:
                msg = protocol.build_fire_candidate(
                    confidence=best_detection.confidence,
                    x=best_detection.normalized_x,
                    y=best_detection.normalized_y,
                )
                if self.uart.send(msg):
                    self.perf.record_uart_sent()

            elif state == config.DetectionState.FIRE_LOST:
                msg = protocol.build_fire_lost(reason="detection_timeout")
                if self.uart.send(msg, force=True):
                    self.perf.record_uart_sent()

            elif state == config.DetectionState.NO_FIRE and state_changed_from in (
                config.DetectionState.FIRE_CANDIDATE,
                config.DetectionState.FIRE_LOST,
            ):
                msg = protocol.build_no_fire()
                if self.uart.send(msg):
                    self.perf.record_uart_sent()

        # Periodic updates while remaining in confirmed state
        elif state == config.DetectionState.FIRE_CONFIRMED and best_detection:
            if now - self._last_confirmed_tx_time >= config.CONFIRMED_UPDATE_INTERVAL:
                msg = protocol.build_fire_confirmed(
                    confidence=best_detection.confidence,
                    x=best_detection.normalized_x,
                    y=best_detection.normalized_y,
                    bbox=[best_detection.x, best_detection.y, best_detection.width, best_detection.height],
                    frame_dims=[frame_w, frame_h],
                )
                if self.uart.send(msg, force=True):
                    self.perf.record_uart_sent()
                    self._last_confirmed_tx_time = now
                else:
                    self.perf.record_uart_failure()

    # ------------------------------------------------------------------
    # Preview Window
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Frame Annotation Helper
    # ------------------------------------------------------------------

    def _annotate_frame(
        self,
        frame: np.ndarray,
        detections: list,
        state: str,
    ) -> np.ndarray:
        """Render detection overlays, target crosshairs, and status bar onto frame."""
        annotated = frame.copy()

        # Draw simulated flame graphic inside bounding box when in simulation mode
        if getattr(self.args, "simulate", False):
            for det in detections:
                fx = int(det.x + det.width // 2)
                fy = int(det.y + det.height - 12)
                flicker = (int(time.time() * 20) % 7) - 3
                # Outer red/orange glow
                cv2.ellipse(
                    annotated,
                    (fx, fy),
                    (max(8, det.width // 3 + flicker), max(16, det.height // 2 + flicker)),
                    0, 0, 360, (0, 69, 255), -1,
                )
                # Mid orange body
                cv2.ellipse(
                    annotated,
                    (fx, fy),
                    (max(5, det.width // 4), max(10, det.height // 3)),
                    0, 0, 360, (0, 165, 255), -1,
                )
                # Yellow flame core
                cv2.ellipse(
                    annotated,
                    (fx, fy + 4),
                    (max(3, det.width // 6), max(6, det.height // 5)),
                    0, 0, 360, (0, 255, 255), -1,
                )

        # Draw detections
        for det in detections:
            color = (0, 0, 255) if state == config.DetectionState.FIRE_CONFIRMED else (0, 165, 255)
            cv2.rectangle(
                annotated,
                (det.x, det.y),
                (det.x + det.width, det.y + det.height),
                color,
                2,
            )
            # Center target crosshair
            cx, cy = int(det.center_x), int(det.center_y)
            cv2.drawMarker(annotated, (cx, cy), color, cv2.MARKER_CROSS, 16, 2)

            label = f"{det.class_name}: {det.confidence:.2f} ({det.normalized_x:.2f},{det.normalized_y:.2f})"
            cv2.putText(
                annotated,
                label,
                (det.x, max(15, det.y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # Status text overlay
        fps = self.perf.get_fps()
        status_text = f"FPS: {fps:.1f} | STATE: {state} | UART: {'OK' if self.uart.is_connected() else 'DISCONNECTED'}"
        cv2.putText(
            annotated,
            status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return annotated

    # ------------------------------------------------------------------
    # Lifecycle & Teardown
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Gracefully stop application."""
        self._running = False
        log_system("Stopping BANTAY-SUNOG fire-detection subsystem...")

        if self.web_server:
            self.web_server.stop()
        if self.camera:
            self.camera.release()
        if self.uart:
            self.uart.disconnect()
        if config.PREVIEW_ENABLED:
            cv2.destroyAllWindows()

        log_system("Shutdown complete.")


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="BANTAY-SUNOG: Raspberry Pi 3B Real-Time Fire Detection System",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Enable live OpenCV display window (requires display/X11)",
    )
    parser.add_argument(
        "--camera",
        default=None,
        help="USB webcam device index or path (default: auto-probe /dev/video1, /dev/video0, etc.)",
    )
    parser.add_argument(
        "--web",
        "--stream",
        action="store_true",
        help="Enable live web browser preview (view stream and telemetry from laptop browser)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=config.WEB_STREAM_PORT,
        help="Web stream port (default: 5000)",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run in simulation mode without physical camera or model",
    )
    parser.add_argument(
        "--detector",
        choices=["tflite", "hsv_fallback", "simulation"],
        help="Override detection backend",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable DEBUG level logging output",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after processing N frames (useful for benchmarking and tests)",
    )
    return parser.parse_args()


def main() -> int:
    """Application entry point."""
    args = parse_arguments()
    app = BantaySunogApp(args)

    # Register OS signal handlers for graceful exit
    def signal_handler(sig, frame):
        log_system(f"Received signal {sig}. Initiating shutdown...")
        app.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not app.initialize():
        log_error("Subsystem initialization failed. Exiting.")
        return 1

    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
