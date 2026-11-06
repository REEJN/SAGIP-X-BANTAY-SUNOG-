"""
BANTAY-SUNOG USB Camera Module

Manages USB webcam acquisition for the Raspberry Pi 3B fire-detection
subsystem. Uses OpenCV VideoCapture with V4L2 backend and dedicated
background capture worker.

Features:
- Configurable device, resolution, FPS, and FourCC format (MJPG / YUYV)
- Threaded background acquisition to keep V4L2 queue drained (0 ms latency, no tearing)
- Low-level stderr silencing for libjpeg non-fatal padding marker warnings
- Automatic device discovery prioritizing video capture nodes over metadata
- Automatic reconnection on camera loss
- Consecutive-failure tracking for CAMERA_ERROR state
"""

import logging
import os
import threading
import time

import cv2
import numpy as np

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_camera, log_error


class USBCamera:
    """USB webcam manager optimised for Raspberry Pi 3B."""

    def __init__(
        self,
        device: str | int | None = None,
        threaded: bool | None = None,
    ) -> None:
        self._device = device if device is not None else config.CAMERA_DEVICE
        self._req_width = config.CAMERA_WIDTH
        self._req_height = config.CAMERA_HEIGHT
        self._req_fps = config.CAMERA_FPS
        self._use_threads = threaded if threaded is not None else getattr(config, "CAMERA_THREADED", True)

        self._cap: cv2.VideoCapture | None = None
        self._actual_width: int = 0
        self._actual_height: int = 0
        self._consecutive_failures: int = 0
        self._opened: bool = False

        self._thread: threading.Thread | None = None
        self._thread_running: bool = False
        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._new_frame_event = threading.Event()

    def _probe_camera_devices(self) -> list:
        """Find candidate camera device paths/indices, prioritizing video capture nodes."""
        candidates = []

        # 1. Configured device if explicitly specified and not auto
        if self._device is not None and str(self._device).lower() != "auto":
            try:
                dev_int = int(self._device)
                if dev_int not in candidates:
                    candidates.append(dev_int)
            except ValueError:
                if self._device not in candidates:
                    candidates.append(str(self._device))

        # 2. Check /dev/v4l/by-id/ on Linux (resolves to hardware webcams)
        # On UVC webcams with multiple endpoints, index0 is video capture, index1 is metadata.
        v4l_by_id = "/dev/v4l/by-id"
        if os.path.isdir(v4l_by_id):
            try:
                entries = sorted(os.listdir(v4l_by_id))
                index0_entries = [e for e in entries if "index0" in e]
                other_entries = [e for e in entries if "index0" not in e]
                for entry in index0_entries + other_entries:
                    full_p = os.path.join(v4l_by_id, entry)
                    real_p = os.path.realpath(full_p)
                    if real_p not in candidates:
                        candidates.append(real_p)
            except Exception:
                pass

        # 3. Check standard /dev/video* nodes (prioritize /dev/video0 over /dev/video1)
        for i in [0, 1, 2]:
            dev_path = f"/dev/video{i}"
            if os.path.exists(dev_path) and dev_path not in candidates:
                candidates.append(dev_path)

        # 4. Fallback integer indices
        for idx in [0, 1, 2]:
            if idx not in candidates:
                candidates.append(idx)
        return candidates

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> bool:
        """
        Open the USB webcam and configure resolution / FPS.
        Automatically probes available camera devices if primary fails.

        Returns True on success, False on failure.
        """
        log_camera("Searching for USB webcam...")
        candidates = self._probe_camera_devices()

        for candidate in candidates:
            try:
                # Prefer V4L2 backend on Linux
                cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture(candidate)

                if cap.isOpened():
                    self._cap = cap
                    self._device = candidate
                    log_camera(f"USB webcam connected: {candidate}")
                    break
            except Exception:
                continue

        if self._cap is None or not self._cap.isOpened():
            log_error(f"No usable USB camera found across probed devices: {candidates}")
            self._opened = False
            return False

        # Set FourCC format (MJPG strongly preferred for low USB bus bandwidth & no tearing)
        fourcc_str = getattr(config, "CAMERA_FOURCC", "MJPG")
        try:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc_str))
        except Exception:
            pass

        # Set requested resolution and FPS
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._req_width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._req_height)
        if self._req_fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, self._req_fps)

        if not self._use_threads:
            # Sequential mode only: reduce buffer size to 1
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Query actual values
        try:
            self._actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self._actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))
        except (ValueError, TypeError):
            self._actual_width = self._req_width
            self._actual_height = self._req_height
            actual_fps = 0.0

        self._opened = True
        self._consecutive_failures = 0

        # Start background capture worker if enabled
        if self._use_threads:
            self._start_capture_thread()

        log_camera(f"USB webcam connected: device {self._device} (threaded={self._use_threads})")
        log_camera(f"Requested: {self._req_width}x{self._req_height} ({fourcc_str})")
        log_camera(f"Actual:    {self._actual_width}x{self._actual_height} @ {actual_fps:.0f} FPS")

        if (self._actual_width != self._req_width or
                self._actual_height != self._req_height):
            log_camera(
                "Camera does not support requested resolution; "
                "using actual resolution.",
                level=logging.WARNING,
            )

        return True

    def _start_capture_thread(self) -> None:
        """Start the background frame acquisition thread."""
        self._thread_running = True
        self._new_frame_event.clear()
        self._thread = threading.Thread(
            target=self._capture_worker,
            name="CameraCaptureWorker",
            daemon=True,
        )
        self._thread.start()

    def _stop_capture_thread(self) -> None:
        """Stop the background frame acquisition thread."""
        self._thread_running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._new_frame_event.clear()

    def _capture_worker(self) -> None:
        """
        Continuously read frames from camera at hardware rate.

        Runs as a daemon thread. Silences C-level stderr during reads to suppress
        spurious non-fatal libjpeg padding marker warnings emitted by camera firmware.
        """
        devnull_fd = None
        saved_stderr_fd = None
        try:
            if os.name == "posix":
                devnull_fd = os.open(os.devnull, os.O_WRONLY)
                saved_stderr_fd = os.dup(2)
                os.dup2(devnull_fd, 2)
        except Exception:
            pass

        try:
            while self._thread_running and self._cap is not None:
                if not self._cap.isOpened():
                    break
                try:
                    res = self._cap.read()
                    if isinstance(res, tuple) and len(res) == 2:
                        ret, frame = res
                    else:
                        ret, frame = False, None
                except Exception:
                    ret, frame = False, None

                if ret and frame is not None:
                    with self._frame_lock:
                        self._latest_frame = frame
                        self._consecutive_failures = 0
                    self._new_frame_event.set()
                else:
                    time.sleep(0.01)
        finally:
            if saved_stderr_fd is not None:
                try:
                    os.dup2(saved_stderr_fd, 2)
                    os.close(saved_stderr_fd)
                except Exception:
                    pass
            if devnull_fd is not None:
                try:
                    os.close(devnull_fd)
                except Exception:
                    pass

    def release(self) -> None:
        """Release the camera resource and stop background capture worker."""
        self._stop_capture_thread()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None
        self._opened = False
        with self._frame_lock:
            self._latest_frame = None
        log_camera("Camera released.")

    def is_opened(self) -> bool:
        """Return True if the camera is currently open and accessible."""
        if not self._opened or self._cap is None:
            return False
        return self._cap.isOpened()

    # ------------------------------------------------------------------
    # Frame acquisition
    # ------------------------------------------------------------------

    def read(self) -> tuple:
        """
        Read the newest available frame from the camera.

        In threaded mode, atomically retrieves the latest frame captured by
        the background worker with zero delay and no frame tearing.
        In sequential mode, drains buffer frames and returns the latest.

        Returns:
            (success: bool, frame: np.ndarray | None)
        """
        if not self.is_opened():
            return False, None

        if self._use_threads:
            # Wait briefly if first frame hasn't arrived yet
            if self._latest_frame is None:
                self._new_frame_event.wait(timeout=0.5)

            with self._frame_lock:
                if self._latest_frame is not None:
                    self._consecutive_failures = 0
                    return True, self._latest_frame.copy()

            self._consecutive_failures += 1
            if self._consecutive_failures >= config.CAMERA_MAX_FAILURES:
                log_error(
                    f"Camera frame acquisition failed "
                    f"({self._consecutive_failures} consecutive failures)."
                )
            return False, None

        # Sequential mode (tests or fallback)
        frame = None
        success = False

        for _ in range(config.CAMERA_BUFFER_DRAIN_COUNT):
            ret, f = self._cap.read()
            if ret and f is not None:
                frame = f
                success = True

        if success:
            self._consecutive_failures = 0
            return True, frame

        self._consecutive_failures += 1
        if self._consecutive_failures >= config.CAMERA_MAX_FAILURES:
            log_error(
                f"Camera frame acquisition failed "
                f"({self._consecutive_failures} consecutive failures)."
            )
        else:
            log_camera(
                f"Frame read failed ({self._consecutive_failures}/"
                f"{config.CAMERA_MAX_FAILURES}).",
                level=logging.DEBUG,
            )
        return False, None

    def has_critical_failure(self) -> bool:
        """Return True if consecutive failures exceed the threshold."""
        return self._consecutive_failures >= config.CAMERA_MAX_FAILURES

    # ------------------------------------------------------------------
    # Reconnection
    # ------------------------------------------------------------------

    def reconnect(self) -> bool:
        """
        Release the camera and attempt to reopen it after a delay.

        Returns True if reconnection succeeds.
        """
        log_camera("Attempting camera reconnection...")
        self.release()
        time.sleep(config.CAMERA_RECONNECT_INTERVAL)
        return self.open()

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def get_resolution(self) -> tuple:
        """Return the actual camera resolution as (width, height)."""
        return self._actual_width, self._actual_height

    def get_resolution_str(self) -> str:
        """Return the actual camera resolution as a 'WxH' string."""
        return f"{self._actual_width}x{self._actual_height}"
