"""
BANTAY-SUNOG Temporal Confirmation Tracker

Implements a sliding-window state machine that prevents false alarms
by requiring consistent positive fire detections across multiple
consecutive processed frames before confirming a fire event.

State transitions::

    IDLE ──(init)──► NO_FIRE
    NO_FIRE ──(detection)──► FIRE_CANDIDATE
    FIRE_CANDIDATE ──(≥ N positives in window)──► FIRE_CONFIRMED
    FIRE_CANDIDATE ──(falls below threshold)──► NO_FIRE
    FIRE_CONFIRMED ──(no detection for timeout)──► FIRE_LOST
    FIRE_LOST ──(reported)──► NO_FIRE

    Any ──(camera fault)──► CAMERA_ERROR
    Any ──(model fault)──► MODEL_ERROR
    Any ──(serial fault)──► SERIAL_ERROR
"""

import time
from collections import deque
from typing import List, Tuple

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_vision


# Re-export Detection for convenience; imported by main.py
from bantay_sunog.detection.fire_detector import Detection


class ConfirmationTracker:
    """
    Temporal fire-detection validator.

    Parameters are read from :mod:`bantay_sunog.config`.
    """

    def __init__(self) -> None:
        self._window_size: int = config.CONFIRMATION_WINDOW_SIZE
        self._required: int = config.CONFIRMATION_REQUIRED_POSITIVES
        self._loss_timeout: float = config.FIRE_LOSS_TIMEOUT

        # Sliding window of booleans (True = fire detected in that frame)
        self._window: deque = deque(maxlen=self._window_size)

        # State
        self._state: str = config.DetectionState.IDLE
        self._prev_state: str = config.DetectionState.IDLE

        # Timing
        self._last_detection_time: float = 0.0

        # Best detection from most recent frame
        self._best_detection: Detection | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self, detections: List[Detection]
    ) -> Tuple[str, str | None, Detection | None]:
        """
        Process detections from the current frame and advance the state
        machine.

        Args:
            detections: List of ``Detection`` objects from the current frame
                        (may be empty).

        Returns:
            A 3-tuple ``(current_state, changed_from_or_None, best_detection_or_None)``.

            * ``changed_from`` is the previous state if a transition occurred,
              otherwise ``None``.
            * ``best_detection`` is the highest-confidence detection from the
              current frame (or the last confirmed one if still in
              ``FIRE_CONFIRMED`` state), or ``None``.
        """
        now = time.monotonic()
        has_detection = len(detections) > 0
        old_state = self._state

        # Pick best detection (highest confidence)
        if has_detection:
            best = max(detections, key=lambda d: d.confidence)
            self._best_detection = best
            self._last_detection_time = now
        else:
            best = None

        # Record in sliding window
        self._window.append(has_detection)
        positives = sum(self._window)

        # ---- State transitions ----

        if self._state == config.DetectionState.IDLE:
            self._transition(config.DetectionState.NO_FIRE)

        if self._state == config.DetectionState.NO_FIRE:
            if has_detection:
                self._transition(config.DetectionState.FIRE_CANDIDATE)
                log_vision("Fire candidate detected.")

        elif self._state == config.DetectionState.FIRE_CANDIDATE:
            if positives >= self._required:
                self._transition(config.DetectionState.FIRE_CONFIRMED)
                log_vision(
                    f"Fire confirmed ({positives}/{self._window_size} frames)."
                )
            elif positives == 0:
                # All recent frames negative — drop back
                self._transition(config.DetectionState.NO_FIRE)
                log_vision("Fire candidate cleared (all frames negative).")

        elif self._state == config.DetectionState.FIRE_CONFIRMED:
            if not has_detection:
                elapsed = now - self._last_detection_time
                if elapsed >= self._loss_timeout:
                    self._transition(config.DetectionState.FIRE_LOST)
                    log_vision("Fire target lost.")

        elif self._state == config.DetectionState.FIRE_LOST:
            # After one cycle of reporting, return to NO_FIRE
            self._transition(config.DetectionState.NO_FIRE)
            self._best_detection = None

        # Determine whether a state change occurred
        changed_from = old_state if self._state != old_state else None

        # For FIRE_CONFIRMED, always return the best (may be from last frame)
        out_detection = best if best is not None else (
            self._best_detection if self._state == config.DetectionState.FIRE_CONFIRMED else None
        )

        return self._state, changed_from, out_detection

    def set_error_state(self, state: str) -> str | None:
        """
        Force the tracker into an error state.

        Args:
            state: One of ``CAMERA_ERROR``, ``MODEL_ERROR``, ``SERIAL_ERROR``.

        Returns:
            The previous state (for logging), or ``None`` if already in that
            error state.
        """
        if self._state == state:
            return None
        old = self._state
        self._state = state
        self._prev_state = old
        return old

    def clear_error(self) -> None:
        """Return to NO_FIRE from an error state."""
        if self._state in (
            config.DetectionState.CAMERA_ERROR,
            config.DetectionState.MODEL_ERROR,
            config.DetectionState.SERIAL_ERROR,
        ):
            self._state = config.DetectionState.NO_FIRE
            self._window.clear()
            self._best_detection = None

    def get_state(self) -> str:
        """Return the current detection state string."""
        return self._state

    def reset(self) -> None:
        """Reset the tracker to NO_FIRE with empty window."""
        self._state = config.DetectionState.NO_FIRE
        self._prev_state = config.DetectionState.NO_FIRE
        self._window.clear()
        self._best_detection = None
        self._last_detection_time = 0.0

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _transition(self, new_state: str) -> None:
        self._prev_state = self._state
        self._state = new_state
