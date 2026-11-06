"""
BANTAY-SUNOG Fire Detector

Core detection engine supporting three backends:

1. **TFLite** – SSD MobileNet-style object detection.
2. **HSV fallback** – Experimental OpenCV colour-space flame detection.
3. **Simulation** – Synthetic detections for testing.

Each backend produces ``Detection`` objects with bounding-box, confidence,
and normalised target coordinates.
"""

import logging
import random
import time
from dataclasses import dataclass, field
from typing import List

import cv2
import numpy as np

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_vision, log_error


# =========================================================================
# Detection dataclass
# =========================================================================

@dataclass
class Detection:
    """A single fire detection result."""

    class_name: str
    confidence: float

    # Bounding box (pixel coordinates in the original frame)
    x: int
    y: int
    width: int
    height: int

    # Frame dimensions
    frame_width: int
    frame_height: int

    # Derived fields (computed in __post_init__)
    center_x: float = field(init=False)
    center_y: float = field(init=False)
    normalized_x: float = field(init=False)
    normalized_y: float = field(init=False)

    def __post_init__(self):
        self.center_x = self.x + (self.width / 2.0)
        self.center_y = self.y + (self.height / 2.0)

        # Normalise to 0.0 – 1.0
        if self.frame_width > 0:
            self.normalized_x = max(0.0, min(1.0, self.center_x / self.frame_width))
        else:
            self.normalized_x = 0.5

        if self.frame_height > 0:
            self.normalized_y = max(0.0, min(1.0, self.center_y / self.frame_height))
        else:
            self.normalized_y = 0.5


# =========================================================================
# Fire Detector
# =========================================================================

class FireDetector:
    """
    Fire-detection engine.

    Instantiate with the ``model_info`` dict returned by
    :func:`bantay_sunog.detection.model_loader.load_model`.
    """

    def __init__(self, model_info: dict) -> None:
        self._backend: str = model_info["backend"]
        self._interpreter = model_info.get("interpreter")
        self._input_details = model_info.get("input_details")
        self._output_details = model_info.get("output_details")
        self._input_shape = model_info.get("input_shape")  # (H, W)
        self._is_quantized: bool = model_info.get("is_quantized", False)
        self._labels: list = model_info.get("labels", ["fire"])

        # HSV fallback state
        self._bg_subtractor = None
        if self._backend == "hsv_fallback":
            self._bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=config.HSV_BG_HISTORY,
                varThreshold=25,
                detectShadows=False,
            )

        log_vision(f"FireDetector initialised (backend={self._backend})")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Run fire detection on a BGR frame.

        Returns a list of ``Detection`` objects (may be empty).
        """
        if self._backend == "tflite":
            return self._detect_tflite(frame)
        elif self._backend == "hsv_fallback":
            return self._detect_hsv(frame)
        elif self._backend == "simulation":
            return self._detect_simulation(frame)
        else:
            log_error(f"Unknown backend: {self._backend}")
            return []

    # ------------------------------------------------------------------
    # TFLite detection
    # ------------------------------------------------------------------

    def _detect_tflite(self, frame: np.ndarray) -> List[Detection]:
        """Run SSD-style TFLite inference."""
        if self._interpreter is None:
            return []

        frame_h, frame_w = frame.shape[:2]
        input_h, input_w = self._input_shape

        # Prepare input tensor
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (input_w, input_h))
        input_data = np.expand_dims(img_resized, axis=0)

        input_dtype = self._input_details[0]["dtype"]
        if input_dtype == np.float32:
            input_data = (np.float32(input_data) - 127.5) / 127.5
        elif input_dtype == np.uint8:
            input_data = input_data.astype(np.uint8)
        elif input_dtype == np.int8:
            input_data = (input_data.astype(np.int32) - 128).astype(np.int8)

        # Invoke
        self._interpreter.set_tensor(self._input_details[0]["index"], input_data)
        self._interpreter.invoke()

        # Parse outputs (SSD format)
        boxes = self._interpreter.get_tensor(self._output_details[0]["index"])[0]
        classes = self._interpreter.get_tensor(self._output_details[1]["index"])[0]
        scores = self._interpreter.get_tensor(self._output_details[2]["index"])[0]
        num_det = int(self._interpreter.get_tensor(self._output_details[3]["index"])[0])

        detections: List[Detection] = []
        for i in range(min(num_det, len(scores))):
            score = float(scores[i])
            if score < config.CONFIDENCE_THRESHOLD:
                continue

            # SSD boxes are [ymin, xmin, ymax, xmax] normalised to 0–1
            ymin = max(0.0, float(boxes[i][0]))
            xmin = max(0.0, float(boxes[i][1]))
            ymax = min(1.0, float(boxes[i][2]))
            xmax = min(1.0, float(boxes[i][3]))

            # Convert to pixel coordinates
            px = int(xmin * frame_w)
            py = int(ymin * frame_h)
            pw = int((xmax - xmin) * frame_w)
            ph = int((ymax - ymin) * frame_h)

            # Determine class name
            cls_idx = int(classes[i])
            cls_name = self._labels[cls_idx] if cls_idx < len(self._labels) else "fire"

            detections.append(Detection(
                class_name=cls_name,
                confidence=round(score, 4),
                x=px, y=py, width=pw, height=ph,
                frame_width=frame_w, frame_height=frame_h,
            ))

        return detections

    # ------------------------------------------------------------------
    # HSV fallback (EXPERIMENTAL)
    # ------------------------------------------------------------------

    def _detect_hsv(self, frame: np.ndarray) -> List[Detection]:
        """
        [EXPERIMENTAL] Detect fire-like regions using HSV colour analysis
        combined with background subtraction for flicker/motion filtering.

        WARNING: This is a heuristic fallback.  It WILL produce false
        positives on orange/red objects, incandescent lights, sunlight, etc.
        Use a trained TFLite model for production deployment.
        """
        frame_h, frame_w = frame.shape[:2]

        # 1. Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Dual-band fire colour masking
        lower1 = np.array(config.HSV_FIRE_LOWER_1, dtype=np.uint8)
        upper1 = np.array(config.HSV_FIRE_UPPER_1, dtype=np.uint8)
        lower2 = np.array(config.HSV_FIRE_LOWER_2, dtype=np.uint8)
        upper2 = np.array(config.HSV_FIRE_UPPER_2, dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        color_mask = cv2.bitwise_or(mask1, mask2)

        # 3. Motion / flicker filtering via background subtraction
        if self._bg_subtractor is not None:
            fg_mask = self._bg_subtractor.apply(frame)
            combined_mask = cv2.bitwise_and(color_mask, fg_mask)
        else:
            combined_mask = color_mask

        # 4. Morphological cleanup
        kernel_size = config.HSV_MORPH_KERNEL_SIZE
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        cleaned = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_DILATE, kernel)

        # 5. Find contours
        contours, _ = cv2.findContours(
            cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections: List[Detection] = []
        total_frame_area = frame_h * frame_w

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < config.HSV_MIN_CONTOUR_AREA:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)

            # Aspect ratio filter
            if bh == 0:
                continue
            aspect = float(bw) / float(bh)
            if aspect < config.HSV_ASPECT_RATIO_MIN or aspect > config.HSV_ASPECT_RATIO_MAX:
                continue

            # Heuristic confidence from area ratio and colour intensity
            area_ratio = min(area / total_frame_area, 0.5)
            # Sample mean saturation within the bounding box
            roi_hsv = hsv[by:by + bh, bx:bx + bw]
            mean_sat = float(np.mean(roi_hsv[:, :, 1])) / 255.0 if roi_hsv.size > 0 else 0.5
            confidence = min(0.95, 0.4 + area_ratio * 2.0 + mean_sat * 0.3)

            if confidence < config.CONFIDENCE_THRESHOLD:
                continue

            detections.append(Detection(
                class_name="fire",
                confidence=round(confidence, 4),
                x=bx, y=by, width=bw, height=bh,
                frame_width=frame_w, frame_height=frame_h,
            ))

        return detections

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def _detect_simulation(self, frame: np.ndarray) -> List[Detection]:
        """Generate synthetic fire detections for testing."""
        frame_h, frame_w = frame.shape[:2]

        if random.random() > config.SIMULATION_FIRE_PROBABILITY:
            return []

        # Random bounding box
        bw = random.randint(60, min(200, frame_w // 3))
        bh = random.randint(80, min(250, frame_h // 3))
        bx = random.randint(0, max(0, frame_w - bw))
        by = random.randint(0, max(0, frame_h - bh))
        conf = round(random.uniform(
            config.SIMULATION_CONFIDENCE_MIN,
            config.SIMULATION_CONFIDENCE_MAX,
        ), 4)

        return [Detection(
            class_name="fire",
            confidence=conf,
            x=bx, y=by, width=bw, height=bh,
            frame_width=frame_w, frame_height=frame_h,
        )]
