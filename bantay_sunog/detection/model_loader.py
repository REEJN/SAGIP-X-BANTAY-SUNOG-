"""
BANTAY-SUNOG Model Loader

Loads and validates the fire-detection backend.  Supports three modes:

1. **tflite**  – TensorFlow Lite interpreter (quantized INT8 SSD MobileNet
   recommended).
2. **hsv_fallback** – No model file; pure OpenCV HSV colour-space detection
   (experimental).
3. **simulation** – Generates synthetic detections for testing.
"""

import os
import logging
from typing import Any

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_model, log_error


# -------------------------------------------------------------------------
# TFLite interpreter import (try multiple package names)
# -------------------------------------------------------------------------

_Interpreter = None  # Will be set if a TFLite runtime is found


def _try_import_tflite():
    """Attempt to import a TFLite interpreter from available packages."""
    global _Interpreter

    # 1. tflite_runtime (lightweight, preferred)
    try:
        from tflite_runtime.interpreter import Interpreter
        _Interpreter = Interpreter
        log_model("TFLite runtime found: tflite_runtime")
        return True
    except ImportError:
        pass

    # 2. ai_edge_litert (Google LiteRT rebranding)
    try:
        from ai_edge_litert.interpreter import Interpreter
        _Interpreter = Interpreter
        log_model("TFLite runtime found: ai_edge_litert")
        return True
    except ImportError:
        pass

    # 3. Full TensorFlow (heavy, not recommended for Pi 3B)
    try:
        import tensorflow as tf
        _Interpreter = tf.lite.Interpreter
        log_model("TFLite runtime found: tensorflow.lite (full TF — heavy)")
        return True
    except ImportError:
        pass

    return False


# -------------------------------------------------------------------------
# Label loading
# -------------------------------------------------------------------------

def load_labels(path: str) -> list:
    """
    Load detection labels from a text file (one label per line).

    Args:
        path: Absolute or relative path to the labels file.

    Returns:
        List of label strings, e.g. ["fire"].
    """
    if not os.path.isfile(path):
        log_model(f"Labels file not found: {path}", level=logging.WARNING)
        return ["fire"]  # sensible default
    try:
        with open(path, "r", encoding="utf-8") as fh:
            labels = [line.strip() for line in fh if line.strip()]
        log_model(f"Loaded {len(labels)} label(s) from {path}")
        return labels
    except Exception as exc:
        log_error(f"Failed to load labels: {exc}")
        return ["fire"]


# -------------------------------------------------------------------------
# Model loading
# -------------------------------------------------------------------------

def load_model(backend: str | None = None) -> dict:
    """
    Load the fire-detection backend.

    Args:
        backend: One of ``"tflite"``, ``"hsv_fallback"``, ``"simulation"``.
                 Defaults to ``config.DETECTION_BACKEND``.

    Returns:
        A dict describing the loaded backend::

            {
                "backend":        str,     # "tflite" | "hsv_fallback" | "simulation"
                "interpreter":    object | None,
                "input_details":  list | None,
                "output_details": list | None,
                "input_shape":    tuple | None,   # (height, width)
                "is_quantized":   bool,
                "labels":         list[str],
            }
    """
    if backend is None:
        backend = config.DETECTION_BACKEND

    result = {
        "backend": backend,
        "interpreter": None,
        "input_details": None,
        "output_details": None,
        "input_shape": None,
        "is_quantized": False,
        "labels": ["fire"],
    }

    # ----- TFLite backend -------------------------------------------------
    if backend == "tflite":
        return _load_tflite(result)

    # ----- HSV fallback (no model file) -----------------------------------
    if backend == "hsv_fallback":
        log_model("[EXPERIMENTAL] Using HSV colour-space fallback detector.")
        log_model("No model file required for HSV fallback.")
        result["input_shape"] = (config.MODEL_INPUT_HEIGHT, config.MODEL_INPUT_WIDTH)
        return result

    # ----- Simulation -----------------------------------------------------
    if backend == "simulation":
        log_model("Using SIMULATION backend — synthetic detections only.")
        result["input_shape"] = (config.MODEL_INPUT_HEIGHT, config.MODEL_INPUT_WIDTH)
        return result

    # Unknown backend
    log_error(f"Unknown detection backend: {backend}")
    raise ValueError(f"Unknown detection backend: {backend}")


def _load_tflite(result: dict) -> dict:
    """Load a TFLite model and populate *result* dict."""
    log_model("Loading TFLite fire-detection model...")

    # Check runtime availability
    if not _try_import_tflite():
        log_error(
            "No TFLite runtime found.  Install one of:\n"
            "  pip install tflite-runtime\n"
            "  pip install ai-edge-litert\n"
            "  pip install tensorflow\n"
            "Falling back to HSV detector."
        )
        result["backend"] = "hsv_fallback"
        log_model("[EXPERIMENTAL] Falling back to HSV colour-space detector.")
        result["input_shape"] = (config.MODEL_INPUT_HEIGHT, config.MODEL_INPUT_WIDTH)
        return result

    # Check model file
    model_path = config.MODEL_PATH
    if not os.path.isfile(model_path):
        log_error(
            f"Model file not found: {model_path}\n"
            "Place a quantized .tflite model in the models/ directory.\n"
            "Falling back to HSV detector."
        )
        result["backend"] = "hsv_fallback"
        log_model("[EXPERIMENTAL] Falling back to HSV colour-space detector.")
        result["input_shape"] = (config.MODEL_INPUT_HEIGHT, config.MODEL_INPUT_WIDTH)
        return result

    # Load interpreter
    try:
        interpreter = _Interpreter(
            model_path=model_path,
            num_threads=config.MODEL_NUM_THREADS,
        )
        interpreter.allocate_tensors()
    except Exception as exc:
        log_error(f"Failed to load TFLite model: {exc}")
        result["backend"] = "hsv_fallback"
        log_model("[EXPERIMENTAL] Falling back to HSV colour-space detector.")
        result["input_shape"] = (config.MODEL_INPUT_HEIGHT, config.MODEL_INPUT_WIDTH)
        return result

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Extract input shape  (typically [1, H, W, 3])
    input_shape = input_details[0]["shape"]
    input_h = int(input_shape[1])
    input_w = int(input_shape[2])

    # Determine quantisation
    input_dtype = input_details[0]["dtype"]
    import numpy as np
    is_quantized = input_dtype in (np.uint8, np.int8)

    result["interpreter"] = interpreter
    result["input_details"] = input_details
    result["output_details"] = output_details
    result["input_shape"] = (input_h, input_w)
    result["is_quantized"] = is_quantized

    # Labels
    result["labels"] = load_labels(config.LABELS_PATH)

    model_size_kb = os.path.getsize(model_path) / 1024
    log_model(f"Model loaded: {model_path}")
    log_model(f"  Size:       {model_size_kb:.0f} KB")
    log_model(f"  Input:      {input_h}x{input_w}  dtype={input_dtype.__name__}")
    log_model(f"  Quantized:  {is_quantized}")
    log_model(f"  Threads:    {config.MODEL_NUM_THREADS}")
    log_model(f"  Labels:     {result['labels']}")
    log_model("Model loaded successfully.")

    return result
