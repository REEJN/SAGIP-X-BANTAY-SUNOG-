"""
BANTAY-SUNOG Configuration Module

Central configuration for the Raspberry Pi 3B fire-detection subsystem.
All tunable parameters are defined here for easy adjustment without
modifying application logic.

Target hardware: Raspberry Pi 3 Model B (1 GB RAM, ARM Cortex-A53 @ 1.2 GHz)
"""

import os

# =============================================================================
# PROJECT METADATA
# =============================================================================

PROJECT_NAME = "BANTAY-SUNOG"
VERSION = "1.0.0"
DESCRIPTION = "Raspberry Pi 3B Real-Time Fire Detection Subsystem"

# =============================================================================
# CAMERA SETTINGS
# =============================================================================

# USB webcam device path
CAMERA_DEVICE = int(os.environ.get("BANTAY_CAMERA_DEVICE", "0"))

# Capture resolution (what the webcam outputs)
CAMERA_WIDTH = int(os.environ.get("BANTAY_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("BANTAY_CAMERA_HEIGHT", "360"))

# Target FPS for capture (0 = camera default)
CAMERA_FPS = int(os.environ.get("BANTAY_CAMERA_FPS", "30"))

# Maximum consecutive frame read failures before CAMERA_ERROR
CAMERA_MAX_FAILURES = 10

# Camera reconnection interval in seconds
CAMERA_RECONNECT_INTERVAL = 3.0

# Number of frames to drain from buffer to get newest frame (1 = single read, no stutter)
CAMERA_BUFFER_DRAIN_COUNT = int(os.environ.get("BANTAY_CAMERA_DRAIN", "1"))

# Preferred FourCC pixel format: "MJPG" (compressed, low USB bandwidth, no tearing) or "YUYV"
CAMERA_FOURCC = os.environ.get("BANTAY_CAMERA_FOURCC", "MJPG")

# Enable dedicated camera acquisition thread (keeps V4L2 queue drained, 0ms latency)
CAMERA_THREADED = os.environ.get("BANTAY_CAMERA_THREADED", "1") in ("1", "true", "True")

# =============================================================================
# DETECTION MODEL SETTINGS
# =============================================================================

# Detection backend: "tflite", "hsv_fallback", "simulation"
DETECTION_BACKEND = os.environ.get("BANTAY_DETECTOR", "hsv_fallback")

# --- TFLite Model ---
MODEL_PATH = os.environ.get(
    "BANTAY_MODEL_PATH",
    os.path.join(os.path.dirname(__file__), "models", "fire_detect.tflite")
)
LABELS_PATH = os.environ.get(
    "BANTAY_LABELS_PATH",
    os.path.join(os.path.dirname(__file__), "models", "labels.txt")
)

# Model input dimensions (standard SSD MobileNet V2 = 300x300)
MODEL_INPUT_WIDTH = int(os.environ.get("BANTAY_MODEL_INPUT_W", "300"))
MODEL_INPUT_HEIGHT = int(os.environ.get("BANTAY_MODEL_INPUT_H", "300"))

# Number of interpreter threads (Pi 3B has 4 cores)
MODEL_NUM_THREADS = int(os.environ.get("BANTAY_MODEL_THREADS", "4"))

# Minimum confidence threshold for a detection to be considered
CONFIDENCE_THRESHOLD = float(os.environ.get("BANTAY_CONFIDENCE", "0.70"))

# --- HSV Fallback (Experimental) ---
# Fire color HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255)
# Band 1: Orange/Yellow/Red flames
HSV_FIRE_LOWER_1 = (0, 70, 180)
HSV_FIRE_UPPER_1 = (25, 255, 255)

# Band 2: Deep red embers / wrap-around
HSV_FIRE_LOWER_2 = (165, 70, 180)
HSV_FIRE_UPPER_2 = (179, 255, 255)

# Minimum contour area (pixels) to be considered a fire region
HSV_MIN_CONTOUR_AREA = 500

# Morphological kernel size
HSV_MORPH_KERNEL_SIZE = 5

# Background subtractor history length
HSV_BG_HISTORY = 50

# Aspect ratio bounds for flame contours (width/height)
HSV_ASPECT_RATIO_MIN = 0.2
HSV_ASPECT_RATIO_MAX = 3.0

# --- Simulation ---
# Simulation fire probability per frame (0.0 - 1.0)
SIMULATION_FIRE_PROBABILITY = float(os.environ.get("BANTAY_SIM_PROB", "0.85"))

# Simulated confidence range
SIMULATION_CONFIDENCE_MIN = 0.65
SIMULATION_CONFIDENCE_MAX = 0.95

# =============================================================================
# FIRE CONFIRMATION SETTINGS
# =============================================================================

# Number of recent frames to consider for temporal validation
CONFIRMATION_WINDOW_SIZE = 5

# Minimum positive detections within the window to confirm fire
CONFIRMATION_REQUIRED_POSITIVES = 3

# Seconds without detection before a confirmed fire is marked as lost
FIRE_LOSS_TIMEOUT = 2.0

# Minimum interval between UART transmissions (seconds)
MIN_TRANSMISSION_INTERVAL = 0.5

# Periodic update interval for confirmed fire coordinates (seconds)
CONFIRMED_UPDATE_INTERVAL = 1.0

# =============================================================================
# UART / SERIAL SETTINGS
# =============================================================================

# Serial port for ESP32-WROOM communication
UART_PORT = os.environ.get("BANTAY_UART_PORT", "/dev/serial0")

# Baud rate
UART_BAUD_RATE = int(os.environ.get("BANTAY_UART_BAUD", "115200"))

# Serial configuration
UART_DATA_BITS = 8
UART_PARITY = "N"  # None
UART_STOP_BITS = 1

# Read/write timeout in seconds
UART_TIMEOUT = 1.0

# Write timeout in seconds
UART_WRITE_TIMEOUT = 1.0

# Reconnection interval on failure (seconds)
UART_RECONNECT_INTERVAL = 5.0

# Maximum reconnection attempts (0 = unlimited)
UART_MAX_RECONNECT_ATTEMPTS = 0

# Enable optional ACK protocol (fire-and-forget if False)
UART_ACK_ENABLED = False

# ACK timeout in seconds (only used if ACK is enabled)
UART_ACK_TIMEOUT = 0.5

# =============================================================================
# LOGGING SETTINGS
# =============================================================================

# Log level: "DEBUG" or "INFO"
LOG_LEVEL = os.environ.get("BANTAY_LOG_LEVEL", "INFO")

# Log directory (relative to project root)
LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")

# Log file name
LOG_FILE = "bantay_sunog.log"

# Maximum log file size in bytes (5 MB)
LOG_MAX_BYTES = 5 * 1024 * 1024

# Number of backup log files to keep
LOG_BACKUP_COUNT = 3

# Enable colored terminal output (auto-detect TTY)
LOG_COLOR_ENABLED = True

# =============================================================================
# DISPLAY / PREVIEW SETTINGS
# =============================================================================

# Enable OpenCV preview window (set False for headless/SSH operation)
PREVIEW_ENABLED = False

# Preview window name
PREVIEW_WINDOW_NAME = "BANTAY-SUNOG Fire Detection"

# Preview overlay: show bounding boxes
PREVIEW_SHOW_BBOX = True

# Preview overlay: show confidence text
PREVIEW_SHOW_CONFIDENCE = True

# Preview overlay: show FPS
PREVIEW_SHOW_FPS = True

# Preview overlay: show detection state
PREVIEW_SHOW_STATE = True

# Web Browser Preview Streaming Settings (viewable on laptop browser)
WEB_STREAM_ENABLED = os.environ.get("BANTAY_WEB_STREAM", "0") in ("1", "true", "True")
WEB_STREAM_HOST = os.environ.get("BANTAY_WEB_HOST", "0.0.0.0")
WEB_STREAM_PORT = int(os.environ.get("BANTAY_WEB_PORT", "5000"))
WEB_STREAM_JPEG_QUALITY = int(os.environ.get("BANTAY_WEB_QUALITY", "70"))

# =============================================================================
# PERFORMANCE MONITORING
# =============================================================================

# Interval for printing performance stats (seconds)
PERF_REPORT_INTERVAL = 10.0

# Rolling window size for FPS calculation (frames)
PERF_FPS_WINDOW = 30

# psutil CPU measurement interval (None = non-blocking)
PERF_CPU_INTERVAL = None

# =============================================================================
# DETECTION STATES
# =============================================================================

class DetectionState:
    """Enumeration of fire detection states."""
    IDLE = "IDLE"
    NO_FIRE = "NO_FIRE"
    FIRE_CANDIDATE = "FIRE_CANDIDATE"
    FIRE_CONFIRMED = "FIRE_CONFIRMED"
    FIRE_LOST = "FIRE_LOST"
    CAMERA_ERROR = "CAMERA_ERROR"
    MODEL_ERROR = "MODEL_ERROR"
    SERIAL_ERROR = "SERIAL_ERROR"
