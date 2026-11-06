"""
BANTAY-SUNOG Structured Logger

Dual-output logger providing timestamped, categorized log messages to both
the terminal (stdout) and rotating log files.

Categories: SYSTEM, CAMERA, MODEL, VISION, UART, UART TX, PERF, ERROR
"""

import logging
import logging.handlers
import os
import sys
from datetime import datetime
from typing import Optional

from bantay_sunog import config


# ---------------------------------------------------------------------------
# ANSI colour codes (auto-disabled when stdout is not a TTY)
# ---------------------------------------------------------------------------

class _Colors:
    """ANSI escape sequences for terminal colouring."""

    RESET = "\033[0m"
    BOLD = "\033[1m"

    # Category colours
    SYSTEM = "\033[96m"    # Cyan
    CAMERA = "\033[93m"    # Yellow
    MODEL = "\033[95m"     # Magenta
    VISION = "\033[92m"    # Green
    UART = "\033[94m"      # Blue
    UART_TX = "\033[1;94m" # Bold Blue
    PERF = "\033[37m"      # Light Gray
    ERROR = "\033[91m"     # Red
    WARNING = "\033[93m"   # Yellow

    # Level colours
    DEBUG_CLR = "\033[37m"
    INFO_CLR = "\033[97m"
    WARN_CLR = "\033[93m"
    ERR_CLR = "\033[91m"
    CRIT_CLR = "\033[1;91m"


_CATEGORY_COLORS = {
    "SYSTEM": _Colors.SYSTEM,
    "CAMERA": _Colors.CAMERA,
    "MODEL": _Colors.MODEL,
    "VISION": _Colors.VISION,
    "UART": _Colors.UART,
    "UART TX": _Colors.UART_TX,
    "PERF": _Colors.PERF,
    "ERROR": _Colors.ERROR,
}

_LEVEL_COLORS = {
    logging.DEBUG: _Colors.DEBUG_CLR,
    logging.INFO: _Colors.INFO_CLR,
    logging.WARNING: _Colors.WARN_CLR,
    logging.ERROR: _Colors.ERR_CLR,
    logging.CRITICAL: _Colors.CRIT_CLR,
}


# ---------------------------------------------------------------------------
# Custom formatter
# ---------------------------------------------------------------------------

class BantayFormatter(logging.Formatter):
    """Formatter producing ``[HH:MM:SS.mmm] [CATEGORY] message`` lines."""

    def __init__(self, use_color: bool = False) -> None:
        super().__init__()
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        # Timestamp
        dt = datetime.fromtimestamp(record.created)
        ts = dt.strftime("%H:%M:%S") + f".{int(dt.microsecond / 1000):03d}"

        # Category (stored as an extra field on the record)
        category: str = getattr(record, "category", "SYSTEM")

        if self.use_color:
            cat_color = _CATEGORY_COLORS.get(category, _Colors.RESET)
            lvl_color = _LEVEL_COLORS.get(record.levelno, _Colors.RESET)
            return (
                f"{_Colors.BOLD}[{ts}]{_Colors.RESET} "
                f"{cat_color}[{category}]{_Colors.RESET} "
                f"{lvl_color}{record.getMessage()}{_Colors.RESET}"
            )
        else:
            return f"[{ts}] [{category}] {record.getMessage()}"


class StartupFormatter(logging.Formatter):
    """Formatter for startup banner messages: ``[CATEGORY] message``."""

    def __init__(self, use_color: bool = False) -> None:
        super().__init__()
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        category: str = getattr(record, "category", "SYSTEM")
        if self.use_color:
            cat_color = _CATEGORY_COLORS.get(category, _Colors.RESET)
            return (
                f"{cat_color}[{category}]{_Colors.RESET} "
                f"{record.getMessage()}"
            )
        else:
            return f"[{category}] {record.getMessage()}"


# ---------------------------------------------------------------------------
# Logger singleton
# ---------------------------------------------------------------------------

_logger: Optional[logging.Logger] = None
_startup_mode: bool = True


def _detect_color() -> bool:
    """Return True if stdout supports ANSI colours."""
    if not config.LOG_COLOR_ENABLED:
        return False
    if os.environ.get("NO_COLOR"):
        return False
    if sys.platform == "win32":
        return os.environ.get("TERM") == "xterm" or os.environ.get("WT_SESSION")
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


def setup_logger() -> logging.Logger:
    """
    Initialise and return the application-wide logger.

    Creates handlers for:
    - Console output (stdout) with optional colour
    - Rotating file output in the configured log directory
    """
    global _logger

    if _logger is not None:
        return _logger

    # Create log directory
    os.makedirs(config.LOG_DIR, exist_ok=True)

    logger = logging.getLogger("bantay_sunog")
    logger.setLevel(getattr(logging, config.LOG_LEVEL.upper(), logging.INFO))
    logger.propagate = False

    use_color = _detect_color()

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logger.level)
    console_handler.setFormatter(BantayFormatter(use_color=use_color))
    logger.addHandler(console_handler)

    # File handler (rotating)
    log_path = os.path.join(config.LOG_DIR, config.LOG_FILE)
    file_handler = logging.handlers.RotatingFileHandler(
        log_path,
        maxBytes=config.LOG_MAX_BYTES,
        backupCount=config.LOG_BACKUP_COUNT,
        encoding="utf-8",
    )
    file_handler.setLevel(logging.DEBUG)  # File always captures DEBUG
    file_handler.setFormatter(BantayFormatter(use_color=False))
    logger.addHandler(file_handler)

    _logger = logger
    return logger


def get_logger() -> logging.Logger:
    """Return the application logger, initialising if needed."""
    global _logger
    if _logger is None:
        return setup_logger()
    return _logger


def set_startup_mode(enabled: bool) -> None:
    """
    Toggle startup banner formatting.

    During startup, messages use ``[CATEGORY] message`` format (no timestamp).
    After startup, messages use ``[HH:MM:SS.mmm] [CATEGORY] message``.
    """
    global _startup_mode
    _startup_mode = enabled
    logger = get_logger()
    use_color = _detect_color()

    for handler in logger.handlers:
        if isinstance(handler, logging.StreamHandler) and not isinstance(
            handler, logging.handlers.RotatingFileHandler
        ):
            if enabled:
                handler.setFormatter(StartupFormatter(use_color=use_color))
            else:
                handler.setFormatter(BantayFormatter(use_color=use_color))


# ---------------------------------------------------------------------------
# Convenience logging functions with category support
# ---------------------------------------------------------------------------

def _log(level: int, category: str, message: str) -> None:
    """Emit a log record with a category extra field."""
    logger = get_logger()
    logger.log(level, message, extra={"category": category})


def log_system(message: str, level: int = logging.INFO) -> None:
    """Log a SYSTEM message."""
    _log(level, "SYSTEM", message)


def log_camera(message: str, level: int = logging.INFO) -> None:
    """Log a CAMERA message."""
    _log(level, "CAMERA", message)


def log_model(message: str, level: int = logging.INFO) -> None:
    """Log a MODEL message."""
    _log(level, "MODEL", message)


def log_vision(message: str, level: int = logging.INFO) -> None:
    """Log a VISION message."""
    _log(level, "VISION", message)


def log_uart(message: str, level: int = logging.INFO) -> None:
    """Log a UART message."""
    _log(level, "UART", message)


def log_uart_tx(message: str, level: int = logging.INFO) -> None:
    """Log a UART TX message (every outgoing message to ESP32)."""
    _log(level, "UART TX", message)


def log_perf(message: str, level: int = logging.INFO) -> None:
    """Log a PERF (performance) message."""
    _log(level, "PERF", message)


def log_error(message: str, level: int = logging.ERROR) -> None:
    """Log an ERROR message."""
    _log(level, "ERROR", message)


def log_banner() -> None:
    """Print the startup banner."""
    set_startup_mode(True)
    log_system("")
    log_system(f"{'=' * 55}")
    log_system(f"  {config.PROJECT_NAME} v{config.VERSION}")
    log_system(f"  {config.DESCRIPTION}")
    log_system(f"{'=' * 55}")
    log_system("")
    log_system("Initializing Raspberry Pi fire-detection subsystem...")
