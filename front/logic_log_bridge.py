"""
log_bridge.py

gazebo_simulator's log_config.json defines the "src" logger with
propagate=False and its own console handler. Records never reach root.

Fix: after every dictConfig call, attach our Qt handler directly to the
"src" logger (and root). No changes needed anywhere in src/.
"""

import logging
import logging.config
from PyQt5.QtCore import QObject, pyqtSignal

# ── Qt signal carrier ─────────────────────────────────────────────────────────


class _LogBridge(QObject):
    new_record = pyqtSignal(logging.LogRecord)


log_bridge = _LogBridge()


# ── Custom handler ────────────────────────────────────────────────────────────


class _QtHandler(logging.Handler):
    def emit(self, record: logging.LogRecord):
        try:
            log_bridge.new_record.emit(record)
        except Exception:
            self.handleError(record)


_qt_handler = _QtHandler()
_qt_handler.setLevel(logging.DEBUG)


def _attach_to_logger(name: str):
    """Add the Qt handler to a named logger if not already there."""
    lgr = logging.getLogger(name)
    for h in lgr.handlers:
        if isinstance(h, _QtHandler):
            return
    lgr.addHandler(_qt_handler)


def _attach_all():
    """Attach Qt handler to root and every logger that has propagate=False."""
    _attach_to_logger("")  # root
    _attach_to_logger("src")  # src.* loggers (propagate=False in log_config)
    # Also catch any other non-propagating loggers that may exist
    for name, lgr in logging.Logger.manager.loggerDict.items():
        if isinstance(lgr, logging.Logger) and not lgr.propagate:
            _attach_to_logger(name)


# ── Monkey-patch dictConfig ───────────────────────────────────────────────────

_original_dictConfig = logging.config.dictConfig


def _patched_dictConfig(config):
    _original_dictConfig(config)
    _attach_all()  # re-attach after dictConfig installs its own handlers


logging.config.dictConfig = _patched_dictConfig


# ── Public helpers ────────────────────────────────────────────────────────────


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


def setup_logging(level: int = logging.DEBUG) -> None:
    """Call once at the top of __main__.py, before any src.* import."""
    # Ensure src.* loggers pass DEBUG records through to our Qt handler
    for name in ("src", "front", ""):
        logging.getLogger(name).setLevel(level)
    _attach_all()

    # Console handler on root for terminal output
    root = logging.getLogger()
    if not any(
        isinstance(h, logging.StreamHandler) and not isinstance(h, _QtHandler)
        for h in root.handlers
    ):
        console = logging.StreamHandler()
        console.setLevel(logging.DEBUG)
        console.setFormatter(
            logging.Formatter(
                "%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
                datefmt="%H:%M:%S",
            )
        )
        root.addHandler(console)
