import threading
import logging
from enum import Enum

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

logger = logging.getLogger(__name__)


class TestStatus(str, Enum):
    IDLE    = "Idle"
    QUEUED  = "Queued"
    RUNNING = "Running"
    PASSED  = "Passed"
    FAILED  = "Failed"


class _Entry:
    __slots__ = ("sensor_id", "func_name", "backend", "func")

    def __init__(self, sensor_id, func_name, backend, func):
        self.sensor_id = str(sensor_id)
        self.func_name = func_name
        self.backend   = backend
        self.func      = func

    def matches(self, sensor_id, func_name) -> bool:
        return self.sensor_id == str(sensor_id) and self.func_name == func_name

    def __repr__(self):
        return f"({self.sensor_id!r}, {self.func_name!r})"


class QueueManager(QObject):

    item_state_changed    = pyqtSignal(str, str, TestStatus)
    item_progress_changed = pyqtSignal(str, str, int)
    item_result           = pyqtSignal(str, str, dict)
    log_line              = pyqtSignal(str)

    _AMBIENT_INTERVAL_MS = 1500
    _AMBIENT_CAP         = 90

    def __init__(self, runner, parent=None):
        super().__init__(parent)
        self._runner   = runner
        self._lock     = threading.Lock()
        self._queue:   list[_Entry]  = []
        self._running: _Entry | None = None
        self._progress = 0
        self._stop_requested = False  # True after cancel() on running test

        self._ambient = QTimer(self)
        self._ambient.setInterval(self._AMBIENT_INTERVAL_MS)
        self._ambient.timeout.connect(self._on_ambient_tick)

        runner.test_finished.connect(self._on_runner_finished)
        runner.test_progress.connect(self._on_runner_progress)
        runner.all_finished.connect(self._on_all_finished)
        runner.log_line.connect(self.log_line)


    def enqueue(self, sensor_id, func_name, backend, func):
        sensor_id = str(sensor_id)
        with self._lock:
            if self._is_active(sensor_id, func_name):
                return
            self._queue.append(_Entry(sensor_id, func_name, backend, func))
        self.log_line.emit(f"{func_name}  added to queue")
        self.item_state_changed.emit(sensor_id, func_name, TestStatus.QUEUED)
        self._try_advance()

    def cancel(self, sensor_id, func_name):
        sensor_id = str(sensor_id)

        removed = False
        with self._lock:
            for e in list(self._queue):
                if e.matches(sensor_id, func_name):
                    self._queue.remove(e)
                    removed = True
                    break

        if removed:
            self.log_line.emit(f"{func_name}  removed from queue")
            self.item_state_changed.emit(sensor_id, func_name, TestStatus.IDLE)
            return

        with self._lock:
            running = self._running
        if running and running.matches(sensor_id, func_name):
            if self._stop_requested:
                return
            self._stop_requested = True
            self.log_line.emit(f"{func_name}  stop requested")
            self.item_state_changed.emit(sensor_id, func_name, TestStatus.IDLE)
            self._runner.stop()
            self._runner.force_kill()

    def cancel_all_for_sensor(self, sensor_id):
        sensor_id = str(sensor_id)
        cancelled = []
        with self._lock:
            keep = []
            for e in self._queue:
                if e.sensor_id == sensor_id:
                    cancelled.append(e)
                else:
                    keep.append(e)
            self._queue = keep

        for e in cancelled:
            self.log_line.emit(f"{e.func_name}  removed from queue")
            self.item_state_changed.emit(e.sensor_id, e.func_name, TestStatus.IDLE)

        with self._lock:
            running = self._running
        if running and running.sensor_id == sensor_id:
            self.log_line.emit(f"{running.func_name}  stop requested")
            self._runner.stop()
            self._runner.force_kill()

    def get_tests(self, backend) -> dict:
        return self._runner.get_tests(backend)

    def is_active(self, sensor_id, func_name) -> bool:
        with self._lock:
            return self._is_active(str(sensor_id), func_name)

    def get_running(self) -> tuple[str, str] | None:
        with self._lock:
            r = self._running
        return (r.sensor_id, r.func_name) if r else None

    def get_queued_for_sensor(self, sensor_id: str) -> dict[str, TestStatus]:
        sid = str(sensor_id)
        with self._lock:
            return {e.func_name: TestStatus.QUEUED for e in self._queue if e.sensor_id == sid}

    def _is_active(self, sensor_id: str, func_name: str) -> bool:
        if self._running and self._running.matches(sensor_id, func_name):
            return True
        return any(e.matches(sensor_id, func_name) for e in self._queue)

    def _try_advance(self):
        with self._lock:
            if self._running is not None or not self._queue:
                return
            entry = self._queue.pop(0)
            self._running = entry

        self._progress = 0
        self.item_state_changed.emit(entry.sensor_id, entry.func_name, TestStatus.RUNNING)
        self._ambient.start()

        self.log_line.emit(
            f"[QueueManager] starting {entry.func_name} for sensor {entry.sensor_id}"
        )
        self._runner.run_one(entry.backend, entry.func_name, entry.func)

    def _on_ambient_tick(self):
        with self._lock:
            running = self._running
        if running is None:
            self._ambient.stop()
            return
        if self._progress < self._AMBIENT_CAP:
            self._progress += 1
            self.item_progress_changed.emit(running.sensor_id, running.func_name, self._progress)

    def _on_runner_progress(self, func_name: str, value: int):
        with self._lock:
            running = self._running
        if running and running.func_name == func_name:
            self._progress = value
            self.item_progress_changed.emit(running.sensor_id, func_name, value)

    def _on_runner_finished(self, func_name: str, result: dict, status_str: str, duration: float):
        self._ambient.stop()

        with self._lock:
            entry = self._running

        if entry is None or entry.func_name != func_name:
            return

        self.item_progress_changed.emit(entry.sensor_id, entry.func_name, 0)

        status_map = {
            "Passed":  TestStatus.PASSED,
            "Failed":  TestStatus.FAILED,
            "Stopped": TestStatus.IDLE,
        }
        ts = status_map.get(status_str, TestStatus.IDLE)
        self.item_state_changed.emit(entry.sensor_id, entry.func_name, ts)

        if result:
            self.item_result.emit(entry.sensor_id, entry.func_name, result)

        try:
            from .sensor_repository import SensorRepository
            repo = SensorRepository.instance()
            passed = result.get("passed", False) if isinstance(result, dict) else bool(result)
            repo.save_test_result(
                sensor_id   = entry.sensor_id,
                test_name   = entry.func_name,
                status      = ts.value,
                result      = result,
                duration    = duration,
            )
        except Exception as exc:
            logger.error("Failed to save test result: %s", exc)

    def _on_all_finished(self):
        try:
            with self._runner._worker_lock:
                self._runner._worker = None
        except Exception:
            pass

        with self._lock:
            self._running = None
        self._stop_requested = False
        self.log_line.emit("[QueueManager] all_finished — advancing queue")
        self._try_advance()