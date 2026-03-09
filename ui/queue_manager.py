import threading
from enum import Enum

from PyQt5.QtCore import QObject, QTimer, pyqtSignal


class TestStatus(str, Enum):
    IDLE    = "Idle"
    QUEUED  = "Queued"
    RUNNING = "Running"
    PASSED  = "Passed"
    FAILED  = "Failed"
    STOPPED = "Stopped"


class _Entry:
    __slots__ = ("sensor_id", "func_name", "backend", "func")

    def __init__(self, sensor_id, func_name, backend, func):
        self.sensor_id = sensor_id
        self.func_name = func_name
        self.backend   = backend
        self.func      = func

    def matches(self, sensor_id, func_name):
        return self.sensor_id == sensor_id and self.func_name == func_name


class QueueManager(QObject):

    item_state_changed    = pyqtSignal(str, str, TestStatus)
    item_progress_changed = pyqtSignal(str, str, int)
    log_line              = pyqtSignal(str)

    _AMBIENT_INTERVAL_MS = 1500
    _AMBIENT_CAP         = 90

    def __init__(self, runner, parent=None):
        super().__init__(parent)
        self._runner   = runner
        self._lock     = threading.Lock()
        self._queue    = []
        self._running  = None
        self._progress = 0

        self._ambient = QTimer(self)
        self._ambient.setInterval(self._AMBIENT_INTERVAL_MS)
        self._ambient.timeout.connect(self._on_ambient_tick)

        runner.test_finished.connect(self._on_runner_finished)
        runner.all_finished.connect(self._try_advance)
        runner.test_progress.connect(self._on_runner_progress)
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
        print(f"[QueueManager] cancel: sensor={sensor_id} func={func_name}")
        print(f"[QueueManager] cancel: queue={[(e.sensor_id, e.func_name) for e in self._queue]}")
        print(f"[QueueManager] cancel: running={( self._running.sensor_id, self._running.func_name) if self._running else None}")

        removed = False
        with self._lock:
            for e in list(self._queue):
                if e.matches(sensor_id, func_name):
                    self._queue.remove(e)
                    removed = True
                    break

        if removed:
            print(f"[QueueManager] cancel: removed from queue → IDLE")
            self.log_line.emit(f"{func_name}  removed from queue")
            self.item_state_changed.emit(sensor_id, func_name, TestStatus.IDLE)
            return

        with self._lock:
            running = self._running
        print(f"[QueueManager] cancel: not in queue, running matches={running and running.matches(sensor_id, func_name)}")
        if running and running.matches(sensor_id, func_name):
            self.log_line.emit(f"{func_name}  stop requested, killing Gazebo")
            self._runner.stop()
            self._runner.force_kill()
        else:
            print(f"[QueueManager] cancel: NOTHING TO CANCEL, not in queue and not running")

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
            self.log_line.emit(f"{running.func_name}  stop requested, killing Gazebo")
            self._runner.stop()
            self._runner.force_kill()

    def get_tests_for_backend(self, backend):
        return self._runner.get_tests(backend)

    def is_active(self, sensor_id, func_name) -> bool:
        with self._lock:
            return self._is_active(str(sensor_id), func_name)

    def _is_active(self, sensor_id, func_name):
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

    def _on_runner_progress(self, func_name, value):
        with self._lock:
            running = self._running
        if running and running.func_name == func_name:
            self._progress = value
            self.item_progress_changed.emit(running.sensor_id, func_name, value)

    def _on_runner_finished(self, func_name, result, status_str, duration):
        self._ambient.stop()
        self._progress = 0

        with self._lock:
            entry = self._running
            self._running = None

        if entry:
            try:
                ts = TestStatus(status_str)
            except ValueError:
                ts = TestStatus.STOPPED
            self.item_progress_changed.emit(entry.sensor_id, entry.func_name, 0)
            self.item_state_changed.emit(entry.sensor_id, entry.func_name, ts)