import time
import traceback

from PyQt5.QtCore import QObject, QThread, pyqtSignal


class TestRunner(QObject):

    log_line      = pyqtSignal(str)
    test_started  = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    all_finished  = pyqtSignal()
    error         = pyqtSignal(str, str)

    def __init__(self, core, parent=None):
        super().__init__(parent)
        self._core   = core
        self._thread = None
        self._worker = None

    def run_tests(self, sensor_backend, test_names: list[str] = None):
        self.stop()

        all_tests = self._core.get_tests(sensor_backend)
        if test_names:
            tests_to_run = {k: v for k, v in all_tests.items() if k in test_names}
        else:
            tests_to_run = all_tests

        if not tests_to_run:
            self.log_line.emit(
                f"No tests found for '{getattr(sensor_backend, 'sensor_name', '?')}'. "
                "Test methods must end with '_test'."
            )
            self.all_finished.emit()
            return

        self._thread = QThread()
        self._worker = _TestWorker(
            core           = self._core,
            sensor_backend = sensor_backend,
            tests          = tests_to_run,
        )
        self._worker.moveToThread(self._thread)

        self._worker.log_line.connect(self.log_line)
        self._worker.test_started.connect(self.test_started)
        self._worker.test_finished.connect(self.test_finished)
        self._worker.all_finished.connect(self.all_finished)
        self._worker.error.connect(self.error)

        self._thread.started.connect(self._worker.run)
        self._worker.all_finished.connect(self._thread.quit)
        self._thread.finished.connect(self._on_thread_finished)

        self._thread.start()

    def stop(self):
        if self._worker is not None:
            self._worker.request_stop()
        self._thread = None
        self._worker = None

    def _on_thread_finished(self):
        self._thread = None
        self._worker = None


class _TestWorker(QObject):

    log_line      = pyqtSignal(str)
    test_started  = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    all_finished  = pyqtSignal()
    error         = pyqtSignal(str, str)

    def __init__(self, core, sensor_backend, tests: dict):
        super().__init__()
        self._core           = core
        self._sensor_backend = sensor_backend
        self._tests          = tests
        self._stop_requested = False

    def request_stop(self):
        self._stop_requested = True

    def run(self):
        sensor_name = getattr(self._sensor_backend, "sensor_name", "unknown")
        self.log_line.emit(f"Starting tests for: {sensor_name}")

        for test_name, test_func in self._tests.items():
            if self._stop_requested:
                self.log_line.emit("--- Stopped by user ---")
                break

            self.log_line.emit(f"▶  {test_name}")
            self.test_started.emit(test_name)

            t0 = time.time()
            try:
                result   = test_func(self._core.simulator)
                duration = time.time() - t0

                if result is None:
                    result = {}

                passed = result.get("passed", False) if isinstance(result, dict) else bool(result)
                status = "Passed" if passed else "Failed"
                mark   = "✔" if passed else "✘"

                self.log_line.emit(f"{mark}  {test_name} — {status}  ({duration:.1f}s)")
                if isinstance(result, dict):
                    for k, v in result.items():
                        if k != "passed":
                            self.log_line.emit(f"     {k}: {v}")

                self.test_finished.emit(test_name, result, status, duration)

            except Exception as exc:
                duration = time.time() - t0
                self.log_line.emit(f"✘  {test_name} raised {type(exc).__name__}: {exc}")
                self.log_line.emit(traceback.format_exc())
                self.error.emit(test_name, str(exc))
                self.test_finished.emit(test_name, {}, "Failed", duration)

        self.log_line.emit("All tests finished")
        self.all_finished.emit()