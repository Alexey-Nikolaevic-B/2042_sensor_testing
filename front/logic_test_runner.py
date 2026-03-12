import ctypes
import time
import traceback
import threading
import logging

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, QMetaObject, Qt

logger = logging.getLogger(__name__)


class _Worker(QObject):

    log_line      = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    test_progress = pyqtSignal(str, int)   # func_name, 0-100
    all_finished  = pyqtSignal()
    error         = pyqtSignal(str, str)

    def __init__(self, core, backend, func_name: str, func, sensor=None):
        super().__init__()
        self._core           = core
        self._backend        = backend
        self._func_name      = func_name
        self._func           = func
        self._sensor         = sensor
        self._stop_requested = False
        self._thread_id: int | None = None

    def request_stop(self):
        self._stop_requested = True

    def raise_in_thread(self, exc_type):
        tid = self._thread_id
        if tid is None:
            self.log_line.emit(f"[Worker:{self._func_name}] raise_in_thread: no thread_id")
            return
        try:
            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
                ctypes.c_ulong(tid),
                ctypes.py_object(exc_type),
            )
            if res == 0:
                self.log_line.emit(f"[Worker:{self._func_name}] raise_in_thread: tid {tid} not found")
            elif res > 1:
                ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(tid), None)
                self.log_line.emit(f"[Worker:{self._func_name}] raise_in_thread: affected {res} threads — undone")
            else:
                self.log_line.emit(f"[Worker:{self._func_name}] {exc_type.__name__} injected into tid {tid}")
        except Exception as exc:
            self.log_line.emit(f"[Worker:{self._func_name}] raise_in_thread error: {exc}")

    @pyqtSlot()
    def run(self):
        func_name = self._func_name
        self._thread_id = threading.current_thread().ident
        self.log_line.emit(f"{func_name}")

        def progress_cb(value: int):
            print(f"[DEBUG runner] progress_cb: func={func_name} value={value}")
            self.test_progress.emit(func_name, value)
            if self._stop_requested:
                raise StopIteration

        # Reset step gate for each test run
        self._core.simulator.advance_step()

        t0 = time.time()
        try:
            if self._sensor is not None:
                result = self._func(self._core.simulator, self._sensor, progress_cb=progress_cb)
            else:
                result = self._func(self._core.simulator, progress_cb=progress_cb)
            duration = time.time() - t0

            if result is None:
                result = {}

            passed = result.get("passed", False) if isinstance(result, dict) else bool(result)
            status = "Passed" if passed else "Failed"

            self.log_line.emit(f"{func_name}  {status}  ({duration:.1f}s)")
            if isinstance(result, dict):
                for k, v in result.items():
                    if k not in ("passed", "duration"):
                        self.log_line.emit(f"     {k}: {v}")

            self.test_finished.emit(func_name, result, status, duration)

        except (StopIteration, SystemExit, KeyboardInterrupt):
            duration = time.time() - t0
            self.log_line.emit(f"{func_name}  Stopped  ({duration:.1f}s)")
            self.test_finished.emit(func_name, {}, "Stopped", duration)

        except Exception as exc:
            duration = time.time() - t0
            tb = traceback.format_exc()
            self.log_line.emit(f"{func_name}  {type(exc).__name__}: {exc}")
            self.log_line.emit(tb)
            logger.error("Test %s raised exception:\n%s", func_name, tb)
            self.error.emit(func_name, str(exc))
            self.test_finished.emit(func_name, {}, "Failed", duration)

        finally:
            self._thread_id = None

        self.all_finished.emit()


class TestRunner(QObject):

    log_line      = pyqtSignal(str)
    test_started  = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    test_progress = pyqtSignal(str, int)
    all_finished  = pyqtSignal()
    error         = pyqtSignal(str, str)

    def __init__(self, core, parent=None):
        super().__init__(parent)
        self._core        = core
        self._worker: _Worker | None = None
        self._worker_lock = threading.Lock()

        self._thread = QThread(self)
        self._thread.start()

    def run_one(self, backend, func_name: str, func, sensor=None):
        worker = _Worker(
            core      = self._core,
            backend   = backend,
            func_name = func_name,
            func      = func,
            sensor    = sensor,
        )
        worker.moveToThread(self._thread)

        worker.log_line.connect(self.log_line)
        worker.test_progress.connect(self.test_progress)   # ← forward progress
        worker.test_finished.connect(self.test_finished)
        worker.all_finished.connect(self.all_finished)
        worker.error.connect(self.error)

        with self._worker_lock:
            self._worker = worker

        self.test_started.emit(func_name)
        QMetaObject.invokeMethod(worker, "run", Qt.QueuedConnection)

    def stop(self):
        with self._worker_lock:
            worker = self._worker
        self.log_line.emit(f"[TestRunner] stop() called, worker={worker}")
        if worker is not None:
            worker.request_stop()

    def force_kill(self):
        import subprocess

        with self._worker_lock:
            worker = self._worker
        self.log_line.emit(f"[TestRunner] force_kill() called, worker={worker}")

        if worker is not None:
            worker.raise_in_thread(SystemExit)

        try:
            proc = getattr(self._core.simulator, "gazebo_process", None)
            if proc is not None:
                try:
                    proc.kill()
                except Exception:
                    pass
            subprocess.run(["pkill", "-9", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-9", "-f", "gzclient"], check=False)
            try:
                self._core.simulator.gazebo_is_running = False
            except Exception:
                pass
        except Exception as exc:
            logger.error("force_kill gazebo error: %s", exc)

    def get_tests(self, backend) -> dict:
        return self._core.get_tests(backend)

    def shutdown(self):
        self.stop()
        self._thread.quit()
        self._thread.wait(10000)