import os
import signal
import time
import traceback
import threading
import multiprocessing as mp

from PyQt5.QtCore import QObject, QThread, pyqtSignal


def _run_test_in_process(func, simulator, func_name, result_queue):
    try:
        result = func(simulator, progress_cb=None)
        result = result or {}
        passed = result.get("passed", False) if isinstance(result, dict) else bool(result)
        result_queue.put({
            "status": "Passed" if passed else "Failed",
            "result": result,
        })
    except Exception as exc:
        result_queue.put({
            "status": "Failed",
            "result": {},
            "error": f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}",
        })


class TestRunner(QObject):

    log_line      = pyqtSignal(str)
    test_started  = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    test_progress = pyqtSignal(str, int)
    all_finished  = pyqtSignal()
    error         = pyqtSignal(str, str)

    def __init__(self, core, parent=None):
        super().__init__(parent)
        self._core   = core
        self._thread = None
        self._worker = None
        self._worker_lock = threading.Lock()

    def run_one(self, backend, func_name: str, func) -> None:
        if self._thread is not None and self._thread.isRunning():
            return

        self._thread = QThread()
        with self._worker_lock:
            self._worker = _Worker(self._core, backend, func_name, func)
        self._worker.moveToThread(self._thread)

        self._worker.log_line.connect(self.log_line)
        self._worker.test_started.connect(self.test_started)
        self._worker.test_finished.connect(self.test_finished)
        self._worker.test_progress.connect(self.test_progress)
        self._worker.error.connect(self.error)

        self._thread.started.connect(self._worker.run)
        self._worker.test_finished.connect(lambda *_: self._thread.quit())
        self._thread.finished.connect(self._on_thread_done)

        self._thread.start()

    def stop(self) -> None:
        with self._worker_lock:
            w = self._worker
        print(f"[TestRunner] stop() called, worker={w}")
        if w is not None:
            w.request_stop()

    def force_kill(self) -> None:
        import subprocess
        print(f"[TestRunner] force_kill() called")

        with self._worker_lock:
            w = self._worker
        print(f"[TestRunner] force_kill: worker={w}")
        if w is not None:
            w.kill_child()

        # Kill Gazebo
        try:
            proc = getattr(self._core.simulator, "gazebo_process", None)
            if proc is not None:
                try:
                    proc.kill()
                except Exception:
                    pass
            subprocess.run(["pkill", "-9", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-9", "-f", "gzclient"], check=False)
            self._core.simulator.gazebo_is_running = False
        except Exception as exc:
            print(f"[TestRunner] force_kill gazebo error: {exc}")

    def get_tests(self, backend) -> dict:
        return self._core.get_tests(backend)

    def _on_thread_done(self):
        with self._worker_lock:
            self._worker = None
        self._thread = None
        self.all_finished.emit()


class _Worker(QObject):

    log_line      = pyqtSignal(str)
    test_started  = pyqtSignal(str)
    test_finished = pyqtSignal(str, dict, str, float)
    test_progress = pyqtSignal(str, int)
    error         = pyqtSignal(str, str)

    _POLL_INTERVAL = 0.05

    def __init__(self, core, backend, func_name: str, func):
        super().__init__()
        self._core      = core
        self._backend   = backend
        self._func_name = func_name
        self._func      = func
        self._stop      = threading.Event()
        self._child_lock = threading.Lock()
        self._child: mp.Process | None = None

    def request_stop(self):
        print(f"[Worker:{self._func_name}] request_stop called, already_set={self._stop.is_set()}")
        self._stop.set()

    def kill_child(self):
        with self._child_lock:
            child = self._child
        print(f"[Worker:{self._func_name}] kill_child called, child={child}, alive={child.is_alive() if child else 'N/A'}")
        if child is not None:
            try:
                os.kill(child.pid, signal.SIGKILL)
                print(f"[Worker:{self._func_name}] SIGKILL sent to pid={child.pid}")
            except (ProcessLookupError, OSError) as e:
                print(f"[Worker:{self._func_name}] kill_child SIGKILL failed: {e}")

    def run(self):
        name = self._func_name
        self.log_line.emit(f"Start {name}")
        self.test_started.emit(name)

        ctx          = mp.get_context("fork")
        result_queue = ctx.Queue()
        child        = ctx.Process(
            target = _run_test_in_process,
            args   = (self._func, self._core.simulator, name, result_queue),
            daemon = True,
        )

        with self._child_lock:
            self._child = child

        t0 = time.time()
        child.start()
        print(f"[Worker:{name}] child started pid={child.pid}")

        while child.is_alive():
            if self._stop.is_set():
                print(f"[Worker:{name}] stop event detected, joining child pid={child.pid} for 0.3s")
                child.join(timeout=0.3)
                if child.is_alive():
                    print(f"[Worker:{name}] child still alive after 0.3s, sending SIGKILL to pid={child.pid}")
                    try:
                        os.kill(child.pid, signal.SIGKILL)
                    except (ProcessLookupError, OSError) as e:
                        print(f"[Worker:{name}] SIGKILL failed: {e}")
                    child.join(timeout=2.0)
                    print(f"[Worker:{name}] after SIGKILL join: alive={child.is_alive()} exitcode={child.exitcode}")
                else:
                    print(f"[Worker:{name}] child exited cleanly within 0.3s, exitcode={child.exitcode}")
                break
            child.join(timeout=self._POLL_INTERVAL)

        duration = time.time() - t0
        print(f"[Worker:{name}] poll loop done, duration={duration:.1f}s exitcode={child.exitcode} stop_set={self._stop.is_set()}")

        with self._child_lock:
            self._child = None

        exitcode = child.exitcode

        was_killed = (exitcode is not None and exitcode < 0)
        print(f"[Worker:{name}] was_killed={was_killed} stop_set={self._stop.is_set()}")
        if self._stop.is_set() or was_killed:
            self.log_line.emit(f"{name}  Stopped  ({duration:.1f}s)")
            self.test_finished.emit(name, {}, "Stopped", duration)
            return

        try:
            data = result_queue.get(timeout=1.0)
        except Exception:
            self.log_line.emit(f"{name}  no result returned (exitcode={exitcode})  ({duration:.1f}s)")
            self.test_finished.emit(name, {}, "Failed", duration)
            return

        if "error" in data:
            self.log_line.emit(f"{name}  {data['error']}")
            self.error.emit(name, data["error"])

        status = data.get("status", "Failed")
        result = data.get("result", {})
        self.log_line.emit(f"{name}  {status}  ({duration:.1f}s)")
        if isinstance(result, dict):
            for k, v in result.items():
                if k != "passed":
                    self.log_line.emit(f"     {k}: {v}")

        self.test_finished.emit(name, result, status, duration)