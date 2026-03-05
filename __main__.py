#!/usr/bin/env python3
"""
Startup sequence
----------------
1.  QApplication
2.  Core()               — Simulator object, no ROS yet
3.  SensorRepository()   — loads DB
4.  TestRunner, Main_UI shown
5.  _SimInitThread.start() — background thread:
      a. launch_ros()
      b. sleep until /rosout service is reachable
      c. emit ready signal → main thread calls launch_node()
"""

import sys
import time
import socket
import threading
import traceback
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication

from src.core import Core
from sensor_repository import SensorRepository
from test_runner import TestRunner
from ui_main import Main_UI


# ── ROS init thread ───────────────────────────────────────────────────────────

class _SimInitThread(QThread):
    """
    Subclasses QThread so run() is guaranteed to execute.

    Sequence inside run():
      1. launch_ros()  — safe from any thread (subprocess.Popen)
      2. Poll /rosout  — wait until roscore is ready
      3. Emit ready    — main thread picks this up and calls launch_node()
                         (rospy.init_node MUST be in main thread)
    """

    log    = pyqtSignal(str)
    ready  = pyqtSignal()
    failed = pyqtSignal(str)

    POLL_INTERVAL = 0.5
    TIMEOUT       = 30.0

    def __init__(self, core: Core, parent=None):
        super().__init__(parent)
        self._core = core

    def run(self):
        import sys
        import socket
        print("[SimInit] Thread run() started", flush=True, file=sys.stderr)

        # Step 1 — start roscore
        self.log.emit("Starting roscore...")
        print("[SimInit] Calling launch_ros()", flush=True, file=sys.stderr)
        self._core.simulator.launch_ros()
        print(f"[SimInit] launch_ros() done. ros_is_running={self._core.simulator.ros_is_running}", flush=True, file=sys.stderr)

        # Step 2 — wait until roscore's XML-RPC master port (11311) is reachable.
        # We use a plain TCP socket so rospy.init_node() is NOT required yet.
        self.log.emit("Waiting for roscore to be ready...")
        deadline = time.time() + self.TIMEOUT
        attempt  = 0
        while time.time() < deadline:
            attempt += 1
            print(f"[SimInit] Checking roscore port attempt {attempt}", flush=True, file=sys.stderr)
            try:
                with socket.create_connection(("localhost", 11311), timeout=1.0):
                    pass
                print("[SimInit] roscore port reachable — emitting ready", flush=True, file=sys.stderr)
                self.log.emit("roscore is ready.")
                self.ready.emit()
                print("[SimInit] ready signal emitted", flush=True, file=sys.stderr)
                return
            except OSError as e:
                print(f"[SimInit] roscore not ready yet: {e}", flush=True, file=sys.stderr)
                time.sleep(self.POLL_INTERVAL)

        print("[SimInit] Timed out waiting for roscore", flush=True, file=sys.stderr)
        self.failed.emit(f"roscore did not become ready within {self.TIMEOUT:.0f}s")


# Keep reference alive for the process lifetime
_sim_init_thread: _SimInitThread = None


def _start_simulator_init(core: Core, on_log, on_fail):
    global _sim_init_thread

    _sim_init_thread = _SimInitThread(core)

    def _on_ready():
        import sys
        print("[SimInit] _on_ready() called — thread:", threading.current_thread().name, flush=True, file=sys.stderr)
        on_log("Initialising ROS node...")
        core.simulator.launch_node()
        print(f"[SimInit] launch_node() done. node_is_running={core.simulator.node_is_running}", flush=True, file=sys.stderr)
        if core.simulator.node_is_running:
            on_log("✔ Simulator ready — you can now run tests.")
        else:
            on_fail("✘ ROS node failed to initialise (check ROS logs).")

    _sim_init_thread.log.connect(on_log)
    _sim_init_thread.ready.connect(_on_ready)
    _sim_init_thread.failed.connect(on_fail)
    _sim_init_thread.start()
    import sys
    print(f"[SimInit] Thread started. isRunning={_sim_init_thread.isRunning()}", flush=True, file=sys.stderr)


# ── DB seed ───────────────────────────────────────────────────────────────────

def _seed_db_from_registry(repo: SensorRepository) -> None:
    """Register backend sensors in the DB on first run."""
    try:
        from src.sensors import REGISTRY, make_sensor
        from config import CONFIG

        for (sensor_type, sensor_name) in REGISTRY.keys():
            if repo.get_sensor_by_name(sensor_name):
                continue
            try:
                instance = make_sensor(sensor_type, sensor_name, CONFIG)
                sdf_path = getattr(instance, "sensor_sdf_path", "")
                repo.add_sensor({
                    "name":     sensor_name,
                    "type":     sensor_type,
                    "sdf_path": sdf_path,
                })
                print(f"[seed] Registered: {sensor_type}/{sensor_name}")
            except Exception as exc:
                print(f"[seed] Could not register {sensor_name}: {exc}")

    except ImportError:
        print("[seed] Backend not available — skipping registry seed")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)

    core   = Core()
    repo   = SensorRepository()
    _seed_db_from_registry(repo)
    runner = TestRunner(core)
    window = Main_UI()
    window.test_page.set_runner(runner, repo)
    window.show()

    _start_simulator_init(
        core   = core,
        on_log = window.test_page.append_log,
        on_fail= window.test_page.append_log,
    )

    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    except Exception:
        print("\nUnexpected error:")
        traceback.print_exc()
    finally:
        # Wait for sim init thread if still running
        if _sim_init_thread is not None:
            try:
                if _sim_init_thread.isRunning():
                    _sim_init_thread.wait(5000)
            except RuntimeError:
                pass

        # Signal the worker to stop, then wait for it to finish cleanly
        # before tearing down ROS. Destroying the thread while it's still
        # running a blocking backend call causes a crash.
        if runner._thread is not None:
            runner._worker.request_stop() if runner._worker else None
            try:
                if runner._thread.isRunning():
                    runner._thread.wait(10000)   # wait up to 10s for clean exit
            except RuntimeError:
                pass
        core.kill()