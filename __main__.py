import sys
import time
import socket
import threading
import traceback

from ui.log_bridge import setup_logging
setup_logging()

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication
from src.core import Core
from ui.sensor_repository import SensorRepository
from ui.test_runner import TestRunner
from ui.queue_manager import QueueManager
from ui.ui_main import Main_UI


class _SimInitThread(QThread):
    log    = pyqtSignal(object)
    ready  = pyqtSignal()
    failed = pyqtSignal(object)

    POLL_INTERVAL = 0.5
    TIMEOUT       = 30.0

    def __init__(self, core: Core, parent=None):
        super().__init__(parent)
        self._core = core

    def run(self):
        self.log.emit(("info",    "__main__", "Starting roscore..."))
        self._core.simulator.launch_ros()
        self.log.emit(("info",    "__main__", "Waiting for roscore to be ready..."))
        deadline = time.time() + self.TIMEOUT
        while time.time() < deadline:
            try:
                with socket.create_connection(("localhost", 11311), timeout=1.0):
                    pass
                self.log.emit(("info", "__main__", "roscore is ready."))
                self.ready.emit()
                return
            except OSError:
                time.sleep(self.POLL_INTERVAL)
        self.failed.emit(("error", "__main__",
                          f"roscore did not become ready within {self.TIMEOUT:.0f}s"))


_sim_init_thread: _SimInitThread = None


def _start_simulator_init(core: Core, on_log, on_fail):
    global _sim_init_thread
    _sim_init_thread = _SimInitThread(core)

    def _on_ready():
        on_log(("info", "__main__", "Initialising ROS node..."))
        core.simulator.launch_node()
        if core.simulator.node_is_running:
            on_log(("info",  "__main__", "Simulator ready, you can now run tests."))
        else:
            on_fail(("error", "__main__", "ROS node failed to initialise (check ROS logs)."))

    _sim_init_thread.log.connect(lambda t: on_log(t))
    _sim_init_thread.ready.connect(_on_ready)
    _sim_init_thread.failed.connect(lambda t: on_fail(t))
    _sim_init_thread.start()


def _seed_db_from_registry(repo: SensorRepository) -> None:
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
            except Exception as exc:
                print(f"[seed] Could not register {sensor_name}: {exc}")
    except ImportError:
        print("[seed] Backend not available")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    core          = Core()
    repo          = SensorRepository()
    runner        = TestRunner(core)
    queue_manager = QueueManager(runner)
    window        = Main_UI()
    window.test_page.set_runner(queue_manager, repo)
    window.show()

    def _sim_log(level: str, msg: str):
        window.col_4.append_log(level, "src.gazebo_simulator", msg)
    core.simulator.on_log = _sim_log

    def _ui_log(t):
        window.col_4.append_log(t[0], t[1], t[2])

    _start_simulator_init(
        core    = core,
        on_log  = _ui_log,
        on_fail = _ui_log,
    )

    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    except Exception:
        print("\nUnexpected error:")
        traceback.print_exc()
    finally:
        if _sim_init_thread is not None:
            try:
                if _sim_init_thread.isRunning():
                    _sim_init_thread.wait(5000)
            except RuntimeError:
                pass
        if runner._thread is not None:
            runner.stop()
            try:
                if runner._thread.isRunning():
                    runner._thread.wait(10000)
            except RuntimeError:
                pass
        core.kill()