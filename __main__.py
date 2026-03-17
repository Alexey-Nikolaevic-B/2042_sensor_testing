import sys
import traceback

from front.logic_log_bridge import setup_logging
setup_logging()

from PyQt5.QtWidgets import QApplication
from src.core import Core
from front.logic_sensor_repository import SensorRepository
from front.logic_test_runner import TestRunner
from front.logic_queue_manager import QueueManager
from front.main import Main_UI


if __name__ == "__main__":
    app = QApplication(sys.argv)

    core          = Core()
    repo          = SensorRepository()
    runner        = TestRunner(core)
    queue_manager = QueueManager(runner)
    window        = Main_UI()

    window.set_core(core)
    window.test_page._runner_log_forward = window.col_4.append_log
    window.test_page.set_runner(queue_manager, repo)
    window.col_4.set_simulator(core.simulator)
    window.test_page.set_simulator(core.simulator)
    window.show()

    def on_sim_log(level: str, msg: str):
        window.col_4.append_log(level, "simulator", msg)

    def on_sim_error(msg: str):
        window.col_4.append_log("error", "simulator", msg)

    def on_sim_ready():
        core.simulator.launch_node()
        if core.simulator.node_is_running:
            window.col_4.append_log("info", "simulator", "Simulator ready, you can now run tests.")
        else:
            window.col_4.append_log("error", "simulator", "ROS node failed to initialise.")

    core.simulator.on_log = on_sim_log

    from front.logic_log_bridge import log_bridge as _log_bridge
    import logging as _logging

    _LEVEL_MAP = {
        _logging.DEBUG:    "debug",
        _logging.INFO:     "info",
        _logging.WARNING:  "warning",
        _logging.ERROR:    "error",
        _logging.CRITICAL: "critical",
    }

    def _on_log_record(record: _logging.LogRecord):
        level  = _LEVEL_MAP.get(record.levelno, "info")
        source = record.name.split(".")[-1]
        window.col_4.append_log(level, source, record.getMessage())

    _log_bridge.new_record.connect(_on_log_record)

    core.simulator.start_async(
        on_ready = on_sim_ready,
        on_log   = on_sim_log,
        on_error = on_sim_error,
    )

    runner.test_started.connect(window.col_4.clear_capture)
    runner.test_started.connect(
        lambda func_name: window.col_4.append_separator(func_name)
    )

    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    except Exception:
        print("\nUnexpected error:")
        traceback.print_exc()

    finally:

        try:
            runner.shutdown()
        except Exception as e:
            print("Runner shutdown error:", e)

        try:
            core.kill()
        except Exception as e:
            print("Core kill error:", e)