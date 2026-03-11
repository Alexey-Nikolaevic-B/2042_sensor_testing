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
    core.simulator.start_async(
        on_ready = on_sim_ready,
        on_log   = on_sim_log,
        on_error = on_sim_error,
    )

    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass
    except Exception:
        print("\nUnexpected error:")
        traceback.print_exc()
    finally:
        runner.stop()
        core.kill()