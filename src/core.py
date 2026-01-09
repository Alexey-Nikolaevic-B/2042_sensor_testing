import sys
from typing import List, Dict

from .sensor_library_utils import get_sensor_types, get_sensors_name_by_type
from .gazebo_simulator import Simulator
from .test_utils import run_tests
from .mono_camera import capture
from .sensors import Sensor

from config import CONFIG


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)


    def prepare_simulator(self) -> None:
        self.simulator.launch()


    def get_sensor_types(self) -> List[str]:
        return get_sensor_types()


    def get_sensors_name_by_type(self, sensor_type: str) -> List[str]:
        return get_sensors_name_by_type(sensor_type)


    def run_test(self, sensor: Sensor) -> Dict:
        return run_tests(self.simulator, sensor)


    def capture_data(self, camera_model_path: str, world_path: str) -> any:
        data = capture(CONFIG, self.simulator, camera_model_path, world_path)
        return data


    def kill(self) -> None:
        self.simulator.kill()
        sys.exit()
