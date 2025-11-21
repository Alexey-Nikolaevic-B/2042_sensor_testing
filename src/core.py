import sys

from .sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from .gazebo_simulator import Simulator
from .tests import _tests
from .mono_camera import capture

from config import CONFIG

class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def prepare_simulator(self) -> None:
        self.simulator.launch()

    def get_sensor_categories(self) -> list:
        return get_sensor_types()

    def get_sensors_by_model(self, sensor_model: str) -> list:
        return get_sensors_by_type(sensor_model)

    def get_sensor(self, sensor_category: str, sensor_model: str) -> dict:
        return get_sensor(sensor_category, sensor_model)
    
    def run_test(self, sensor_category: str, sensor: str) -> tuple:
        return _tests.run(self.simulator, CONFIG, sensor_category, sensor)

    def capture_data(self, camera_model_path: str, world_path: str) -> any:
        data = capture(CONFIG, self.simulator, camera_model_path, world_path)
        return data

    def kill(self) -> None:
        self.simulator.kill()
        sys.exit()