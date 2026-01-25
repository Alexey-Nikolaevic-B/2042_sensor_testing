import sys
from typing import List, Dict, Any

from .sensor_library_utils import get_sensor_types, get_sensors_name_by_type
from .gazebo_simulator import Simulator
from .test_utils import load_test_functions
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


    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        return load_test_functions(sensor)


    def capture_data(self, sensor: Sensor, world_path, **kwargs) -> any:
        data = sensor.capture_data(self.simulator, world_path=world_path, **kwargs)
        return data


    def kill(self) -> None:
        self.simulator.kill()
        sys.exit()
