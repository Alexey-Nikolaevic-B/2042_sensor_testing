import sys

from .sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from .simulator.simulator_factory import SimulatorFactory
from .tests import _tests

from config import CONFIG

class Core:
    def __init__(self) -> None:
        self.simulator = SimulatorFactory.create_simulator('gazebo', CONFIG)

    def get_sensor_types(self) -> list:
        return get_sensor_types()

    def get_sensors_by_type(self, sensor_type: str) -> list:
        return get_sensors_by_type(sensor_type)

    def get_sensor(self, sensor_type: str, sensor_name: str) -> dict:
        return get_sensor(sensor_type, sensor_name)
    
    def run_test(self, sensor_type: str, sensor: str) -> tuple:
        return _tests.run(self.simulator, CONFIG, sensor_type, sensor)

    def kill(self) -> None:
        print('\nЗавершение процессов:')
        self.simulator.kill()
        sys.exit()