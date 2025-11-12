import sys

from .sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from .test_manager import TestManager
from .simulator.simulator_factory import SimulatorFactory

from config import CONFIG

class Core:
    def __init__(self) -> None:
        self.simulator = SimulatorFactory.create_simulator('gazebo', CONFIG)
        self.test_manager = TestManager(self.simulator, CONFIG)

    def get_sensor_types(self) -> list[str]:
        return get_sensor_types()

    def get_sensors_by_type(self, sensor_type: str) -> list[str]:
        return get_sensors_by_type(sensor_type)

    def get_sensor(self, sensor_type: str, sensor_name: str) -> dict:
        return get_sensor(sensor_type, sensor_name)
    
    def run_test(self, sensor_type: str, sensor: str) -> tuple:
        return self.test_manager.run_test(sensor_type, sensor)

    def kill(self) -> None:
        print('\nЗавершение процессов:')
        self.simulator.kill()
        sys.exit()