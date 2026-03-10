import sys
from typing import List, Dict, Any

from .gazebo_simulator import Simulator
from .sensors import REGISTRY, Sensor

from config import CONFIG


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def prepare_simulator(self) -> None:
        self.simulator.launch()

    def get_sensor_types(self) -> List[str]:
        return sorted({sensor_type for sensor_type, _ in REGISTRY.keys()})

    def get_sensors_by_type(self, sensor_type: str) -> List[str]:
        return sorted({name for registered_type, name in REGISTRY.keys() if registered_type == sensor_type})

    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        return {
            attr: getattr(sensor, attr)
            for attr in dir(sensor)
            if attr.endswith('_test') and callable(getattr(sensor, attr))
        }

    def capture_data(self, sensor: Sensor, world_path, **kwargs) -> Any:
        return sensor.capture_data(self.simulator, world_path=world_path, **kwargs)

    def kill(self) -> None:
        self.simulator.kill()