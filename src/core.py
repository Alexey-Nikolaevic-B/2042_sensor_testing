import logging
from typing import Optional

from .gazebo_simulator import Simulator
from .sensors.sensor import Sensor
from .sensors.detector import detect_sensor_type
from config import CONFIG

logger = logging.getLogger(__name__)


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def detect_sensor_type(self, sdf_path: str) -> Optional[str]:
        return detect_sensor_type(sdf_path)

    def read_sensor_params(self, sensor: Sensor) -> dict:
        import src.database.sensor_storage as db
        type_def = db.get_sensor_type(sensor.sensor_type)
        if not type_def:
            return {}
        param_names = [p["name"] for p in type_def.get("params", []) if p.get("name")]
        return sensor.read_params_from_sdf(param_names)

    def get_world_for_test(self, sensor_type: str, func_name: str) -> str:
        import src.database.sensor_storage as db
        tests = {t["func_name"]: t for t in db.get_type_tests(sensor_type)}
        return tests.get(func_name, {}).get("world_path", "")

    def get_tests(self, sensor) -> dict[str, callable]:
        from src.tests import get_tests_for_type
        return get_tests_for_type(sensor.sensor_type)

    def get_tests_for_sensor(self, sensor) -> dict[str, callable]:
        return self.get_tests(sensor)

    def kill(self) -> None:
        self.simulator.kill()