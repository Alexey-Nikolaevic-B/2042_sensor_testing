from typing import List, Dict, Any, Optional

from .gazebo_simulator import Simulator
from .sensors import REGISTRY, Sensor
from .sensors.detector import detect_sensor_type as _detect

from config import CONFIG


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def get_sensor_types(self) -> List[str]:
        return sorted(REGISTRY.keys())

    def get_tests(self, sensor) -> Dict[str, Any]:
        """Return {func_name: callable} for all tests registered for this sensor's type."""
        from src.tests.tests import get_tests_for_type
        sensor_type = getattr(sensor, "sensor_type", None)
        if not sensor_type:
            return {}
        return get_tests_for_type(sensor_type)

    def detect_sensor_type(self, sdf_path: str) -> Optional[str]:
        return _detect(sdf_path)

    def read_sensor_params(self, sensor) -> dict:
        """Read param values from SDF using the path/name defs stored in the sensor type DB."""
        import src.database.sensor_storage as db

        sensor_type = getattr(sensor, "sensor_type", None)
        sdf_path    = getattr(sensor, "sdf_path", None)
        if not sensor_type or not sdf_path:
            return {}

        type_def = db.get_sensor_type(sensor_type)
        if not type_def:
            return {}

        param_defs = []
        for p in type_def.get("params", []):
            if isinstance(p, dict) and p.get("name"):
                param_defs.append(p)
            elif isinstance(p, str) and p:
                param_defs.append(p)
        if not param_defs:
            return {}

        return sensor.read_params_from_sdf(param_defs)

    def save_sensor_params(self, sensor_id: str, params: dict, repo) -> None:
        sensor_data = repo.get_sensor(sensor_id)
        if sensor_data is None:
            raise KeyError(f"No sensor with id {sensor_id!r}")

        sensor_type = sensor_data.get("type")
        sdf_path    = sensor_data.get("sdf_path")

        SensorClass = REGISTRY.get(sensor_type)
        if SensorClass is None:
            raise ValueError(f"No sensor class registered for type {sensor_type!r}")

        instance = SensorClass(sdf_path)
        instance.save_params_to_sdf(sdf_path, params)
        repo.update_sensor(sensor_id, {"params": params})

    def kill(self) -> None:
        self.simulator.kill()