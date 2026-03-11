from typing import List, Dict, Any

from .gazebo_simulator import Simulator
from .sensors import REGISTRY, Sensor

from config import CONFIG


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def get_sensor_types(self) -> List[str]:
        return sorted(REGISTRY.keys())

    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        """Return {func_name: bound_method} for all *_test methods on sensor."""
        return {
            attr: getattr(sensor, attr)
            for attr in dir(sensor)
            if attr.endswith("_test") and callable(getattr(sensor, attr))
        }

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