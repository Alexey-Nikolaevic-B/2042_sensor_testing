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

    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        return {
            attr: getattr(sensor, attr)
            for attr in dir(sensor)
            if attr.endswith("_test") and callable(getattr(sensor, attr))
        }

    def detect_sensor_type(self, sdf_path: str) -> Optional[str]:
        return _detect(sdf_path)

    def read_sensor_params(self, sensor) -> dict:
        import re
        import src.database.sensor_storage as db

        sensor_type = getattr(sensor, "sensor_type", None)
        sdf_path    = getattr(sensor, "sdf_path", None)
        if not sensor_type or not sdf_path:
            return {}

        type_def = db.get_sensor_type(sensor_type)
        if not type_def:
            return {}

        param_names = [p["name"] for p in type_def.get("params", []) if p.get("name")]
        if not param_names:
            return {}

        try:
            with open(sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError:
            return {}

        result = {}
        for name in param_names:
            m = re.search(rf"<{re.escape(name)}>\s*(.*?)\s*</{re.escape(name)}>",
                          content, re.DOTALL)
            if m:
                result[name] = m.group(1).strip()
        return result

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