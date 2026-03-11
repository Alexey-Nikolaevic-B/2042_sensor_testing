from abc import ABC, abstractmethod
from typing import Dict, Type, Optional, Any


class Sensor(ABC):
    sensor_type: str
    sdf_path:    str

    @abstractmethod
    def get_params(self) -> Dict[str, Any]:
        """Return current sensor parameters as a dict."""
        raise NotImplementedError

    @abstractmethod
    def save_params_to_sdf(self, sdf_path: str, params: dict) -> None:
        """Write params into the SDF file at sdf_path."""
        raise NotImplementedError


# Registry key is sensor_type only
REGISTRY: Dict[str, Type["Sensor"]] = {}


def register_sensor(sensor_type: str):
    """Decorator to register a sensor class by type."""
    def deco(SensorClass: Type[Sensor]) -> Type[Sensor]:
        SensorClass.sensor_type = sensor_type
        REGISTRY[sensor_type] = SensorClass
        return SensorClass
    return deco


def make_sensor(sensor_type: str, sdf_path: str) -> Optional[Sensor]:
    SensorClass = REGISTRY.get(sensor_type)
    if SensorClass is None:
        return None
    return SensorClass(sdf_path)