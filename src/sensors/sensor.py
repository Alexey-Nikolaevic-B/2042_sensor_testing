from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Type, Optional, Tuple


class Sensor(ABC):
    sensor_name: str
    sensor_type: str
    sensor_sdf_path: str
    test_to_world: Dict[str, str]


    @abstractmethod
    def set_params(self) -> None:
        """Для изменения параметров датчика"""
        raise NotImplementedError


    @abstractmethod
    def print_params(self) -> None:
        """Выводит параметры датчика"""
        raise NotImplementedError


REGISTRY: Dict[Tuple[str, str], Type[Sensor]] = {}


def register_sensor(sensor_type: str, sensor_name: str):
    """Декоратор для добавления датчика в фабрику"""
    def deco(SensorType: Type[Sensor]) -> Type[Sensor]:
        key = (sensor_type, sensor_name)
        REGISTRY[key] = SensorType

        SensorType.sensor_type = sensor_type
        SensorType.sensor_name = sensor_name

        return SensorType
    return deco


def make_sensor(sensor_type: str, sensor_name: str, CONFIG) -> Optional[Sensor]:
    SensorType = REGISTRY.get((sensor_type, sensor_name))
    if SensorType == None:
        return None
    return SensorType(CONFIG)
