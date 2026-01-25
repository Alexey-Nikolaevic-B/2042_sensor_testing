from abc import ABC, abstractmethod
from typing import Dict, Type, Optional, Tuple, Any


class Sensor(ABC):
    sensor_name: str
    sensor_type: str
    sensor_sdf_path: str
    test_to_world: Dict[str, str]


    @abstractmethod
    def capture_data(self, simulator, world_path : Optional[str] = None, **kwargs) -> Optional[Dict[str, Any]]:
        """
        Сделать одно измерение датчика
        Если вызывается внутри теста - world_path = None
        Если из ui - нужно передать корректный world_path
        """
        raise NotImplementedError


    @abstractmethod
    def set_params(self, **params) -> None:
        """Принять новые параметры"""
        raise NotImplementedError


    @abstractmethod
    def get_params(self) -> None:
        """Вернуть текущие параметры"""
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
