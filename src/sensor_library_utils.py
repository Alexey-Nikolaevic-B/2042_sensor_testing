from typing import List
from .sensors import REGISTRY


def get_sensor_types() -> List[str]:
    """Возвращает список из всех типов датчиков"""
    sensor_types = set()
    for type, _ in REGISTRY.keys():
        sensor_types.add(type)

    return sorted(list(sensor_types))


def get_sensors_name_by_type(sensor_type: str) -> List[str]:
    """Возвращает все датчики, тип которых совпадает с sensor_type"""
    sensors = set()
    for type, name in REGISTRY.keys():
        if type == sensor_type:
            sensors.add(name)

    return sorted(list(sensors))
