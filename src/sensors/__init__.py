from .sensor import Sensor, register_sensor, make_sensor, REGISTRY

from . import rfid

__all__ = [
    'rfid',
    'mono_camera',
    'depth_camera',
    'Sensor',
    'register_sensor',
    'make_sensor',
    'REGISTRY',
]
