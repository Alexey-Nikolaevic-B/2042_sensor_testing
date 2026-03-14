from .sensor import Sensor, register_sensor, make_sensor, REGISTRY

from . import (
    rfid,
    mono_camera,
    depth_camera,
    uvc_profile_640x480_60deg,
    camera_sdf_registry,
    tactile_sdf_registry,
)

__all__ = [
    'rfid',
    'mono_camera',
    'depth_camera',
    'uvc_profile_640x480_60deg',
    'camera_sdf_registry',
    'tactile_sdf_registry',
    'Sensor',
    'register_sensor',
    'make_sensor',
    'REGISTRY',
]
