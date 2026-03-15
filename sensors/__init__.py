from .sensor import Sensor, register_sensor, make_sensor, REGISTRY

# чтобы выполнились декораторы
from . import (
    rfid,
    mono_camera,
    depth_camera,
    uvc_profile_640x480_60deg,
    camera_sdf_registry,
)
