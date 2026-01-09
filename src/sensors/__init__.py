from .sensor import Sensor, register_sensor, make_sensor, REGISTRY

# чтобы выполнились декораторы
from . import rfid, mono_camera, depth_camera
