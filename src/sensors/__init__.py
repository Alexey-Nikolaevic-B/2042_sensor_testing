from .sensor import Sensor, register_sensor, make_sensor, REGISTRY

# чтобы выполнились декораторы
from . import (
    rfid,
    mono_camera,
    depth_camera,
    uvc_profile_640x480_60deg,
    axis_wide_110deg,
    picam3_std,
    d435_like,
    d455_like,
    azure_kinect_wfov,
    zed2i_like,
    bumblebee2_like,
    zed_mini,
)
