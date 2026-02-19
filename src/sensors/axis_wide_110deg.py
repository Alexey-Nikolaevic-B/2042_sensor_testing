from .mono_profile_base import MonoProfileBase
from .sensor import register_sensor


@register_sensor("camera", "axis_wide_110deg")
class AxisWide110Deg(MonoProfileBase):
    IMAGE_TOPIC = "/axis_wide_110deg/image_raw"

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.920
    CLIP_NEAR = 0.1
    CLIP_FAR = 100.0
