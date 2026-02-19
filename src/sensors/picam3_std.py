from .mono_profile_base import MonoProfileBase
from .sensor import register_sensor


@register_sensor("camera", "picam3_std")
class Picam3Std(MonoProfileBase):
    IMAGE_TOPIC = "/picam3_std/image_raw"

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.152
    CLIP_NEAR = 0.1
    CLIP_FAR = 50.0
