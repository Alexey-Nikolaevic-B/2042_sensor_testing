from .sensor import register_sensor
from .stereo_profile_base import StereoProfileBase


@register_sensor("camera", "bumblebee2_like")
class Bumblebee2Like(StereoProfileBase):
    LEFT_IMAGE_TOPIC = "/bumblebee2_like/left/image_raw"
    RIGHT_IMAGE_TOPIC = "/bumblebee2_like/right/image_raw"

    IMAGE_WIDTH = 1024
    IMAGE_HEIGHT = 768
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.222
    CLIP_NEAR = 0.3
    CLIP_FAR = 10.0
    BASELINE_M = 0.12

    MIN_DISPARITY_PX = 2
