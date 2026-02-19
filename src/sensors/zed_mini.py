from .sensor import register_sensor
from .stereo_profile_base import StereoProfileBase


@register_sensor("camera", "zed_mini")
class ZedMini(StereoProfileBase):
    LEFT_IMAGE_TOPIC = "/zed_mini/left/image_raw"
    RIGHT_IMAGE_TOPIC = "/zed_mini/right/image_raw"

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.780
    CLIP_NEAR = 0.3
    CLIP_FAR = 20.0
    BASELINE_M = 0.063

    MIN_DISPARITY_PX = 2
