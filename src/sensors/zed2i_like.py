from .sensor import register_sensor
from .stereo_profile_base import StereoProfileBase


@register_sensor("camera", "zed2i_like")
class Zed2iLike(StereoProfileBase):
    LEFT_IMAGE_TOPIC = "/zed2i_like/left/image_raw"
    RIGHT_IMAGE_TOPIC = "/zed2i_like/right/image_raw"

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1200
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.920
    CLIP_NEAR = 0.3
    CLIP_FAR = 20.0
    BASELINE_M = 0.12

    MIN_DISPARITY_PX = 2
