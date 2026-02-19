from .depth_profile_base import DepthProfileBase
from .sensor import register_sensor


@register_sensor("camera", "d455_like")
class D455Like(DepthProfileBase):
    DEPTH_TOPIC = "/d455_like/depth/image_raw"
    IMAGE_TOPIC = "/d455_like/image_raw"

    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 720
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.501
    CLIP_NEAR = 0.52
    CLIP_FAR = 12.0

    TEST_DISTANCES = (1.0, 3.0, 5.0)
    MAX_ABS_ERROR_M = 0.9
