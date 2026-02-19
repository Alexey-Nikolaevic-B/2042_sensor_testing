from .depth_profile_base import DepthProfileBase
from .sensor import register_sensor


@register_sensor("camera", "azure_kinect_wfov")
class AzureKinectWfov(DepthProfileBase):
    DEPTH_TOPIC = "/azure_kinect_wfov/depth/image_raw"
    IMAGE_TOPIC = "/azure_kinect_wfov/image_raw"

    IMAGE_WIDTH = 512
    IMAGE_HEIGHT = 512
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 2.094
    CLIP_NEAR = 0.25
    CLIP_FAR = 2.88

    TEST_DISTANCES = (0.8, 1.5, 2.5)
    MAX_ABS_ERROR_M = 0.5
