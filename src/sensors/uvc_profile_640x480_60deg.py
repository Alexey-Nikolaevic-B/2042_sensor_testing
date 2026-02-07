"""
Что это за файл:
- Регистрация профиля виртуальной mono RGB камеры `uvc_profile_640x480_60deg`
  в реестре сенсоров проекта через декоратор @register_sensor.

Как запускать:
- Через основной CLI проекта: `python3 __main__.py` -> выбрать тип `camera` и этот профиль.
- Через suite раннер: `python3 catkin_ws/src/scenario_test_pkg/scripts/run_camera_suite.py --camera uvc_profile_640x480_60deg`.

Топики:
- Основной кадр: /uvc_profile_640x480_60deg/image_raw

Сцены/тесты:
- Сцены: scene_c1.launch, scene_c4.launch, scene_c7.launch
- Тесты: C1/C4/C7 (через run_camera_suite.py)
"""

from math import radians

from .mono_camera import MonoCamera
from .sensor import register_sensor


@register_sensor("camera", "uvc_profile_640x480_60deg")
class UvcProfile640x48060Deg(MonoCamera):
    """Профиль виртуальной моно-камеры 640x480, FOV 60deg, clip 0.1..50, 30 FPS."""

    IMAGE_TOPIC = "/uvc_profile_640x480_60deg/image_raw"
    CAMERA_MODEL_NAME = "uvc_profile_640x480_60deg_model"

    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    UPDATE_RATE = 30

    HORIZONTAL_FOV_DEG = 60.0
    HORIZONTAL_FOV_RAD = radians(HORIZONTAL_FOV_DEG)

    CLIP_NEAR = 0.1
    CLIP_FAR = 50.0

    def __init__(self, CONFIG):
        super().__init__(CONFIG)

        # Явно фиксируем параметры профиля в Python-объекте,
        # чтобы раннер мог читать ожидаемые значения из одного места.
        self.image_width = self.IMAGE_WIDTH
        self.image_height = self.IMAGE_HEIGHT
        self.horizontal_fov = self.HORIZONTAL_FOV_RAD
        self.clip_near = self.CLIP_NEAR
        self.clip_far = self.CLIP_FAR
        self.update_rate = self.UPDATE_RATE
