import re
import rospy
from pathlib import Path

from sensor_msgs.msg import Image

from typing import Optional, Dict, Any, List
from .sensor import Sensor, register_sensor


@register_sensor('camera', 'mono_camera')
class MonoCamera(Sensor):
    IMAGE_TOPIC = '/mono_camera/image_raw'

    """Моно камера"""
    def __init__(self, CONFIG):
        super().__init__()

        root = Path(CONFIG.get("ROOT_PATH", "") or "")
        sensors_root = Path(CONFIG["SENSORS_PATH"])
        if not sensors_root.is_absolute():
            sensors_root = root / sensors_root

        self.sensor_sdf_path = str((sensors_root / self.sensor_type / f"{self.sensor_name}.sdf").resolve())

        self.test_to_world = {}

        self._load_params_from_sdf()


    def capture_data(
        self,
        simulator,
        world_path : Optional[str] = None,
        timeout: float = 1.0,
        convert2cv = False
    ) -> Optional[Dict[str, Any]]:
        if world_path:
            if not simulator.open_scene(
                world_path,
                self.sensor_sdf_path,
                expected_topics=self.get_expected_topics(),
                sensor_name=self.sensor_name,
            ):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)

        msg = rospy.wait_for_message(self.IMAGE_TOPIC, Image, timeout=timeout)

        if msg is None:
            return None
        
        result = {"raw_image": msg}

        if convert2cv:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            result["cv_image"] = cv

        return result


    def _read_sdf(self) -> str:
        with open(self.sensor_sdf_path, "r", encoding="utf-8") as f:
            return f.read()


    def _write_sdf(self, text: str) -> None:
        with open(self.sensor_sdf_path, "w", encoding="utf-8") as f:
            f.write(text)


    def _replace_one(self, pattern: str, replacement: str, what: str) -> None:
        text = self._read_sdf()
        new_text, n = re.subn(pattern, replacement, text, count=1, flags=re.S)
        if n == 0:
            raise RuntimeError(f"{what} not found in {self.sensor_sdf_path}")
        self._write_sdf(new_text)


    def _extract_one(self, pattern: str, cast, default):
        text = self._read_sdf()
        m = re.search(pattern, text, flags=re.S)
        if not m:
            return default
        try:
            return cast(m.group(1).strip())
        except Exception:
            return default


    def _cam_prefix(self) -> str:
        return r'<camera\b[^>]*>.*?'


    def _load_params_from_sdf(self) -> None:
        # дефотные значения
        self.horizontal_fov = 1.3
        self.image_width = 1280
        self.image_height = 720
        self.image_format = "R8G8B8"
        self.clip_near = 0.05
        self.clip_far = 100.0
        self.noise_type = "gaussian"
        self.noise_mean = 0.0
        self.noise_stddev = 0.0

        cam = self._cam_prefix()

        self.horizontal_fov = self._extract_one(
            cam + r'<horizontal_fov>\s*([^<]+)\s*</horizontal_fov>',
            float, self.horizontal_fov
        )

        self.image_width = self._extract_one(
            cam + r'<image>.*?<width>\s*([^<]+)\s*</width>',
            int, self.image_width
        )
        self.image_height = self._extract_one(
            cam + r'<image>.*?<height>\s*([^<]+)\s*</height>',
            int, self.image_height
        )
        self.image_format = self._extract_one(
            cam + r'<image>.*?<format>\s*([^<]+)\s*</format>',
            str, self.image_format
        )

        self.clip_near = self._extract_one(
            cam + r'<clip>.*?<near>\s*([^<]+)\s*</near>',
            float, self.clip_near
        )
        self.clip_far = self._extract_one(
            cam + r'<clip>.*?<far>\s*([^<]+)\s*</far>',
            float, self.clip_far
        )

        self.noise_type = self._extract_one(
            cam + r'<noise>.*?<type>\s*([^<]+)\s*</type>',
            str, self.noise_type
        )
        self.noise_mean = self._extract_one(
            cam + r'<noise>.*?<mean>\s*([^<]+)\s*</mean>',
            float, self.noise_mean
        )
        self.noise_stddev = self._extract_one(
            cam + r'<noise>.*?<stddev>\s*([^<]+)\s*</stddev>',
            float, self.noise_stddev
        )


    def set_horizontal_fov(self, value: Optional[float] = None) -> None:
        if value is not None:
            self.horizontal_fov = float(value)
        pattern = r'(<camera\b[^>]*>.*?<horizontal_fov>)(.*?)(</horizontal_fov>)'
        self._replace_one(pattern, rf'\1{self.horizontal_fov}\3', "horizontal_fov")


    def set_image_width(self, value: Optional[int] = None) -> None:
        if value is not None:
            self.image_width = int(value)
        pattern = r'(<camera\b[^>]*>.*?<image>.*?<width>)(.*?)(</width>)'
        self._replace_one(pattern, rf'\1{self.image_width}\3', "image width")


    def set_image_height(self, value: Optional[int] = None) -> None:
        if value is not None:
            self.image_height = int(value)
        pattern = r'(<camera\b[^>]*>.*?<image>.*?<height>)(.*?)(</height>)'
        self._replace_one(pattern, rf'\1{self.image_height}\3', "image height")


    def set_image_format(self, value: Optional[str] = None) -> None:
        if value is not None:
            self.image_format = str(value)
        pattern = r'(<camera\b[^>]*>.*?<image>.*?<format>)(.*?)(</format>)'
        self._replace_one(pattern, rf'\1{self.image_format}\3', "image format")


    def set_clip_near(self, value: Optional[float] = None) -> None:
        if value is not None:
            self.clip_near = float(value)
        pattern = r'(<camera\b[^>]*>.*?<clip>.*?<near>)(.*?)(</near>)'
        self._replace_one(pattern, rf'\1{self.clip_near}\3', "clip near")


    def set_clip_far(self, value: Optional[float] = None) -> None:
        if value is not None:
            self.clip_far = float(value)
        pattern = r'(<camera\b[^>]*>.*?<clip>.*?<far>)(.*?)(</far>)'
        self._replace_one(pattern, rf'\1{self.clip_far}\3', "clip far")


    def set_noise_type(self, value: Optional[str] = None) -> None:
        if value is not None:
            self.noise_type = str(value)
        pattern = r'(<camera\b[^>]*>.*?<noise>.*?<type>)(.*?)(</type>)'
        self._replace_one(pattern, rf'\1{self.noise_type}\3', "noise type")


    def set_noise_mean(self, value: Optional[float] = None) -> None:
        if value is not None:
            self.noise_mean = float(value)
        pattern = r'(<camera\b[^>]*>.*?<noise>.*?<mean>)(.*?)(</mean>)'
        self._replace_one(pattern, rf'\1{self.noise_mean}\3', "noise mean")


    def set_noise_stddev(self, value: Optional[float] = None) -> None:
        if value is not None:
            self.noise_stddev = float(value)
        pattern = r'(<camera\b[^>]*>.*?<noise>.*?<stddev>)(.*?)(</stddev>)'
        self._replace_one(pattern, rf'\1{self.noise_stddev}\3', "noise stddev")


    def get_params(self) -> Dict[str, Any]:
        return {
            "horizontal_fov": self.horizontal_fov,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "image_format": self.image_format,
            "clip_near": self.clip_near,
            "clip_far": self.clip_far,
            "noise_type": self.noise_type,
            "noise_mean": self.noise_mean,
            "noise_stddev": self.noise_stddev,
        }
    

    def set_params(self, **params) -> None:
        if "horizontal_fov" in params:
            self.set_horizontal_fov(float(params["horizontal_fov"]))
        if "image_width" in params:
            self.set_image_width(int(params["image_width"]))
        if "image_height" in params:
            self.set_image_height(int(params["image_height"]))
        if "image_format" in params:
            self.set_image_format(str(params["image_format"]))
        if "clip_near" in params:
            self.set_clip_near(float(params["clip_near"]))
        if "clip_far" in params:
            self.set_clip_far(float(params["clip_far"]))
        if "noise_type" in params:
            self.set_noise_type(str(params["noise_type"]))
        if "noise_mean" in params:
            self.set_noise_mean(float(params["noise_mean"]))
        if "noise_stddev" in params:
            self.set_noise_stddev(float(params["noise_stddev"]))

    def get_expected_topics(self) -> List[str]:
        return [str(self.IMAGE_TOPIC)]
