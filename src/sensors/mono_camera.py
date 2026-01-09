import re
from typing import Optional
from .sensor import Sensor, register_sensor


@register_sensor('camera', 'mono_camera')
class MonoCamera(Sensor):
    """Моно камера"""
    def __init__(self, CONFIG):
        super().__init__()

        WORLDS_PATH = CONFIG["WORLDS_PATH"]
        SENSORS_PATH = CONFIG["SENSORS_PATH"]

        self.sensor_sdf_path = f'{SENSORS_PATH}{self.sensor_type}/{self.sensor_name}.sdf'

        self.test_to_world = {}

        self._load_params_from_sdf()


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


    def print_params(self) -> None:
        print('mono camera с параметрами:')
        print(f'угол обзора (fov) = {self.horizontal_fov}')
        print(f'разрешение изображения = {self.image_width}x{self.image_height}')
        print(f'формат изображения = {self.image_format}')
        print(f'ближняя плоскость отсечения = {self.clip_near}')
        print(f'дальняя плоскость отсечения = {self.clip_far}')
        print(f'тип шума = {self.noise_type}')
        print(f'математическое ожидание шума = {self.noise_mean}')
        print(f'среднеквадратическое отклонение шума = {self.noise_stddev}')


    def set_params(self) -> None:
        self.horizontal_fov = float(input('введите угол обзора (fov, рад): '))
        self.set_horizontal_fov()

        self.image_width = int(input('введите ширину изображения: '))
        self.set_image_width()

        self.image_height = int(input('введите высоту изображения: '))
        self.set_image_height()

        self.image_format = input('введите формат изображения (например R8G8B8): ').strip()
        self.set_image_format()

        self.clip_near = float(input('введите ближнюю плоскость отсечения (м): '))
        self.set_clip_near()

        self.clip_far = float(input('введите дальнюю плоскость отсечения (м): '))
        self.set_clip_far()

        self.noise_type = input('введите тип шума (например gaussian): ').strip()
        self.set_noise_type()

        self.noise_mean = float(input('введите математическое ожидание шума: '))
        self.set_noise_mean()

        self.noise_stddev = float(input('введите среднеквадратическое отклонение шума: '))
        self.set_noise_stddev()
