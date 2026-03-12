import re
import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class Sensor:
    def __init__(self, sensor_type: str, sensor_name: str, sdf_path: str,
                 topic: str = "", description: str = "",
                 image_path: str = "", params: dict = None):
        self.sensor_type = sensor_type
        self.sensor_name = sensor_name
        self.sdf_path    = sdf_path
        self.topic       = topic
        self.description = description
        self.image_path  = image_path
        self.params      = params or {}

    def param(self, name: str, default=None):
        return self.params.get(name, default)

    def read_params_from_sdf(self, param_names: list[str]) -> dict:
        if not self.sdf_path:
            return {}
        try:
            with open(self.sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError as e:
            logger.error("read_params_from_sdf: cannot read %r: %s", self.sdf_path, e)
            return {}
        result = {}
        for name in param_names:
            m = re.search(rf"<{re.escape(name)}>\s*(.*?)\s*</{re.escape(name)}>",
                          content, re.DOTALL)
            if m:
                result[name] = m.group(1).strip()
        return result

    def write_params_to_sdf(self, params: dict) -> None:
        if not self.sdf_path:
            raise ValueError("sdf_path is not set")
        try:
            with open(self.sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError as e:
            raise OSError(f"Cannot read SDF: {e}") from e
        for name, value in params.items():
            content, n = re.subn(
                rf"(<{re.escape(name)}>)\s*.*?\s*(</{re.escape(name)}>)",
                rf"\g<1>{value}\g<2>",
                content, count=1, flags=re.DOTALL,
            )
            if n == 0:
                logger.warning("write_params_to_sdf: tag <%s> not found in %s", name, self.sdf_path)
        with open(self.sdf_path, "w", encoding="utf-8") as f:
            f.write(content)
        self.params.update(params)

    def capture_data(self, msg_type, window: float = 2.0,
                     timeout: float = 0.25) -> dict:
        import rospy
        results  = {}
        deadline = time.time() + window
        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(self.topic, msg_type, timeout=timeout)
                results[msg.header.frame_id] = msg.pose
            except Exception:
                continue
        return results

    def __repr__(self):
        return f"<Sensor {self.sensor_type!r} name={self.sensor_name!r}>"