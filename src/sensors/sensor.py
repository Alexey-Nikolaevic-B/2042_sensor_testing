import re
import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


def detect_topics_from_sdf(sdf_path: str) -> list[str]:
    """
    Scan an SDF file and extract all topic-like values.
    Looks for any XML tag whose name ends with 'topic'
    and whose value starts with '/'.
    Returns a deduplicated list preserving order.
    """
    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error("detect_topics_from_sdf: cannot read %r: %s", sdf_path, e)
        return []

    seen   = set()
    topics = []
    for m in re.finditer(r"<\w*[Tt]opic\w*>\s*(/[^<\s]+)\s*</\w*[Tt]opic\w*>", content):
        t = m.group(1).strip()
        if t and t not in seen:
            seen.add(t)
            topics.append(t)
    return topics


class Sensor:
    def __init__(self, sensor_type: str, sensor_name: str, sdf_path: str,
                 topics: list = None, description: str = "",
                 image_path: str = "", params: dict = None):
        self.sensor_type = sensor_type
        self.sensor_name = sensor_name
        self.sdf_path    = sdf_path
        self.topics      = list(topics or [])
        self.description = description
        self.image_path  = image_path
        self.params      = params or {}

    @property
    def topic(self) -> str:
        """Primary topic — first in list, for backwards compat."""
        return self.topics[0] if self.topics else ""

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

    def capture_data(self, msg_type, topic: str = "", window: float = 2.0,
                     timeout: float = 0.25) -> dict:
        """
        Read messages from a ROS topic for `window` seconds.
        Uses `topic` if given, otherwise falls back to self.topic (first in list).
        Returns {frame_id: pose} for each message received.
        """
        import rospy
        t = topic or self.topic
        results  = {}
        deadline = time.time() + window
        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(t, msg_type, timeout=timeout)
                results[msg.header.frame_id] = msg.pose
            except Exception:
                continue
        return results

    def __repr__(self):
        return f"<Sensor {self.sensor_type!r} name={self.sensor_name!r}>"


# ── Legacy REGISTRY (used by rfid.py, core.py, widget_col_3.py) ───────────────
from typing import Dict, Type as _Type

REGISTRY: Dict[str, _Type] = {}


def register_sensor(sensor_type: str):
    """Decorator to register a concrete sensor class by type string."""
    def deco(cls):
        cls.sensor_type = sensor_type
        REGISTRY[sensor_type] = cls
        return cls
    return deco