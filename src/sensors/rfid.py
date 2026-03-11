import re
import time
import rospy

from typing import Optional, Dict, Any
from .sensor import Sensor, register_sensor
from geometry_msgs.msg import PoseStamped


@register_sensor("rfid")
class Rfid(Sensor):
    DETECTED_TOPIC       = "/detected_tags"
    DEFAULT_READ_DISTANCE = 3

    def __init__(self, sdf_path: str):
        super().__init__()
        from config import CONFIG

        self.sensor_sdf_path   = sdf_path
        self.rfid_map_path     = CONFIG["RFID_MAP_PATH"]
        self.read_distance     = self._read_distance_from_sdf(sdf_path)

        # Model name used in Gazebo for the antenna itself.
        # Tests reference this so different antenna SDFs can have different names.
        self.antenna_model_name = CONFIG.get("RFID_ANTENNA_MODEL_NAME", "rfid_antenna")

        worlds = CONFIG["WORLDS_PATH"]
        self.test_to_world = {
            "max_stable_read_distance_test": f"{worlds}rfid/change_distance.world",
            "min_stable_read_distance_test": f"{worlds}rfid/change_distance.world",
            "mass_read_test":                f"{worlds}rfid/mass_read.world",
            "overlap_tags_test":             f"{worlds}rfid/overlap_tags.world",
            "angle_dependence_test":         f"{worlds}rfid/angle_dependence.world",
            "move_tags_test":                f"{worlds}rfid/move_tags.world",
            "antenna_rotation_test":         f"{worlds}rfid/antenna_rotation.world",
        }

    @staticmethod
    def _read_distance_from_sdf(sdf_path: str) -> int:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        m = re.search(r"<rzero>\s*(\d+)\s*</rzero>", content)
        if not m:
            raise RuntimeError(f"<rzero> tag not found in {sdf_path!r}")
        return int(m.group(1))

    def get_params(self) -> Dict[str, Any]:
        return {"read_distance": self.read_distance}

    def save_params_to_sdf(self, sdf_path: str, params: dict) -> None:
        if "read_distance" not in params:
            return
        read_distance = int(params["read_distance"])
        if read_distance <= 0:
            raise ValueError("read_distance must be > 0")

        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        content, n = re.subn(
            r"<rzero>.*?</rzero>",
            f"<rzero>{read_distance}</rzero>",
            content, count=1, flags=re.S,
        )
        if n == 0:
            raise RuntimeError(f"<rzero> tag not found in {sdf_path!r}")
        with open(sdf_path, "w", encoding="utf-8") as f:
            f.write(content)

        self.read_distance = read_distance

    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        window: float = 0.5,
        timeout_per_msg: float = 0.25,
    ) -> Dict[str, Any]:

        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return {}

        tags: Dict[str, Any] = {}
        deadline = time.time() + max(0.0, float(window))

        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(
                    self.DETECTED_TOPIC, PoseStamped, timeout=timeout_per_msg
                )
                tags[msg.header.frame_id] = msg.pose
            except rospy.ROSException:
                continue

        return tags