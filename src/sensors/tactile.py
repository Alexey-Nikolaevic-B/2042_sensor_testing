import math
import re
import time
from typing import Optional, Dict, Any

import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ContactsState

from .sensor import Sensor, register_sensor


@register_sensor("tactile", "tactile_sensor")
class Tactile(Sensor):
    """Тактильный датчик (во всех тестах предполагаем, что датчик находится в начале координат)"""
    BUMPER_TOPIC = "/tactile_sensor/bumper_states"

    """Тактильный датчик"""
    def __init__(self, CONFIG, sensor_sdf_path: Optional[str] = None):
        super().__init__()

        SENSORS_PATH = CONFIG["SENSORS_PATH"]
        WORLDS_PATH = CONFIG["WORLDS_PATH"]

        self.sensor_sdf_path = sensor_sdf_path if sensor_sdf_path else f'{SENSORS_PATH}{self.sensor_type}/{self.sensor_name}.sdf'

        self.test_to_world = {
            'min_force_test': f'{WORLDS_PATH}tactile/min_force.world',
        }

        self.update_rate = self.update_rate_from_sdf(self.sensor_sdf_path)


    @staticmethod
    def update_rate_from_sdf(sdf_path: str) -> int:
        """Получает параметр датчика update_rate из sdf файла"""
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        m = re.search(r"<update_rate>\s*(\d+)\s*</update_rate>", content)
        if not m:
            raise RuntimeError(f"update_rate not found in {sdf_path}")
        return int(m.group(1))


    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        window: float = 0.5,
        timeout_per_msg: float = 0.25,
        **kwargs,
    ) -> Optional[Dict[str, Any]]:
        """Собирает сырые сообщения с топика bumper_states (ContactsState) в течение window секунд."""
        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None

        messages = []
        deadline = time.time() + max(0.0, float(window))
        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(
                    self.BUMPER_TOPIC, ContactsState, timeout=timeout_per_msg
                )
                messages.append(msg)
            except rospy.ROSException:
                continue

        return {"contacts_states": messages}


    def get_params(self) -> Dict[str, Any]:
        return {"update_rate": self.update_rate}


    def set_params(self, **params) -> None:
        if "update_rate" in params:
            update_rate = params["update_rate"]
            if update_rate < 1:
                raise ValueError("update_rate must be greater than 1 or equal to 1")
            self.update_rate = update_rate


    def min_force_test(self, simulator, progress_cb=None) -> Optional[Dict[str, Any]]:
        """Определение минимальной силы, при которой датчик стабильно срабатывает"""
        max_force = 10
        step = 0.5
        threshold_force = 2.0
        current_force = 0
        attempts = 3
        result = None
        test_start_time = time.time()

        if not simulator.open_scene(self.test_to_world['min_force_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        while current_force <= max_force:
            for _ in range(attempts):
                simulator.apply_body_wrench(
                    body_name="presser::link",
                    force_x=0.0,
                    force_y=0.0,
                    force_z=-current_force,
                    duration_sec=1.0,
                )
                time.sleep(0.5)
                data = self.capture_data(simulator)
                time.sleep(0.5)

                if data and data["contacts_states"]:
                    result = current_force
                    break
            current_force += step

        test_duration_in_seconds = time.time() - test_start_time
        is_test_passed = bool(result is not None and result <= threshold_force)

        return {
            'passed': is_test_passed,
            'expected_result': f"≤ {threshold_force}",
            'duration_in_seconds': test_duration_in_seconds,
            'result': result,
        }
