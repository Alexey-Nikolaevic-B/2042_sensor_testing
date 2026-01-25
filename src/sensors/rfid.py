import re
import time
import rospy

from .sensor import Sensor, register_sensor
from geometry_msgs.msg import PoseStamped
from typing import Optional, Dict, Any


@register_sensor("rfid", "rfid_antenna")
class Rfid(Sensor):
    DETECTED_TOPIC = "/detected_tags"

    """"Rfid антенна"""
    def __init__(self, CONFIG):
        super().__init__()

        WORLDS_PATH = CONFIG["WORLDS_PATH"]
        SENSORS_PATH = CONFIG["SENSORS_PATH"]

        self.sensor_sdf_path = f'{SENSORS_PATH}{self.sensor_type}/{self.sensor_name}.sdf'

        self.test_to_world = {
            'max_stable_read_distance_test': f'{WORLDS_PATH}rfid/change_distance.world',
            'min_stable_read_distance_test': f'{WORLDS_PATH}rfid/change_distance.world'
        }

        self.rfid_map_path = CONFIG['RFID_MAP_PATH']

        # значение по умолчанию для read_distance = 10
        self.set_read_distance(10)


    def set_read_distance(self, read_distance: int) -> None:
        """Меняет в sdf дальность считывания антенны"""
        self.read_distance = read_distance

        with open(self.sensor_sdf_path, "r", encoding="utf-8") as f:
            sdf_before_replace = f.read()

        sdf_after_replace, n = re.subn(r"<rzero>.*?</rzero>", f"<rzero>{self.read_distance}</rzero>", sdf_before_replace, count=1, flags=re.S)
        if n == 0:
            raise RuntimeError("rzero not found")

        with open(self.sensor_sdf_path, "w", encoding="utf-8") as f:
            f.write(sdf_after_replace)


    def get_params(self) -> Dict[str, Any]:
        return {"read_distance": self.read_distance}


    def set_params(self, **params) -> None:
        if "read_distance" in params:
            rd = int(params["read_distance"])
            if rd <= 0:
                raise ValueError("read_distance must be > 0")
            self.set_read_distance(rd)
    
    
    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        window: float = 0.5,
        timeout_per_msg: float = 0.25
    ) -> Dict[str, Any]:
        """Считать все метки"""
        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        tags: Dict[str, Any] = {}
        deadline = time.time() + max(0.0, float(window))

        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(self.DETECTED_TOPIC, PoseStamped, timeout=timeout_per_msg)
            except rospy.ROSException:
                continue
            
            tags[msg.header.frame_id] = msg.pose

        return tags


    def max_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения дальности считывания"""
        # плагин читает файл map.txt и по нему создает метки в gazebo
        with open(self.rfid_map_path, 'w') as f:
            f.write(f'fix1 1 0.5 0 0\n')
            f.flush()

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            return None
    
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        # ждем спавна rfid_tag1
        tag_name = "rfid_tag1"
        is_tag_spawned = simulator.wait_for_model_spawn(tag_name, 30)
        if not is_tag_spawned:
            raise RuntimeError("tag not spawned")

        current_dist = 0.5
        reset_distance = 25
        max_dist = 0

        while current_dist <= self.read_distance + 1e-9:

            detected_count = 0

            # делаем 10 попыток считывания метки
            for _ in range(10):
                try:
                    # перемещаем метку далеко, чтобы сбросить попытку
                    simulator.set_pose(tag_name, reset_distance, 0, 0)
                    time.sleep(0.005)

                    simulator.set_pose(tag_name, current_dist, 0, 0)

                    data = self.capture_data(simulator=simulator, world_path=None, window=1)

                    if data is not None and tag_name in data:
                        detected_count += 1

                except:
                    continue

            is_tag_detected = bool(detected_count >= 7)

            if is_tag_detected:
                max_dist = current_dist

            current_dist = round(current_dist + 0.5, 1)

        return {'max_read_distance': max_dist}


    def min_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения минимальной дальности считывания"""

        with open(self.rfid_map_path, 'w') as f:
            f.write(f'fix1 1 0.5 0 0\n')
            f.flush()

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            return None
        
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        tag_name = "rfid_tag1"
        if not simulator.wait_for_model_spawn(tag_name, 30):
            raise RuntimeError("tag not spawned")

        min_dist = current_dist = 0.25
        reset_distance = 25

        while current_dist >= 0:

            detected_count = 0

            for _ in range(4):
                try:
                    simulator.set_pose(tag_name, reset_distance, 0, 0)
                    time.sleep(0.005)

                    simulator.set_pose(tag_name, current_dist, 0, 0)
                    data = self.capture_data(simulator=simulator, world_path=None, window=1)

                    if tag_name in data:
                        detected_count += 1
                except:
                    continue

            is_tag_detected = bool(detected_count >= 3)

            if is_tag_detected:
                min_dist = current_dist

            current_dist = round(current_dist - 0.01, 5)

        return {'min_read_distance': min_dist}
