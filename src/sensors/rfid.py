from .sensor import Sensor, register_sensor
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from typing import Optional, Dict, Any


import re
import time
import rospy


@register_sensor("rfid", "rfid_antenna")
class Rfid(Sensor):
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


    def print_params(self):
        print('rfid антенна с параметрами:')
        print(f'дальность считывания = {self.read_distance}')


    def set_params(self) -> None:
        distance = int(input('введите дальность считывания: '))
        if distance <= 0:
            raise ValueError("дальность считывания должна быть > 0")
        self.set_read_distance(distance)


    @staticmethod
    def _set_pose(set_state, model, x, y, z) -> None:
        """Вспомогательный метод для перемещения метки в Gazebo"""
        state = ModelState()
        state.model_name = model
        state.reference_frame = "world"
        state.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
        response = set_state(state)
        if not response.success:
            raise RuntimeError(response.status_message)
        

    @staticmethod
    def _wait_for_tag_spawn(tag_name: str, timeout = 10) -> bool:
        start_time = time.time()
        while (time.time() - start_time < timeout):
            try:
                msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
                if tag_name in msg.name:
                    return True
            except rospy.ROSException:
                continue
        return False


    def max_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения дальности считывания"""

        # плагин читает файл map.txt и по нему создает метки в gazebo
        with open(self.rfid_map_path, 'w') as f:
            f.write(f'fix1 1 0.5 0 0\n')
            f.flush()

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            return None

        # ждем спавна rfid_tag1
        tag_name = "rfid_tag1"
        is_tag_spawned = self._wait_for_tag_spawn(tag_name, 30)
        if not is_tag_spawned:
            raise RuntimeError("tag not spawned")

        # создаем сервис для перемещения rfid_tag1
        rospy.wait_for_service("/gazebo/set_model_state", timeout=5)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        current_dist = 0.5
        reset_distance = 50
        max_dist = 0

        while current_dist <= 10.0 + 1e-9:

            detected_count = 0

            # делаем 20 попыток считывания метки
            for _ in range(20):
                try:
                    # перемещаем метку далеко, чтобы сбросить попытку
                    self._set_pose(set_state, tag_name, reset_distance, 0, 0)
                    time.sleep(0.025)
                    # перемещаем метку на тестовую дистанцию
                    self._set_pose(set_state, tag_name, current_dist, 0, 0)
                    msg = rospy.wait_for_message('/detected_tags', PoseStamped, timeout=0.1)
                    if msg.header.frame_id == tag_name:
                        detected_count += 1
                except:
                    continue

            is_tag_detected = bool(detected_count >= 8)

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

        tag_name = "rfid_tag1"
        if not self._wait_for_tag_spawn(tag_name, 30):
            raise RuntimeError("tag not spawned")

        rospy.wait_for_service("/gazebo/set_model_state", timeout=5)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        min_dist = current_dist = 0.5
        reset_distance = 50

        while current_dist >= 0:

            detected_count = 0

            for _ in range(5):
                try:
                    self._set_pose(set_state, tag_name, reset_distance, 0, 0)
                    time.sleep(0.005)
                    self._set_pose(set_state, tag_name, current_dist, 0, 0)
                    msg = rospy.wait_for_message('/detected_tags', PoseStamped, timeout=0.1)
                    if msg.header.frame_id == tag_name:
                        detected_count += 1
                except:
                    continue

            is_tag_detected = bool(detected_count >= 4)

            if is_tag_detected:
                min_dist = current_dist

            current_dist = round(current_dist - 0.01, 5)

        return {'min_read_distance': min_dist}
