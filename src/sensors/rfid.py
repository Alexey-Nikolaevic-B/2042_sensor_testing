import re
import time
import rospy
import math

from typing import Optional, Dict, Any
from .sensor import Sensor, register_sensor
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3


@register_sensor("rfid", "rfid_antenna")
class Rfid(Sensor):
    """Во всех тестах предполагаем, что антенна находится в начале координат!"""
    DETECTED_TOPIC = "/detected_tags"
    DEFAULT_READ_DISTANCE = 3

    """"Rfid антенна"""
    def __init__(self, CONFIG, sensor_sdf_path : Optional[str] = None):
        super().__init__()

        WORLDS_PATH = CONFIG["WORLDS_PATH"]
        SENSORS_PATH = CONFIG["SENSORS_PATH"]

        self.sensor_sdf_path = sensor_sdf_path if sensor_sdf_path else f'{SENSORS_PATH}{self.sensor_type}/{self.sensor_name}.sdf'

        self.test_to_world = {
            'max_stable_read_distance_test': f'{WORLDS_PATH}rfid/change_distance.world',
            'min_stable_read_distance_test': f'{WORLDS_PATH}rfid/change_distance.world',
            'mass_read_test': f'{WORLDS_PATH}rfid/mass_read.world',
            'overlap_tags_test': f'{WORLDS_PATH}rfid/overlap_tags.world',
            'angle_dependence_test': f'{WORLDS_PATH}rfid/angle_dependence.world',
            'move_tags_test': f'{WORLDS_PATH}rfid/move_tags.world',
            'antenna_rotation_test': f'{WORLDS_PATH}rfid/antenna_rotation.world',
        }

        self.rfid_map_path = CONFIG['RFID_MAP_PATH']

        if sensor_sdf_path is None:
            # если не передан явно путь к sdf, то мы записываем дефолтное значение дальности считывания
            self.set_read_distance(self.DEFAULT_READ_DISTANCE)
        else:
            self.read_distance = self.read_distance_from_sdf(self.sensor_sdf_path)


    @staticmethod
    def read_distance_from_sdf(sdf_path: str) -> int:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        m = re.search(r"<rzero>\s*(\d+)\s*</rzero>", content)
        if not m:
            raise RuntimeError(f"rzero not found in {sdf_path}")
        return int(m.group(1))


    def set_read_distance(self, read_distance: int) -> None:
        """
        Меняет в sdf дальность считывания антенны
        Нужно для того, чтобы в консольной версии приложения можно было менять параметры датчика
        """
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
            read_distance = int(params["read_distance"])
            if read_distance <= 0:
                raise ValueError("read_distance must be > 0")
            self.set_read_distance(read_distance)


    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        window: float = 0.5,
        timeout_per_msg: float = 0.25
    ) -> Dict[str, Any]:
        """Считывает все метрки и возвращает dict: tag_id -> pose"""
        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None

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
        """Тест для определения дальности считывания rfid антенны"""
        with open(self.rfid_map_path, 'w') as f:
            f.write(f'fix1 1 0.5 0 0\n')

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        # ждем спавна rfid_tag1
        tag_name = "rfid_tag1"
        is_tag_spawned = simulator.wait_for_model_spawn(tag_name, 30)
        if not is_tag_spawned:
            raise RuntimeError("tag not spawned")

        current_dist = 0.5
        reset_distance = self.read_distance * 5
        max_dist = 0

        start_test_time = time.time()

        while current_dist <= self.read_distance + 1e-9:
            detected = False

            try:
                simulator.set_pose(tag_name, reset_distance, 0, 0)
                time.sleep(0.005)

                simulator.set_pose(tag_name, current_dist, 0, 0)

                data = self.capture_data(simulator=simulator, world_path=None, window=3)

                if data is not None and tag_name in data:
                    detected = True
            except Exception:
                continue

            if detected:
                max_dist = current_dist

            current_dist = round(current_dist + 0.5, 1)

        test_duration_in_seconds = time.time() - start_test_time
        is_test_passed = bool(abs(max_dist - self.read_distance) <= 0.5)
        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'max_read_distance': max_dist,
        }


    def min_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения минимальной дальности считывания rfid антенны"""
        with open(self.rfid_map_path, 'w') as f:
            f.write('fix1 1 0.5 0 0\n')

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        tag_name = "rfid_tag1"
        if not simulator.wait_for_model_spawn(tag_name, 30):
            raise RuntimeError("tag not spawned")

        min_dist = current_dist = 0.25
        reset_distance = 25
        start_test_time = time.time()

        while current_dist >= 0:
            detected = False
            try:
                simulator.set_pose(tag_name, reset_distance, 0, 0)
                time.sleep(0.005)

                simulator.set_pose(tag_name, current_dist, 0, 0)
                data = self.capture_data(simulator=simulator, world_path=None, window=2)

                if tag_name in data:
                    detected = True
            except Exception:
                continue

            if detected:
                min_dist = current_dist
            current_dist = round(current_dist - 0.01, 5)

        test_duration_in_seconds = time.time() - start_test_time
        is_test_passed = bool(min_dist <= 0.05)
        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'min_read_distance': min_dist,
        }


    def mass_read_test(self, simulator) -> Optional[Dict[str, Any]]:
        # TODO: сейчас для дефолтного rfid датчика считывается только 13 меток из 75
        """Оценка корректности считывания большого количества меток"""
        tags_count = 75 # в условиии теста ровно 75 меток

        # Будем считать, что метки расположены на окружности радиусом read_distance / 2
        radius = self.read_distance / 2

        with open(self.rfid_map_path, 'w') as f:
            for i in range(tags_count):
                x = radius * math.cos(2 * math.pi / tags_count * i)
                y = radius * math.sin(2 * math.pi / tags_count * i)
                f.write(f'fix{i + 1} {i + 1} {x} {y} 0\n')

        if not simulator.open_scene(self.test_to_world['mass_read_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f'rfid_tag{i + 1}', 30):
                raise RuntimeError(f"tag {i + 1} not spawned")

        test_start_time = time.time()
        tag_id2pose = self.capture_data(simulator=simulator, world_path=None, window=20)
        test_duration_in_seconds = time.time() - test_start_time

        is_test_passed = bool(len(tag_id2pose) / tags_count >= 0.75)
        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'tags_detected_count': len(tag_id2pose),
        }


    def overlap_tags_test(self, simulator) -> Optional[Dict[str, Any]]:
        """
        Считывание при частичном перекрытии меток
        Метки расположены на одной окружности на разных расстояниях друг от друга
        Тест возвращает минимальное расстояние между метками, при котором все они были считаны
        None - если на любом расстоянии не удавалось считать все метки
        """
        distances_between_tags = [0.2, 0.1, 0.05, 0.02]
        radius = self.read_distance / 2 # радиус окружности, на которой расположены метки
        tags_count = 5 # в условиии теста ровно 5 меток
        dist_result = None

        start_test_time = time.time()

        for distance in distances_between_tags:
            with open(self.rfid_map_path, 'w') as f:
                "Создаем tags_count меток на окружности радиусом read_distance / 2 на расстоянии distance друг от друга"
                for i in range(tags_count):
                    x = radius * math.cos(distance / radius * i)
                    y = radius * math.sin(distance / radius * i)
                    f.write(f'fix{i + 1} {i + 1} {x} {y} 0\n')

            if not simulator.open_scene(self.test_to_world['overlap_tags_test'], self.sensor_sdf_path):
                raise RuntimeError("failed to open gazebo scene")
        
            for i in range(tags_count):
                if not simulator.wait_for_model_spawn(f'rfid_tag{i + 1}', 30):
                    raise RuntimeError(f"tag {i + 1} not spawned")

            tag_id2pose = self.capture_data(simulator=simulator, world_path=None, window=5)
            if len(tag_id2pose) == tags_count:
                dist_result = distance

        test_duration_in_seconds = time.time() - start_test_time
        is_test_passed = bool(dist_result is not None)
        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'dist_result': dist_result,
        }


    def angle_dependence_test(self, simulator) -> Optional[Dict[str, Any]]:
        """
        Тест для определение зависимости успешности считывания от угла расположения меток
        Критерий прохождения теста: все метки были считаны антенной
        """
        angles = [0, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
        radius = self.read_distance / 2

        with open(self.rfid_map_path, 'w') as f:
            # располагаем метки над считывателем
            for tag_num, angle in enumerate(angles):
                x = radius * math.sin(angle)
                z = radius * math.cos(angle)
                f.write(f'fix{tag_num + 1} {tag_num + 1} {x} 0 {z}\n')

        if not simulator.open_scene(self.test_to_world['angle_dependence_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        for i in range(len(angles)):
            if not simulator.wait_for_model_spawn(f'rfid_tag{i + 1}', 30):
                raise RuntimeError(f"tag {i + 1} not spawned")

        test_start_time = time.time()
        tag_id2pose = self.capture_data(simulator=simulator, world_path=None, window=20)
        duration_in_seconds = time.time() - test_start_time
        is_test_passed = bool(len(tag_id2pose) == len(angles))
        return {
            "passed": is_test_passed,
            "duration": duration_in_seconds,
            "tags_detected_count": len(tag_id2pose),
        }

    def move_tags_test(self, simulator) -> Optional[Dict[str, Any]]:
        """
        Оценка устойчивости считывания при движении меток
        Критерий прохождения теста: хотя бы одна метка была считана антенной
        Замечание: с текущим плагином rfid_tag_plugin тест не проходит, так как метки в нем static
        """
        velocities = [Vector3(0.5, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)]
        result_velocity = None
        dist_in_antenna_radius = 1.5
        start_distance = -1 * self.read_distance * dist_in_antenna_radius
        start_test_time = time.time()

        for velocity in velocities:
            with open(self.rfid_map_path, 'w') as f:
                f.write(f'fix1 1 {start_distance} 0 0\n')
            if not simulator.open_scene(self.test_to_world['move_tags_test'], self.sensor_sdf_path):
                raise RuntimeError("failed to open gazebo scene")

            if not simulator.wait_for_model_spawn(f'rfid_tag1', 30):
                raise RuntimeError("tag not spawned")

            simulator.set_pose(f'rfid_tag1', x=start_distance, y=0, z=0, linear_velocity=velocity)
            time_to_reach_antenna = dist_in_antenna_radius * self.read_distance / velocity.x
            tag_id2pose = self.capture_data(simulator=simulator, world_path=None, window=time_to_reach_antenna * 2)
            result_velocity = velocity if len(tag_id2pose) == 1 else result_velocity
            if result_velocity is None:
                break

        test_duration_in_seconds = time.time() - start_test_time
        is_test_passed = bool(result_velocity is not None)
        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'max_detected_velocity': result_velocity,
        }


    def antenna_rotation_test(self, simulator) -> Optional[Dict[str, Any]]:
        """
        Оценка устойчивости считывания при вращении антенны
        Критерий прохождения теста: при ориентации 0 градусов все метки были считаны
        """
        angles = [0, math.pi / 6, math.pi / 3, math.pi / 2]
        tags_count = 10 # в условии теста 10 меток
        radius = self.read_distance / 2
        is_test_passed = False
        angle2tags_count = {}

        with open(self.rfid_map_path, 'w') as f:
            for i in range(tags_count):
                x = radius * math.cos(2 * math.pi / tags_count * i)
                y = radius * math.sin(2 * math.pi / tags_count * i)
                f.write(f'fix{i + 1} {i + 1} {x} {y} 0\n')

        if not simulator.open_scene(self.test_to_world['antenna_rotation_test'], self.sensor_sdf_path):
            raise RuntimeError("failed to open gazebo scene")

        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f'rfid_tag{i + 1}', 30):
                raise RuntimeError(f"tag {i + 1} not spawned")

        test_start_time = time.time()

        for angle in angles:
            quaternion = Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
            simulator.set_pose(f'rfid_antenna', x=0, y=0, z=0, quaternion=quaternion)
            time.sleep(0.01)
            tag_id2pose = self.capture_data(simulator=simulator, world_path=None, window=7)
            if len(tag_id2pose) == tags_count and angle == 0:
                is_test_passed = True
            angle2tags_count[math.degrees(angle)] = len(tag_id2pose)

        test_duration_in_seconds = time.time() - test_start_time

        return {
            'passed': is_test_passed,
            'duration': test_duration_in_seconds,
            'angle2tags_count': angle2tags_count,
        }
