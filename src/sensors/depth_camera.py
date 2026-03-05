import time
import numpy as np
from pathlib import Path

import rospy
from typing import Optional, Dict, Any, List

from sensor_msgs.msg import Image

from .sensor import register_sensor
from .mono_camera import MonoCamera


@register_sensor('camera', 'depth_camera')
class DepthCamera(MonoCamera):
    DEPTH_TOPIC = '/depth_camera/depth/image_raw'
    IMAGE_TOPIC = '/depth_camera/image_raw'

    """Камера глубины"""
    def __init__(self, CONFIG):
        super().__init__(CONFIG)

        root = Path(CONFIG.get("ROOT_PATH", "") or "")
        worlds_root = Path(CONFIG["WORLDS_PATH"])
        if not worlds_root.is_absolute():
            worlds_root = root / worlds_root
        sensors_root = Path(CONFIG["SENSORS_PATH"])
        if not sensors_root.is_absolute():
            sensors_root = root / sensors_root

        self.sensor_sdf_path = str((sensors_root / self.sensor_type / f"{self.sensor_name}.sdf").resolve())

        self.test_to_world = {
            'depth_perception_test': str(worlds_root / "depth_camera" / "depth_perception_test.world"),
        }


    @staticmethod
    def _depth_msg_to_np(msg: Image) -> np.ndarray:
        h, w = msg.height, msg.width

        if msg.encoding == "32FC1":
            arr = np.frombuffer(msg.data, dtype=np.float32)
            return arr.reshape(h, w)

        if msg.encoding == "16UC1":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            return arr.astype(np.float32) * 0.001

        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")


    @staticmethod
    def _color_msg_to_np(msg: Image) -> np.ndarray:
        h, w = msg.height, msg.width

        if msg.encoding in ("rgb8", "R8G8B8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return arr  # RGB
        if msg.encoding == "bgr8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return arr  # BGR

        raise ValueError(f"Unsupported color encoding: {msg.encoding}")


    @staticmethod
    def __apply_depth(color_cv: np.ndarray, depth_cv: np.ndarray, focus_distance: float) -> np.ndarray:
        depth_cv = depth_cv.copy()
        depth_cv[np.isnan(depth_cv)] = 0

        mx = float(depth_cv.max())
        if mx <= 1e-9:
            return color_cv.astype(np.uint8)

        focus_distance_norm = focus_distance / mx
        depth_norm = (depth_cv - depth_cv.min()) / (mx - depth_cv.min() + 1e-9)

        blur_strength = depth_norm - focus_distance_norm

        negative_mask = blur_strength < -(1 / mx)
        positive_mask = blur_strength >= 0

        blur_strength[negative_mask] = np.abs(blur_strength[negative_mask]) * 20
        blur_strength[positive_mask] = blur_strength[positive_mask]

        blur_strength = np.clip(blur_strength, 0, 1)
        blur_strength = np.clip(blur_strength * 5, 0, 1)

        k = int(max(3, min(51, round(mx))))
        if k % 2 == 0:
            k += 1

        import cv2
        blurred_img = cv2.GaussianBlur(color_cv, (k, k), 0)
        out = (1 - blur_strength[:, :, np.newaxis]) * color_cv + blur_strength[:, :, np.newaxis] * blurred_img
        return out.astype(np.uint8)


    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        timeout: float = 1.0,
        focused_image: bool = False,
        focus_distance: float = 2.0
    ) -> Optional[Dict[str, Any]]:
        """
        Snapshot:
          - depth_cv: numpy float32 (метры)
          - focused_depth_m: центральный пиксель
          - если focused_image=True: dof_cv (картинка с DOF)
        """
        if world_path:
            if not simulator.open_scene(
                world_path,
                self.sensor_sdf_path,
                expected_topics=self.get_expected_topics(),
                sensor_name=self.sensor_name,
            ):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)

        try:
            depth_msg = rospy.wait_for_message(self.DEPTH_TOPIC, Image, timeout=timeout)
        except rospy.ROSException:
            return None

        result: Dict[str, Any] = {"raw_depth": depth_msg}

        try:
            depth_cv = self._depth_msg_to_np(depth_msg)
        except Exception as e:
            result["error"] = str(e)
            return result

        h, w = depth_cv.shape[:2]
        result["focused_depth_m"] = float(depth_cv[h // 2, w // 2])
        result["depth_cv"] = depth_cv

        if focused_image:
            try:
                color_msg = rospy.wait_for_message(self.IMAGE_TOPIC, Image, timeout=timeout)
                color_cv = self._color_msg_to_np(color_msg)
                result["raw_color"] = color_msg

                if color_msg.encoding in ("rgb8", "R8G8B8"):
                    color_cv_for_cv2 = color_cv[:, :, ::-1]  # RGB -> BGR
                else:
                    color_cv_for_cv2 = color_cv  # уже BGR

                result["dof_cv"] = self.__apply_depth(color_cv_for_cv2, depth_cv, focus_distance)

            except rospy.ROSException:
                result["error"] = "No color image (timeout)"
            except Exception as e:
                result["error"] = f"focused_image failed: {e}"

        return result


    def depth_perception_test(self, simulator):
        """Тест точности измерения глубины на дистанциях 1м, 3м, 5м"""

        world_path = self.test_to_world['depth_perception_test']

        cube_name = 'close_green_cube'
        cube_z = 0.25
        reset_x = 50.0
        distances = [1.0, 3.0, 5.0]

        if not simulator.open_scene(
            world_path,
            self.sensor_sdf_path,
            expected_topics=self.get_expected_topics(),
            sensor_name=self.sensor_name,
        ):
            return None

        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        is_cube_spawned = simulator.wait_for_model_spawn(cube_name, 30)
        if not is_cube_spawned:
            raise RuntimeError(f'cube not spawned: {cube_name}')

        results = []

        for d in distances:
            simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
            time.sleep(0.5)

            simulator.set_pose(cube_name, d + 0.25, 0.0, cube_z)
            time.sleep(0.5)

            data = self.capture_data(simulator, world_path=None, timeout=1.0, focused_image=False)
            if data is None:
                results.append({'distance': d, 'error': 'No message'})
                continue
            if "error" in data:
                results.append({'distance': d, 'error': data["error"]})
                continue

            z = data.get('focused_depth_m')
            if z is None or np.isnan(z) or z == 0.0:
                results.append({'distance': d, 'error': 'Invalid measurement'})
                continue

            abs_err = abs(z - d)
            rel_err = abs_err / d * 100.0

            results.append({
                'distance': d,
                'measured': z,
                'abs_error': abs_err,
                'rel_error': rel_err
            })

        return results

    def get_expected_topics(self) -> List[str]:
        topics: List[str] = [str(self.DEPTH_TOPIC)]
        image_topic = str(self.IMAGE_TOPIC or "").strip()
        if image_topic:
            topics.append(image_topic)
        return topics
