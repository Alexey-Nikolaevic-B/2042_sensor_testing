from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from .sensor import Sensor


class StereoProfileBase(Sensor):
    LEFT_IMAGE_TOPIC = ""
    RIGHT_IMAGE_TOPIC = ""

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.92
    CLIP_NEAR = 0.3
    CLIP_FAR = 20.0
    BASELINE_M = 0.12

    C1_CUBE_NAME = "test_cube"
    C7_FRONT_CUBE_NAME = "front_cube"
    C7_BACK_CUBE_NAME = "back_cube"

    C4_MIN_PIXELS = 1500
    C7_CASES = {"occ_25": 0.10, "occ_50": 0.20}
    C7_MIN_PIXELS = 800
    MIN_DISPARITY_PX = 2

    def __init__(self, CONFIG):
        super().__init__()
        self.CONFIG = CONFIG

        worlds_root = CONFIG["WORLDS_PATH"]
        if not os.path.isabs(worlds_root):
            worlds_root = os.path.join(CONFIG["ROOT_PATH"], worlds_root)

        sensors_root = CONFIG["SENSORS_PATH"]
        if not os.path.isabs(sensors_root):
            sensors_root = os.path.join(CONFIG["ROOT_PATH"], sensors_root)

        self.sensor_sdf_path = os.path.join(sensors_root, self.sensor_type, f"{self.sensor_name}.sdf")

        self.test_to_world = {
            "stereo_topics_presence_test": os.path.join(worlds_root, "camera_c4_geometries.world"),
            "stereo_disparity_test": os.path.join(worlds_root, "camera_c1_single_cube.world"),
            "stereo_occlusion_test": os.path.join(worlds_root, "camera_c7_occlusion.world"),
        }

        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)
        self.baseline = float(self.BASELINE_M)

    def _results_dir(self) -> str:
        path = os.path.join(self.CONFIG["ROOT_PATH"], "results", self.sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    def _captured_dir(self) -> str:
        path = os.path.join(self._results_dir(), "captured_images")
        os.makedirs(path, exist_ok=True)
        return path

    def _save_frame(self, name: str, frame: np.ndarray) -> str:
        out = os.path.join(self._captured_dir(), name)
        cv2.imwrite(out, frame)
        return out

    def _open_test_scene(self, simulator, test_name: str) -> None:
        world = self.test_to_world[test_name]
        if not simulator.open_scene(world, self.sensor_sdf_path):
            raise RuntimeError(f"Failed to open scene for {test_name}: {world}")

        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

    @staticmethod
    def _msg_to_bgr(msg: Image) -> np.ndarray:
        h, w = msg.height, msg.width
        enc = (msg.encoding or "").lower()

        if enc in ("rgb8", "r8g8b8"):
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if enc == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

        if enc in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def _wait_pair(self, timeout: float = 35.0) -> Tuple[Image, Image]:
        left = rospy.wait_for_message(self.LEFT_IMAGE_TOPIC, Image, timeout=timeout)
        right = rospy.wait_for_message(self.RIGHT_IMAGE_TOPIC, Image, timeout=timeout)
        return left, right

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        return closed

    def _red_mask(self, hsv: np.ndarray) -> np.ndarray:
        lower_1 = np.array([0, 90, 60], dtype=np.uint8)
        upper_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_2 = np.array([170, 90, 60], dtype=np.uint8)
        upper_2 = np.array([180, 255, 255], dtype=np.uint8)
        mask_1 = cv2.inRange(hsv, lower_1, upper_1)
        mask_2 = cv2.inRange(hsv, lower_2, upper_2)
        return self._clean_mask(cv2.bitwise_or(mask_1, mask_2))

    def _color_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        if color == "red":
            return self._red_mask(hsv)

        ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
            "green": ((40, 80, 60), (85, 255, 255)),
            "blue": ((100, 90, 60), (135, 255, 255)),
            "yellow": ((20, 90, 90), (40, 255, 255)),
        }
        if color not in ranges:
            raise ValueError(f"Unsupported color: {color}")

        lower, upper = ranges[color]
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        return self._clean_mask(mask)

    @staticmethod
    def _bbox(mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, (0, 0, 0, 0)

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        return int(w * h), (int(x), int(y), int(w), int(h))

    @staticmethod
    def _count_pixels(mask: np.ndarray) -> int:
        return int(cv2.countNonZero(mask))

    @staticmethod
    def _move_and_settle(simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.8) -> None:
        simulator.set_pose(model_name, x=x, y=y, z=z)
        time.sleep(settle_s)

    def capture_data(
        self,
        simulator,
        world_path: str | None = None,
        timeout: float = 1.0,
        convert2cv: bool = False,
    ) -> Dict[str, Any] | None:
        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        left_msg, right_msg = self._wait_pair(timeout=timeout)
        result: Dict[str, Any] = {
            "raw_left": left_msg,
            "raw_right": right_msg,
        }
        if convert2cv:
            result["left_cv"] = self._msg_to_bgr(left_msg)
            result["right_cv"] = self._msg_to_bgr(right_msg)
        return result

    def set_params(self, **params) -> None:
        if "baseline" in params:
            baseline = float(params["baseline"])
            if baseline <= 0:
                raise ValueError("baseline must be > 0")
            self.baseline = baseline

    def get_params(self) -> Dict[str, Any]:
        return {
            "image_width": self.image_width,
            "image_height": self.image_height,
            "horizontal_fov": self.horizontal_fov,
            "clip_near": self.clip_near,
            "clip_far": self.clip_far,
            "baseline": self.baseline,
        }

    def stereo_topics_presence_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_topics_presence_test")

        data = self.capture_data(simulator, world_path=None, timeout=35.0, convert2cv=True)
        if data is None:
            raise RuntimeError("No stereo frames")

        left = data["left_cv"]
        right = data["right_cv"]

        if left.shape[:2] != (self.image_height, self.image_width):
            raise AssertionError(f"Left frame shape mismatch: {left.shape[:2]} != {(self.image_height, self.image_width)}")
        if right.shape[:2] != (self.image_height, self.image_width):
            raise AssertionError(f"Right frame shape mismatch: {right.shape[:2]} != {(self.image_height, self.image_width)}")

        metrics: Dict[str, Any] = {"left": {}, "right": {}, "threshold": int(self.C4_MIN_PIXELS)}

        for side, frame in (("left", left), ("right", right)):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            counts = {color: self._count_pixels(self._color_mask(hsv, color)) for color in ("red", "green", "blue", "yellow")}
            metrics[side] = counts
            self._save_frame(f"stereo_topics_{side}.png", frame)

        missing: Dict[str, Dict[str, int]] = {}
        for side in ("left", "right"):
            low = {c: n for c, n in metrics[side].items() if int(n) <= int(self.C4_MIN_PIXELS)}
            if low:
                missing[side] = low

        if missing:
            raise AssertionError(f"Stereo C4 presence failed: {missing}")

        return {"id": "STEREO_TOPICS", "metrics": metrics}

    def stereo_disparity_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_disparity_test")

        if not simulator.wait_for_model_spawn(self.C1_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C1_CUBE_NAME}")

        self._move_and_settle(simulator, self.C1_CUBE_NAME, x=3.0, y=0.0, z=0.25)

        data = self.capture_data(simulator, world_path=None, timeout=35.0, convert2cv=True)
        if data is None:
            raise RuntimeError("No stereo frames")

        left = data["left_cv"]
        right = data["right_cv"]

        left_hsv = cv2.cvtColor(left, cv2.COLOR_BGR2HSV)
        right_hsv = cv2.cvtColor(right, cv2.COLOR_BGR2HSV)

        l_area, (lx, ly, lw, lh) = self._bbox(self._red_mask(left_hsv))
        r_area, (rx, ry, rw, rh) = self._bbox(self._red_mask(right_hsv))

        if l_area == 0 or r_area == 0:
            raise AssertionError("Failed to detect red cube in stereo frames")

        l_center_x = lx + lw / 2.0
        r_center_x = rx + rw / 2.0
        disparity_px = abs(l_center_x - r_center_x)

        left_dbg = left.copy()
        right_dbg = right.copy()
        cv2.rectangle(left_dbg, (lx, ly), (lx + lw, ly + lh), (255, 255, 255), 2)
        cv2.rectangle(right_dbg, (rx, ry), (rx + rw, ry + rh), (255, 255, 255), 2)
        self._save_frame("stereo_disparity_left.png", left_dbg)
        self._save_frame("stereo_disparity_right.png", right_dbg)

        if disparity_px < float(self.MIN_DISPARITY_PX):
            raise AssertionError(f"Disparity too small: {disparity_px} px < {self.MIN_DISPARITY_PX}")

        return {
            "id": "STEREO_DISPARITY",
            "metrics": {
                "left_center_x": float(l_center_x),
                "right_center_x": float(r_center_x),
                "disparity_px": float(disparity_px),
                "min_disparity_px": float(self.MIN_DISPARITY_PX),
            },
        }

    def stereo_occlusion_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_occlusion_test")

        if not simulator.wait_for_model_spawn(self.C7_FRONT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_FRONT_CUBE_NAME}")
        if not simulator.wait_for_model_spawn(self.C7_BACK_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_BACK_CUBE_NAME}")

        self._move_and_settle(simulator, self.C7_FRONT_CUBE_NAME, x=3.0, y=0.0, z=0.25)

        metrics: Dict[str, Any] = {
            "left": {"blue_pixels": {}},
            "right": {"blue_pixels": {}},
            "cases": dict(self.C7_CASES),
            "threshold": int(self.C7_MIN_PIXELS),
        }

        for case_name, y in self.C7_CASES.items():
            self._move_and_settle(simulator, self.C7_BACK_CUBE_NAME, x=3.0, y=float(y), z=0.25)

            data = self.capture_data(simulator, world_path=None, timeout=35.0, convert2cv=True)
            if data is None:
                raise RuntimeError(f"No stereo frames for case {case_name}")

            for side, frame in (("left", data["left_cv"]), ("right", data["right_cv"])):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                blue = self._color_mask(hsv, "blue")
                blue_count = self._count_pixels(blue)
                metrics[side]["blue_pixels"][case_name] = int(blue_count)

                dbg = frame.copy()
                cv2.putText(dbg, f"{case_name}: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                self._save_frame(f"stereo_c7_{side}_{case_name}.png", dbg)

        for side in ("left", "right"):
            blue_25 = metrics[side]["blue_pixels"].get("occ_25", 0)
            blue_50 = metrics[side]["blue_pixels"].get("occ_50", 0)

            relation_ok = blue_25 > blue_50
            threshold_ok = blue_25 > self.C7_MIN_PIXELS and blue_50 > self.C7_MIN_PIXELS

            if not (relation_ok and threshold_ok):
                raise AssertionError(
                    f"Stereo C7 failed on {side}: blue_25={blue_25}, blue_50={blue_50}, threshold={self.C7_MIN_PIXELS}"
                )

        return {"id": "STEREO_C7", "metrics": metrics}
