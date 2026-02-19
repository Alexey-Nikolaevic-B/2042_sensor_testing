from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from .mono_camera import MonoCamera


class MonoProfileBase(MonoCamera):
    IMAGE_TOPIC = ""

    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.0471975512
    CLIP_NEAR = 0.1
    CLIP_FAR = 50.0

    C1_CUBE_NAME = "test_cube"
    C7_FRONT_CUBE_NAME = "front_cube"
    C7_BACK_CUBE_NAME = "back_cube"

    C1_POSITIONS = (1.0, 3.0, 5.0)
    C1_MIN_MARGIN_RATIO = 1.10
    C4_MIN_PIXELS = 1500
    C7_CASES = {"occ_25": 0.10, "occ_50": 0.20}
    C7_MIN_PIXELS = 800

    def __init__(self, CONFIG):
        super().__init__(CONFIG)

        self.CONFIG = CONFIG
        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)

        worlds_root = CONFIG["WORLDS_PATH"]
        if not os.path.isabs(worlds_root):
            worlds_root = os.path.join(CONFIG["ROOT_PATH"], worlds_root)

        self.test_to_world = {
            "c1_size_order_test": os.path.join(worlds_root, "camera_c1_single_cube.world"),
            "c4_geometries_presence_test": os.path.join(worlds_root, "camera_c4_geometries.world"),
            "c7_occlusion_test": os.path.join(worlds_root, "camera_c7_occlusion.world"),
        }

    def _results_dir(self) -> str:
        path = os.path.join(self.CONFIG["ROOT_PATH"], "results", self.sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    def _captured_dir(self) -> str:
        path = os.path.join(self._results_dir(), "captured_images")
        os.makedirs(path, exist_ok=True)
        return path

    def _wait_image(self, timeout: float = 35.0) -> Image:
        return rospy.wait_for_message(self.IMAGE_TOPIC, Image, timeout=timeout)

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
    def _bbox_area(mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, (0, 0, 0, 0)

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        return int(w * h), (int(x), int(y), int(w), int(h))

    @staticmethod
    def _count_pixels(mask: np.ndarray) -> int:
        return int(cv2.countNonZero(mask))

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
    def _move_and_settle(simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.8) -> None:
        simulator.set_pose(model_name, x=x, y=y, z=z)
        time.sleep(settle_s)

    def c1_size_order_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "positions": list(self.C1_POSITIONS),
            "bbox_area_px": {},
            "min_margin_ratio": float(self.C1_MIN_MARGIN_RATIO),
        }

        self._open_test_scene(simulator, "c1_size_order_test")
        if not simulator.wait_for_model_spawn(self.C1_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C1_CUBE_NAME}")

        for x in self.C1_POSITIONS:
            label = f"x{int(x)}"
            self._move_and_settle(simulator, self.C1_CUBE_NAME, x=float(x), y=0.0, z=0.25)

            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            red = self._red_mask(hsv)
            area, (bx, by, bw, bh) = self._bbox_area(red)
            metrics["bbox_area_px"][label] = int(area)

            debug = frame.copy()
            if area > 0:
                cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
            cv2.putText(debug, f"{label}: area={area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            artifacts.append(self._save_frame(f"c1_{label}.png", debug))

        x1 = metrics["bbox_area_px"].get("x1", 0)
        x3 = metrics["bbox_area_px"].get("x3", 0)
        x5 = metrics["bbox_area_px"].get("x5", 0)
        order_ok = x1 > x3 > x5
        margin_ok = (x1 >= x3 * self.C1_MIN_MARGIN_RATIO) and (x3 >= x5 * self.C1_MIN_MARGIN_RATIO)
        metrics["checks"] = {"size_order": bool(order_ok), "size_margin": bool(margin_ok)}

        if not (order_ok and margin_ok):
            raise AssertionError(f"C1 checks failed: {metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}")

        return {"id": "C1", "metrics": metrics, "artifacts": artifacts}

    def c4_geometries_presence_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {"pixel_counts": {}, "threshold": int(self.C4_MIN_PIXELS)}

        self._open_test_scene(simulator, "c4_geometries_presence_test")
        msg = self._wait_image(timeout=35.0)
        frame = self._msg_to_bgr(msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color in ("red", "green", "blue", "yellow"):
            metrics["pixel_counts"][color] = self._count_pixels(self._color_mask(hsv, color))

        artifacts.append(self._save_frame("c4_geometries.png", frame))
        missing = {c: n for c, n in metrics["pixel_counts"].items() if int(n) <= int(metrics["threshold"])}
        metrics["checks"] = {"all_present": len(missing) == 0, "missing_or_low": missing}

        if missing:
            raise AssertionError(f"C4 checks failed: missing_or_low={missing}, threshold={metrics['threshold']}")

        return {"id": "C4", "metrics": metrics, "artifacts": artifacts}

    def c7_occlusion_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "cases": dict(self.C7_CASES),
            "blue_pixels": {},
            "threshold": int(self.C7_MIN_PIXELS),
        }

        self._open_test_scene(simulator, "c7_occlusion_test")
        if not simulator.wait_for_model_spawn(self.C7_FRONT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_FRONT_CUBE_NAME}")
        if not simulator.wait_for_model_spawn(self.C7_BACK_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_BACK_CUBE_NAME}")

        self._move_and_settle(simulator, self.C7_FRONT_CUBE_NAME, x=3.0, y=0.0, z=0.25)

        for case_name, y in self.C7_CASES.items():
            self._move_and_settle(simulator, self.C7_BACK_CUBE_NAME, x=3.0, y=float(y), z=0.25)

            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            blue = self._color_mask(hsv, "blue")
            blue_count = self._count_pixels(blue)
            metrics["blue_pixels"][case_name] = int(blue_count)

            debug = frame.copy()
            cv2.putText(debug, f"{case_name}: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            artifacts.append(self._save_frame(f"c7_{case_name}.png", debug))

        blue_25 = metrics["blue_pixels"].get("occ_25", 0)
        blue_50 = metrics["blue_pixels"].get("occ_50", 0)
        relation_ok = blue_25 > blue_50
        threshold_ok = blue_25 > self.C7_MIN_PIXELS and blue_50 > self.C7_MIN_PIXELS
        metrics["checks"] = {"occlusion_relation": bool(relation_ok), "threshold_ok": bool(threshold_ok)}

        if not (relation_ok and threshold_ok):
            raise AssertionError(
                f"C7 checks failed: {metrics['checks']}, blue_pixels={metrics['blue_pixels']}, "
                f"threshold={self.C7_MIN_PIXELS}"
            )

        return {"id": "C7", "metrics": metrics, "artifacts": artifacts}
