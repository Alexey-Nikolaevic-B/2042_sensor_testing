from __future__ import annotations

import json
import os
import time
from math import atan, degrees
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
    C2_SPHERE_A_NAME = "sphere_a"
    C2_SPHERE_B_NAME = "sphere_b"
    C7_FRONT_CUBE_NAME = "front_cube"
    C7_BACK_CUBE_NAME = "back_cube"
    C9_SPHERE_NAME = "fov_sphere"
    C10_CUBE_NAME = "clip_cube"

    C1_POSITIONS = (1.0, 3.0, 5.0)
    C2_DISTANCES = (0.20, 0.15, 0.10, 0.05, 0.02)
    C2_MIN_CONTOUR_AREA = 80
    C1_MIN_MARGIN_RATIO = 1.10
    C4_MIN_PIXELS = 1500
    C7_CASES = {"occ_25": 0.10, "occ_50": 0.20}
    C7_MIN_PIXELS = 800
    C9_STEP = 0.05
    C9_MAX_Y = 4.0
    C9_MIN_CONTOUR_AREA = 120
    C10_NEAR_START_X = 0.05
    C10_NEAR_SEARCH_END_X = 2.0
    C10_NEAR_STEP = 0.01
    C10_FAR_COARSE_STEP = 0.5
    C10_FAR_FINE_STEP = 0.01
    C10_MIN_RED_PIXELS = 20
    C11_DURATION_S = 60.0
    C11_MAX_JITTER_S = 0.015

    def __init__(self, CONFIG):
        super().__init__(CONFIG)

        self.CONFIG = CONFIG
        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)

        worlds_root = Path(CONFIG["WORLDS_PATH"])

        self.test_to_world = {
            "c1_size_order_test": str(worlds_root / "camera_c1_single_cube.world"),
            "c2_resolution_test": str(worlds_root / "camera_c2_resolution.world"),
            "c4_geometries_presence_test": str(worlds_root / "camera_c4_geometries.world"),
            "c7_occlusion_test": str(worlds_root / "camera_c7_occlusion.world"),
            "c9_fov_test": str(worlds_root / "camera_c9_fov.world"),
            "c10_clipping_test": str(worlds_root / "camera_c10_clipping.world"),
            "c11_fps_stability_test": str(worlds_root / "camera_c11_fps_static_load.world"),
        }

    def _results_dir(self) -> str:
        path = os.path.join(self.CONFIG["ROOT_PATH"], "results", self.sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    def _captured_dir(self) -> str:
        path = os.path.join(self._results_dir(), "captured_images")
        os.makedirs(path, exist_ok=True)
        return path

    def _metrics_dir(self) -> str:
        path = os.path.join(self._results_dir(), "metrics")
        os.makedirs(path, exist_ok=True)
        return path

    def _save_metrics_json(self, name: str, payload: Dict[str, Any]) -> str:
        out = os.path.join(self._metrics_dir(), name)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return out

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

    @staticmethod
    def _iter_float_range(start: float, stop: float, step: float) -> List[float]:
        values: List[float] = []
        cur = float(start)
        while cur <= float(stop) + 1e-9:
            values.append(round(cur, 4))
            cur += float(step)
        return values

    def _white_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, 175], dtype=np.uint8),
            np.array([180, 85, 255], dtype=np.uint8),
        )
        return self._clean_mask(mask)

    @staticmethod
    def _large_contours(
        mask: np.ndarray,
        min_area: float,
        border_margin: int = 3,
    ) -> List[Tuple[float, Tuple[int, int, int, int]]]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = mask.shape[:2]

        result: List[Tuple[float, Tuple[int, int, int, int]]] = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < float(min_area):
                continue

            x, y, cw, ch = cv2.boundingRect(cnt)
            if x <= border_margin or y <= border_margin:
                continue
            if x + cw >= (w - border_margin) or y + ch >= (h - border_margin):
                continue

            result.append((area, (int(x), int(y), int(cw), int(ch))))
        return result

    def _red_visibility(self, frame_bgr: np.ndarray) -> Tuple[bool, int]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        red = self._red_mask(hsv)
        red_pixels = self._count_pixels(red)
        return red_pixels >= int(self.C10_MIN_RED_PIXELS), int(red_pixels)

    @staticmethod
    def _annotate(frame: np.ndarray, lines: List[str]) -> np.ndarray:
        debug = frame.copy()
        y = 30
        for line in lines:
            cv2.putText(debug, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            y += 28
        return debug

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

    def c2_resolution_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "distances_m": [float(d) for d in self.C2_DISTANCES],
            "large_contours": {},
            "separated": {},
            "min_contour_area_px": int(self.C2_MIN_CONTOUR_AREA),
            "timings_s": {},
        }

        self._open_test_scene(simulator, "c2_resolution_test")
        for model in (self.C2_SPHERE_A_NAME, self.C2_SPHERE_B_NAME):
            if not simulator.wait_for_model_spawn(model, timeout=20):
                raise RuntimeError(f"Model not spawned: {model}")

        self._move_and_settle(simulator, self.C2_SPHERE_A_NAME, x=3.0, y=0.0, z=0.25, settle_s=0.6)

        d_min: Optional[float] = None
        for d in self.C2_DISTANCES:
            started = time.perf_counter()
            self._move_and_settle(simulator, self.C2_SPHERE_B_NAME, x=3.0, y=float(d), z=0.25, settle_s=0.5)

            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            white = self._white_mask(frame)
            contours = self._large_contours(white, min_area=self.C2_MIN_CONTOUR_AREA, border_margin=4)

            d_key = f"{d:.2f}"
            contour_count = len(contours)
            separated = contour_count >= 2
            metrics["large_contours"][d_key] = int(contour_count)
            metrics["separated"][d_key] = bool(separated)
            metrics["timings_s"][d_key] = round(time.perf_counter() - started, 4)

            if separated and (d_min is None or d < d_min):
                d_min = float(d)

            debug = frame.copy()
            for _, (x, y, w, h) in contours:
                cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 255, 255), 2)
            debug = self._annotate(debug, [f"d={d:.2f}m", f"contours={contour_count}", f"separated={separated}"])
            artifacts.append(self._save_frame(f"c2_d_{d_key.replace('.', '_')}.png", debug))

        checks = {
            "d_0_10_separated": bool(metrics["separated"].get("0.10", False)),
            "d_0_05_separated": bool(metrics["separated"].get("0.05", False)),
            "d_min_le_0_05": bool(d_min is not None and d_min <= 0.05),
        }
        checks["pass"] = bool((checks["d_0_10_separated"] and checks["d_0_05_separated"]) or checks["d_min_le_0_05"])
        metrics["d_min_m"] = None if d_min is None else round(float(d_min), 4)
        metrics["checks"] = checks

        metrics_path = self._save_metrics_json("c2_resolution_metrics.json", metrics)
        if not checks["pass"]:
            raise AssertionError(f"C2 checks failed: {checks}, large_contours={metrics['large_contours']}")

        return {"id": "C2", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c9_fov_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        x_fixed = 2.0

        metrics: Dict[str, Any] = {
            "x_fixed_m": float(x_fixed),
            "step_y_m": float(self.C9_STEP),
            "target_fov_rad": float(self.horizontal_fov),
            "samples": [],
        }

        self._open_test_scene(simulator, "c9_fov_test")
        if not simulator.wait_for_model_spawn(self.C9_SPHERE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C9_SPHERE_NAME}")

        last_visible: Optional[Tuple[float, np.ndarray, int]] = None
        first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None

        for y in self._iter_float_range(0.0, float(self.C9_MAX_Y), float(self.C9_STEP)):
            self._move_and_settle(simulator, self.C9_SPHERE_NAME, x=x_fixed, y=float(y), z=0.2, settle_s=0.35)

            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            white = self._white_mask(frame)
            contours = self._large_contours(white, min_area=self.C9_MIN_CONTOUR_AREA, border_margin=4)
            contour_count = len(contours)
            visible = contour_count > 0

            metrics["samples"].append({"y_m": float(y), "visible": bool(visible), "contours": int(contour_count)})

            if visible:
                last_visible = (float(y), frame, contour_count)
            elif last_visible is not None:
                first_not_visible = (float(y), frame, contour_count)
                break

        if last_visible is None:
            raise AssertionError("C9 failed: object was never detected in frame")
        if first_not_visible is None:
            raise AssertionError("C9 failed: object did not disappear within tested Y range")

        y_max = float(last_visible[0])
        y_lost = float(first_not_visible[0])
        measured_fov = float(2.0 * atan(y_max / x_fixed))
        target_fov = float(self.horizontal_fov)
        rel_error = float(abs(measured_fov - target_fov) / target_fov) if target_fov > 0 else float("inf")

        dbg_visible = self._annotate(
            last_visible[1],
            [f"Ymax={y_max:.2f} m", f"FOVmeasured={measured_fov:.5f} rad", "visible=True"],
        )
        artifacts.append(self._save_frame("c9_ymax_visible.png", dbg_visible))

        dbg_lost = self._annotate(
            first_not_visible[1],
            [f"Y={y_lost:.2f} m", f"FOVtarget={target_fov:.5f} rad", "visible=False"],
        )
        artifacts.append(self._save_frame("c9_after_ymax_not_visible.png", dbg_lost))

        metrics["y_max_visible_m"] = y_max
        metrics["y_first_not_visible_m"] = y_lost
        metrics["fov_measured_rad"] = measured_fov
        metrics["fov_measured_deg"] = float(degrees(measured_fov))
        metrics["fov_target_deg"] = float(degrees(target_fov))
        metrics["relative_error"] = rel_error
        metrics["checks"] = {"rel_error_le_0_02": bool(rel_error <= 0.02)}

        metrics_path = self._save_metrics_json("c9_fov_metrics.json", metrics)
        if rel_error > 0.02:
            raise AssertionError(
                f"C9 failed: measured={measured_fov:.6f} rad, target={target_fov:.6f} rad, rel_error={rel_error:.4f}"
            )

        return {"id": "C9", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c10_clipping_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "near_clip_target_m": float(self.clip_near),
            "far_clip_target_m": float(self.clip_far),
            "near_search": {"start_x": float(self.C10_NEAR_START_X), "end_x": float(self.C10_NEAR_SEARCH_END_X)},
            "far_search": {"coarse_step": float(self.C10_FAR_COARSE_STEP), "fine_step": float(self.C10_FAR_FINE_STEP)},
            "min_red_pixels": int(self.C10_MIN_RED_PIXELS),
        }

        self._open_test_scene(simulator, "c10_clipping_test")
        if not simulator.wait_for_model_spawn(self.C10_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C10_CUBE_NAME}")

        near_before: Optional[Tuple[float, np.ndarray, int]] = None
        near_after: Optional[Tuple[float, np.ndarray, int]] = None

        for x in self._iter_float_range(self.C10_NEAR_START_X, self.C10_NEAR_SEARCH_END_X, self.C10_NEAR_STEP):
            self._move_and_settle(simulator, self.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.2)
            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            visible, red_pixels = self._red_visibility(frame)
            if visible:
                near_after = (float(x), frame, red_pixels)
                break
            near_before = (float(x), frame, red_pixels)

        if near_after is None:
            raise AssertionError("C10 failed: clip_cube did not appear in near search range")

        near_x = float(near_after[0])
        metrics["x_near_m"] = near_x

        far_last_visible = near_after
        far_first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None

        far_search_stop = float(self.clip_far) + 5.0
        for x in self._iter_float_range(
            near_x + float(self.C10_FAR_COARSE_STEP),
            far_search_stop,
            float(self.C10_FAR_COARSE_STEP),
        ):
            self._move_and_settle(simulator, self.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.2)
            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            visible, red_pixels = self._red_visibility(frame)
            if visible:
                far_last_visible = (float(x), frame, red_pixels)
            else:
                far_first_not_visible = (float(x), frame, red_pixels)
                break

        if far_first_not_visible is None:
            raise AssertionError("C10 failed: clip_cube did not disappear in far search range")

        fine_start = max(float(near_x), float(far_last_visible[0]) - float(self.C10_FAR_COARSE_STEP))
        fine_end = float(far_first_not_visible[0])
        far_last_visible_fine = far_last_visible
        far_first_not_visible_fine = far_first_not_visible

        for x in self._iter_float_range(fine_start, fine_end, float(self.C10_FAR_FINE_STEP)):
            self._move_and_settle(simulator, self.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.15)
            msg = self._wait_image(timeout=35.0)
            frame = self._msg_to_bgr(msg)
            visible, red_pixels = self._red_visibility(frame)
            if visible:
                far_last_visible_fine = (float(x), frame, red_pixels)
            else:
                far_first_not_visible_fine = (float(x), frame, red_pixels)
                break

        far_x = float(far_last_visible_fine[0])
        metrics["x_far_m"] = far_x
        metrics["x_far_first_not_visible_m"] = float(far_first_not_visible_fine[0])

        near_ok = abs(near_x - float(self.clip_near)) <= 0.05
        far_ok = abs(far_x - float(self.clip_far)) <= 0.05
        metrics["checks"] = {
            "near_abs_error_m": float(abs(near_x - float(self.clip_near))),
            "far_abs_error_m": float(abs(far_x - float(self.clip_far))),
            "near_ok": bool(near_ok),
            "far_ok": bool(far_ok),
        }

        if near_before is not None:
            dbg = self._annotate(
                near_before[1],
                [f"near_before x={near_before[0]:.2f}", f"red_px={near_before[2]}", "visible=False"],
            )
            artifacts.append(self._save_frame("c10_near_before.png", dbg))

        dbg = self._annotate(
            near_after[1],
            [f"near_after x={near_after[0]:.2f}", f"red_px={near_after[2]}", "visible=True"],
        )
        artifacts.append(self._save_frame("c10_near_after.png", dbg))

        dbg = self._annotate(
            far_last_visible_fine[1],
            [f"far_before x={far_last_visible_fine[0]:.2f}", f"red_px={far_last_visible_fine[2]}", "visible=True"],
        )
        artifacts.append(self._save_frame("c10_far_before.png", dbg))

        dbg = self._annotate(
            far_first_not_visible_fine[1],
            [f"far_after x={far_first_not_visible_fine[0]:.2f}", f"red_px={far_first_not_visible_fine[2]}", "visible=False"],
        )
        artifacts.append(self._save_frame("c10_far_after.png", dbg))

        metrics_path = self._save_metrics_json("c10_clipping_metrics.json", metrics)
        if not (near_ok and far_ok):
            raise AssertionError(
                f"C10 failed: near={near_x:.3f} (target {self.clip_near:.3f}), "
                f"far={far_x:.3f} (target {self.clip_far:.3f})"
            )

        return {"id": "C10", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c11_fps_stability_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "duration_target_s": float(self.C11_DURATION_S),
            "update_rate_target_hz": float(self.update_rate),
            "jitter_limit_s": float(self.C11_MAX_JITTER_S),
        }

        self._open_test_scene(simulator, "c11_fps_stability_test")
        self._wait_image(timeout=35.0)

        timestamps: List[float] = []
        first_msg: Dict[str, Optional[Image]] = {"msg": None}
        last_msg: Dict[str, Optional[Image]] = {"msg": None}

        def _on_image(msg: Image) -> None:
            stamp = float(msg.header.stamp.to_sec())
            if stamp <= 0.0:
                stamp = float(rospy.Time.now().to_sec())
            timestamps.append(stamp)
            if first_msg["msg"] is None:
                first_msg["msg"] = msg
            last_msg["msg"] = msg

        sub = rospy.Subscriber(self.IMAGE_TOPIC, Image, _on_image, queue_size=2000)
        started_wall = time.perf_counter()
        try:
            while (time.perf_counter() - started_wall) < float(self.C11_DURATION_S):
                time.sleep(0.1)
        finally:
            sub.unregister()

        metrics["duration_actual_s"] = round(time.perf_counter() - started_wall, 4)
        if len(timestamps) < 2:
            raise AssertionError(f"C11 failed: not enough frames captured ({len(timestamps)})")

        monotonic_stamps: List[float] = []
        for ts in timestamps:
            if not monotonic_stamps or ts > monotonic_stamps[-1]:
                monotonic_stamps.append(float(ts))

        if len(monotonic_stamps) < 2:
            raise AssertionError("C11 failed: no monotonic timestamp sequence")

        total_dt = float(monotonic_stamps[-1] - monotonic_stamps[0])
        if total_dt <= 0.0:
            raise AssertionError(f"C11 failed: invalid timestamps interval ({total_dt})")

        n_frames = len(monotonic_stamps)
        fps_actual = float(n_frames / total_dt)
        ideal_dt = float(1.0 / float(self.update_rate))
        deltas = np.diff(np.array(monotonic_stamps, dtype=np.float64))
        jitter = float(np.max(np.abs(deltas - ideal_dt))) if deltas.size > 0 else 0.0
        max_dt = float(np.max(deltas)) if deltas.size > 0 else 0.0
        dropouts = int(np.sum(deltas > (2.0 * ideal_dt))) if deltas.size > 0 else 0

        fps_ok = fps_actual >= (0.95 * float(self.update_rate))
        jitter_ok = jitter <= float(self.C11_MAX_JITTER_S)
        dropouts_ok = dropouts == 0

        metrics.update(
            {
                "frames_captured": int(n_frames),
                "timestamps_interval_s": total_dt,
                "fps_actual_hz": fps_actual,
                "ideal_dt_s": ideal_dt,
                "jitter_s": jitter,
                "max_dt_s": max_dt,
                "dropouts_count": dropouts,
                "checks": {
                    "fps_ok": bool(fps_ok),
                    "jitter_ok": bool(jitter_ok),
                    "dropouts_ok": bool(dropouts_ok),
                },
            }
        )

        if first_msg["msg"] is not None:
            frame_first = self._msg_to_bgr(first_msg["msg"])
            debug_first = self._annotate(frame_first, ["C11 first frame", f"fps={fps_actual:.2f}", f"jitter={jitter:.4f}s"])
            artifacts.append(self._save_frame("c11_first_frame.png", debug_first))
        if last_msg["msg"] is not None:
            frame_last = self._msg_to_bgr(last_msg["msg"])
            debug_last = self._annotate(frame_last, ["C11 last frame", f"dropouts={dropouts}", f"max_dt={max_dt:.4f}s"])
            artifacts.append(self._save_frame("c11_last_frame.png", debug_last))

        metrics_path = self._save_metrics_json("c11_fps_stability_metrics.json", metrics)
        if not (fps_ok and jitter_ok and dropouts_ok):
            raise AssertionError(
                f"C11 failed: fps={fps_actual:.3f} (target>={0.95 * self.update_rate:.3f}), "
                f"jitter={jitter:.4f}s (limit<={self.C11_MAX_JITTER_S:.4f}s), dropouts={dropouts}"
            )

        return {"id": "C11", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
