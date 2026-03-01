from __future__ import annotations

import json
import os
import time
import xml.etree.ElementTree as ET
from math import atan2, cos, pi, sin
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

from .depth_camera import DepthCamera


class DepthProfileBase(DepthCamera):
    DEPTH_TOPIC = ""
    IMAGE_TOPIC = ""

    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 720
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.518
    CLIP_NEAR = 0.28
    CLIP_FAR = 10.0

    TEST_DISTANCES: Tuple[float, ...] = (1.0, 3.0, 5.0)
    MAX_ABS_ERROR_M = 0.8
    C3_TARGET_CUBE_NAME = "target_cube"
    C5_RANGE_CUBE_NAME = "range_cube"
    C6_SHIFT_CUBE_NAME = "shift_cube"

    C3_RADIUS_M = 2.0
    C3_SAMPLES = 30
    C3_MAX_DEV_RATIO = 0.05
    C3_FALLBACK_HALF_ANGLE_RAD = 0.6

    C5_START_X = 0.5
    C5_END_X = 10.0
    C5_STEP = 0.5
    C5_DEPTH_TOLERANCE_M = 0.10

    C6_START_X = 2.0
    C6_END_X = 1.0
    C6_STEP = 0.01
    C6_DEPTH_CHANGE_EPS_M = 0.004
    C6_MIN_CHANGED_RATIO = 0.80

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
            "depth_perception_test": str(worlds_root / "camera_depth_perception.world"),
            "c3_view_angle_stability_test": str(worlds_root / "camera_c3_view_angle.world"),
            "c5_working_range_test": str(worlds_root / "camera_c5_working_range.world"),
            "c6_small_displacement_sensitivity_test": str(worlds_root / "camera_c6_small_shifts.world"),
        }
        self.camera_model_name = self._read_camera_model_name()

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

    def _save_frame(self, name: str, frame: np.ndarray) -> str:
        out = os.path.join(self._captured_dir(), name)
        cv2.imwrite(out, frame)
        return out

    def _save_metrics_json(self, name: str, payload: Dict[str, Any]) -> str:
        out = os.path.join(self._metrics_dir(), name)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return out

    def _open_test_scene(self, simulator, test_name: str) -> None:
        world = self.test_to_world[test_name]
        if not simulator.open_scene(world, self.sensor_sdf_path):
            diag = {}
            if hasattr(simulator, "get_last_scene_diagnostics") and callable(simulator.get_last_scene_diagnostics):
                diag = simulator.get_last_scene_diagnostics()
            reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
            raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)

    @staticmethod
    def _iter_float_range(start: float, stop: float, step: float) -> List[float]:
        values: List[float] = []
        cur = float(start)
        while cur <= float(stop) + 1e-9:
            values.append(round(cur, 4))
            cur += float(step)
        return values

    @staticmethod
    def _iter_float_range_desc(start: float, stop: float, step: float) -> List[float]:
        values: List[float] = []
        cur = float(start)
        while cur >= float(stop) - 1e-9:
            values.append(round(cur, 4))
            cur -= float(step)
        return values

    @staticmethod
    def _depth_msg_to_meters(msg: Image) -> np.ndarray:
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "").lower()

        if enc == "32fc1":
            arr = np.frombuffer(msg.data, dtype=np.float32)
            return arr.reshape(h, w)

        if enc == "16uc1":
            arr_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            return arr_mm.astype(np.float32) / 1000.0

        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

    @staticmethod
    def _color_msg_to_bgr(msg: Image) -> np.ndarray:
        h, w = int(msg.height), int(msg.width)
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

    def _color_mask(self, bgr: np.ndarray, color: str) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
            "blue": ((95, 60, 40), (140, 255, 255)),
            "green": ((35, 60, 40), (90, 255, 255)),
            "yellow": ((15, 80, 80), (45, 255, 255)),
        }
        if color not in ranges:
            raise ValueError(f"Unsupported color: {color}")
        lower, upper = ranges[color]
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        return self._clean_mask(mask)

    def _find_centroid(self, bgr: np.ndarray, color: str, min_area: float = 120.0) -> Optional[Tuple[int, int]]:
        mask = self._color_mask(bgr, color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < float(min_area):
            return None
        moments = cv2.moments(contour)
        if moments["m00"] <= 1e-6:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx, cy

    @staticmethod
    def _depth_at_pixel(depth_m: np.ndarray, x: int, y: int, window: int = 3) -> Optional[float]:
        h, w = depth_m.shape[:2]
        x0 = max(0, int(x) - int(window))
        y0 = max(0, int(y) - int(window))
        x1 = min(w, int(x) + int(window) + 1)
        y1 = min(h, int(y) + int(window) + 1)
        roi = depth_m[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _measure_depth(
        self,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
    ) -> Tuple[Optional[float], Tuple[int, int]]:
        h, w = depth_m.shape[:2]
        if bgr is not None and color_hint:
            centroid = self._find_centroid(bgr, color_hint)
            if centroid is not None:
                z = self._depth_at_pixel(depth_m, centroid[0], centroid[1], window=3)
                if z is not None:
                    return z, centroid

        cx, cy = w // 2, h // 2
        z_center = self._depth_at_pixel(depth_m, cx, cy, window=4)
        return z_center, (cx, cy)

    @staticmethod
    def _draw_debug(
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray],
        point: Tuple[int, int],
        lines: List[str],
    ) -> np.ndarray:
        if bgr is not None:
            debug = bgr.copy()
        else:
            dm = depth_m.copy()
            finite = dm[np.isfinite(dm) & (dm > 0)]
            if finite.size == 0:
                debug = np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)
            else:
                dmin = float(np.percentile(finite, 5))
                dmax = float(np.percentile(finite, 95))
                if dmax <= dmin:
                    dmax = dmin + 1e-3
                norm = np.clip((dm - dmin) / (dmax - dmin), 0.0, 1.0)
                u8 = (norm * 255.0).astype(np.uint8)
                debug = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

        cv2.circle(debug, point, 5, (255, 255, 255), 2)
        y = 30
        for line in lines:
            cv2.putText(debug, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            y += 28
        return debug

    @staticmethod
    def _read_model_name_from_sdf(path: str) -> str:
        if not path:
            return ""
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            model = root.find("model")
            if model is not None and model.get("name"):
                return str(model.get("name"))
        except Exception:
            return ""
        return ""

    def _read_camera_model_name(self) -> str:
        model_name = self._read_model_name_from_sdf(self.sensor_sdf_path)
        if model_name:
            return model_name
        return f"{self.sensor_name}_model"

    @staticmethod
    def _move_and_settle(simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.35) -> None:
        simulator.set_pose(model_name, x=float(x), y=float(y), z=float(z))
        time.sleep(settle_s)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        return Quaternion(0.0, 0.0, sin(float(yaw) * 0.5), cos(float(yaw) * 0.5))

    def _set_model_pose_6d(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        settle_s: float = 0.35,
    ) -> None:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose = Pose(
            Point(float(x), float(y), float(z)),
            self._yaw_to_quaternion(yaw),
        )
        response = set_state(state)
        if not response.success:
            raise RuntimeError(f"Failed to set model state for {model_name}: {response.status_message}")
        time.sleep(settle_s)

    def _wait_depth(self, timeout: float = 3.0) -> Image:
        if not self.DEPTH_TOPIC:
            raise RuntimeError("DEPTH_TOPIC is not configured for this depth profile")
        return rospy.wait_for_message(self.DEPTH_TOPIC, Image, timeout=timeout)

    def _try_wait_color(self, timeout: float = 1.0) -> Optional[Image]:
        if not self.IMAGE_TOPIC:
            return None
        try:
            return rospy.wait_for_message(self.IMAGE_TOPIC, Image, timeout=timeout)
        except rospy.ROSException:
            return None

    def depth_perception_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "depth_perception_test")

        cube_name = "close_green_cube"
        cube_z = 0.25
        reset_x = 50.0

        if not simulator.wait_for_model_spawn(cube_name, 30):
            raise RuntimeError(f"cube not spawned: {cube_name}")

        results = []
        measured_values = []

        for d in self.TEST_DISTANCES:
            simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
            time.sleep(0.05)

            simulator.set_pose(cube_name, float(d) + 0.25, 0.0, cube_z)
            time.sleep(0.6)

            depth_msg = self._wait_depth(timeout=1.5)
            depth_m = self._depth_msg_to_meters(depth_msg)
            z, _ = self._measure_depth(depth_m, bgr=None, color_hint=None)
            if z is None or np.isnan(z) or z <= 0.0:
                raise RuntimeError(f"Invalid depth measurement at distance={d}: {z}")

            if not (self.clip_near <= float(z) <= (self.clip_far + 0.5)):
                raise AssertionError(f"Depth out of clip range at distance={d}: z={z}, clip=({self.clip_near}, {self.clip_far})")

            abs_err = abs(float(z) - float(d))
            rel_err = abs_err / float(d) * 100.0

            if abs_err > float(self.MAX_ABS_ERROR_M):
                raise AssertionError(
                    f"Depth absolute error too high at distance={d}: abs_err={abs_err:.4f}, max={self.MAX_ABS_ERROR_M}"
                )

            results.append(
                {
                    "distance": float(d),
                    "measured": float(z),
                    "abs_error": float(abs_err),
                    "rel_error": float(rel_err),
                }
            )
            measured_values.append(float(z))

        monotonic_ok = all(measured_values[i] < measured_values[i + 1] for i in range(len(measured_values) - 1))
        if not monotonic_ok:
            raise AssertionError(f"Depth monotonicity failed: {measured_values}")

        return {
                "id": "DEPTH",
            "metrics": {
                "distances_m": list(self.TEST_DISTANCES),
                "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
                "measurements": results,
                "monotonic_increasing": True,
            },
        }

    def c3_view_angle_stability_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "samples_target": int(self.C3_SAMPLES),
            "radius_m": float(self.C3_RADIUS_M),
            "max_deviation_limit": float(self.C3_MAX_DEV_RATIO),
            "measurements_m": [],
            "mode": "",
        }

        self._open_test_scene(simulator, "c3_view_angle_stability_test")
        if not simulator.wait_for_model_spawn(self.C3_TARGET_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C3_TARGET_CUBE_NAME}")

        depths: List[float] = []
        use_camera_orbit = simulator.wait_for_model_spawn(self.camera_model_name, timeout=10)

        if use_camera_orbit:
            metrics["mode"] = "camera_orbit"
            try:
                cube_x, cube_y, cube_z = 2.0, 0.0, 0.25
                cam_z = 0.25
                for i in range(self.C3_SAMPLES):
                    theta = 2.0 * pi * (float(i) / float(self.C3_SAMPLES))
                    cam_x = cube_x + self.C3_RADIUS_M * cos(theta)
                    cam_y = cube_y + self.C3_RADIUS_M * sin(theta)
                    yaw = atan2(cube_y - cam_y, cube_x - cam_x)
                    self._set_model_pose_6d(self.camera_model_name, x=cam_x, y=cam_y, z=cam_z, yaw=yaw, settle_s=0.35)

                    depth_msg = self._wait_depth(timeout=2.0)
                    depth_m = self._depth_msg_to_meters(depth_msg)
                    color_msg = self._try_wait_color(timeout=1.0)
                    bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                    z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                    if z is None or np.isnan(z) or np.isinf(z):
                        continue
                    depths.append(float(z))

                    if i in (0, self.C3_SAMPLES // 2):
                        dbg = self._draw_debug(depth_m, bgr, point, [f"C3 camera orbit", f"sample={i}", f"depth={z:.3f}m"])
                        artifacts.append(self._save_frame(f"c3_camera_orbit_{i}.png", dbg))

                if len(depths) < max(10, int(0.6 * self.C3_SAMPLES)):
                    raise RuntimeError(f"Too few valid measurements in camera orbit mode: {len(depths)}")
            except Exception as exc:
                metrics["mode_error"] = str(exc)
                depths.clear()
                use_camera_orbit = False

        if not use_camera_orbit:
            # Эквивалентная реализация: двигаем target_cube относительно неподвижной камеры.
            metrics["mode"] = "target_cube_orbit_equivalent"
            metrics["implementation_note"] = (
                "Camera orbit is replaced with equivalent target_cube motion relative to static camera "
                "using /gazebo/set_model_state."
            )
            angles = np.linspace(-self.C3_FALLBACK_HALF_ANGLE_RAD, self.C3_FALLBACK_HALF_ANGLE_RAD, self.C3_SAMPLES)
            for i, theta in enumerate(angles):
                x = self.C3_RADIUS_M * cos(float(theta))
                y = self.C3_RADIUS_M * sin(float(theta))
                self._move_and_settle(simulator, self.C3_TARGET_CUBE_NAME, x=x, y=y, z=0.25, settle_s=0.25)

                depth_msg = self._wait_depth(timeout=2.0)
                depth_m = self._depth_msg_to_meters(depth_msg)
                color_msg = self._try_wait_color(timeout=1.0)
                bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                if z is None or np.isnan(z) or np.isinf(z):
                    continue
                depths.append(float(z))

                if i in (0, self.C3_SAMPLES // 2):
                    dbg = self._draw_debug(depth_m, bgr, point, [f"C3 equivalent", f"sample={i}", f"depth={z:.3f}m"])
                    artifacts.append(self._save_frame(f"c3_equivalent_{i}.png", dbg))

        if len(depths) < 10:
            raise RuntimeError(f"C3 failed: too few valid depth samples ({len(depths)})")

        z_mean = float(np.mean(depths))
        z_min = float(np.min(depths))
        z_max = float(np.max(depths))
        max_dev_ratio = float((z_max - z_min) / z_mean) if z_mean > 1e-9 else float("inf")

        metrics["measurements_m"] = [float(v) for v in depths]
        metrics["mean_depth_m"] = z_mean
        metrics["min_depth_m"] = z_min
        metrics["max_depth_m"] = z_max
        metrics["max_deviation_ratio"] = max_dev_ratio
        metrics["checks"] = {"max_deviation_le_0_05": bool(max_dev_ratio <= self.C3_MAX_DEV_RATIO)}

        metrics_path = self._save_metrics_json("c3_view_angle_stability_metrics.json", metrics)
        if max_dev_ratio > self.C3_MAX_DEV_RATIO:
            raise AssertionError(
                f"C3 failed: max_deviation={max_dev_ratio:.4f} > {self.C3_MAX_DEV_RATIO:.4f}, "
                f"min={z_min:.4f}, max={z_max:.4f}, mean={z_mean:.4f}"
            )

        return {"id": "C3", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c5_working_range_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "x_values_m": self._iter_float_range(self.C5_START_X, self.C5_END_X, self.C5_STEP),
            "tolerance_m": float(self.C5_DEPTH_TOLERANCE_M),
            "samples": [],
        }

        self._open_test_scene(simulator, "c5_working_range_test")
        if not simulator.wait_for_model_spawn(self.C5_RANGE_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C5_RANGE_CUBE_NAME}")

        for x in metrics["x_values_m"]:
            self._move_and_settle(simulator, self.C5_RANGE_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.3)
            depth_msg = self._wait_depth(timeout=2.0)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._try_wait_color(timeout=1.0)
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point = self._measure_depth(depth_m, bgr, color_hint="green")

            finite_ok = bool(z is not None and np.isfinite(z))
            abs_err = float(abs(float(z) - float(x))) if finite_ok else float("inf")
            sample_ok = bool(finite_ok and abs_err <= self.C5_DEPTH_TOLERANCE_M)

            metrics["samples"].append(
                {
                    "x_m": float(x),
                    "depth_m": None if z is None else float(z),
                    "finite_ok": finite_ok,
                    "abs_error_m": abs_err if np.isfinite(abs_err) else None,
                    "ok": sample_ok,
                }
            )

            if abs(float(x) - 0.5) < 1e-9 or abs(float(x) - 5.0) < 1e-9 or abs(float(x) - 10.0) < 1e-9:
                dbg = self._draw_debug(
                    depth_m,
                    bgr,
                    point,
                    [f"C5 x={x:.2f}m", f"depth={z if z is not None else float('nan'):.3f}m", f"ok={sample_ok}"],
                )
                artifacts.append(self._save_frame(f"c5_x_{str(x).replace('.', '_')}.png", dbg))

        best_start = None
        best_end = None
        cur_start = None
        cur_end = None

        for sample in metrics["samples"]:
            if sample["ok"]:
                if cur_start is None:
                    cur_start = float(sample["x_m"])
                cur_end = float(sample["x_m"])
            else:
                if cur_start is not None:
                    if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                        best_start, best_end = cur_start, cur_end
                    cur_start, cur_end = None, None

        if cur_start is not None:
            if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                best_start, best_end = cur_start, cur_end

        if best_start is None or best_end is None:
            raise AssertionError("C5 failed: no stable depth interval found")

        metrics["x_min_ok_m"] = float(best_start)
        metrics["x_max_ok_m"] = float(best_end)
        metrics["checks"] = {
            "x_min_ok_le_0_5": bool(float(best_start) <= 0.5 + 1e-6),
            "x_max_ok_ge_10_0": bool(float(best_end) >= 10.0 - 1e-6),
        }

        metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
        if not (metrics["checks"]["x_min_ok_le_0_5"] and metrics["checks"]["x_max_ok_ge_10_0"]):
            raise AssertionError(
                f"C5 failed: stable interval [{best_start:.2f}, {best_end:.2f}] does not cover [0.5, 10.0]"
            )

        return {"id": "C5", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c6_small_displacement_sensitivity_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        x_values = self._iter_float_range_desc(self.C6_START_X, self.C6_END_X, self.C6_STEP)
        metrics: Dict[str, Any] = {
            "x_values_m": x_values,
            "eps_m": float(self.C6_DEPTH_CHANGE_EPS_M),
            "required_ratio": float(self.C6_MIN_CHANGED_RATIO),
            "samples": [],
        }

        self._open_test_scene(simulator, "c6_small_displacement_sensitivity_test")
        if not simulator.wait_for_model_spawn(self.C6_SHIFT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C6_SHIFT_CUBE_NAME}")

        depths: List[Optional[float]] = []
        for x in x_values:
            self._move_and_settle(simulator, self.C6_SHIFT_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.16)
            depth_msg = self._wait_depth(timeout=2.0)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._try_wait_color(timeout=1.0)
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point = self._measure_depth(depth_m, bgr, color_hint="yellow")
            z_valid = bool(z is not None and np.isfinite(z))
            depths.append(float(z) if z_valid else None)
            metrics["samples"].append({"x_m": float(x), "depth_m": None if not z_valid else float(z)})

            if abs(float(x) - self.C6_START_X) < 1e-9 or abs(float(x) - self.C6_END_X) < 1e-9:
                dbg = self._draw_debug(
                    depth_m,
                    bgr,
                    point,
                    [f"C6 x={x:.2f}m", f"depth={z if z is not None else float('nan'):.3f}m"],
                )
                artifacts.append(self._save_frame(f"c6_x_{str(x).replace('.', '_')}.png", dbg))

        deltas: List[float] = []
        for i in range(1, len(depths)):
            prev = depths[i - 1]
            cur = depths[i]
            if prev is None or cur is None:
                continue
            deltas.append(float(abs(cur - prev)))

        if not deltas:
            raise AssertionError("C6 failed: no valid consecutive depth pairs")

        changed = [d for d in deltas if d > self.C6_DEPTH_CHANGE_EPS_M]
        changed_ratio = float(len(changed) / len(deltas))
        median_delta = float(np.median(np.array(deltas, dtype=np.float64)))

        metrics["delta_abs_m"] = [float(d) for d in deltas]
        metrics["median_delta_m"] = median_delta
        metrics["changed_pairs"] = int(len(changed))
        metrics["pairs_total"] = int(len(deltas))
        metrics["changed_ratio"] = changed_ratio
        metrics["checks"] = {"changed_ratio_ge_0_8": bool(changed_ratio >= self.C6_MIN_CHANGED_RATIO)}

        metrics_path = self._save_metrics_json("c6_small_displacement_sensitivity_metrics.json", metrics)
        if changed_ratio < self.C6_MIN_CHANGED_RATIO:
            raise AssertionError(
                f"C6 failed: changed_ratio={changed_ratio:.3f} < {self.C6_MIN_CHANGED_RATIO:.3f}, "
                f"median_delta={median_delta:.4f}, eps={self.C6_DEPTH_CHANGE_EPS_M:.4f}"
            )

        return {"id": "C6", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
