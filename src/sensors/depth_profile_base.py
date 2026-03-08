from __future__ import annotations

import copy
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
    C5_CLIP_MARGIN_M = 0.05
    C5_TARGET_SIZE_X_M = 0.5
    DEPTH_ROI_HALF_WINDOW = 2  # 5x5 ROI

    C6_START_X = 2.0
    C6_END_X = 1.0
    C6_STEP = 0.01
    C6_TARGET_SIZE_X_M = 0.5
    C6_DEPTH_CHANGE_EPS_M = 0.004
    C6_MIN_CHANGED_RATIO = 0.80
    C6_MONOTONIC_TOLERANCE_M = 0.002
    C6_DELTA_TOLERANCE_M = 0.003
    C6_ABS_ERROR_TOLERANCE_M = 0.03
    DEPTH_TOPIC_WARMUP_TIMEOUT_S = 20.0
    DEPTH_WAIT_PER_CANDIDATE_S = 1.2
    FRAME_FRESH_TIMEOUT_S = 4.0
    CLIP_SATURATION_EPS_M = 0.02
    DEPTH_POST_MOVE_CONFIRMATION_FRAMES = 2
    C3_TARGET_SIZE_X_M = 0.5
    C3_MEAN_ABS_ERROR_M = 0.08
    C3_MAX_ABS_ERROR_M = 0.15

    def __init__(self, CONFIG):
        super().__init__(CONFIG)
        self.CONFIG = CONFIG
        self._last_test_diagnostics: Dict[str, Any] = {}

        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)

        worlds_root = Path(CONFIG["WORLDS_PATH"])
        if not worlds_root.is_absolute():
            worlds_root = Path(CONFIG["ROOT_PATH"]) / worlds_root

        self.test_to_world = {
            "depth_perception_test": str(worlds_root / "camera_depth_perception.world"),
            "c3_view_angle_stability_test": str(worlds_root / "camera_c3_view_angle.world"),
            "c5_working_range_test": str(worlds_root / "camera_c5_working_range.world"),
            "c6_small_displacement_sensitivity_test": str(worlds_root / "camera_c6_small_shifts.world"),
        }
        self.camera_model_name = self._read_camera_model_name()
        self._resolved_depth_topic = ""
        self._resolved_image_topic = ""

    def _set_test_diagnostics(self, **kwargs) -> None:
        self._last_test_diagnostics.update(kwargs)

    def get_last_test_diagnostics(self) -> Dict[str, Any]:
        return copy.deepcopy(self._last_test_diagnostics)

    def get_expected_topics(self) -> List[str]:
        topics: List[str] = [str(self.DEPTH_TOPIC)]
        image_topic = str(self.IMAGE_TOPIC or "").strip()
        if image_topic:
            topics.append(image_topic)
        return topics

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
        self._last_test_diagnostics = {}
        self._resolved_depth_topic = ""
        self._resolved_image_topic = ""
        world = self.test_to_world[test_name]
        display_env = self._ensure_render_display_env()
        if display_env:
            self._set_test_diagnostics(render_display_env=display_env)
        if not simulator.open_scene(
            world,
            self.sensor_sdf_path,
            expected_topics=self.get_expected_topics(),
            sensor_name=self.sensor_name,
        ):
            diag = {}
            if hasattr(simulator, "get_last_scene_diagnostics") and callable(simulator.get_last_scene_diagnostics):
                diag = simulator.get_last_scene_diagnostics()
            reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
            raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")
        self._update_resolved_topics(simulator)
        self._set_test_diagnostics(
            scene_open_success=True,
            world_file=str(world),
            expected_depth_topic=str(self.DEPTH_TOPIC),
            expected_image_topic=str(self.IMAGE_TOPIC or ""),
            resolved_depth_topic=str(self._resolved_depth_topic or self.DEPTH_TOPIC),
            resolved_image_topic=str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
        )
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
            dtype = np.float32
            bytes_per_px = 4
        elif enc == "16uc1":
            dtype = np.uint16
            bytes_per_px = 2
        else:
            raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

        row_stride_bytes = int(msg.step) if int(msg.step) > 0 else int(w * bytes_per_px)
        min_row_bytes = int(w * bytes_per_px)
        if row_stride_bytes < min_row_bytes:
            raise ValueError(
                f"Invalid depth step for encoding={msg.encoding}: step={row_stride_bytes}, min_required={min_row_bytes}"
            )

        cols_with_stride = row_stride_bytes // bytes_per_px
        raw = np.frombuffer(msg.data, dtype=dtype)
        expected_size = int(h * cols_with_stride)
        if raw.size < expected_size:
            raise ValueError(
                f"Depth buffer too small for encoding={msg.encoding}: got={raw.size}, expected={expected_size}"
            )

        arr = raw[:expected_size].reshape(h, cols_with_stride)[:, :w]
        if enc == "16uc1":
            arr_mm = arr.astype(np.uint16, copy=False)
            return arr_mm.astype(np.float32) / 1000.0

        return arr.astype(np.float32, copy=False)

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
    def _depth_at_pixel_with_meta(depth_m: np.ndarray, x: int, y: int, half_window: int = 2) -> Tuple[Optional[float], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        x0 = max(0, int(x) - int(half_window))
        y0 = max(0, int(y) - int(half_window))
        x1 = min(w, int(x) + int(half_window) + 1)
        y1 = min(h, int(y) + int(half_window) + 1)
        roi = depth_m[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        meta = {
            "pixel": {"x": int(x), "y": int(y)},
            "roi_xyxy": [int(x0), int(y0), int(x1), int(y1)],
            "roi_shape": [int(max(0, y1 - y0)), int(max(0, x1 - x0))],
            "valid_count": int(valid.size),
            "aggregator": "mean",
        }
        if valid.size == 0:
            return None, meta
        return float(np.mean(valid)), meta

    def _depth_at_pixel(self, depth_m: np.ndarray, x: int, y: int, window: int = 2) -> Optional[float]:
        z, _ = self._depth_at_pixel_with_meta(depth_m, x=x, y=y, half_window=window)
        return z

    def _depth_frame_stats(self, depth_msg: Image, depth_m: np.ndarray) -> Dict[str, Any]:
        finite = depth_m[np.isfinite(depth_m)]
        positive = finite[finite > 0.0] if finite.size > 0 else np.array([], dtype=np.float32)
        stats: Dict[str, Any] = {
            "encoding": str(depth_msg.encoding),
            "dtype": str(depth_m.dtype),
            "shape": [int(depth_m.shape[0]), int(depth_m.shape[1])],
            "step_bytes": int(depth_msg.step),
            "is_bigendian": int(depth_msg.is_bigendian),
            "finite_count": int(finite.size),
            "positive_count": int(positive.size),
            "finite_min_m": None,
            "finite_max_m": None,
        }
        if finite.size > 0:
            stats["finite_min_m"] = float(np.min(finite))
            stats["finite_max_m"] = float(np.max(finite))
        return stats

    @staticmethod
    def _depth_preview(depth_m: np.ndarray) -> np.ndarray:
        finite = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
        if finite.size == 0:
            return np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)
        dmin = float(np.percentile(finite, 5))
        dmax = float(np.percentile(finite, 95))
        if dmax <= dmin:
            dmax = dmin + 1e-3
        norm = np.clip((depth_m - dmin) / (dmax - dmin), 0.0, 1.0)
        u8 = (norm * 255.0).astype(np.uint8)
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

    @staticmethod
    def _roi_depth_stats(depth_m: np.ndarray, roi_xyxy: List[int]) -> Dict[str, Any]:
        if not roi_xyxy or len(roi_xyxy) != 4:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        x0, y0, x1, y1 = [int(v) for v in roi_xyxy]
        h, w = depth_m.shape[:2]
        x0 = max(0, min(w, x0))
        x1 = max(0, min(w, x1))
        y0 = max(0, min(h, y0))
        y1 = max(0, min(h, y1))
        if x1 <= x0 or y1 <= y0:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        roi = depth_m[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        return {
            "valid_count": int(valid.size),
            "min_m": float(np.min(valid)),
            "max_m": float(np.max(valid)),
            "mean_m": float(np.mean(valid)),
            "median_m": float(np.median(valid)),
        }

    @staticmethod
    def _fallback_point_from_finite_depth(depth_m: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        y0 = int(h * 0.2)
        y1 = int(h * 0.8)
        x0 = int(w * 0.2)
        x1 = int(w * 0.8)
        central = depth_m[y0:y1, x0:x1]
        central_valid = np.isfinite(central) & (central > 0.0)

        meta: Dict[str, Any] = {
            "strategy": "finite_depth_min",
            "central_window_xyxy": [int(x0), int(y0), int(x1), int(y1)],
            "central_valid_count": int(np.count_nonzero(central_valid)),
            "fallback_scope": "central",
        }

        if np.count_nonzero(central_valid) > 0:
            central_depth = np.where(central_valid, central, np.inf)
            idx = np.unravel_index(int(np.argmin(central_depth)), central_depth.shape)
            py = int(y0 + idx[0])
            px = int(x0 + idx[1])
            meta["chosen_depth_m"] = float(depth_m[py, px])
            return (px, py), meta

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        meta["fallback_scope"] = "global"
        meta["global_valid_count"] = int(np.count_nonzero(valid))
        if np.count_nonzero(valid) == 0:
            return None, meta

        masked = np.where(valid, depth_m, np.inf)
        idx = np.unravel_index(int(np.argmin(masked)), masked.shape)
        py = int(idx[0])
        px = int(idx[1])
        meta["chosen_depth_m"] = float(depth_m[py, px])
        return (px, py), meta

    def _measure_depth_with_meta(
        self,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
    ) -> Tuple[Optional[float], Tuple[int, int], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        if bgr is not None and color_hint:
            centroid = self._find_centroid(bgr, color_hint)
            if centroid is not None:
                z, roi_meta = self._depth_at_pixel_with_meta(
                    depth_m,
                    centroid[0],
                    centroid[1],
                    half_window=int(self.DEPTH_ROI_HALF_WINDOW),
                )
                if z is not None:
                    roi_meta["source"] = "color_centroid"
                    roi_meta["color_hint"] = str(color_hint)
                    return z, centroid, roi_meta

        fallback_point, fallback_diag = self._fallback_point_from_finite_depth(depth_m)
        if fallback_point is not None:
            z_fallback, roi_meta = self._depth_at_pixel_with_meta(
                depth_m,
                fallback_point[0],
                fallback_point[1],
                half_window=int(self.DEPTH_ROI_HALF_WINDOW),
            )
            if z_fallback is not None:
                roi_meta["source"] = "finite_depth_fallback"
                roi_meta["color_hint"] = str(color_hint) if color_hint else None
                roi_meta["fallback_meta"] = fallback_diag
                return z_fallback, fallback_point, roi_meta

        cx, cy = w // 2, h // 2
        z_center, roi_meta = self._depth_at_pixel_with_meta(
            depth_m,
            cx,
            cy,
            half_window=int(self.DEPTH_ROI_HALF_WINDOW),
        )
        roi_meta["source"] = "frame_center"
        roi_meta["color_hint"] = str(color_hint) if color_hint else None
        return z_center, (cx, cy), roi_meta

    def _measure_depth(
        self,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
    ) -> Tuple[Optional[float], Tuple[int, int]]:
        z, point, _ = self._measure_depth_with_meta(depth_m=depth_m, bgr=bgr, color_hint=color_hint)
        return z, point

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

    def _save_failure_artifacts(
        self,
        prefix: str,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray],
        point: Tuple[int, int],
        roi_meta: Dict[str, Any],
        extra_lines: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
        roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))

        if bgr is not None:
            rgb_dbg = bgr.copy()
        else:
            rgb_dbg = self._depth_preview(depth_m)
        cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
        lines = [
            f"source={roi_meta.get('source', 'unknown')}",
            f"valid={roi_stats.get('valid_count', 0)}",
            f"mean={roi_stats.get('mean_m') if roi_stats.get('mean_m') is not None else float('nan'):.3f}",
            f"median={roi_stats.get('median_m') if roi_stats.get('median_m') is not None else float('nan'):.3f}",
        ]
        for line in (extra_lines or []):
            lines.append(str(line))
        y = 30
        for line in lines:
            cv2.putText(rgb_dbg, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            y += 24
        rgb_path = self._save_frame(f"{prefix}_rgb.png", rgb_dbg)

        depth_dbg = self._depth_preview(depth_m)
        cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
        depth_path = self._save_frame(f"{prefix}_depth.png", depth_dbg)

        return {
            "rgb_artifact": rgb_path,
            "depth_artifact": depth_path,
            "roi_stats": roi_stats,
            "roi_meta": roi_meta,
        }

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

    @staticmethod
    def _list_image_topics() -> List[str]:
        try:
            published = rospy.get_published_topics()
        except Exception:
            return []
        return sorted([name for name, msg_type in published if msg_type == "sensor_msgs/Image"])

    @staticmethod
    def _choose_depth_topic(candidates: List[str]) -> str:
        if not candidates:
            return ""
        prioritized = sorted(
            candidates,
            key=lambda t: (0 if t.endswith("/depth/image_raw") else 1, len(t), t),
        )
        return prioritized[0]

    def _resolve_depth_topic(self, warmup_timeout: float) -> Tuple[str, Dict[str, Any]]:
        expected = str(self.DEPTH_TOPIC or "")
        sensor_name = str(self.sensor_name)
        preferred = [
            (expected, "expected"),
            (f"/{sensor_name}/depth/image_raw", "name_namespace"),
            (f"/{sensor_name}_depth/image_raw", "name_underscore"),
            (f"/{sensor_name}_depth/depth/image_raw", "name_underscore_nested"),
            (f"/{sensor_name}/depth_image_raw", "name_alt"),
        ]

        deadline = time.time() + float(warmup_timeout)
        last_topics: List[str] = []
        while time.time() < deadline:
            topics = self._list_image_topics()
            last_topics = topics
            for topic, source in preferred:
                if topic and topic in topics:
                    diag = {
                        "expected_depth_topic": expected,
                        "selected_depth_topic": topic,
                        "selected_source": source,
                        "topics_found": topics,
                        "topic_mapping_changed": bool(topic != expected),
                    }
                    return topic, diag
            time.sleep(0.2)

        if expected and expected in last_topics:
            diag = {
                "expected_depth_topic": expected,
                "selected_depth_topic": expected,
                "selected_source": "expected_after_warmup",
                "topics_found": last_topics,
                "topic_mapping_changed": False,
            }
            return expected, diag

        token = sensor_name
        generic = [
            t for t in last_topics
            if ("depth" in t and ("image_raw" in t or t.endswith("/image")))
        ]
        in_namespace = [t for t in generic if token in t]
        selected = self._choose_depth_topic(in_namespace) or self._choose_depth_topic(generic) or expected
        diag = {
            "expected_depth_topic": expected,
            "selected_depth_topic": selected,
            "selected_source": "heuristic_fallback",
            "topics_found": last_topics,
            "topic_mapping_changed": bool(selected != expected),
        }
        return selected, diag

    @staticmethod
    def _scene_diag(simulator) -> Dict[str, Any]:
        if hasattr(simulator, "get_last_scene_diagnostics") and callable(simulator.get_last_scene_diagnostics):
            try:
                diag = simulator.get_last_scene_diagnostics()
                if isinstance(diag, dict):
                    return diag
            except Exception:
                pass
        return {}

    @staticmethod
    def _ensure_render_display_env() -> Dict[str, str]:
        applied: Dict[str, str] = {}
        if os.environ.get("DISPLAY"):
            return applied

        xauthority = os.environ.get("XAUTHORITY", "").strip()
        if not xauthority:
            default_xauthority = os.path.expanduser("~/.Xauthority")
            if os.path.exists(default_xauthority):
                os.environ["XAUTHORITY"] = default_xauthority
                applied["XAUTHORITY"] = default_xauthority

        for display in (":0",):
            display_socket = f"/tmp/.X11-unix/X{display.lstrip(':')}"
            if os.path.exists(display_socket):
                os.environ["DISPLAY"] = display
                applied["DISPLAY"] = display
                break

        return applied

    @staticmethod
    def _msg_stamp_s(msg: Image) -> float:
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            return float(time.time())
        return stamp

    def _update_resolved_topics(self, simulator) -> Dict[str, Any]:
        scene_diag = self._scene_diag(simulator)
        resolved_topics = scene_diag.get("resolved_topics", {}) if isinstance(scene_diag, dict) else {}
        if isinstance(resolved_topics, dict):
            depth_candidate = str(resolved_topics.get(self.DEPTH_TOPIC, "") or "").strip()
            image_candidate = str(resolved_topics.get(self.IMAGE_TOPIC, "") or "").strip()
            if depth_candidate:
                self._resolved_depth_topic = depth_candidate
            if image_candidate:
                self._resolved_image_topic = image_candidate
        return scene_diag

    def _resolved_color_topic(self) -> str:
        return str(self._resolved_image_topic or self.IMAGE_TOPIC or "").strip()

    def _wait_message_after(
        self,
        prev_stamp_s: Optional[float],
        topic: str,
        timeout: float,
        stage: str,
    ) -> Image:
        target_topic = str(topic or "").strip()
        if not target_topic:
            raise RuntimeError(f"No topic configured for stage={stage}")

        start = time.time()
        saw_message = False
        while (time.time() - start) < float(timeout):
            remaining = max(0.2, float(timeout) - (time.time() - start))
            try:
                msg = rospy.wait_for_message(target_topic, Image, timeout=min(1.0, remaining))
            except rospy.ROSException:
                continue

            saw_message = True
            stamp_s = self._msg_stamp_s(msg)
            if prev_stamp_s is None or stamp_s > (float(prev_stamp_s) + 1e-6):
                return msg

        if saw_message:
            raise RuntimeError(
                f"No fresh frame on topic={target_topic} after stage={stage}; prev_stamp_s={prev_stamp_s}"
            )
        raise RuntimeError(f"No frame received on topic={target_topic} during stage={stage}")

    def _wait_depth_after(self, prev_stamp_s: Optional[float], timeout: Optional[float] = None, stage: str = "") -> Image:
        if not self.DEPTH_TOPIC:
            raise RuntimeError("DEPTH_TOPIC is not configured for this depth profile")
        if not self._resolved_depth_topic:
            selected, diag = self._resolve_depth_topic(self.DEPTH_TOPIC_WARMUP_TIMEOUT_S)
            self._resolved_depth_topic = selected or self.DEPTH_TOPIC
            self._set_test_diagnostics(depth_topic_resolution=diag)
        return self._wait_message_after(
            prev_stamp_s=prev_stamp_s,
            topic=self._resolved_depth_topic,
            timeout=float(timeout or self.FRAME_FRESH_TIMEOUT_S),
            stage=str(stage or "depth_wait_after"),
        )

    def _wait_color_after(self, prev_stamp_s: Optional[float], timeout: Optional[float] = None, stage: str = "") -> Optional[Image]:
        target_topic = self._resolved_color_topic()
        if not target_topic:
            return None
        return self._wait_message_after(
            prev_stamp_s=prev_stamp_s,
            topic=target_topic,
            timeout=float(timeout or self.FRAME_FRESH_TIMEOUT_S),
            stage=str(stage or "color_wait_after"),
        )

    def _is_far_clip_saturated(self, z: Optional[float], roi_stats: Dict[str, Any]) -> bool:
        if z is None or not np.isfinite(z):
            return False
        far_threshold = float(self.clip_far) - float(self.CLIP_SATURATION_EPS_M)
        if float(z) < far_threshold:
            return False

        roi_values = [
            roi_stats.get("min_m"),
            roi_stats.get("max_m"),
            roi_stats.get("mean_m"),
            roi_stats.get("median_m"),
        ]
        finite_values = [float(v) for v in roi_values if v is not None and np.isfinite(v)]
        if not finite_values:
            return True
        return all(value >= far_threshold for value in finite_values)

    def _wait_confirmed_depth_after(
        self,
        prev_stamp_s: Optional[float],
        fresh_frames: Optional[int] = None,
        stage: str = "",
    ) -> Tuple[Image, List[float]]:
        count = max(1, int(fresh_frames or self.DEPTH_POST_MOVE_CONFIRMATION_FRAMES))
        stamps: List[float] = []
        latest_prev = prev_stamp_s
        msg: Optional[Image] = None
        for idx in range(count):
            msg = self._wait_depth_after(
                prev_stamp_s=latest_prev,
                timeout=self.FRAME_FRESH_TIMEOUT_S,
                stage=f"{stage}_confirm_{idx + 1}",
            )
            latest_prev = self._msg_stamp_s(msg)
            stamps.append(float(latest_prev))
        assert msg is not None
        return msg, stamps

    @staticmethod
    def _front_face_depth(center_x_m: float, size_x_m: float) -> float:
        return float(center_x_m) - (float(size_x_m) * 0.5)

    def _c5_expected_in_contract_range(self, expected_depth: float) -> bool:
        near_limit = float(self.clip_near) + float(self.C5_CLIP_MARGIN_M)
        far_limit = float(self.clip_far) - float(self.C5_CLIP_MARGIN_M)
        return bool(near_limit <= float(expected_depth) <= far_limit)

    def _wait_depth(self, timeout: float = 3.0) -> Image:
        if not self.DEPTH_TOPIC:
            raise RuntimeError("DEPTH_TOPIC is not configured for this depth profile")
        if not self._resolved_depth_topic:
            selected, diag = self._resolve_depth_topic(self.DEPTH_TOPIC_WARMUP_TIMEOUT_S)
            self._resolved_depth_topic = selected or self.DEPTH_TOPIC
            self._set_test_diagnostics(depth_topic_resolution=diag)

        candidates = [self._resolved_depth_topic]
        if self._resolved_depth_topic != self.DEPTH_TOPIC:
            candidates.append(self.DEPTH_TOPIC)

        errors: List[str] = []
        for topic in candidates:
            try:
                wait_t = min(float(timeout), float(self.DEPTH_WAIT_PER_CANDIDATE_S))
                return rospy.wait_for_message(topic, Image, timeout=wait_t)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{topic}: {exc}")
                continue
        raise RuntimeError(f"Failed to receive depth frame. candidates={candidates}, errors={errors}")

    def _try_wait_color(self, timeout: float = 1.0) -> Optional[Image]:
        target_topic = self._resolved_color_topic()
        if not target_topic:
            return None
        try:
            return rospy.wait_for_message(target_topic, Image, timeout=timeout)
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
        first_frame_diagnostics: Optional[Dict[str, Any]] = None
        last_depth_stamp_s: Optional[float] = None
        metrics_payload: Dict[str, Any] = {
            "status": "RUNNING",
            "error_reason": "",
            "world_file": str(self.test_to_world["depth_perception_test"]),
            "scene_open_success": True,
            "expected_depth_topic": str(self.DEPTH_TOPIC),
            "expected_image_topic": str(self.IMAGE_TOPIC or ""),
            "resolved_depth_topic": str(self._resolved_depth_topic or self.DEPTH_TOPIC),
            "resolved_image_topic": str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
            "distances_m": list(self.TEST_DISTANCES),
            "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
            "clip_near_m": float(self.clip_near),
            "clip_far_m": float(self.clip_far),
            "measurements": results,
        }

        def _persist_depth_metrics(status: str, error_reason: str = "") -> str:
            metrics_payload["status"] = str(status)
            metrics_payload["error_reason"] = str(error_reason)
            metrics_payload["first_frame_diagnostics"] = first_frame_diagnostics or {}
            metrics_payload["topic_diagnostics"] = self.get_last_test_diagnostics().get("depth_topic_resolution", {})
            metrics_payload["selected_depth_topic"] = str(self._resolved_depth_topic or self.DEPTH_TOPIC)
            metrics_payload["selected_image_topic"] = str(self._resolved_image_topic or self.IMAGE_TOPIC or "")
            path = self._save_metrics_json("depth_perception_metrics.json", metrics_payload)
            self._set_test_diagnostics(depth_perception_metrics_json=path)
            return path

        for d in self.TEST_DISTANCES:
            sample: Dict[str, Any] = {
                "distance_m": float(d),
                "status": "RUNNING",
                "resolved_depth_topic": str(self._resolved_depth_topic or self.DEPTH_TOPIC),
                "resolved_image_topic": str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
            }
            try:
                if last_depth_stamp_s is None:
                    baseline_msg = self._wait_depth(timeout=2.0)
                    last_depth_stamp_s = self._msg_stamp_s(baseline_msg)
                    sample["baseline_stamp_s"] = float(last_depth_stamp_s)

                simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
                reset_depth_msg = self._wait_depth_after(
                    prev_stamp_s=last_depth_stamp_s,
                    timeout=3.0,
                    stage=f"depth_perception_reset_d_{str(d).replace('.', '_')}",
                )
                reset_stamp_s = self._msg_stamp_s(reset_depth_msg)
                sample["reset_frame_stamp_s"] = float(reset_stamp_s)

                simulator.set_pose(cube_name, float(d) + 0.25, 0.0, cube_z)
                depth_msg, confirmation_stamps = self._wait_confirmed_depth_after(
                    prev_stamp_s=reset_stamp_s,
                    stage=f"depth_perception_target_d_{str(d).replace('.', '_')}",
                )
                depth_stamp_s = self._msg_stamp_s(depth_msg)
                last_depth_stamp_s = depth_stamp_s
                sample["depth_frame_stamp_s"] = float(depth_stamp_s)
                sample["post_move_depth_frame_stamps_s"] = [float(v) for v in confirmation_stamps]

                self._set_test_diagnostics(depth_topic_selected=self._resolved_depth_topic)
                depth_m = self._depth_msg_to_meters(depth_msg)
                color_msg = self._wait_color_after(
                    prev_stamp_s=depth_stamp_s - 1e-6,
                    timeout=2.0,
                    stage=f"depth_perception_color_d_{str(d).replace('.', '_')}",
                )
                bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                if color_msg is not None:
                    sample["color_frame_stamp_s"] = float(self._msg_stamp_s(color_msg))

                z, point, roi_meta = self._measure_depth_with_meta(depth_m, bgr=bgr, color_hint="green")
                sample["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                sample["measurement_roi"] = roi_meta
                sample["measurement_source"] = str(roi_meta.get("source", ""))
                if z is None or np.isnan(z) or z <= 0.0:
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[f"distance={float(d):.2f}", "reason=invalid_measurement"],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "invalid_measurement"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="invalid_measurement")
                    raise RuntimeError(f"Invalid depth measurement at distance={d}: {z}")

                if first_frame_diagnostics is None:
                    first_frame_diagnostics = self._depth_frame_stats(depth_msg, depth_m)
                    first_frame_diagnostics["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                    first_frame_diagnostics["measurement_roi"] = roi_meta
                    self._set_test_diagnostics(depth_perception_first_frame=first_frame_diagnostics)

                roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))
                far_clip_saturated = self._is_far_clip_saturated(z, roi_stats)
                sample["roi_depth_stats"] = roi_stats
                sample["measured_depth_m"] = float(z)
                sample["far_clip_saturated"] = bool(far_clip_saturated)

                if far_clip_saturated:
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_saturation_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[
                            f"distance={float(d):.2f}",
                            f"z={float(z):.3f}",
                            f"clip_far={float(self.clip_far):.3f}",
                            "reason=far_clip_saturation",
                        ],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "far_clip_saturation"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="far_clip_saturation")
                    raise AssertionError(
                        f"Depth measurement saturated at far clip at distance={d}: z={z}, clip_far={self.clip_far}"
                    )

                if not (self.clip_near <= float(z) <= (self.clip_far + 0.5)):
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_clip_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[f"distance={float(d):.2f}", f"z={float(z):.3f}", "reason=clip_range"],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "clip_range"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="clip_range")
                    raise AssertionError(
                        f"Depth out of clip range at distance={d}: z={z}, clip=({self.clip_near}, {self.clip_far})"
                    )

                abs_err = abs(float(z) - float(d))
                rel_err = abs_err / float(d) * 100.0
                sample["abs_error_m"] = float(abs_err)
                sample["rel_error_pct"] = float(rel_err)

                if abs_err > float(self.MAX_ABS_ERROR_M):
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_abs_err_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[
                            f"distance={float(d):.2f}",
                            f"z={float(z):.3f}",
                            f"abs_err={float(abs_err):.3f}",
                            "reason=abs_error",
                        ],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "abs_error"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="abs_error")
                    raise AssertionError(
                        f"Depth absolute error too high at distance={d}: abs_err={abs_err:.4f}, max={self.MAX_ABS_ERROR_M}"
                    )

                sample["status"] = "PASS"
                results.append(sample)
                measured_values.append(float(z))
            except Exception:
                if sample not in results:
                    results.append(sample)
                raise

            x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
            if bgr is not None:
                rgb_dbg = bgr.copy()
            else:
                rgb_dbg = self._depth_preview(depth_m)
            cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                rgb_dbg,
                f"d={float(d):.2f} z={float(z):.3f}m",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._save_frame(f"depth_perception_rgb_d_{str(d).replace('.', '_')}.png", rgb_dbg)

            depth_dbg = self._depth_preview(depth_m)
            cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                depth_dbg,
                f"roi_min={roi_stats['min_m'] if roi_stats['min_m'] is not None else float('nan'):.3f} "
                f"roi_max={roi_stats['max_m'] if roi_stats['max_m'] is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._save_frame(f"depth_perception_depth_d_{str(d).replace('.', '_')}.png", depth_dbg)

        monotonic_ok = all(measured_values[i] < measured_values[i + 1] for i in range(len(measured_values) - 1))
        if not monotonic_ok:
            _persist_depth_metrics(status="FAIL", error_reason="monotonicity")
            raise AssertionError(f"Depth monotonicity failed: {measured_values}")

        metrics_json = _persist_depth_metrics(status="PASS")
        return {
            "id": "DEPTH",
            "metrics": {
                "distances_m": list(self.TEST_DISTANCES),
                "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
                "measurements": results,
                "monotonic_increasing": True,
                "first_frame_diagnostics": first_frame_diagnostics or {},
                "topic_diagnostics": self.get_last_test_diagnostics().get("depth_topic_resolution", {}),
                "selected_depth_topic": self._resolved_depth_topic,
                "selected_image_topic": self._resolved_image_topic,
            },
            "metrics_json": metrics_json,
        }

    def c3_view_angle_stability_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "samples_target": int(self.C3_SAMPLES),
            "radius_m": float(self.C3_RADIUS_M),
            "expected_curve": "front_face_depth = radius*cos(theta) - target_half_depth",
            "mean_abs_error_limit_m": float(self.C3_MEAN_ABS_ERROR_M),
            "max_abs_error_limit_m": float(self.C3_MAX_ABS_ERROR_M),
            "samples": [],
            "mode": "",
        }

        self._open_test_scene(simulator, "c3_view_angle_stability_test")
        if not simulator.wait_for_model_spawn(self.C3_TARGET_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C3_TARGET_CUBE_NAME}")

        samples: List[Dict[str, Any]] = []
        use_camera_orbit = simulator.wait_for_model_spawn(self.camera_model_name, timeout=10)
        last_depth_stamp_s: Optional[float] = None

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

                    depth_msg = self._wait_depth_after(
                        prev_stamp_s=last_depth_stamp_s,
                        timeout=3.0,
                        stage=f"c3_camera_orbit_{i}",
                    )
                    last_depth_stamp_s = self._msg_stamp_s(depth_msg)
                    depth_m = self._depth_msg_to_meters(depth_msg)
                    color_msg = self._wait_color_after(
                        prev_stamp_s=last_depth_stamp_s - 1e-6,
                        timeout=2.0,
                        stage=f"c3_camera_orbit_color_{i}",
                    )
                    bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                    z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                    if z is None or np.isnan(z) or np.isinf(z):
                        continue
                    expected_depth = max(
                        0.0,
                        float(self.C3_RADIUS_M) - (float(self.C3_TARGET_SIZE_X_M) * 0.5),
                    )
                    abs_err = abs(float(z) - expected_depth)
                    samples.append(
                        {
                            "sample_index": int(i),
                            "theta_rad": float(theta),
                            "camera_x_m": float(cam_x),
                            "camera_y_m": float(cam_y),
                            "measured_depth_m": float(z),
                            "expected_depth_m": float(expected_depth),
                            "abs_error_m": float(abs_err),
                        }
                    )

                    if i in (0, self.C3_SAMPLES // 2):
                        dbg = self._draw_debug(depth_m, bgr, point, [f"C3 camera orbit", f"sample={i}", f"depth={z:.3f}m"])
                        artifacts.append(self._save_frame(f"c3_camera_orbit_{i}.png", dbg))

                if len(samples) < max(10, int(0.6 * self.C3_SAMPLES)):
                    raise RuntimeError(f"Too few valid measurements in camera orbit mode: {len(samples)}")
            except Exception as exc:
                metrics["mode_error"] = str(exc)
                samples.clear()
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

                depth_msg = self._wait_depth_after(
                    prev_stamp_s=last_depth_stamp_s,
                    timeout=3.0,
                    stage=f"c3_target_orbit_{i}",
                )
                last_depth_stamp_s = self._msg_stamp_s(depth_msg)
                depth_m = self._depth_msg_to_meters(depth_msg)
                color_msg = self._wait_color_after(
                    prev_stamp_s=last_depth_stamp_s - 1e-6,
                    timeout=2.0,
                    stage=f"c3_target_orbit_color_{i}",
                )
                bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                if z is None or np.isnan(z) or np.isinf(z):
                    continue
                expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C3_TARGET_SIZE_X_M))
                abs_err = abs(float(z) - expected_depth)
                samples.append(
                    {
                        "sample_index": int(i),
                        "theta_rad": float(theta),
                        "target_center_x_m": float(x),
                        "target_center_y_m": float(y),
                        "expected_depth_m": float(expected_depth),
                        "measured_depth_m": float(z),
                        "abs_error_m": float(abs_err),
                    }
                )

                if i in (0, self.C3_SAMPLES // 2):
                    dbg = self._draw_debug(
                        depth_m,
                        bgr,
                        point,
                        [f"C3 equivalent", f"sample={i}", f"depth={z:.3f}m", f"expected={expected_depth:.3f}m"],
                    )
                    artifacts.append(self._save_frame(f"c3_equivalent_{i}.png", dbg))

        if len(samples) < 10:
            metrics["samples"] = samples
            metrics_path = self._save_metrics_json("c3_view_angle_stability_metrics.json", metrics)
            raise RuntimeError(f"C3 failed: too few valid depth samples ({len(samples)})")

        abs_errors = [float(sample["abs_error_m"]) for sample in samples]
        measured_depths = [float(sample["measured_depth_m"]) for sample in samples]
        expected_depths = [float(sample["expected_depth_m"]) for sample in samples]

        mean_abs_error = float(np.mean(abs_errors))
        max_abs_error = float(np.max(abs_errors))
        metrics["samples"] = samples
        metrics["mean_measured_depth_m"] = float(np.mean(measured_depths))
        metrics["mean_expected_depth_m"] = float(np.mean(expected_depths))
        metrics["mean_abs_error_m"] = mean_abs_error
        metrics["max_abs_error_m"] = max_abs_error
        metrics["checks"] = {
            "mean_abs_error_ok": bool(mean_abs_error <= self.C3_MEAN_ABS_ERROR_M),
            "max_abs_error_ok": bool(max_abs_error <= self.C3_MAX_ABS_ERROR_M),
        }

        metrics_path = self._save_metrics_json("c3_view_angle_stability_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C3 failed: mean_abs_error={mean_abs_error:.4f} (limit={self.C3_MEAN_ABS_ERROR_M:.4f}), "
                f"max_abs_error={max_abs_error:.4f} (limit={self.C3_MAX_ABS_ERROR_M:.4f})"
            )

        return {"id": "C3", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c5_working_range_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "x_values_m": self._iter_float_range(self.C5_START_X, self.C5_END_X, self.C5_STEP),
            "tolerance_m": float(self.C5_DEPTH_TOLERANCE_M),
            "clip_margin_m": float(self.C5_CLIP_MARGIN_M),
            "target_size_x_m": float(self.C5_TARGET_SIZE_X_M),
            "samples": [],
            "first_frame_diagnostics": {},
        }

        self._open_test_scene(simulator, "c5_working_range_test")
        if not simulator.wait_for_model_spawn(self.C5_RANGE_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C5_RANGE_CUBE_NAME}")

        first_frame_diagnostics: Optional[Dict[str, Any]] = None
        last_depth_stamp_s: Optional[float] = None
        for x in metrics["x_values_m"]:
            self._move_and_settle(simulator, self.C5_RANGE_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.3)
            depth_msg = self._wait_depth_after(
                prev_stamp_s=last_depth_stamp_s,
                timeout=3.0,
                stage=f"c5_x_{str(x).replace('.', '_')}",
            )
            last_depth_stamp_s = self._msg_stamp_s(depth_msg)
            self._set_test_diagnostics(depth_topic_selected=self._resolved_depth_topic)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._wait_color_after(
                prev_stamp_s=last_depth_stamp_s - 1e-6,
                timeout=2.0,
                stage=f"c5_color_x_{str(x).replace('.', '_')}",
            )
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point, roi_meta = self._measure_depth_with_meta(depth_m, bgr, color_hint="green")

            if first_frame_diagnostics is None:
                first_frame_diagnostics = self._depth_frame_stats(depth_msg, depth_m)
                first_frame_diagnostics["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                first_frame_diagnostics["measurement_roi"] = roi_meta
                self._set_test_diagnostics(c5_first_frame=first_frame_diagnostics)

            roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))
            expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C5_TARGET_SIZE_X_M))
            expected_in_sensor_range = self._c5_expected_in_contract_range(expected_depth)

            finite_ok = bool(z is not None and np.isfinite(z))
            abs_err = float(abs(float(z) - float(expected_depth))) if finite_ok else float("inf")
            sample_ok = bool(finite_ok and expected_in_sensor_range and abs_err <= self.C5_DEPTH_TOLERANCE_M)

            metrics["samples"].append(
                {
                    "x_m": float(x),
                    "expected_front_face_depth_m": float(expected_depth),
                    "expected_in_sensor_range": bool(expected_in_sensor_range),
                    "depth_m": None if z is None else float(z),
                    "measurement_roi": roi_meta,
                    "roi_depth_stats": roi_stats,
                    "finite_ok": finite_ok,
                    "abs_error_m": abs_err if np.isfinite(abs_err) else None,
                    "ok": sample_ok,
                }
            )

            x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
            if bgr is not None:
                rgb_dbg = bgr.copy()
            else:
                rgb_dbg = self._depth_preview(depth_m)
            cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                rgb_dbg,
                f"x={float(x):.2f} z={float(z) if z is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            artifacts.append(self._save_frame(f"c5_rgb_x_{str(x).replace('.', '_')}.png", rgb_dbg))

            depth_dbg = self._depth_preview(depth_m)
            cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                depth_dbg,
                f"roi_min={roi_stats['min_m'] if roi_stats['min_m'] is not None else float('nan'):.3f} "
                f"roi_max={roi_stats['max_m'] if roi_stats['max_m'] is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            artifacts.append(self._save_frame(f"c5_depth_x_{str(x).replace('.', '_')}.png", depth_dbg))

            if abs(float(x) - 0.5) < 1e-9 or abs(float(x) - 5.0) < 1e-9 or abs(float(x) - 10.0) < 1e-9:
                dbg = self._draw_debug(
                    depth_m,
                    bgr,
                    point,
                    [f"C5 x={x:.2f}m", f"depth={z if z is not None else float('nan'):.3f}m", f"ok={sample_ok}"],
                )
                artifacts.append(self._save_frame(f"c5_x_{str(x).replace('.', '_')}.png", dbg))

        if first_frame_diagnostics is not None:
            metrics["first_frame_diagnostics"] = first_frame_diagnostics

        expected_ok_x_values = [
            float(x)
            for x in metrics["x_values_m"]
            if self._c5_expected_in_contract_range(
                self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C5_TARGET_SIZE_X_M))
            )
        ]
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

        metrics["expected_ok_x_values_m"] = expected_ok_x_values
        metrics["expected_x_min_ok_m"] = expected_ok_x_values[0] if expected_ok_x_values else None
        metrics["expected_x_max_ok_m"] = expected_ok_x_values[-1] if expected_ok_x_values else None

        if not expected_ok_x_values:
            metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
            raise AssertionError("C5 failed: no sampled positions fall inside the sensor clip range")

        if best_start is None or best_end is None:
            metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
            raise AssertionError("C5 failed: no stable depth interval found")

        metrics["x_min_ok_m"] = float(best_start)
        metrics["x_max_ok_m"] = float(best_end)
        metrics["topic_diagnostics"] = self.get_last_test_diagnostics().get("depth_topic_resolution", {})
        metrics["selected_depth_topic"] = self._resolved_depth_topic
        metrics["selected_image_topic"] = self._resolved_image_topic
        metrics["checks"] = {
            "x_min_ok_covers_expected": bool(float(best_start) <= float(expected_ok_x_values[0]) + 1e-6),
            "x_max_ok_covers_expected": bool(float(best_end) >= float(expected_ok_x_values[-1]) - 1e-6),
        }

        metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C5 failed: stable interval [{best_start:.2f}, {best_end:.2f}] does not cover "
                f"[{expected_ok_x_values[0]:.2f}, {expected_ok_x_values[-1]:.2f}]"
            )

        return {"id": "C5", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c6_small_displacement_sensitivity_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        x_values = self._iter_float_range_desc(self.C6_START_X, self.C6_END_X, self.C6_STEP)
        metrics: Dict[str, Any] = {
            "x_values_m": x_values,
            "eps_m": float(self.C6_DEPTH_CHANGE_EPS_M),
            "required_ratio": float(self.C6_MIN_CHANGED_RATIO),
            "target_size_x_m": float(self.C6_TARGET_SIZE_X_M),
            "monotonic_tolerance_m": float(self.C6_MONOTONIC_TOLERANCE_M),
            "delta_tolerance_m": float(self.C6_DELTA_TOLERANCE_M),
            "abs_error_tolerance_m": float(self.C6_ABS_ERROR_TOLERANCE_M),
            "samples": [],
        }

        self._open_test_scene(simulator, "c6_small_displacement_sensitivity_test")
        if not simulator.wait_for_model_spawn(self.C6_SHIFT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C6_SHIFT_CUBE_NAME}")

        depths: List[Optional[float]] = []
        last_depth_stamp_s: Optional[float] = None
        for x in x_values:
            self._move_and_settle(simulator, self.C6_SHIFT_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.16)
            depth_msg = self._wait_depth_after(
                prev_stamp_s=last_depth_stamp_s,
                timeout=3.0,
                stage=f"c6_x_{str(x).replace('.', '_')}",
            )
            last_depth_stamp_s = self._msg_stamp_s(depth_msg)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._wait_color_after(
                prev_stamp_s=last_depth_stamp_s - 1e-6,
                timeout=2.0,
                stage=f"c6_color_x_{str(x).replace('.', '_')}",
            )
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point = self._measure_depth(depth_m, bgr, color_hint="yellow")
            z_valid = bool(z is not None and np.isfinite(z))
            depths.append(float(z) if z_valid else None)
            expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C6_TARGET_SIZE_X_M))
            abs_err = abs(float(z) - expected_depth) if z_valid else None
            metrics["samples"].append(
                {
                    "x_m": float(x),
                    "expected_front_face_depth_m": float(expected_depth),
                    "depth_m": None if not z_valid else float(z),
                    "abs_error_m": None if abs_err is None else float(abs_err),
                }
            )

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
        monotonic_violations = 0
        for i in range(1, len(depths)):
            prev = depths[i - 1]
            cur = depths[i]
            if prev is None or cur is None:
                continue
            if float(cur) > float(prev) + float(self.C6_MONOTONIC_TOLERANCE_M):
                monotonic_violations += 1

        expected_step_m = float(self.C6_STEP)
        abs_errors = [
            float(sample["abs_error_m"])
            for sample in metrics["samples"]
            if sample.get("abs_error_m") is not None
        ]
        mean_abs_error = float(np.mean(abs_errors)) if abs_errors else float("inf")

        metrics["delta_abs_m"] = [float(d) for d in deltas]
        metrics["median_delta_m"] = median_delta
        metrics["changed_pairs"] = int(len(changed))
        metrics["pairs_total"] = int(len(deltas))
        metrics["changed_ratio"] = changed_ratio
        metrics["expected_step_m"] = expected_step_m
        metrics["monotonic_violations"] = int(monotonic_violations)
        metrics["mean_abs_error_m"] = mean_abs_error
        metrics["checks"] = {
            "changed_ratio_ge_0_8": bool(changed_ratio >= self.C6_MIN_CHANGED_RATIO),
            "monotonic_nonincreasing": bool(monotonic_violations == 0),
            "median_delta_matches_step": bool(abs(median_delta - expected_step_m) <= self.C6_DELTA_TOLERANCE_M),
            "mean_abs_error_ok": bool(mean_abs_error <= self.C6_ABS_ERROR_TOLERANCE_M),
        }

        metrics_path = self._save_metrics_json("c6_small_displacement_sensitivity_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C6 failed: changed_ratio={changed_ratio:.3f}, monotonic_violations={monotonic_violations}, "
                f"median_delta={median_delta:.4f}, expected_step={expected_step_m:.4f}, mean_abs_error={mean_abs_error:.4f}"
            )

        return {"id": "C6", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
