from __future__ import annotations

import copy
import os
import threading
import time
from math import tan
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
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
    # Смещения подобраны так, чтобы occ_25 имел меньшую окклюзию (больше blue-пикселей),
    # а occ_50 — большую окклюзию (меньше blue-пикселей).
    C7_CASES = {"occ_25": 0.20, "occ_50": 0.10}
    C7_MIN_PIXELS = 800
    MIN_DISPARITY_PX = 2
    C8_OBJECTS = ("obj_near_cube", "obj_near_sphere", "obj_far_cube", "obj_far_sphere", "wall_left", "wall_right")
    S1_MAX_REL_ERROR = 0.10
    S1_MIN_PASS_OBJECTS = 3
    S2_MIN_VALID_GAIN = 0.05
    S2_CROP_BLOCK_SIZE = 21
    S2_CROP_TEXTURE_THRESHOLD = 10
    S2_CROP_UNIQUENESS_RATIO = 10
    S2_ROI_Y0_RATIO = 0.26
    S2_ROI_Y1_RATIO = 0.58
    S2_ROI_X0_RATIO = 0.24
    S2_ROI_X1_RATIO = 0.76
    # Первичный прогрев топиков/кадров для stereo делаем длиннее,
    # иначе на "холодном" запуске часто прилетают таймауты.
    PAIR_TIMEOUT_S = 25.0
    PAIR_RETRIES = 3
    PAIR_QUEUE_SIZE = 20
    PAIR_SLOP_S = 0.08
    PAIR_MAX_SKEW_S = 0.08
    TOPIC_WARMUP_TIMEOUT_S = 25.0

    def __init__(self, CONFIG):
        super().__init__()
        self.CONFIG = CONFIG

        worlds_root = Path(CONFIG["WORLDS_PATH"])
        if not worlds_root.is_absolute():
            worlds_root = Path(CONFIG["ROOT_PATH"]) / worlds_root

        sensors_root = CONFIG["SENSORS_PATH"]
        if not os.path.isabs(sensors_root):
            sensors_root = os.path.join(CONFIG["ROOT_PATH"], sensors_root)

        self.sensor_sdf_path = os.path.join(sensors_root, self.sensor_type, f"{self.sensor_name}.sdf")

        self.test_to_world = {
            "stereo_topics_presence_test": str(worlds_root / "camera_c4_geometries.world"),
            "stereo_disparity_test": str(worlds_root / "camera_c1_single_cube.world"),
            "stereo_occlusion_test": str(worlds_root / "camera_c7_occlusion.world"),
            "s1_stereo_accuracy_test": str(worlds_root / "camera_c8_stereo_complex.world"),
            "s2_texture_vs_smooth_stability_test": str(worlds_root / "camera_c8_stereo_complex.world"),
        }

        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)
        self.baseline = float(self.BASELINE_M)
        self._last_test_diagnostics: Dict[str, Any] = {}
        self._last_scene_diag: Dict[str, Any] = {}
        self._resolved_left_topic = str(self.LEFT_IMAGE_TOPIC)
        self._resolved_right_topic = str(self.RIGHT_IMAGE_TOPIC)

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

    def _set_test_diagnostics(self, **kwargs) -> None:
        self._last_test_diagnostics.update(kwargs)

    def get_last_test_diagnostics(self) -> Dict[str, Any]:
        return copy.deepcopy(self._last_test_diagnostics)

    def get_expected_topics(self) -> List[str]:
        topics: List[str] = [str(self.LEFT_IMAGE_TOPIC), str(self.RIGHT_IMAGE_TOPIC)]
        return [topic for topic in topics if topic.strip()]

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

    def _reset_resolved_stereo_topics(self) -> None:
        self._last_scene_diag = {}
        self._resolved_left_topic = str(self.LEFT_IMAGE_TOPIC)
        self._resolved_right_topic = str(self.RIGHT_IMAGE_TOPIC)

    def _update_resolved_stereo_topics(self, simulator) -> Dict[str, Any]:
        scene_diag = self._scene_diag(simulator)
        self._last_scene_diag = copy.deepcopy(scene_diag) if isinstance(scene_diag, dict) else {}

        resolved_topics = scene_diag.get("resolved_topics", {}) if isinstance(scene_diag, dict) else {}
        if isinstance(resolved_topics, dict):
            left_candidate = str(resolved_topics.get(self.LEFT_IMAGE_TOPIC, "") or "").strip()
            right_candidate = str(resolved_topics.get(self.RIGHT_IMAGE_TOPIC, "") or "").strip()
            if left_candidate:
                self._resolved_left_topic = left_candidate
            if right_candidate:
                self._resolved_right_topic = right_candidate

        return scene_diag

    def _open_test_scene(self, simulator, test_name: str) -> None:
        self._last_test_diagnostics = {}
        self._reset_resolved_stereo_topics()
        display_env = self._ensure_render_display_env()
        if display_env:
            self._set_test_diagnostics(stereo_render_env={"display_env": dict(display_env)})
        world = self.test_to_world[test_name]
        if not simulator.open_scene(
            world,
            self.sensor_sdf_path,
            expected_topics=self.get_expected_topics(),
            sensor_name=self.sensor_name,
        ):
            diag = self._scene_diag(simulator)
            self._last_scene_diag = copy.deepcopy(diag) if isinstance(diag, dict) else {}
            reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
            raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")

        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
        scene_diag = self._update_resolved_stereo_topics(simulator)
        self._set_test_diagnostics(
            stereo_scene={
                "display_env": dict(display_env),
                "scene_reason": str(scene_diag.get("reason", "")) if scene_diag else "",
                "expected_left": str(self.LEFT_IMAGE_TOPIC),
                "expected_right": str(self.RIGHT_IMAGE_TOPIC),
                "resolved_left": str(self._resolved_left_topic),
                "resolved_right": str(self._resolved_right_topic),
                "topic_mapping_changed": bool(
                    str(self._resolved_left_topic) != str(self.LEFT_IMAGE_TOPIC)
                    or str(self._resolved_right_topic) != str(self.RIGHT_IMAGE_TOPIC)
                ),
            }
        )

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
        left, right, _ = self._wait_pair_closest(timeout=timeout, retries=1, max_skew_s=float(self.PAIR_MAX_SKEW_S))
        return left, right

    @staticmethod
    def _msg_stamp(msg: Image) -> float:
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            return float(time.time())
        return stamp

    @staticmethod
    def _pair_stamp(left_msg: Image, right_msg: Image) -> float:
        return float(max(StereoProfileBase._msg_stamp(left_msg), StereoProfileBase._msg_stamp(right_msg)))

    @staticmethod
    def _list_image_topics() -> List[str]:
        try:
            published = rospy.get_published_topics()
        except Exception:
            return []
        return sorted([name for name, msg_type in published if msg_type == "sensor_msgs/Image"])

    @staticmethod
    def _looks_like_side_topic(topic: str, side: str) -> bool:
        normalized = str(topic or "").strip()
        if not normalized:
            return False
        if side == "left":
            return ("/left/" in normalized) or ("_left/" in normalized) or normalized.endswith("_left/image_raw")
        if side == "right":
            return ("/right/" in normalized) or ("_right/" in normalized) or normalized.endswith("_right/image_raw")
        return False

    @classmethod
    def _is_valid_stereo_pair(cls, left_topic: str, right_topic: str) -> bool:
        left = str(left_topic or "").strip()
        right = str(right_topic or "").strip()
        if not left or not right or left == right:
            return False
        return bool(cls._looks_like_side_topic(left, "left") and cls._looks_like_side_topic(right, "right"))

    def _resolve_stereo_topics(self, warmup_timeout: float) -> Tuple[str, str, Dict[str, Any]]:
        expected_left = str(self.LEFT_IMAGE_TOPIC)
        expected_right = str(self.RIGHT_IMAGE_TOPIC)
        cached_left = str(self._resolved_left_topic or "").strip()
        cached_right = str(self._resolved_right_topic or "").strip()
        sensor_name = str(self.sensor_name)

        preferred_pairs: List[Tuple[str, str, str]] = []
        if self._is_valid_stereo_pair(cached_left, cached_right):
            preferred_pairs.append((cached_left, cached_right, "scene_resolved"))
        if expected_left and expected_right and (expected_left, expected_right) != (cached_left, cached_right):
            preferred_pairs.append((expected_left, expected_right, "expected"))
        preferred_pairs.extend(
            [
                (f"/{sensor_name}_left/image_raw", f"/{sensor_name}_right/image_raw", "name_underscore"),
                (f"/{sensor_name}/left/image_raw", f"/{sensor_name}/right/image_raw", "name_namespace"),
            ]
        )

        deadline = time.time() + float(warmup_timeout)
        last_topics: List[str] = []
        while time.time() < deadline:
            topics = self._list_image_topics()
            last_topics = topics
            for left_topic, right_topic, source in preferred_pairs:
                if self._is_valid_stereo_pair(left_topic, right_topic) and left_topic in topics and right_topic in topics:
                    return left_topic, right_topic, {
                        "expected_left": expected_left,
                        "expected_right": expected_right,
                        "selected_left": left_topic,
                        "selected_right": right_topic,
                        "selected_source": source,
                        "topics_found": topics,
                        "topic_mapping_changed": bool(left_topic != expected_left or right_topic != expected_right),
                    }
            time.sleep(0.2)

        if self._is_valid_stereo_pair(expected_left, expected_right) and expected_left in last_topics and expected_right in last_topics:
            return expected_left, expected_right, {
                "expected_left": expected_left,
                "expected_right": expected_right,
                "selected_left": expected_left,
                "selected_right": expected_right,
                "selected_source": "expected_after_warmup",
                "topics_found": last_topics,
                "topic_mapping_changed": False,
            }

        for left_topic, right_topic, source in preferred_pairs[1:]:
            if self._is_valid_stereo_pair(left_topic, right_topic) and left_topic in last_topics and right_topic in last_topics:
                return left_topic, right_topic, {
                    "expected_left": expected_left,
                    "expected_right": expected_right,
                    "selected_left": left_topic,
                    "selected_right": right_topic,
                    "selected_source": f"{source}_after_warmup",
                    "topics_found": last_topics,
                    "topic_mapping_changed": True,
                }
        safe_candidates = [
            {
                "left": left_topic,
                "right": right_topic,
                "source": source,
                "published": bool(left_topic in last_topics and right_topic in last_topics),
            }
            for left_topic, right_topic, source in preferred_pairs
            if self._is_valid_stereo_pair(left_topic, right_topic)
        ]
        published_left_candidates = [topic for topic in last_topics if self._looks_like_side_topic(topic, "left")]
        published_right_candidates = [topic for topic in last_topics if self._looks_like_side_topic(topic, "right")]
        sensor_namespace_left = [topic for topic in published_left_candidates if sensor_name and sensor_name in topic]
        sensor_namespace_right = [topic for topic in published_right_candidates if sensor_name and sensor_name in topic]

        return "", "", {
            "expected_left": expected_left,
            "expected_right": expected_right,
            "selected_left": "",
            "selected_right": "",
            "selected_source": "unresolved_no_safe_pair",
            "topics_found": last_topics,
            "topic_mapping_changed": False,
            "scene_resolved_left": cached_left,
            "scene_resolved_right": cached_right,
            "safe_candidates": safe_candidates,
            "published_left_candidates": published_left_candidates,
            "published_right_candidates": published_right_candidates,
            "sensor_namespace_left_candidates": sensor_namespace_left,
            "sensor_namespace_right_candidates": sensor_namespace_right,
            "resolve_reason": "no_safe_stereo_pair_after_warmup",
        }

    def _wait_pair_closest(
        self,
        timeout: float = 25.0,
        retries: int = 3,
        max_skew_s: float = 0.08,
        min_pair_stamp_s: Optional[float] = None,
    ) -> Tuple[Image, Image, float]:
        try:
            import message_filters
        except Exception as exc:  # noqa: BLE001
            self._set_test_diagnostics(stereo_pair_capture={"reason": "message_filters_import_error", "error": str(exc)})
            raise RuntimeError(f"message_filters import failed: {exc}")

        left_topic, right_topic, topic_diag = self._resolve_stereo_topics(self.TOPIC_WARMUP_TIMEOUT_S)
        pair_diag: Dict[str, Any] = dict(topic_diag)
        pair_diag.update(
            {
                "timeout_s": float(timeout),
                "retries": int(retries),
                "max_skew_s": float(max_skew_s),
                "min_pair_stamp_s": None if min_pair_stamp_s is None else float(min_pair_stamp_s),
                "queue_size": int(self.PAIR_QUEUE_SIZE),
                "slop_s": float(self.PAIR_SLOP_S),
                "attempts": [],
            }
        )

        if not left_topic or not right_topic:
            pair_diag["reason"] = "stereo_topics_unresolved"
            self._set_test_diagnostics(stereo_pair_capture=pair_diag)
            raise RuntimeError("Stereo topics could not be resolved")
        if left_topic == right_topic:
            pair_diag["reason"] = "stereo_topics_collapsed"
            self._set_test_diagnostics(stereo_pair_capture=pair_diag)
            raise RuntimeError(f"Stereo topics collapsed to one topic: {left_topic}")

        for attempt in range(1, int(retries) + 1):
            attempt_diag: Dict[str, Any] = {"attempt": int(attempt), "left_msgs": 0, "right_msgs": 0}
            lock = threading.Lock()
            pair_holder: Dict[str, Any] = {}

            def _left_count(_msg: Image) -> None:
                attempt_diag["left_msgs"] += 1

            def _right_count(_msg: Image) -> None:
                attempt_diag["right_msgs"] += 1

            def _pair_cb(left_msg: Image, right_msg: Image) -> None:
                with lock:
                    if pair_holder:
                        return
                    pair_stamp = self._pair_stamp(left_msg, right_msg)
                    if min_pair_stamp_s is not None and pair_stamp <= float(min_pair_stamp_s) + 1e-6:
                        attempt_diag["pairs_rejected_before_min_stamp"] = int(
                            attempt_diag.get("pairs_rejected_before_min_stamp", 0)
                        ) + 1
                        return
                    skew = abs(self._msg_stamp(left_msg) - self._msg_stamp(right_msg))
                    pair_holder["left"] = left_msg
                    pair_holder["right"] = right_msg
                    pair_holder["skew"] = float(skew)
                    pair_holder["pair_stamp"] = float(pair_stamp)

            left_counter_sub = rospy.Subscriber(left_topic, Image, _left_count, queue_size=200)
            right_counter_sub = rospy.Subscriber(right_topic, Image, _right_count, queue_size=200)

            left_mf = message_filters.Subscriber(left_topic, Image)
            right_mf = message_filters.Subscriber(right_topic, Image)
            sync = message_filters.ApproximateTimeSynchronizer(
                [left_mf, right_mf],
                queue_size=int(self.PAIR_QUEUE_SIZE),
                slop=float(self.PAIR_SLOP_S),
                allow_headerless=False,
            )
            sync.registerCallback(_pair_cb)

            started = time.time()
            try:
                while (time.time() - started) < float(timeout):
                    with lock:
                        if pair_holder:
                            break
                    time.sleep(0.02)
            finally:
                left_counter_sub.unregister()
                right_counter_sub.unregister()
                try:
                    left_mf.sub.unregister()
                    right_mf.sub.unregister()
                except Exception:
                    pass

            with lock:
                has_pair = bool(pair_holder)
                if has_pair:
                    skew = float(pair_holder["skew"])
                    pair_stamp = float(pair_holder["pair_stamp"])
                    attempt_diag["pair_received"] = True
                    attempt_diag["pair_skew_s"] = skew
                    attempt_diag["pair_stamp_s"] = pair_stamp
                    pair_diag["attempts"].append(attempt_diag)
                    self._set_test_diagnostics(stereo_pair_capture=pair_diag)
                    if skew <= float(max_skew_s):
                        return pair_holder["left"], pair_holder["right"], skew
                    attempt_diag["pair_rejected"] = True
                    attempt_diag["pair_reject_reason"] = "skew_above_threshold"
                else:
                    attempt_diag["pair_received"] = False

            pair_diag["attempts"].append(attempt_diag)

        pair_diag["reason"] = "pair_timeout"
        self._set_test_diagnostics(stereo_pair_capture=pair_diag)
        raise RuntimeError("Failed to capture left/right pair with timestamp proximity")

    @staticmethod
    def _fx_from_fov(width_px: int, horizontal_fov_rad: float) -> float:
        if width_px <= 0 or horizontal_fov_rad <= 0.0:
            raise RuntimeError(f"Invalid camera intrinsics for fx estimation: width={width_px}, fov={horizontal_fov_rad}")
        return float((width_px / 2.0) / tan(horizontal_fov_rad / 2.0))

    def _compute_disparity_and_depth(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        if left_bgr.shape[:2] != right_bgr.shape[:2]:
            raise RuntimeError(f"Stereo size mismatch: left={left_bgr.shape[:2]}, right={right_bgr.shape[:2]}")

        if self.baseline <= 0.0:
            raise RuntimeError(f"Invalid baseline: {self.baseline}")

        gray_left = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray_left.shape[:2]
        fx = self._fx_from_fov(w, float(self.horizontal_fov))

        num_disp = max(16, min(256, ((w // 4) // 16) * 16))
        if num_disp < 16:
            num_disp = 16
        block_size = 7

        matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=int(num_disp),
            blockSize=int(block_size),
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            preFilterCap=31,
            uniquenessRatio=8,
            speckleWindowSize=50,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = disparity > 0.0
        depth[valid] = float(fx * float(self.baseline)) / disparity[valid]
        return disparity, depth, float(fx)

    @staticmethod
    def _disparity_to_viz(disparity: np.ndarray) -> np.ndarray:
        valid = np.isfinite(disparity) & (disparity > 0.0)
        h, w = disparity.shape[:2]
        norm_u8 = np.zeros((h, w), dtype=np.uint8)

        if np.any(valid):
            vals = disparity[valid]
            lo = float(np.percentile(vals, 5))
            hi = float(np.percentile(vals, 95))
            if hi <= lo:
                hi = lo + 1e-3
            scaled = np.clip((disparity - lo) / (hi - lo), 0.0, 1.0)
            norm_u8[valid] = (scaled[valid] * 255.0).astype(np.uint8)

        return cv2.applyColorMap(norm_u8, cv2.COLORMAP_TURBO)

    @staticmethod
    def _expand_bbox(
        bbox: Tuple[int, int, int, int],
        width: int,
        height: int,
        pad: int = 6,
    ) -> Tuple[int, int, int, int]:
        x, y, bw, bh = bbox
        x0 = max(0, int(x) - int(pad))
        y0 = max(0, int(y) - int(pad))
        x1 = min(int(width), int(x + bw + pad))
        y1 = min(int(height), int(y + bh + pad))
        return int(x0), int(y0), int(max(1, x1 - x0)), int(max(1, y1 - y0))

    @staticmethod
    def _median_depth_in_bbox(
        depth_map: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[Optional[float], int]:
        x, y, w, h = bbox
        roi = depth_map[y:y + h, x:x + w]
        with np.errstate(invalid="ignore"):
            finite = np.isfinite(roi)
        valid = roi[finite]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None, 0
        return float(np.median(valid)), int(valid.size)

    @staticmethod
    def _valid_ratio(
        disparity: np.ndarray,
        mask: np.ndarray,
    ) -> float:
        area = int(np.count_nonzero(mask))
        if area <= 0:
            return 0.0
        valid = mask & np.isfinite(disparity) & (disparity > 0.0)
        return float(np.count_nonzero(valid) / area)

    @staticmethod
    def _crop_valid_ratio_bm(
        gray_left: np.ndarray,
        gray_right: np.ndarray,
        block_size: int,
        texture_threshold: int,
        uniqueness_ratio: int,
    ) -> float:
        crop_h, crop_w = gray_left.shape[:2]
        if crop_h <= 0 or crop_w <= 0:
            return 0.0

        num_disp = max(16, min(256, ((crop_w // 4) // 16) * 16))
        if num_disp < 16:
            num_disp = 16

        matcher = cv2.StereoBM_create(numDisparities=int(num_disp), blockSize=int(block_size))
        matcher.setTextureThreshold(int(texture_threshold))
        matcher.setUniquenessRatio(int(uniqueness_ratio))
        matcher.setSpeckleWindowSize(50)
        matcher.setSpeckleRange(2)
        matcher.setDisp12MaxDiff(1)

        disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
        valid = np.isfinite(disparity) & (disparity > 0.0)
        return float(np.count_nonzero(valid) / max(1, valid.size))

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

        if color == "blue":
            # Для stereo C7 допускаем более широкий диапазон по S/V,
            # т.к. при headless-рендере синий объект часто темнее ожидаемого.
            blue_main = cv2.inRange(
                hsv,
                np.array([85, 40, 25], dtype=np.uint8),
                np.array([145, 255, 255], dtype=np.uint8),
            )
            blue_dark = cv2.inRange(
                hsv,
                np.array([80, 20, 10], dtype=np.uint8),
                np.array([150, 180, 170], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(blue_main, blue_dark)
            cleaned = self._clean_mask(mask)
            kernel = np.ones((3, 3), np.uint8)
            return cv2.dilate(cleaned, kernel, iterations=1)

        ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
            "green": ((40, 80, 60), (85, 255, 255)),
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

    @staticmethod
    def _set_model_pose_and_readback(model_name: str, x: float, y: float, z: float, settle_s: float = 0.5) -> Dict[str, Any]:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0.0, 0.0, 0.0, 1.0))

        set_resp = set_state(state)
        time.sleep(float(settle_s))

        pose_diag: Dict[str, Any] = {
            "request_pose": {"x": float(x), "y": float(y), "z": float(z)},
            "set_model_state": {
                "success": bool(set_resp.success),
                "status_message": str(set_resp.status_message),
            },
        }

        try:
            get_resp = get_state(model_name, "world")
            pose_diag["get_model_state"] = {
                "success": bool(get_resp.success),
                "status_message": str(get_resp.status_message),
                "pose": {
                    "x": float(get_resp.pose.position.x),
                    "y": float(get_resp.pose.position.y),
                    "z": float(get_resp.pose.position.z),
                },
            }
        except Exception as exc:  # noqa: BLE001
            pose_diag["get_model_state_error"] = str(exc)
        return pose_diag

    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        timeout: float = 1.0,
        convert2cv: bool = False,
    ) -> Optional[Dict[str, Any]]:
        if world_path:
            self._reset_resolved_stereo_topics()
            display_env = self._ensure_render_display_env()
            if display_env:
                self._set_test_diagnostics(stereo_render_env={"display_env": dict(display_env)})
            if not simulator.open_scene(
                world_path,
                self.sensor_sdf_path,
                expected_topics=self.get_expected_topics(),
                sensor_name=self.sensor_name,
            ):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
            self._update_resolved_stereo_topics(simulator)

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
        pair_diag = self.get_last_test_diagnostics().get("stereo_pair_capture", {})
        if pair_diag:
            metrics["topic_diagnostics"] = pair_diag

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

        warm_left_msg, warm_right_msg, _ = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        prev_pair_stamp = self._pair_stamp(warm_left_msg, warm_right_msg)
        self._move_and_settle(simulator, self.C1_CUBE_NAME, x=3.0, y=0.0, z=0.25)
        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
            min_pair_stamp_s=float(prev_pair_stamp),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)

        left_hsv = cv2.cvtColor(left, cv2.COLOR_BGR2HSV)
        right_hsv = cv2.cvtColor(right, cv2.COLOR_BGR2HSV)

        l_area, (lx, ly, lw, lh) = self._bbox(self._red_mask(left_hsv))
        r_area, (rx, ry, rw, rh) = self._bbox(self._red_mask(right_hsv))

        if l_area == 0 or r_area == 0:
            raise AssertionError("Failed to detect red cube in stereo frames")

        l_center_x = lx + lw / 2.0
        r_center_x = rx + rw / 2.0
        disparity_px_signed = float(l_center_x - r_center_x)
        disparity_px = float(abs(disparity_px_signed))

        left_dbg = left.copy()
        right_dbg = right.copy()
        cv2.rectangle(left_dbg, (lx, ly), (lx + lw, ly + lh), (255, 255, 255), 2)
        cv2.rectangle(right_dbg, (rx, ry), (rx + rw, ry + rh), (255, 255, 255), 2)
        self._save_frame("stereo_disparity_left.png", left_dbg)
        self._save_frame("stereo_disparity_right.png", right_dbg)

        metrics = {
            "left_center_x": float(l_center_x),
            "right_center_x": float(r_center_x),
            "disparity_px": float(disparity_px),
            "disparity_px_signed": float(disparity_px_signed),
            "min_disparity_px": float(self.MIN_DISPARITY_PX),
            "pair_skew_s": float(skew),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
        }
        self._set_test_diagnostics(stereo_disparity={"metrics": dict(metrics)})

        if disparity_px_signed < float(self.MIN_DISPARITY_PX):
            raise AssertionError(
                f"Signed disparity too small or inverted: {disparity_px_signed} px < {self.MIN_DISPARITY_PX}"
            )

        return {
            "id": "STEREO_DISPARITY",
            "metrics": metrics,
        }

    def stereo_occlusion_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_occlusion_test")

        if not simulator.wait_for_model_spawn(self.C7_FRONT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_FRONT_CUBE_NAME}")
        if not simulator.wait_for_model_spawn(self.C7_BACK_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_BACK_CUBE_NAME}")

        back_x = 3.6
        front_x = 3.0
        baseline_front_y = 0.40

        # Базово делаем синий объект (back cube) видимым в центре,
        # затем двигаем передний окклюдер по Y.
        move_back = self._set_model_pose_and_readback(self.C7_BACK_CUBE_NAME, x=back_x, y=0.0, z=0.25, settle_s=0.45)
        if not move_back.get("set_model_state", {}).get("success", False):
            raise RuntimeError(f"Failed to position back cube: {move_back}")

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
        move_before = self._set_model_pose_and_readback(
            self.C7_FRONT_CUBE_NAME,
            x=front_x,
            y=baseline_front_y,
            z=0.25,
            settle_s=0.45,
        )
        if not move_before.get("set_model_state", {}).get("success", False):
            raise RuntimeError(f"Failed to position front cube before occlusion cases: {move_before}")

        metrics: Dict[str, Any] = {
            "left": {"blue_pixels": {}},
            "right": {"blue_pixels": {}},
            "cases": dict(self.C7_CASES),
            "threshold": int(self.C7_MIN_PIXELS),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
            "occluder_model": str(self.C7_FRONT_CUBE_NAME),
            "occluded_model": str(self.C7_BACK_CUBE_NAME),
            "occluder_motion": {"before": move_before, "cases": {}, "back_cube": move_back},
            "pair_skew_s": {"before": None, "cases": {}},
        }

        # Базовый кадр до окклюзии
        base_left_msg, base_right_msg, base_skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        prev_pair_stamp = self._pair_stamp(base_left_msg, base_right_msg)
        metrics["pair_skew_s"]["before"] = float(base_skew)
        for side, frame in (("left", self._msg_to_bgr(base_left_msg)), ("right", self._msg_to_bgr(base_right_msg))):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blue = self._color_mask(hsv, "blue")
            blue_count = self._count_pixels(blue)
            metrics[side]["blue_pixels"]["before"] = int(blue_count)

            contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            overlay = frame.copy()
            if contours:
                contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(overlay, f"before: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            self._save_frame(f"stereo_c7_{side}_before_raw.png", frame)
            self._save_frame(f"stereo_c7_{side}_before_blue_mask.png", blue)
            self._save_frame(f"stereo_c7_{side}_before_overlay.png", overlay)

        for case_name, y in self.C7_CASES.items():
            move_diag = self._set_model_pose_and_readback(
                self.C7_FRONT_CUBE_NAME,
                x=front_x,
                y=float(y),
                z=0.25,
                settle_s=0.45,
            )
            metrics["occluder_motion"]["cases"][case_name] = move_diag
            if not move_diag.get("set_model_state", {}).get("success", False):
                raise RuntimeError(f"set_model_state failed for {case_name}: {move_diag}")

            left_msg, right_msg, skew = self._wait_pair_closest(
                timeout=float(self.PAIR_TIMEOUT_S),
                retries=int(self.PAIR_RETRIES),
                max_skew_s=float(self.PAIR_MAX_SKEW_S),
                min_pair_stamp_s=float(prev_pair_stamp),
            )
            prev_pair_stamp = self._pair_stamp(left_msg, right_msg)
            metrics["pair_skew_s"]["cases"][case_name] = float(skew)

            for side, frame in (("left", self._msg_to_bgr(left_msg)), ("right", self._msg_to_bgr(right_msg))):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                blue = self._color_mask(hsv, "blue")
                blue_count = self._count_pixels(blue)
                metrics[side]["blue_pixels"][case_name] = int(blue_count)

                contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                dbg = frame.copy()
                if contours:
                    contour = max(contours, key=cv2.contourArea)
                    x, yb, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(dbg, (x, yb), (x + w, yb + h), (255, 255, 255), 2)
                cv2.putText(dbg, f"{case_name}: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                self._save_frame(f"stereo_c7_{side}_{case_name}_raw.png", frame)
                self._save_frame(f"stereo_c7_{side}_{case_name}_blue_mask.png", blue)
                self._save_frame(f"stereo_c7_{side}_{case_name}_overlay.png", dbg)

        for side in ("left", "right"):
            blue_25 = metrics[side]["blue_pixels"].get("occ_25", 0)
            blue_50 = metrics[side]["blue_pixels"].get("occ_50", 0)

            relation_ok = blue_25 > blue_50
            threshold_ok = blue_25 > self.C7_MIN_PIXELS and blue_50 > self.C7_MIN_PIXELS
            metrics[side]["checks"] = {
                "occlusion_relation": bool(relation_ok),
                "threshold_ok": bool(threshold_ok),
            }

            if not (relation_ok and threshold_ok):
                self._set_test_diagnostics(stereo_occlusion={"metrics": copy.deepcopy(metrics)})
                raise AssertionError(
                    f"Stereo C7 failed on {side}: blue_25={blue_25}, blue_50={blue_50}, threshold={self.C7_MIN_PIXELS}"
                )

        self._set_test_diagnostics(stereo_occlusion={"metrics": copy.deepcopy(metrics)})
        return {"id": "STEREO_C7", "metrics": metrics}

    def s1_stereo_accuracy_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "s1_stereo_accuracy_test")
        for model_name in self.C8_OBJECTS:
            if not simulator.wait_for_model_spawn(model_name, timeout=20):
                raise RuntimeError(f"Model not spawned: {model_name}")

        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)
        disparity, depth_map, fx = self._compute_disparity_and_depth(left, right)

        h, w = left.shape[:2]
        left_hsv = cv2.cvtColor(left, cv2.COLOR_BGR2HSV)
        disparity_viz = self._disparity_to_viz(disparity)

        objects = {
            "obj_near_cube": {"color": "red", "depth_gt": 2.0},
            "obj_near_sphere": {"color": "green", "depth_gt": 2.0},
            "obj_far_cube": {"color": "blue", "depth_gt": 4.5},
            "obj_far_sphere": {"color": "yellow", "depth_gt": 4.5},
        }

        metrics: Dict[str, Any] = {
            "pair_skew_s": float(skew),
            "baseline_m": float(self.baseline),
            "horizontal_fov_rad": float(self.horizontal_fov),
            "fx_px": float(fx),
            "objects": {},
            "max_rel_error": float(self.S1_MAX_REL_ERROR),
            "min_pass_objects": int(self.S1_MIN_PASS_OBJECTS),
        }
        pair_diag = self.get_last_test_diagnostics().get("stereo_pair_capture", {})
        if pair_diag:
            metrics["topic_diagnostics"] = pair_diag

        left_dbg = left.copy()
        disp_dbg = disparity_viz.copy()
        passed = 0

        for obj_name, cfg in objects.items():
            mask = self._color_mask(left_hsv, cfg["color"])
            area, bbox = self._bbox(mask)
            if area <= 0:
                metrics["objects"][obj_name] = {"detected": False, "reason": "color contour not found"}
                continue

            roi = self._expand_bbox(bbox, width=w, height=h, pad=6)
            depth_est, valid_px = self._median_depth_in_bbox(depth_map, roi)
            if depth_est is None:
                metrics["objects"][obj_name] = {
                    "detected": True,
                    "depth_gt_m": float(cfg["depth_gt"]),
                    "depth_est_m": None,
                    "valid_px": int(valid_px),
                    "ok": False,
                    "reason": "no valid disparity in ROI",
                }
                continue

            rel_err = abs(float(depth_est) - float(cfg["depth_gt"])) / float(cfg["depth_gt"])
            ok = rel_err <= float(self.S1_MAX_REL_ERROR)
            if ok:
                passed += 1

            metrics["objects"][obj_name] = {
                "detected": True,
                "depth_gt_m": float(cfg["depth_gt"]),
                "depth_est_m": float(depth_est),
                "valid_px": int(valid_px),
                "relative_error": float(rel_err),
                "ok": bool(ok),
            }

            x, y, bw, bh = roi
            color = (0, 255, 0) if ok else (0, 0, 255)
            cv2.rectangle(left_dbg, (x, y), (x + bw, y + bh), color, 2)
            cv2.rectangle(disp_dbg, (x, y), (x + bw, y + bh), color, 2)
            label = f"{obj_name}: z={depth_est:.2f}m gt={cfg['depth_gt']:.2f} err={rel_err:.2f}"
            cv2.putText(left_dbg, label, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2, cv2.LINE_AA)

        metrics["passed_objects"] = int(passed)
        metrics["checks"] = {"pass_count_ok": bool(passed >= int(self.S1_MIN_PASS_OBJECTS))}

        artifacts = [
            self._save_frame("s1_c8_left.png", left),
            self._save_frame("s1_c8_right.png", right),
            self._save_frame("s1_c8_disparity.png", disparity_viz),
            self._save_frame("s1_c8_left_rois.png", left_dbg),
            self._save_frame("s1_c8_disparity_rois.png", disp_dbg),
        ]

        if passed < int(self.S1_MIN_PASS_OBJECTS):
            raise AssertionError(
                f"S1 failed: passed_objects={passed} < {self.S1_MIN_PASS_OBJECTS}, "
                f"objects={metrics['objects']}"
            )

        return {"id": "S1", "metrics": metrics, "artifacts": artifacts}

    def s2_texture_vs_smooth_stability_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "s2_texture_vs_smooth_stability_test")
        for model_name in self.C8_OBJECTS:
            if not simulator.wait_for_model_spawn(model_name, timeout=20):
                raise RuntimeError(f"Model not spawned: {model_name}")

        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)
        disparity, depth_map, _ = self._compute_disparity_and_depth(left, right)
        disparity_viz = self._disparity_to_viz(disparity)
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        h, w = disparity.shape[:2]
        y0 = int(float(self.S2_ROI_Y0_RATIO) * h)
        y1 = int(float(self.S2_ROI_Y1_RATIO) * h)
        x0 = int(float(self.S2_ROI_X0_RATIO) * w)
        x1 = int(float(self.S2_ROI_X1_RATIO) * w)
        xm = (x0 + x1) // 2

        wall_band = np.zeros((h, w), dtype=bool)
        wall_band[y0:y1, x0:x1] = True

        with np.errstate(invalid="ignore"):
            finite_depth = np.isfinite(depth_map)
        wall_depth_mask = np.zeros((h, w), dtype=bool)
        with np.errstate(invalid="ignore"):
            wall_depth_mask[finite_depth] = (depth_map[finite_depth] >= 4.2) & (depth_map[finite_depth] <= 5.8)
        wall_mask = wall_band & wall_depth_mask

        left_half_mask = np.zeros((h, w), dtype=bool)
        right_half_mask = np.zeros((h, w), dtype=bool)
        left_half_mask[y0:y1, x0:xm] = True
        right_half_mask[y0:y1, xm:x1] = True

        left_wall = wall_mask & left_half_mask
        right_wall = wall_mask & right_half_mask

        # Fallback if depth-based wall isolation is too sparse.
        if np.count_nonzero(left_wall) < 200 or np.count_nonzero(right_wall) < 200:
            left_wall = left_half_mask
            right_wall = right_half_mask

        left_texture = float(np.std(gray[left_wall])) if np.count_nonzero(left_wall) > 0 else 0.0
        right_texture = float(np.std(gray[right_wall])) if np.count_nonzero(right_wall) > 0 else 0.0

        left_crop_left = gray[y0:y1, x0:xm]
        left_crop_right = gray_right[y0:y1, x0:xm]
        right_crop_left = gray[y0:y1, xm:x1]
        right_crop_right = gray_right[y0:y1, xm:x1]

        left_ratio = self._crop_valid_ratio_bm(
            left_crop_left,
            left_crop_right,
            block_size=int(self.S2_CROP_BLOCK_SIZE),
            texture_threshold=int(self.S2_CROP_TEXTURE_THRESHOLD),
            uniqueness_ratio=int(self.S2_CROP_UNIQUENESS_RATIO),
        )
        right_ratio = self._crop_valid_ratio_bm(
            right_crop_left,
            right_crop_right,
            block_size=int(self.S2_CROP_BLOCK_SIZE),
            texture_threshold=int(self.S2_CROP_TEXTURE_THRESHOLD),
            uniqueness_ratio=int(self.S2_CROP_UNIQUENESS_RATIO),
        )

        if right_texture >= left_texture:
            textured_side = "right"
            smooth_side = "left"
            valid_ratio_textured = float(right_ratio)
            valid_ratio_smooth = float(left_ratio)
        else:
            textured_side = "left"
            smooth_side = "right"
            valid_ratio_textured = float(left_ratio)
            valid_ratio_smooth = float(right_ratio)

        gain = float(valid_ratio_textured - valid_ratio_smooth)
        check_gain_005 = gain >= float(self.S2_MIN_VALID_GAIN)
        check_non_worse = valid_ratio_textured >= valid_ratio_smooth

        metrics = {
            "pair_skew_s": float(skew),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
            "assignment_by_texture_std": {
                "left_std": float(left_texture),
                "right_std": float(right_texture),
                "textured_side": textured_side,
                "smooth_side": smooth_side,
            },
            "crop_matcher": {
                "type": "StereoBM",
                "block_size": int(self.S2_CROP_BLOCK_SIZE),
                "texture_threshold": int(self.S2_CROP_TEXTURE_THRESHOLD),
                "uniqueness_ratio": int(self.S2_CROP_UNIQUENESS_RATIO),
                "crop_bounds": {"x0": int(x0), "x1": int(x1), "xm": int(xm), "y0": int(y0), "y1": int(y1)},
            },
            "valid_ratio": {
                "left": float(left_ratio),
                "right": float(right_ratio),
                "textured": float(valid_ratio_textured),
                "smooth": float(valid_ratio_smooth),
                "gain_textured_minus_smooth": float(gain),
            },
            "checks": {
                "textured_ge_smooth_plus_0_05": bool(check_gain_005),
                "textured_ge_smooth": bool(check_non_worse),
            },
        }

        dbg = disparity_viz.copy()
        left_color = (255, 255, 0) if textured_side == "left" else (200, 200, 200)
        right_color = (255, 255, 0) if textured_side == "right" else (200, 200, 200)
        cv2.rectangle(dbg, (x0, y0), (xm, y1), left_color, 2)
        cv2.rectangle(dbg, (xm, y0), (x1, y1), right_color, 2)
        cv2.putText(dbg, f"left ratio={left_ratio:.3f} std={left_texture:.1f}", (x0, max(20, y0 - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, left_color, 2, cv2.LINE_AA)
        cv2.putText(dbg, f"right ratio={right_ratio:.3f} std={right_texture:.1f}", (xm, max(20, y0 - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, right_color, 2, cv2.LINE_AA)
        cv2.putText(dbg, f"gain={gain:.3f}", (x0, max(20, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        artifacts = [
            self._save_frame("s2_c8_left.png", left),
            self._save_frame("s2_c8_right.png", right),
            self._save_frame("s2_c8_disparity.png", disparity_viz),
            self._save_frame("s2_c8_disparity_rois.png", dbg),
        ]

        self._set_test_diagnostics(stereo_s2={"metrics": copy.deepcopy(metrics)})

        if not (check_non_worse and check_gain_005):
            raise AssertionError(
                "S2 failed: "
                f"textured_ratio={valid_ratio_textured:.4f}, "
                f"smooth_ratio={valid_ratio_smooth:.4f}, "
                f"gain={gain:.4f}, "
                f"required_gain>={self.S2_MIN_VALID_GAIN:.4f}"
            )

        return {"id": "S2", "metrics": metrics, "artifacts": artifacts}
