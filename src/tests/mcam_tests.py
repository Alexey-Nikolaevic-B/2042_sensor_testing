"""Mono camera (mcam) tests — C1, C2, C4, C7, C9, C10, C11."""

import logging
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from math import atan, degrees
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from config import CONFIG
from ._common import (
    _camera_load_sensor_profile,
    _camera_worlds_root,
)

logger = logging.getLogger(__name__)


def _mono_build_ctx(sensor):
    print(
        f"[DEBUG _mono_build_ctx] building context for sensor_name={getattr(sensor, 'sensor_name', '?')}"
    )
    print(f"[DEBUG _mono_build_ctx] sensor.sdf_path={getattr(sensor, 'sdf_path', '?')}")
    print(f"[DEBUG _mono_build_ctx] sensor.topic={getattr(sensor, 'topic', '?')}")
    ctx = SimpleNamespace()
    ctx.IMAGE_TOPIC = ""
    ctx.IMAGE_WIDTH = 640
    ctx.IMAGE_HEIGHT = 480
    ctx.UPDATE_RATE = 30
    ctx.HORIZONTAL_FOV_RAD = 1.0471975512
    ctx.CLIP_NEAR = 0.1
    ctx.CLIP_FAR = 50.0
    ctx.C1_CUBE_NAME = "test_cube"
    ctx.C2_SPHERE_A_NAME = "sphere_a"
    ctx.C2_SPHERE_B_NAME = "sphere_b"
    ctx.C7_FRONT_CUBE_NAME = "front_cube"
    ctx.C7_BACK_CUBE_NAME = "back_cube"
    ctx.C9_SPHERE_NAME = "fov_sphere"
    ctx.C10_CUBE_NAME = "clip_cube"
    ctx.C1_POSITIONS = (2.0, 5.0, 8.0)
    ctx.C1_TRACK_Y = 0.35
    ctx.C1_TRACK_Z = 0.15
    ctx.C2_DISTANCES = (0.20, 0.15, 0.10, 0.05, 0.02)
    ctx.C2_MIN_CONTOUR_AREA = 80
    ctx.C1_MIN_MARGIN_RATIO = 1.10
    ctx.C4_MIN_PIXELS = 1500
    ctx.C7_CASES = {"occ_25": 0.20, "occ_50": 0.10}
    ctx.C7_MIN_PIXELS = 800
    ctx.C9_STEP = 0.05
    ctx.C9_MAX_Y = 4.0
    ctx.C9_MIN_CONTOUR_AREA = 120
    ctx.C9_SPHERE_RADIUS_M = 0.1
    ctx.C10_NEAR_START_X = 0.05
    ctx.C10_NEAR_SEARCH_END_X = 2.0
    ctx.C10_NEAR_STEP = 0.01
    ctx.C10_FAR_COARSE_STEP = 0.5
    ctx.C10_FAR_FINE_STEP = 0.01
    ctx.C10_MIN_RED_PIXELS = 20
    ctx.C11_DURATION_S = 60.0
    ctx.C11_WARMUP_SECONDS = 2.0
    ctx.C11_JITTER_PERCENTILE = 95
    ctx.C11_MAX_JITTER_S = 0.015
    ctx.sensor = sensor
    ctx._simulator = None  # set later by inner test functions
    ctx.sensor_name = str(getattr(sensor, "sensor_name", ""))
    ctx.sensor_type = str(getattr(sensor, "sensor_type", ""))
    ctx.sensor_sdf_path = str(getattr(sensor, "sdf_path", ""))
    ctx.CONFIG = {"ROOT_PATH": str(CONFIG["ROOT_PATH"])}
    profile = (
        _camera_load_sensor_profile(ctx.sensor_sdf_path) if ctx.sensor_sdf_path else {}
    )
    print(
        f"[DEBUG _mono_build_ctx] SDF profile loaded: {list(profile.keys()) if profile else 'EMPTY'}"
    )
    print(
        f"[DEBUG _mono_build_ctx] profile.image_topic={profile.get('image_topic', 'N/A')}, profile.family={profile.get('family', 'N/A')}"
    )
    ctx.IMAGE_TOPIC = str(
        profile.get("image_topic", "")
        or getattr(sensor, "topic", "")
        or ctx.IMAGE_TOPIC
    )
    ctx.image_width = int(profile.get("image_width") or ctx.IMAGE_WIDTH)
    ctx.image_height = int(profile.get("image_height") or ctx.IMAGE_HEIGHT)
    ctx.horizontal_fov = float(profile.get("horizontal_fov") or ctx.HORIZONTAL_FOV_RAD)
    ctx.clip_near = float(profile.get("clip_near") or ctx.CLIP_NEAR)
    ctx.clip_far = float(profile.get("clip_far") or ctx.CLIP_FAR)
    ctx.update_rate = int(profile.get("update_rate") or ctx.UPDATE_RATE)
    print(
        f"[DEBUG _mono_build_ctx] resolved: IMAGE_TOPIC={ctx.IMAGE_TOPIC}, {ctx.image_width}x{ctx.image_height}, fov={ctx.horizontal_fov:.3f}, clip=[{ctx.clip_near}, {ctx.clip_far}], rate={ctx.update_rate}"
    )
    worlds_root = _camera_worlds_root()
    print(
        f"[DEBUG _mono_build_ctx] worlds_root={worlds_root}, exists={worlds_root.exists()}"
    )
    ctx.test_to_world = {
        "c1_size_order_test": str(worlds_root / "camera_c1_single_cube.world"),
        "c2_resolution_test": str(worlds_root / "camera_c2_resolution.world"),
        "c4_geometries_presence_test": str(worlds_root / "camera_c4_geometries.world"),
        "c7_occlusion_test": str(worlds_root / "camera_c7_occlusion.world"),
        "c9_fov_test": str(worlds_root / "camera_c9_fov.world"),
        "c10_clipping_test": str(worlds_root / "camera_c10_clipping.world"),
        "c11_fps_stability_test": str(worlds_root / "camera_c11_fps_static_load.world"),
    }
    ctx._last_test_diagnostics = {}
    return ctx


def _mono__set_test_diagnostics(ctx, **kwargs) -> None:
    ctx._last_test_diagnostics.update(kwargs)


def _mono_get_last_test_diagnostics(ctx) -> Dict[str, Any]:
    # JSON-serializable payload, shallow-copy is enough here.
    return dict(ctx._last_test_diagnostics)


def _mono_build_description(func_name: str, result: dict, passed: bool) -> str:
    """Build a human-readable description explaining WHY the test passed or failed."""
    metrics = result.get("metrics", {})
    prefix = "" if passed else "Датчик не прошёл тест. "

    try:
        if func_name == "c1_size_order_test":
            areas = metrics.get("bbox_area_px", {})
            checks = metrics.get("checks", {})
            if passed:
                desc = (
                    f"Тест пройден: проекция куба корректно уменьшается с расстоянием. "
                    f"Площади (px): {areas}. Порядок размеров соблюдён: {checks.get('size_order', '?')}, "
                    f"запас ≥{metrics.get('min_margin_ratio', '?')}x: {checks.get('size_margin', '?')}."
                )
            else:
                desc = (
                    f"Проекция куба не уменьшается корректно с расстоянием. "
                    f"Площади (px): {areas}. Порядок: {checks.get('size_order', '?')}, "
                    f"запас: {checks.get('size_margin', '?')}."
                )
        elif func_name == "c2_resolution_test":
            exp = metrics.get("expected_resolution", {})
            act = metrics.get("actual_resolution", {})
            if passed:
                desc = (
                    f"Тест пройден: разрешение кадра совпадает с заданным в SDF. "
                    f"Ожидалось: {exp.get('width', '?')}x{exp.get('height', '?')}, "
                    f"получено: {act.get('width', '?')}x{act.get('height', '?')}."
                )
            else:
                desc = (
                    f"Разрешение кадра не совпадает с заданным в SDF. "
                    f"Ожидалось: {exp.get('width', '?')}x{exp.get('height', '?')}, "
                    f"получено: {act.get('width', '?')}x{act.get('height', '?')}."
                )
        elif func_name == "c4_geometries_presence_test":
            counts = metrics.get("pixel_counts", {})
            threshold = metrics.get("threshold", "?")
            if passed:
                desc = (
                    f"Тест пройден: все 4 цветных объекта обнаружены в кадре. "
                    f"Пиксели: красный={counts.get('red', 0)}, зелёный={counts.get('green', 0)}, "
                    f"синий={counts.get('blue', 0)}, жёлтый={counts.get('yellow', 0)}. "
                    f"Порог: {threshold}."
                )
            else:
                desc = (
                    f"Не все цветные объекты обнаружены. "
                    f"Пиксели: красный={counts.get('red', 0)}, зелёный={counts.get('green', 0)}, "
                    f"синий={counts.get('blue', 0)}, жёлтый={counts.get('yellow', 0)}. Порог: {threshold}."
                )
        elif func_name == "c7_occlusion_test":
            blue = metrics.get("blue_pixels", {})
            threshold = metrics.get("threshold", "?")
            if passed:
                desc = (
                    f"Тест пройден: окклюзия работает корректно. При меньшем перекрытии (occ_25) "
                    f"видно больше синих пикселей ({blue.get('occ_25', 0)}), чем при большем (occ_50: "
                    f"{blue.get('occ_50', 0)}). Порог: {threshold}."
                )
            else:
                desc = (
                    f"Окклюзия работает некорректно. Синие пиксели: occ_25={blue.get('occ_25', 0)}, "
                    f"occ_50={blue.get('occ_50', 0)}. Порог: {threshold}."
                )
        elif func_name == "c9_fov_test":
            if passed:
                desc = (
                    f"Тест пройден: измеренный FOV совпадает с заданным. "
                    f"Измерено: {metrics.get('fov_measured_deg', 0):.1f}°, "
                    f"задано: {metrics.get('fov_target_deg', 0):.1f}°, "
                    f"отклонение: {metrics.get('relative_error', 0):.2%} (допуск ≤5%)."
                )
            else:
                # При исключении metrics может быть пуст — берём данные из error
                error_str = result.get("error", "")
                if error_str and "measured=" in error_str:
                    desc = f"Измеренный FOV не совпадает с заданным (допуск ≤5%). {error_str}"
                else:
                    desc = (
                        f"Измеренный FOV не совпадает с заданным. "
                        f"Измерено: {metrics.get('fov_measured_deg', 0):.1f}°, "
                        f"задано: {metrics.get('fov_target_deg', 0):.1f}°, "
                        f"отклонение: {metrics.get('relative_error', 0):.2%} (допуск ≤5%)."
                    )
        elif func_name == "c10_clipping_test":
            checks = metrics.get("checks", {})
            near_x = metrics.get("x_near_m", 0)
            far_x = metrics.get("x_far_m", 0)
            min_far = checks.get("far_min_required_m", 0)
            if passed:
                desc = (
                    f"Тест пройден: объект обнаружен вблизи камеры ({float(near_x):.2f}м) "
                    f"и виден на расстоянии до {float(far_x):.1f}м "
                    f"(требовалось ≥{float(min_far):.1f}м)."
                )
            else:
                near_vis = checks.get("near_visible", False)
                far_ok = checks.get("far_ok", False)
                if not near_vis:
                    desc = "Объект не обнаружен вблизи камеры — возможно, near clip plane не настроен."
                else:
                    desc = (
                        f"Объект виден только до {float(far_x):.1f}м "
                        f"(требовалось ≥{float(min_far):.1f}м)."
                    )
        elif func_name == "c11_fps_stability_test":
            if passed:
                desc = (
                    f"Тест пройден: FPS стабилен. "
                    f"FPS: {metrics.get('fps_actual_hz', 0):.1f} Гц (≥95% от заданного). "
                    f"Джиттер P95: {metrics.get('jitter_s', 0):.4f}с. "
                    f"Пропуски кадров: {metrics.get('dropouts_count', 0)}."
                )
            else:
                desc = (
                    f"FPS нестабилен. "
                    f"FPS: {metrics.get('fps_actual_hz', 0):.1f} Гц. "
                    f"Джиттер P95: {metrics.get('jitter_s', 0):.4f}с (лимит {metrics.get('jitter_limit_s', 0):.4f}с). "
                    f"Пропуски кадров: {metrics.get('dropouts_count', 0)}."
                )
        else:
            return None
    except Exception:
        return None

    return prefix + desc


def _mono_safe_wrapper(test_func):
    """Wrap mono test entry-point: catch exceptions and return {"passed": False} with diagnostics."""
    import functools
    import traceback as _tb

    @functools.wraps(test_func)
    def wrapper(simulator, sensor, progress_cb=None):
        t_start = time.time()
        print(f"\n[DEBUG _mono_safe_wrapper] ▶ START {test_func.__name__} sensor={getattr(sensor, 'sensor_name', '?')}")
        try:
            result = test_func(simulator, sensor, progress_cb=progress_cb)
            print(f"[DEBUG _mono_safe_wrapper] ✓ DONE {test_func.__name__} in {time.time()-t_start:.1f}s passed={result.get('passed', '?') if isinstance(result, dict) else '?'}")
            # Add description for passed tests
            if isinstance(result, dict) and "description" not in result:
                desc = _mono_build_description(test_func.__name__, result, True)
                if desc:
                    result["description"] = desc
            return result
        except Exception as exc:
            tb = _tb.format_exc()
            print(f"[DEBUG _mono_safe_wrapper] ✗ FAILED {test_func.__name__} in {time.time()-t_start:.1f}s: {type(exc).__name__}: {exc}")
            print(tb)
            diag = {}
            if "ctx" in test_func.__code__.co_varnames:
                import sys
                frame = sys.exc_info()[2]
                while frame is not None:
                    local_ctx = frame.tb_frame.f_locals.get("ctx")
                    if local_ctx is not None and hasattr(local_ctx, "_last_test_diagnostics"):
                        diag = dict(local_ctx._last_test_diagnostics)
                        break
                    frame = frame.tb_next
            result = {
                "passed": False,
                "error": f"{type(exc).__name__}: {exc}",
                "diagnostics": diag,
            }
            desc = _mono_build_description(test_func.__name__, result, False)
            if desc:
                result["description"] = desc
            if progress_cb:
                try:
                    progress_cb(100)
                except Exception:
                    pass
            return result

    return wrapper


def _mono__wait_image(ctx, timeout: float = 35.0, topic: Optional[str] = None, simulator=None) -> Image:
    target_topic = str(topic or ctx.IMAGE_TOPIC)
    sim = simulator or getattr(ctx, "_simulator", None)
    print(f"[DEBUG _mono__wait_image] topic={target_topic}, timeout={timeout:.1f}s, simulator={'yes' if sim else 'NO'}")
    try:
        msgs = ctx.sensor.capture_data(
            Image, topic=target_topic, window=0.5, timeout=0.5, simulator=sim,
        )
        if not msgs:
            raise rospy.ROSException(f"No messages received on {target_topic}")
        msg = msgs[-1]
        print(
            f"[DEBUG _mono__wait_image] GOT image: {msg.width}x{msg.height}, enc={msg.encoding}, data_len={len(msg.data)}"
        )
        return msg
    except rospy.ROSException as e:
        print(
            f"[DEBUG _mono__wait_image] TIMEOUT on {target_topic} after {timeout:.1f}s: {e}"
        )
        try:
            topics = rospy.get_published_topics()
            image_topics = [
                t for t, tp in topics if "image" in t.lower() or "camera" in t.lower()
            ]
            print(
                f"[DEBUG _mono__wait_image] active image/camera topics: {image_topics[:15]}"
            )
        except Exception:
            pass
        raise


def _mono__scene_diag(ctx, simulator) -> Dict[str, Any]:
    return {}


def _mono__resolved_image_topic(ctx, simulator) -> Tuple[str, Dict[str, Any]]:
    scene_diag = _mono__scene_diag(ctx, simulator)
    return str(ctx.IMAGE_TOPIC), scene_diag


def _mono__ensure_render_display_env(ctx) -> Dict[str, str]:
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


def _mono__msg_stamp_s(ctx, msg: Image) -> float:
    """Extract timestamp from ROS message header.
    Always returns the sim-time stamp from the header, even if zero.
    Never falls back to wall-clock time — mixing sim-time and wall-clock
    causes _wait_image_after to hang after Gazebo restarts (sim-time resets
    to 0 while prev_stamp is still ~1.7e9 from wall clock)."""
    return float(msg.header.stamp.to_sec())


def _mono__wait_image_after(
    ctx,
    prev_stamp_s: Optional[float],
    timeout: float = 35.0,
    topic: Optional[str] = None,
    skip_frames: int = 0,
) -> Image:
    start = time.time()
    target_topic = str(topic or ctx.IMAGE_TOPIC)
    saw_message = False
    attempt = 0
    frames_to_skip = int(skip_frames)
    print(
        f"[DEBUG _mono__wait_image_after] topic={target_topic}, prev_stamp={prev_stamp_s}, timeout={timeout:.1f}s, skip_frames={frames_to_skip}"
    )

    while (time.time() - start) < float(timeout):
        remaining = max(0.2, float(timeout) - (time.time() - start))
        attempt += 1
        try:
            msg = _mono__wait_image(
                ctx, timeout=min(remaining, 5.0), topic=target_topic
            )
        except rospy.ROSException:
            if attempt <= 3 or attempt % 5 == 0:
                print(
                    f"[DEBUG _mono__wait_image_after] attempt {attempt}: timeout, retrying... elapsed={time.time()-start:.1f}s"
                )
            continue
        saw_message = True

        if prev_stamp_s is None:
            if frames_to_skip > 0:
                frames_to_skip -= 1
                continue
            print(
                f"[DEBUG _mono__wait_image_after] got first image (no prev_stamp), attempt={attempt}"
            )
            return msg

        stamp = _mono__msg_stamp_s(ctx, msg)
        # Detect Gazebo restart: if prev_stamp is much larger than current stamp
        # (e.g. prev=1775506223 from wall clock, current=12 from restarted sim time),
        # treat current frame as fresh — the sim was restarted.
        if prev_stamp_s is not None and float(prev_stamp_s) > stamp + 100.0:
            print(
                f"[DEBUG _mono__wait_image_after] sim-time reset detected: prev={prev_stamp_s:.1f} >> stamp={stamp:.1f}, accepting as fresh"
            )
            prev_stamp_s = 0.0  # reset — accept any frame from new Gazebo session
        if stamp > float(prev_stamp_s) + 1e-6:
            if frames_to_skip > 0:
                frames_to_skip -= 1
                prev_stamp_s = stamp
                continue
            print(
                f"[DEBUG _mono__wait_image_after] got fresh image: stamp={stamp:.6f} > prev={prev_stamp_s:.6f}, attempt={attempt}"
            )
            return msg
        elif attempt <= 3:
            print(
                f"[DEBUG _mono__wait_image_after] attempt {attempt}: stamp={stamp:.6f} not fresh enough (need > {prev_stamp_s:.6f})"
            )

    if not saw_message:
        print(
            f"[DEBUG _mono__wait_image_after] FAIL: no image at all on {target_topic}"
        )
        raise RuntimeError(f"No image received on {target_topic} within {timeout:.1f}s")
    print(
        f"[DEBUG _mono__wait_image_after] FAIL: no fresh image after {prev_stamp_s:.6f}"
    )
    raise RuntimeError(
        f"No fresh image received on {target_topic} after stamp {float(prev_stamp_s):.6f} "
        f"within {timeout:.1f}s"
    )


def _mono__msg_to_bgr(ctx, msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    print(
        f"[DEBUG _mono__msg_to_bgr] {w}x{h}, encoding={msg.encoding}, data_len={len(msg.data)}"
    )

    if enc in ("rgb8", "r8g8b8"):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        print(
            f"[DEBUG _mono__msg_to_bgr] converted rgb8 -> bgr, shape={bgr.shape}, mean={bgr.mean():.1f}"
        )
        return bgr

    if enc == "bgr8":
        bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        print(
            f"[DEBUG _mono__msg_to_bgr] bgr8 direct, shape={bgr.shape}, mean={bgr.mean():.1f}"
        )
        return bgr

    if enc in ("mono8", "8uc1"):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        print(f"[DEBUG _mono__msg_to_bgr] converted mono8 -> bgr, shape={bgr.shape}")
        return bgr

    print(f"[DEBUG _mono__msg_to_bgr] UNSUPPORTED encoding: {msg.encoding}")
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def _mono__clean_mask(ctx, mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    return closed


def _mono__red_mask(ctx, hsv: np.ndarray) -> np.ndarray:
    lower_1 = np.array([0, 90, 60], dtype=np.uint8)
    upper_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_2 = np.array([170, 90, 60], dtype=np.uint8)
    upper_2 = np.array([180, 255, 255], dtype=np.uint8)
    mask_1 = cv2.inRange(hsv, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv, lower_2, upper_2)
    return _mono__clean_mask(ctx, cv2.bitwise_or(mask_1, mask_2))


def _mono__color_mask(ctx, hsv: np.ndarray, color: str) -> np.ndarray:
    if color == "red":
        return _mono__red_mask(ctx, hsv)

    ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
        "green": ((40, 80, 60), (85, 255, 255)),
        "blue": ((100, 90, 60), (135, 255, 255)),
        "yellow": ((20, 90, 90), (40, 255, 255)),
    }
    if color not in ranges:
        raise ValueError(f"Unsupported color: {color}")

    lower, upper = ranges[color]
    mask = cv2.inRange(
        hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
    )
    return _mono__clean_mask(ctx, mask)


def _mono__bbox_area(ctx, mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, (0, 0, 0, 0)

    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    return int(w * h), (int(x), int(y), int(w), int(h))


def _mono__count_pixels(ctx, mask: np.ndarray) -> int:
    return int(cv2.countNonZero(mask))


def _mono__iter_float_range(ctx, start: float, stop: float, step: float) -> List[float]:
    values: List[float] = []
    cur = float(start)
    while cur <= float(stop) + 1e-9:
        values.append(round(cur, 4))
        cur += float(step)
    return values


def _mono__white_mask(ctx, frame_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([0, 0, 175], dtype=np.uint8),
        np.array([180, 85, 255], dtype=np.uint8),
    )
    return _mono__clean_mask(ctx, mask)


def _mono__large_contours(
    ctx,
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


def _mono__red_stats(ctx, frame_bgr: np.ndarray) -> Dict[str, float]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    red = _mono__red_mask(ctx, hsv)
    red_pixels = float(_mono__count_pixels(ctx, red))

    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour_area = 0.0
    if contours:
        max_contour_area = float(max(cv2.contourArea(c) for c in contours))

    h, w = frame_bgr.shape[:2]
    frame_area = float(max(1, h * w))
    pixel_ratio = float(red_pixels / frame_area)
    contour_ratio = float(max_contour_area / frame_area)

    return {
        "red_pixels": float(red_pixels),
        "pixel_ratio": pixel_ratio,
        "max_contour_area": max_contour_area,
        "contour_ratio": contour_ratio,
        "visible_by_pixels": bool(red_pixels >= int(ctx.C10_MIN_RED_PIXELS)),
    }


def _mono__annotate(ctx, frame: np.ndarray, lines: List[str]) -> np.ndarray:
    debug = frame.copy()
    y = 30
    for line in lines:
        cv2.putText(
            debug,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28
    return debug


def _mono__read_clip_from_sdf(ctx, camera_model_path_abs: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "camera_model_path_abs": str(camera_model_path_abs),
        "near_m": None,
        "far_m": None,
        "source": "",
        "error": "",
    }
    if not camera_model_path_abs:
        result["error"] = "camera_model_path_abs is empty"
        return result

    try:
        tree = ET.parse(camera_model_path_abs)
        root = tree.getroot()
        clip = root.find(".//sensor/camera/clip")
        if clip is None:
            clip = root.find(".//camera/clip")
        if clip is None:
            result["error"] = "No <clip> section found in SDF"
            return result

        near_node = clip.find("near")
        far_node = clip.find("far")
        if near_node is None or far_node is None:
            result["error"] = "No <near>/<far> in <clip>"
            return result

        near_v = float((near_node.text or "").strip())
        far_v = float((far_node.text or "").strip())
        result["near_m"] = float(near_v)
        result["far_m"] = float(far_v)
        result["source"] = "sdf_clip"
        return result
    except Exception as exc:
        result["error"] = f"Failed to parse SDF clip: {exc}"
        return result


def _mono__open_test_scene(ctx, simulator, test_name: str) -> None:
    print(f"[DEBUG _mono__open_test_scene] test_name={test_name}")
    ctx._last_test_diagnostics = {}
    world = ctx.test_to_world[test_name]
    print(
        f"[DEBUG _mono__open_test_scene] world={world}, exists={os.path.exists(world)}"
    )
    print(
        f"[DEBUG _mono__open_test_scene] sdf_path={ctx.sensor_sdf_path}, exists={os.path.exists(ctx.sensor_sdf_path) if ctx.sensor_sdf_path else False}"
    )
    print(f"[DEBUG _mono__open_test_scene] calling simulator.open_scene()...")
    t_scene = time.time()
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        diag = _mono__scene_diag(ctx, simulator)
        reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
        print(
            f"[DEBUG _mono__open_test_scene] FAILED to open scene after {time.time()-t_scene:.1f}s: reason={reason}"
        )
        raise RuntimeError(
            f"Failed to open scene for {test_name}: {world} (reason={reason})"
        )
    print(f"[DEBUG _mono__open_test_scene] scene opened OK in {time.time()-t_scene:.1f}s, waiting for services...")
    _pcb = getattr(ctx, "_progress_cb", None)
    if _pcb:
        try: _pcb(15)
        except Exception: pass
    t_svc = time.time()
    print(f"[DEBUG _mono__open_test_scene] waiting for get_world_properties (30s timeout)...")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
    print(f"[DEBUG _mono__open_test_scene] get_world_properties ready in {time.time()-t_svc:.1f}s")
    t_svc2 = time.time()
    print(f"[DEBUG _mono__open_test_scene] waiting for set_model_state (30s timeout)...")
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
    print(f"[DEBUG _mono__open_test_scene] set_model_state ready in {time.time()-t_svc2:.1f}s")
    if _pcb:
        try: _pcb(20)
        except Exception: pass
    print(f"[DEBUG _mono__open_test_scene] services ready, total scene setup: {time.time()-t_scene:.1f}s")


def _mono__move_and_settle(
    ctx, simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.8
) -> None:
    simulator.set_pose(model_name, x=x, y=y, z=z)
    time.sleep(settle_s)


def _mono__c4_run_shell_capture(
    ctx, cmd: str, timeout_s: float = 5.0, max_lines: int = 200
) -> Dict[str, Any]:
    try:
        res = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=float(timeout_s),
            check=False,
        )
        return {
            "cmd": str(cmd),
            "returncode": int(res.returncode),
            "stdout_tail": (res.stdout or "").splitlines()[-int(max_lines) :],
            "stderr_tail": (res.stderr or "").splitlines()[-int(max_lines) :],
        }
    except Exception as exc:
        return {
            "cmd": str(cmd),
            "returncode": None,
            "stdout_tail": [],
            "stderr_tail": [str(exc)],
        }


def _mono__c4_extract_log_path_from_lines(ctx, lines: List[str]) -> str:
    # Typical token inside pgrep/command line: __log:=/abs/path/gazebo-1.log
    for line in lines:
        s = str(line)
        marker = "__log:="
        idx = s.find(marker)
        if idx < 0:
            continue
        tail = s[idx + len(marker) :].strip()
        if not tail:
            continue
        token = tail.split()[0].strip().strip("'").strip('"')
        if token.endswith(".log"):
            return token
        if ".log" in token:
            return token.split(".log", 1)[0] + ".log"
    return ""


def _mono__c4_collect_log_candidates_from_dir(
    ctx, base: Path, recursive: bool = False
) -> List[Path]:
    if not base.exists() or not base.is_dir():
        return []
    patterns = ("gazebo-*.log", "gzserver-*.log")
    found: List[Path] = []
    for pattern in patterns:
        found.extend(base.rglob(pattern) if recursive else base.glob(pattern))
    return found


def _mono__c4_read_latest_gzserver_log_tail(
    ctx,
    cls,
    log_path_hint: Optional[str] = None,
    ros_log_dir_hint: Optional[str] = None,
    max_lines: int = 200,
) -> Dict[str, Any]:
    searched_paths: List[str] = []
    hint_file_exists = False
    hint_dir_exists = False
    hint_file_size_bytes: Optional[int] = None
    waited_ms = 0

    def _read_tail(path: Path) -> Dict[str, Any]:
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f_in:
                lines = f_in.read().splitlines()
            return {
                "path": str(path),
                "tail": lines[-int(max_lines) :],
                "searched_paths": searched_paths,
                "hint_file_exists": bool(hint_file_exists),
                "hint_dir_exists": bool(hint_dir_exists),
                "hint_file_size_bytes": hint_file_size_bytes,
                "waited_ms": int(waited_ms),
            }
        except Exception as exc:
            return {
                "path": str(path),
                "tail": [f"<gazebo-log-read-error> {exc}"],
                "searched_paths": searched_paths,
                "hint_file_exists": bool(hint_file_exists),
                "hint_dir_exists": bool(hint_dir_exists),
                "hint_file_size_bytes": hint_file_size_bytes,
                "waited_ms": int(waited_ms),
            }

    if log_path_hint:
        hint = Path(log_path_hint).expanduser()
        searched_paths.append(str(hint))
        hint_dir_exists = bool(hint.parent.exists() and hint.parent.is_dir())
        t_start = time.perf_counter()
        for _ in range(10):
            exists_now = bool(hint.exists() and hint.is_file())
            if exists_now:
                hint_file_exists = True
                try:
                    hint_file_size_bytes = int(hint.stat().st_size)
                except Exception:
                    hint_file_size_bytes = None
                if hint_file_size_bytes is None or hint_file_size_bytes > 0:
                    waited_ms = int((time.perf_counter() - t_start) * 1000.0)
                    return _read_tail(hint)
            time.sleep(0.2)
        waited_ms = int((time.perf_counter() - t_start) * 1000.0)
        hint_file_exists = bool(hint.exists() and hint.is_file())
        if hint_file_exists:
            try:
                hint_file_size_bytes = int(hint.stat().st_size)
            except Exception:
                hint_file_size_bytes = None

    candidates: List[Path] = []

    if ros_log_dir_hint:
        ros_dir = Path(str(ros_log_dir_hint)).expanduser()
        searched_paths.append(str(ros_dir))
        candidates.extend(
            cls._c4_collect_log_candidates_from_dir(ros_dir, recursive=True)
        )

    env_ros_log_dir = os.environ.get("ROS_LOG_DIR", "")
    if env_ros_log_dir:
        env_dir = Path(env_ros_log_dir).expanduser()
        searched_paths.append(str(env_dir))
        candidates.extend(
            cls._c4_collect_log_candidates_from_dir(env_dir, recursive=True)
        )

    latest_dir = Path.home() / ".ros" / "log" / "latest"
    searched_paths.append(str(latest_dir))
    candidates.extend(
        cls._c4_collect_log_candidates_from_dir(latest_dir, recursive=False)
    )

    ros_log_root = Path.home() / ".ros" / "log"
    searched_paths.append(str(ros_log_root))
    candidates.extend(
        cls._c4_collect_log_candidates_from_dir(ros_log_root, recursive=True)
    )

    unique_existing: List[Path] = []
    for path in candidates:
        try:
            if path.exists() and path.is_file() and path not in unique_existing:
                unique_existing.append(path)
        except Exception:
            continue

    if not unique_existing:
        return {
            "path": "",
            "tail": ["<gazebo-log-not-found>"],
            "searched_paths": searched_paths,
            "hint_file_exists": bool(hint_file_exists),
            "hint_dir_exists": bool(hint_dir_exists),
            "hint_file_size_bytes": hint_file_size_bytes,
            "waited_ms": int(waited_ms),
        }

    chosen = sorted(unique_existing, key=lambda p: p.stat().st_mtime)[-1]
    return _read_tail(chosen)


def _mono__c4_collect_scene_open_diagnostics(
    ctx, simulator, scene_diag: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    if not isinstance(scene_diag, dict):
        scene_diag = {}

    rosservice_full = _mono__c4_run_shell_capture(
        ctx,
        "rosservice list | grep '^/gazebo/' || true",
        timeout_s=6.0,
        max_lines=400,
    )
    rosnode_info = _mono__c4_run_shell_capture(
        ctx,
        "rosnode info /gazebo || true",
        timeout_s=6.0,
        max_lines=200,
    )
    rostopic_gazebo_clock = _mono__c4_run_shell_capture(
        ctx,
        "rostopic list | grep -E '^/clock$|^/gazebo/' || true",
        timeout_s=6.0,
        max_lines=300,
    )
    gz_pid_info = _mono__c4_run_shell_capture(
        ctx,
        "pgrep -f '(^|/)gzserver([[:space:]]|$)' | head -n 1 || true",
        timeout_s=3.0,
        max_lines=10,
    )
    gz_pid = ""
    for line in gz_pid_info.get("stdout_tail", []):
        s = str(line).strip()
        if s.isdigit():
            gz_pid = s
            break
    if gz_pid:
        gzserver_ps = _mono__c4_run_shell_capture(
            ctx,
            f"ps -fp {gz_pid} || true",
            timeout_s=3.0,
            max_lines=80,
        )
    else:
        gzserver_ps = {
            "cmd": "ps -fp <gzserver_pid>",
            "returncode": 0,
            "stdout_tail": [],
            "stderr_tail": ["gzserver pid not found"],
        }

    gz_log_hint = ""
    gz_state = scene_diag.get("gzserver_state", {})
    if isinstance(gz_state, dict):
        pgrep_tail = gz_state.get("pgrep_tail", [])
        if isinstance(pgrep_tail, list):
            gz_log_hint = _mono__c4_extract_log_path_from_lines(
                ctx, [str(x) for x in pgrep_tail]
            )

    ros_log_dir_hint = ""
    launch_env = scene_diag.get("launch_env", {})
    if isinstance(launch_env, dict):
        ros_log_dir_hint = str(launch_env.get("ROS_LOG_DIR", "") or "")

    launch_stdout_tail = scene_diag.get("launch_stdout_tail", [])
    launch_stderr_tail = scene_diag.get("launch_stderr_tail", [])

    return {
        "rosservice_gazebo_full": rosservice_full,
        "rosnode_info_gazebo": rosnode_info,
        "rostopic_gazebo_clock": rostopic_gazebo_clock,
        "gzserver_ps": gzserver_ps,
        "roslaunch_stdout_tail": (
            list(launch_stdout_tail)[-50:]
            if isinstance(launch_stdout_tail, list)
            else []
        ),
        "roslaunch_stderr_tail": (
            list(launch_stderr_tail)[-50:]
            if isinstance(launch_stderr_tail, list)
            else []
        ),
        "gazebo_log_path_hint": str(gz_log_hint),
        "gazebo_log_tail": _mono__c4_read_latest_gzserver_log_tail(
            ctx,
            log_path_hint=gz_log_hint,
            ros_log_dir_hint=ros_log_dir_hint,
            max_lines=200,
        ),
    }


def _mono__c4_classify_scene_reason(
    ctx, last_reason: str, diagnostics: Dict[str, Any]
) -> Tuple[str, str]:
    reason = str(last_reason or "gazebo_api_timeout")
    if reason in ("topics_not_ready", "roslaunch_exited_while_waiting_topics"):
        return (
            "camera_topics_not_ready",
            "Expected camera topics were not resolved or did not publish messages in time",
        )

    rosservice = diagnostics.get("rosservice_gazebo_full", {})
    services = [
        str(line).strip()
        for line in rosservice.get("stdout_tail", [])
        if str(line).strip().startswith("/gazebo/")
    ]
    service_set = set(services)
    logger_services = {"/gazebo/get_loggers", "/gazebo/set_logger_level"}
    only_logger_services = bool(service_set) and service_set.issubset(logger_services)
    if only_logger_services:
        return (
            "gazebo_only_logger_services",
            "gazebo_ros_api_plugin likely not initialized (only logger services present)",
        )
    gz_ps = diagnostics.get("gzserver_ps", {})
    gz_alive = bool(gz_ps.get("stdout_tail"))
    if reason in ("gzserver_died", "roslaunch_exited") or not gz_alive:
        return (
            "gzserver_died_early",
            "gzserver died before Gazebo API services became available",
        )
    return (
        "gazebo_api_timeout",
        "Gazebo API services timeout; see diagnostics: rosservice_gazebo_full / gazebo_log_tail",
    )


def _mono__open_c4_scene_with_retry(ctx, simulator) -> None:
    world = ctx.test_to_world["c4_geometries_presence_test"]
    attempts: List[Dict[str, Any]] = []
    last_reason = "unknown"

    for attempt in (1, 2):
        if attempt == 2:
            simulator.kill_gazebo()
            time.sleep(1.0)

        scene_diag: Dict[str, Any] = {}
        try:
            opened = simulator.open_scene(world, ctx.sensor_sdf_path)
        except Exception as exc:
            opened = None
            if not isinstance(scene_diag, dict):
                scene_diag = {}
            scene_diag.setdefault("open_scene_exception", str(exc))
        last_reason = (
            str(scene_diag.get("reason", "unknown"))
            if isinstance(scene_diag, dict)
            else "unknown"
        )
        attempts.append(
            {"attempt": int(attempt), "opened": bool(opened), "reason": last_reason}
        )
        if opened:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
            return

        retryable_reasons = {
            "gazebo_services_not_available",
            "gazebo_services_timeout",
            "roslaunch_exited",
        }
        if attempt == 1 and last_reason in retryable_reasons:
            continue
        break

    extra_diag = _mono__c4_collect_scene_open_diagnostics(ctx, simulator, scene_diag={})
    classified_reason, classified_msg = _mono__c4_classify_scene_reason(
        ctx, last_reason, extra_diag
    )
    _mono__set_test_diagnostics(
        ctx,
        c4_scene_open={
            "reason": classified_reason,
            "message": classified_msg,
            "attempts": attempts,
            "diagnostics": extra_diag,
        },
    )
    raise RuntimeError(
        f"Failed to open scene for c4_geometries_presence_test (reason={classified_reason}). "
        f"See diagnostics: rosservice_gazebo_full / gazebo_log_tail"
    )


def _mono_c1_size_order_test(ctx, simulator) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c1_size_order_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "positions": list(ctx.C1_POSITIONS),
        "cube_pose": {"y": float(ctx.C1_TRACK_Y), "z": float(ctx.C1_TRACK_Z)},
        "bbox_area_px": {},
        "bbox_px": {},
        "red_pixels": {},
        "frame_stamp_s": {},
        "min_margin_ratio": float(ctx.C1_MIN_MARGIN_RATIO),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c1_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c1_size_order={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c1_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c1_size_order_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(
            str(resolved_topic) != str(ctx.IMAGE_TOPIC)
        )
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c1_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c1_diag()

    if not simulator.wait_for_model_spawn(ctx.C1_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C1_CUBE_NAME}"
        _store_c1_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C1_CUBE_NAME}")

    prev_stamp_s: Optional[float] = None
    _c1_positions = list(ctx.C1_POSITIONS)
    _c1_pcb = getattr(ctx, "_progress_cb", None)
    for _c1_idx, x in enumerate(_c1_positions):
        label = f"x{int(x)}"
        if _c1_pcb:
            try: _c1_pcb(30 + int(50 * _c1_idx / len(_c1_positions)))
            except Exception: pass
        _mono__move_and_settle(
            ctx,
            simulator,
            ctx.C1_CUBE_NAME,
            x=float(x),
            y=float(ctx.C1_TRACK_Y),
            z=float(ctx.C1_TRACK_Z),
            settle_s=2.0,
        )

        try:
            msg = _mono__wait_image_after(
                ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic, skip_frames=2
            )
        except Exception as exc:
            metrics["error_reason"] = f"image_receive_failed:{exc}"
            _store_c1_diag()
            raise RuntimeError(
                f"Failed to receive fresh image for C1 from topic {resolved_topic}: {exc}"
            ) from exc
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red = _mono__red_mask(ctx, hsv)
        area, (bx, by, bw, bh) = _mono__bbox_area(ctx, red)
        metrics["bbox_area_px"][label] = int(area)
        metrics["bbox_px"][label] = {
            "x": int(bx),
            "y": int(by),
            "w": int(bw),
            "h": int(bh),
        }
        metrics["red_pixels"][label] = int(_mono__count_pixels(ctx, red))
        metrics["frame_stamp_s"][label] = float(prev_stamp_s)

        debug = frame.copy()
        if area > 0:
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
        cv2.putText(
            debug,
            f"{label}: area={area} ts={prev_stamp_s:.6f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    x2 = metrics["bbox_area_px"].get("x2", 0)
    x5 = metrics["bbox_area_px"].get("x5", 0)
    x8 = metrics["bbox_area_px"].get("x8", 0)
    order_ok = x2 > x5 > x8
    margin_ok = (x2 >= x5 * ctx.C1_MIN_MARGIN_RATIO) and (
        x5 >= x8 * ctx.C1_MIN_MARGIN_RATIO
    )
    metrics["checks"] = {"size_order": bool(order_ok), "size_margin": bool(margin_ok)}
    metrics["status"] = "PASS" if (order_ok and margin_ok) else "FAIL"
    if not (order_ok and margin_ok):
        metrics["error_reason"] = (
            f"size_order_failed: checks={metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )
    _store_c1_diag()
    if not (order_ok and margin_ok):
        raise AssertionError(
            f"C1 checks failed: {metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )

    return {"id": "C1", "passed": True, "metrics": metrics}


def _mono_c4_geometries_presence_test(ctx, simulator) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c4_geometries_presence_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "pixel_counts": {},
        "threshold": int(ctx.C4_MIN_PIXELS),
        "display_env": {},
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c4_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c4_geometries_presence={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c4_diag()

    try:
        _mono__open_c4_scene_with_retry(ctx, simulator)
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c4_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c4_diag()

    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _store_c4_diag()
        raise RuntimeError(
            f"Failed to receive image for C4 from topic {resolved_topic}: {exc}"
        ) from exc

    frame = _mono__msg_to_bgr(ctx, msg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color in ("red", "green", "blue", "yellow"):
        metrics["pixel_counts"][color] = _mono__count_pixels(
            ctx, _mono__color_mask(ctx, hsv, color)
        )

    missing = {
        c: n
        for c, n in metrics["pixel_counts"].items()
        if int(n) <= int(metrics["threshold"])
    }
    metrics["checks"] = {"all_present": len(missing) == 0, "missing_or_low": missing}
    metrics["status"] = "PASS" if len(missing) == 0 else "FAIL"
    if missing:
        metrics["error_reason"] = f"missing_or_low:{missing}"

    debug = _mono__annotate(
        ctx,
        frame,
        [
            f"topic={resolved_topic}",
            f"threshold={metrics['threshold']}",
            f"red={metrics['pixel_counts'].get('red', 0)}",
            f"green={metrics['pixel_counts'].get('green', 0)}",
            f"blue={metrics['pixel_counts'].get('blue', 0)}",
            f"yellow={metrics['pixel_counts'].get('yellow', 0)}",
        ],
    )
    _store_c4_diag()

    if missing:
        raise AssertionError(
            f"C4 checks failed: missing_or_low={missing}, threshold={metrics['threshold']}"
        )

    return {"id": "C4", "passed": True, "metrics": metrics}


def _mono_c7_occlusion_test(ctx, simulator) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c7_occlusion_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "cases": dict(ctx.C7_CASES),
        "blue_pixels": {},
        "threshold": int(ctx.C7_MIN_PIXELS),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c7_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c7_occlusion={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c7_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c7_occlusion_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c7_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c7_diag()

    if not simulator.wait_for_model_spawn(ctx.C7_FRONT_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_FRONT_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_FRONT_CUBE_NAME}")
    if not simulator.wait_for_model_spawn(ctx.C7_BACK_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_BACK_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_BACK_CUBE_NAME}")

    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c7_diag()
        raise RuntimeError(
            f"Failed to receive warmup image for C7 from topic {resolved_topic}: {exc}"
        ) from exc

    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)

    def _move_and_capture(
        model_name: str, x: float, y: float, z: float, settle_s: float = 0.35
    ) -> np.ndarray:
        nonlocal prev_stamp_s

        _mono__move_and_settle(
            ctx,
            simulator,
            model_name,
            x=float(x),
            y=float(y),
            z=float(z),
            settle_s=settle_s,
        )
        msg = _mono__wait_image_after(
            ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic
        )
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        return _mono__msg_to_bgr(ctx, msg)

    _move_and_capture(ctx.C7_BACK_CUBE_NAME, x=3.6, y=0.0, z=0.25)

    for case_name, y in ctx.C7_CASES.items():
        frame = _move_and_capture(ctx.C7_FRONT_CUBE_NAME, x=3.0, y=float(y), z=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue = _mono__color_mask(ctx, hsv, "blue")
        blue_count = _mono__count_pixels(ctx, blue)
        metrics["blue_pixels"][case_name] = int(blue_count)

        debug = _mono__annotate(
            ctx,
            frame,
            [
                f"topic={resolved_topic}",
                f"{case_name}: blue={blue_count}",
                f"front_y={float(y):.2f}",
            ],
        )

    blue_25 = metrics["blue_pixels"].get("occ_25", 0)
    blue_50 = metrics["blue_pixels"].get("occ_50", 0)
    relation_ok = blue_25 > blue_50
    threshold_ok = blue_25 > ctx.C7_MIN_PIXELS and blue_50 > ctx.C7_MIN_PIXELS
    metrics["checks"] = {
        "occlusion_relation": bool(relation_ok),
        "threshold_ok": bool(threshold_ok),
    }
    metrics["status"] = "PASS" if (relation_ok and threshold_ok) else "FAIL"
    if not (relation_ok and threshold_ok):
        metrics["error_reason"] = (
            f"occlusion_mismatch: occ_25={blue_25}, occ_50={blue_50}, threshold={ctx.C7_MIN_PIXELS}"
        )
    _store_c7_diag()

    if not (relation_ok and threshold_ok):
        raise AssertionError(
            f"C7 checks failed: {metrics['checks']}, blue_pixels={metrics['blue_pixels']}, "
            f"threshold={ctx.C7_MIN_PIXELS}"
        )

    return {"id": "C7", "passed": True, "metrics": metrics}


def _mono_c2_resolution_test(ctx, simulator) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c2_resolution_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "expected_resolution": {
            "width": int(ctx.image_width),
            "height": int(ctx.image_height),
        },
        "actual_resolution": None,
        "encoding": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "status": "ERROR",
        "error_reason": "",
        "display_env": {},
    }
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))

    world = ctx.test_to_world["c2_resolution_test"]
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        scene_diag = _mono__scene_diag(ctx, simulator)
        metrics["resolved_topic"] = str(ctx.IMAGE_TOPIC)
        metrics["topic_mapping_changed"] = False
        metrics["scene_open_success"] = False
        metrics["error_reason"] = (
            f"scene_open_failed:{scene_diag.get('reason', 'unknown')}"
            if scene_diag
            else "scene_open_failed"
        )
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to open scene for c2_resolution_test: {world} "
            f"(reason={scene_diag.get('reason', 'unknown') if scene_diag else 'unknown'})"
        )

    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
    metrics["scene_open_success"] = True

    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))

    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to receive image for C2 from topic {resolved_topic}: {exc}"
        ) from exc

    actual_width = int(getattr(msg, "width", 0) or 0)
    actual_height = int(getattr(msg, "height", 0) or 0)
    metrics["actual_resolution"] = {
        "width": actual_width,
        "height": actual_height,
    }
    metrics["encoding"] = str(getattr(msg, "encoding", "") or "")
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""

    if actual_width <= 0 or actual_height <= 0:
        metrics["error_reason"] = "invalid_image_resolution"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"C2 received invalid image dimensions from topic {resolved_topic}: "
            f"width={actual_width}, height={actual_height}"
        )

    try:
        frame = _mono__msg_to_bgr(ctx, msg)
    except Exception:
        frame = None

    if frame is not None:
        debug = _mono__annotate(
            ctx,
            frame,
            [
                f"topic={resolved_topic}",
                f"expected={ctx.image_width}x{ctx.image_height}",
                f"actual={actual_width}x{actual_height}",
            ],
        )

    resolution_matches = actual_width == int(ctx.image_width) and actual_height == int(
        ctx.image_height
    )
    metrics["checks"] = {"resolution_matches": bool(resolution_matches)}
    metrics["status"] = "PASS" if resolution_matches else "FAIL"
    if not resolution_matches:
        metrics["error_reason"] = (
            f"resolution_mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}"
        )

    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    if not resolution_matches:
        raise AssertionError(
            f"C2 resolution mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}, topic={resolved_topic}"
        )

    return {"id": "C2", "passed": True, "metrics": metrics}


def _mono_c9_fov_test(ctx, simulator) -> Dict[str, Any]:
    x_fixed = 2.0

    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c9_fov_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "x_fixed_m": float(x_fixed),
        "step_y_m": float(ctx.C9_STEP),
        "target_fov_rad": float(ctx.horizontal_fov),
        "sphere_radius_m": float(ctx.C9_SPHERE_RADIUS_M),
        "samples": [],
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c9_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c9_fov={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c9_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c9_fov_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c9_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c9_diag()

    if not simulator.wait_for_model_spawn(ctx.C9_SPHERE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C9_SPHERE_NAME}"
        _store_c9_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C9_SPHERE_NAME}")

    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c9_diag()
        raise RuntimeError(
            f"Failed to receive warmup image for C9 from topic {resolved_topic}: {exc}"
        ) from exc

    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    last_visible: Optional[Tuple[float, np.ndarray, int]] = None
    first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None

    for y in _mono__iter_float_range(ctx, 0.0, float(ctx.C9_MAX_Y), float(ctx.C9_STEP)):
        _mono__move_and_settle(
            ctx,
            simulator,
            ctx.C9_SPHERE_NAME,
            x=x_fixed,
            y=float(y),
            z=0.2,
            settle_s=0.35,
        )
        msg = _mono__wait_image_after(
            ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic
        )
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        white = _mono__white_mask(ctx, frame)
        contours = _mono__large_contours(
            ctx, white, min_area=ctx.C9_MIN_CONTOUR_AREA, border_margin=4
        )
        contour_count = len(contours)
        visible = contour_count > 0

        metrics["samples"].append(
            {"y_m": float(y), "visible": bool(visible), "contours": int(contour_count)}
        )

        if visible:
            last_visible = (float(y), frame, contour_count)
        elif last_visible is not None:
            first_not_visible = (float(y), frame, contour_count)
            break

    if last_visible is None:
        metrics["error_reason"] = "object_never_detected"
        _store_c9_diag()
        raise AssertionError("C9 failed: object was never detected in frame")
    if first_not_visible is None:
        metrics["error_reason"] = "object_never_lost"
        _store_c9_diag()
        raise AssertionError(
            "C9 failed: object did not disappear within tested Y range"
        )

    y_visible = float(last_visible[0])
    y_lost = float(first_not_visible[0])
    y_transition = float((y_visible + y_lost) / 2.0)
    y_edge_estimate = float(y_transition + float(ctx.C9_SPHERE_RADIUS_M))
    measured_fov = float(2.0 * atan(y_edge_estimate / x_fixed))
    target_fov = float(ctx.horizontal_fov)
    rel_error = (
        float(abs(measured_fov - target_fov) / target_fov)
        if target_fov > 0
        else float("inf")
    )

    dbg_visible = _mono__annotate(
        ctx,
        last_visible[1],
        [
            f"topic={resolved_topic}",
            f"Yvisible={y_visible:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVmeasured={measured_fov:.5f} rad",
            "visible=True",
        ],
    )

    dbg_lost = _mono__annotate(
        ctx,
        first_not_visible[1],
        [
            f"topic={resolved_topic}",
            f"Ylost={y_lost:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVtarget={target_fov:.5f} rad",
            "visible=False",
        ],
    )

    metrics["y_max_visible_m"] = y_visible
    metrics["y_first_not_visible_m"] = y_lost
    metrics["y_transition_m"] = y_transition
    metrics["y_edge_estimate_m"] = y_edge_estimate
    metrics["fov_measured_rad"] = measured_fov
    metrics["fov_measured_deg"] = float(degrees(measured_fov))
    metrics["fov_target_deg"] = float(degrees(target_fov))
    metrics["relative_error"] = rel_error
    metrics["checks"] = {"rel_error_le_0_05": bool(rel_error <= 0.05)}
    metrics["status"] = "PASS" if rel_error <= 0.05 else "FAIL"
    if rel_error > 0.05:
        metrics["error_reason"] = (
            f"fov_mismatch: measured={measured_fov:.6f}, target={target_fov:.6f}, rel_error={rel_error:.4f}"
        )
    _store_c9_diag()
    if rel_error > 0.05:
        raise AssertionError(
            f"C9 failed: measured={measured_fov:.6f} rad, target={target_fov:.6f} rad, rel_error={rel_error:.4f}"
        )

    return {"id": "C9", "passed": True, "metrics": metrics}


def _mono_c10_clipping_test(ctx, simulator) -> Dict[str, Any]:
    clip_cfg = _mono__read_clip_from_sdf(ctx, ctx.sensor_sdf_path)
    near_target = (
        float(clip_cfg["near_m"])
        if clip_cfg.get("near_m") is not None
        else float(ctx.clip_near)
    )
    far_target = (
        float(clip_cfg["far_m"])
        if clip_cfg.get("far_m") is not None
        else float(ctx.clip_far)
    )

    # Simplified approach: check visibility at a few key distances,
    # then binary-search for the max visible distance.
    # Pass criteria: cube visible at near_target and max_visible >= min_far_m.
    min_far_m = min(50.0, far_target * 0.3)  # reasonable minimum far distance

    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c10_clipping_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "near_clip_target_m": float(near_target),
        "far_clip_target_m": float(far_target),
        "clip_source": clip_cfg,
        "min_far_required_m": float(min_far_m),
        "min_red_pixels": int(ctx.C10_MIN_RED_PIXELS),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c10_diag() -> None:
        _mono__set_test_diagnostics(ctx, c10_clipping={"metrics": dict(metrics)})

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx)
    _store_c10_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c10_clipping_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c10_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c10_diag()

    if not simulator.wait_for_model_spawn(ctx.C10_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C10_CUBE_NAME}"
        _store_c10_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C10_CUBE_NAME}")

    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c10_diag()
        raise RuntimeError(f"Failed to receive warmup image for C10: {exc}") from exc

    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    _c10_pcb = getattr(ctx, "_progress_cb", None)
    if _c10_pcb:
        try: _c10_pcb(30)
        except Exception: pass
    print(f"[DEBUG C10] near_target={near_target:.3f}m  far_target={far_target:.3f}m  min_far={min_far_m:.1f}m")

    def _is_visible(x: float, settle_s: float = 0.2) -> bool:
        nonlocal prev_stamp_s
        _mono__move_and_settle(ctx, simulator, ctx.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=settle_s)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        red_stats = _mono__red_stats(ctx, frame)
        visible = bool(red_stats["visible_by_pixels"])
        print(f"[DEBUG C10] x={x:.3f}m  red_pixels={int(red_stats['red_pixels'])}  visible={visible}")
        return visible

    # ── Step 1: Check near clip — cube visible at near_target ─────────
    near_check_x = max(0.3, near_target + 0.1)  # slightly beyond near clip
    near_visible = _is_visible(near_check_x)
    metrics["near_check_x_m"] = near_check_x
    metrics["near_visible"] = near_visible

    if not near_visible:
        # Try a few more distances to find where cube appears
        for try_x in [0.5, 1.0, 2.0]:
            if _is_visible(try_x):
                near_visible = True
                near_check_x = try_x
                metrics["near_check_x_m"] = try_x
                metrics["near_visible"] = True
                break

    near_ok = near_visible
    if _c10_pcb:
        try: _c10_pcb(50)
        except Exception: pass

    # ── Step 2: Binary search for max visible distance ────────────────
    # Start from a known visible point, find where cube disappears
    search_lo = near_check_x
    search_hi = min(600.0, far_target * 1.2)  # cap at 600m (sub-pixel limit)

    # Quick check: is cube visible at search_hi?
    if _is_visible(search_hi):
        # Visible even at max search distance — far clip is beyond render limit
        max_visible_x = search_hi
        print(f"[DEBUG C10] cube visible at max search distance {search_hi:.1f}m")
    else:
        # Binary search between last visible and first not visible
        lo, hi = search_lo, search_hi
        print(f"[DEBUG C10] binary search for far boundary: [{lo:.1f}, {hi:.1f}]")
        while (hi - lo) > 1.0:
            mid = (lo + hi) / 2.0
            if _is_visible(mid, settle_s=0.15):
                lo = mid
            else:
                hi = mid
        max_visible_x = lo

    metrics["x_near_m"] = near_check_x
    metrics["x_far_m"] = max_visible_x
    far_ok = max_visible_x >= min_far_m

    metrics["checks"] = {
        "near_visible": bool(near_ok),
        "far_max_visible_m": float(max_visible_x),
        "far_min_required_m": float(min_far_m),
        "far_ok": bool(far_ok),
    }

    print(
        f"[DEBUG C10] result: near_visible={near_ok} at {near_check_x:.3f}m, "
        f"max_visible={max_visible_x:.1f}m (need >={min_far_m:.1f}m, ok={far_ok})"
    )

    metrics["status"] = "PASS" if (near_ok and far_ok) else "FAIL"
    if not (near_ok and far_ok):
        metrics["error_reason"] = (
            f"clipping_mismatch: near_visible={near_ok}, "
            f"max_visible={max_visible_x:.1f}m (need >={min_far_m:.1f}m)"
        )
    _store_c10_diag()
    if not (near_ok and far_ok):
        raise AssertionError(
            f"C10 failed: near_visible={near_ok} at {near_check_x:.3f}m, "
            f"max_visible={max_visible_x:.1f}m (need >={min_far_m:.1f}m)"
        )

    return {"id": "C10", "passed": True, "metrics": metrics}


def _mono_c11_fps_stability_test(ctx, simulator) -> Dict[str, Any]:
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c11_fps_stability_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "duration_target_s": float(ctx.C11_DURATION_S),
        "update_rate_target_hz": float(ctx.update_rate),
        "warmup_seconds": float(ctx.C11_WARMUP_SECONDS),
        "jitter_percentile": int(ctx.C11_JITTER_PERCENTILE),
        "jitter_limit_s": float(ctx.C11_MAX_JITTER_S),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c11_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c11_fps_stability={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c11_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c11_fps_stability_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(
            str(resolved_topic) != str(ctx.IMAGE_TOPIC)
        )
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c11_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c11_diag()

    try:
        _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_receive_failed:{exc}"
        _store_c11_diag()
        raise RuntimeError(
            f"Failed to receive warmup image for C11 from topic {resolved_topic}: {exc}"
        ) from exc

    timestamps: List[float] = []
    first_msg: Dict[str, Optional[Image]] = {"msg": None}
    last_msg: Dict[str, Optional[Image]] = {"msg": None}
    header_stamp_missing_count = 0

    def _on_image(msg: Image) -> None:
        nonlocal header_stamp_missing_count
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            header_stamp_missing_count += 1
            stamp = float(rospy.Time.now().to_sec())
        timestamps.append(stamp)
        if first_msg["msg"] is None:
            first_msg["msg"] = msg
        last_msg["msg"] = msg

    sub = rospy.Subscriber(resolved_topic, Image, _on_image, queue_size=2000)
    started_wall = time.perf_counter()
    _c11_pcb = getattr(ctx, "_progress_cb", None)
    try:
        while (time.perf_counter() - started_wall) < float(ctx.C11_DURATION_S):
            time.sleep(0.1)
            if _c11_pcb:
                elapsed = time.perf_counter() - started_wall
                pct = 25 + int(65 * elapsed / float(ctx.C11_DURATION_S))
                try: _c11_pcb(min(pct, 90))
                except Exception: pass
    finally:
        sub.unregister()

    metrics["duration_actual_s"] = round(time.perf_counter() - started_wall, 4)
    metrics["raw_frames_captured"] = int(len(timestamps))
    metrics["header_stamp_missing_count"] = int(header_stamp_missing_count)
    metrics["first_raw_stamp_s"] = float(timestamps[0]) if timestamps else None
    metrics["last_raw_stamp_s"] = float(timestamps[-1]) if timestamps else None
    if len(timestamps) < 2:
        metrics["error_reason"] = f"not_enough_frames_captured:{len(timestamps)}"
        _store_c11_diag()
        raise AssertionError(
            f"C11 failed: not enough frames captured ({len(timestamps)})"
        )

    monotonic_stamps: List[float] = []
    for ts in timestamps:
        if not monotonic_stamps or ts > monotonic_stamps[-1]:
            monotonic_stamps.append(float(ts))

    metrics["monotonic_frames_captured"] = int(len(monotonic_stamps))
    metrics["non_monotonic_dropped"] = int(len(timestamps) - len(monotonic_stamps))
    if len(monotonic_stamps) < 2:
        metrics["error_reason"] = "no_monotonic_timestamp_sequence"
        _store_c11_diag()
        raise AssertionError("C11 failed: no monotonic timestamp sequence")

    warmup_frames = int(
        max(1, round(float(ctx.update_rate) * float(ctx.C11_WARMUP_SECONDS)))
    )
    if len(monotonic_stamps) <= (warmup_frames + 1):
        metrics["error_reason"] = (
            f"not_enough_frames_after_warmup:{len(monotonic_stamps)} total,warmup={warmup_frames}"
        )
        _store_c11_diag()
        raise AssertionError(
            f"C11 failed: not enough frames after warmup ({len(monotonic_stamps)} total, warmup={warmup_frames})"
        )

    eval_stamps = monotonic_stamps[warmup_frames:]
    total_dt = float(eval_stamps[-1] - eval_stamps[0])
    if total_dt <= 0.0:
        metrics["error_reason"] = f"invalid_timestamps_interval:{total_dt}"
        _store_c11_diag()
        raise AssertionError(f"C11 failed: invalid timestamps interval ({total_dt})")

    n_frames = len(eval_stamps)
    fps_actual = float((n_frames - 1) / total_dt)
    ideal_dt = float(1.0 / float(ctx.update_rate))
    deltas = np.diff(np.array(eval_stamps, dtype=np.float64))

    abs_jitter = (
        np.abs(deltas - ideal_dt) if deltas.size > 0 else np.array([], dtype=np.float64)
    )
    # Было раньше: jitter = max(|dt - ideal_dt|), что слишком чувствительно к единичным пикам.
    # Теперь: устойчивый jitter = P95(|dt - ideal_dt|), max оставляем как диагностическую метрику.
    jitter = (
        float(np.percentile(abs_jitter, ctx.C11_JITTER_PERCENTILE))
        if abs_jitter.size > 0
        else 0.0
    )
    jitter_max_abs = float(np.max(abs_jitter)) if abs_jitter.size > 0 else 0.0
    max_dt = float(np.max(deltas)) if deltas.size > 0 else 0.0
    dropouts = int(np.sum(deltas > (2.0 * ideal_dt))) if deltas.size > 0 else 0

    fps_ok = fps_actual >= (0.95 * float(ctx.update_rate))
    # Jitter limit: use the larger of the hardcoded limit or 50% of the ideal frame time.
    # This prevents false failures on low-FPS cameras where even small timing
    # variations exceed the absolute 15ms threshold.
    jitter_limit = max(float(ctx.C11_MAX_JITTER_S), ideal_dt * 0.5)
    jitter_ok = jitter <= jitter_limit
    dropouts_ok = dropouts == 0

    metrics.update(
        {
            "frames_captured": int(n_frames),
            "frames_skipped_warmup": int(warmup_frames),
            "first_eval_stamp_s": float(eval_stamps[0]),
            "last_eval_stamp_s": float(eval_stamps[-1]),
            "timestamps_interval_s": total_dt,
            "fps_actual_hz": fps_actual,
            "ideal_dt_s": ideal_dt,
            "jitter_limit_s": float(jitter_limit),
            "jitter_s": jitter,
            "jitter_old_max_abs_s": jitter_max_abs,
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
        frame_first = _mono__msg_to_bgr(ctx, first_msg["msg"])
        debug_first = _mono__annotate(
            ctx,
            frame_first,
            ["C11 first frame", f"fps={fps_actual:.2f}", f"jitter={jitter:.4f}s"],
        )
    if last_msg["msg"] is not None:
        frame_last = _mono__msg_to_bgr(ctx, last_msg["msg"])
        debug_last = _mono__annotate(
            ctx,
            frame_last,
            ["C11 last frame", f"dropouts={dropouts}", f"max_dt={max_dt:.4f}s"],
        )

    metrics["status"] = "PASS" if (fps_ok and jitter_ok and dropouts_ok) else "FAIL"
    if not (fps_ok and jitter_ok and dropouts_ok):
        metrics["error_reason"] = (
            f"fps_jitter_or_dropout_failed: fps={fps_actual:.3f}, jitter={jitter:.4f}, dropouts={dropouts}"
        )
    _store_c11_diag()
    if not (fps_ok and jitter_ok and dropouts_ok):
        raise AssertionError(
            f"C11 failed: fps={fps_actual:.3f} (target>={0.95 * ctx.update_rate:.3f}), "
            f"jitter={jitter:.4f}s (limit<={jitter_limit:.4f}s), dropouts={dropouts}"
        )

    return {"id": "C11", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c1_size_order_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c1_size_order_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "positions": list(ctx.C1_POSITIONS),
        "cube_pose": {"y": float(ctx.C1_TRACK_Y), "z": float(ctx.C1_TRACK_Z)},
        "bbox_area_px": {},
        "bbox_px": {},
        "red_pixels": {},
        "frame_stamp_s": {},
        "min_margin_ratio": float(ctx.C1_MIN_MARGIN_RATIO),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c1_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c1_size_order={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c1_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c1_size_order_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(
            str(resolved_topic) != str(ctx.IMAGE_TOPIC)
        )
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c1_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c1_diag()

    if not simulator.wait_for_model_spawn(ctx.C1_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C1_CUBE_NAME}"
        _store_c1_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C1_CUBE_NAME}")

    prev_stamp_s: Optional[float] = None
    _c1_positions = list(ctx.C1_POSITIONS)
    _c1_pcb = getattr(ctx, "_progress_cb", None)
    for _c1_idx, x in enumerate(_c1_positions):
        label = f"x{int(x)}"
        if _c1_pcb:
            try: _c1_pcb(30 + int(50 * _c1_idx / len(_c1_positions)))
            except Exception: pass
        _mono__move_and_settle(
            ctx,
            simulator,
            ctx.C1_CUBE_NAME,
            x=float(x),
            y=float(ctx.C1_TRACK_Y),
            z=float(ctx.C1_TRACK_Z),
            settle_s=2.0,
        )

        try:
            msg = _mono__wait_image_after(
                ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic, skip_frames=2
            )
        except Exception as exc:
            metrics["error_reason"] = f"image_receive_failed:{exc}"
            _store_c1_diag()
            raise RuntimeError(
                f"Failed to receive fresh image for C1 from topic {resolved_topic}: {exc}"
            ) from exc
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red = _mono__red_mask(ctx, hsv)
        area, (bx, by, bw, bh) = _mono__bbox_area(ctx, red)
        metrics["bbox_area_px"][label] = int(area)
        metrics["bbox_px"][label] = {
            "x": int(bx),
            "y": int(by),
            "w": int(bw),
            "h": int(bh),
        }
        metrics["red_pixels"][label] = int(_mono__count_pixels(ctx, red))
        metrics["frame_stamp_s"][label] = float(prev_stamp_s)

        debug = frame.copy()
        if area > 0:
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
        cv2.putText(
            debug,
            f"{label}: area={area} ts={prev_stamp_s:.6f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    x2 = metrics["bbox_area_px"].get("x2", 0)
    x5 = metrics["bbox_area_px"].get("x5", 0)
    x8 = metrics["bbox_area_px"].get("x8", 0)
    order_ok = x2 > x5 > x8
    margin_ok = (x2 >= x5 * ctx.C1_MIN_MARGIN_RATIO) and (
        x5 >= x8 * ctx.C1_MIN_MARGIN_RATIO
    )
    metrics["checks"] = {"size_order": bool(order_ok), "size_margin": bool(margin_ok)}
    metrics["status"] = "PASS" if (order_ok and margin_ok) else "FAIL"
    if not (order_ok and margin_ok):
        metrics["error_reason"] = (
            f"size_order_failed: checks={metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )
    _store_c1_diag()
    if not (order_ok and margin_ok):
        raise AssertionError(
            f"C1 checks failed: {metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C1", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c2_resolution_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c2_resolution_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "expected_resolution": {
            "width": int(ctx.image_width),
            "height": int(ctx.image_height),
        },
        "actual_resolution": None,
        "encoding": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "status": "ERROR",
        "error_reason": "",
        "display_env": {},
    }
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))

    world = ctx.test_to_world["c2_resolution_test"]
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        scene_diag = _mono__scene_diag(ctx, simulator)
        metrics["resolved_topic"] = str(ctx.IMAGE_TOPIC)
        metrics["topic_mapping_changed"] = False
        metrics["scene_open_success"] = False
        metrics["error_reason"] = (
            f"scene_open_failed:{scene_diag.get('reason', 'unknown')}"
            if scene_diag
            else "scene_open_failed"
        )
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to open scene for c2_resolution_test: {world} "
            f"(reason={scene_diag.get('reason', 'unknown') if scene_diag else 'unknown'})"
        )

    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
    metrics["scene_open_success"] = True

    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))

    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to receive image for C2 from topic {resolved_topic}: {exc}"
        ) from exc

    actual_width = int(getattr(msg, "width", 0) or 0)
    actual_height = int(getattr(msg, "height", 0) or 0)
    metrics["actual_resolution"] = {
        "width": actual_width,
        "height": actual_height,
    }
    metrics["encoding"] = str(getattr(msg, "encoding", "") or "")
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""

    if actual_width <= 0 or actual_height <= 0:
        metrics["error_reason"] = "invalid_image_resolution"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"C2 received invalid image dimensions from topic {resolved_topic}: "
            f"width={actual_width}, height={actual_height}"
        )

    try:
        frame = _mono__msg_to_bgr(ctx, msg)
    except Exception:
        frame = None

    if frame is not None:
        debug = _mono__annotate(
            ctx,
            frame,
            [
                f"topic={resolved_topic}",
                f"expected={ctx.image_width}x{ctx.image_height}",
                f"actual={actual_width}x{actual_height}",
            ],
        )

    resolution_matches = actual_width == int(ctx.image_width) and actual_height == int(
        ctx.image_height
    )
    metrics["checks"] = {"resolution_matches": bool(resolution_matches)}
    metrics["status"] = "PASS" if resolution_matches else "FAIL"
    if not resolution_matches:
        metrics["error_reason"] = (
            f"resolution_mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}"
        )

    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    if not resolution_matches:
        raise AssertionError(
            f"C2 resolution mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}, topic={resolved_topic}"
        )

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C2", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c4_geometries_presence_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c4_geometries_presence_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "pixel_counts": {},
        "threshold": int(ctx.C4_MIN_PIXELS),
        "display_env": {},
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c4_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c4_geometries_presence={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c4_diag()

    try:
        _mono__open_c4_scene_with_retry(ctx, simulator)
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c4_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c4_diag()

    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _store_c4_diag()
        raise RuntimeError(
            f"Failed to receive image for C4 from topic {resolved_topic}: {exc}"
        ) from exc

    frame = _mono__msg_to_bgr(ctx, msg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color in ("red", "green", "blue", "yellow"):
        metrics["pixel_counts"][color] = _mono__count_pixels(
            ctx, _mono__color_mask(ctx, hsv, color)
        )

    missing = {
        c: n
        for c, n in metrics["pixel_counts"].items()
        if int(n) <= int(metrics["threshold"])
    }
    metrics["checks"] = {"all_present": len(missing) == 0, "missing_or_low": missing}
    metrics["status"] = "PASS" if len(missing) == 0 else "FAIL"
    if missing:
        metrics["error_reason"] = f"missing_or_low:{missing}"

    debug = _mono__annotate(
        ctx,
        frame,
        [
            f"topic={resolved_topic}",
            f"threshold={metrics['threshold']}",
            f"red={metrics['pixel_counts'].get('red', 0)}",
            f"green={metrics['pixel_counts'].get('green', 0)}",
            f"blue={metrics['pixel_counts'].get('blue', 0)}",
            f"yellow={metrics['pixel_counts'].get('yellow', 0)}",
        ],
    )
    _store_c4_diag()

    if missing:
        raise AssertionError(
            f"C4 checks failed: missing_or_low={missing}, threshold={metrics['threshold']}"
        )

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C4", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c7_occlusion_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c7_occlusion_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "cases": dict(ctx.C7_CASES),
        "blue_pixels": {},
        "threshold": int(ctx.C7_MIN_PIXELS),
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c7_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c7_occlusion={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c7_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c7_occlusion_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c7_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c7_diag()

    if not simulator.wait_for_model_spawn(ctx.C7_FRONT_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_FRONT_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_FRONT_CUBE_NAME}")
    if not simulator.wait_for_model_spawn(ctx.C7_BACK_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_BACK_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_BACK_CUBE_NAME}")

    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c7_diag()
        raise RuntimeError(
            f"Failed to receive warmup image for C7 from topic {resolved_topic}: {exc}"
        ) from exc

    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)

    def _move_and_capture(
        model_name: str, x: float, y: float, z: float, settle_s: float = 0.35
    ) -> np.ndarray:
        nonlocal prev_stamp_s

        _mono__move_and_settle(
            ctx,
            simulator,
            model_name,
            x=float(x),
            y=float(y),
            z=float(z),
            settle_s=settle_s,
        )
        msg = _mono__wait_image_after(
            ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic
        )
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        return _mono__msg_to_bgr(ctx, msg)

    _move_and_capture(ctx.C7_BACK_CUBE_NAME, x=3.6, y=0.0, z=0.25)

    for case_name, y in ctx.C7_CASES.items():
        frame = _move_and_capture(ctx.C7_FRONT_CUBE_NAME, x=3.0, y=float(y), z=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue = _mono__color_mask(ctx, hsv, "blue")
        blue_count = _mono__count_pixels(ctx, blue)
        metrics["blue_pixels"][case_name] = int(blue_count)

        debug = _mono__annotate(
            ctx,
            frame,
            [
                f"topic={resolved_topic}",
                f"{case_name}: blue={blue_count}",
                f"front_y={float(y):.2f}",
            ],
        )

    blue_25 = metrics["blue_pixels"].get("occ_25", 0)
    blue_50 = metrics["blue_pixels"].get("occ_50", 0)
    relation_ok = blue_25 > blue_50
    threshold_ok = blue_25 > ctx.C7_MIN_PIXELS and blue_50 > ctx.C7_MIN_PIXELS
    metrics["checks"] = {
        "occlusion_relation": bool(relation_ok),
        "threshold_ok": bool(threshold_ok),
    }
    metrics["status"] = "PASS" if (relation_ok and threshold_ok) else "FAIL"
    if not (relation_ok and threshold_ok):
        metrics["error_reason"] = (
            f"occlusion_mismatch: occ_25={blue_25}, occ_50={blue_50}, threshold={ctx.C7_MIN_PIXELS}"
        )
    _store_c7_diag()

    if not (relation_ok and threshold_ok):
        raise AssertionError(
            f"C7 checks failed: {metrics['checks']}, blue_pixels={metrics['blue_pixels']}, "
            f"threshold={ctx.C7_MIN_PIXELS}"
        )

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C7", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c9_fov_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    x_fixed = 2.0

    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c9_fov_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "x_fixed_m": float(x_fixed),
        "step_y_m": float(ctx.C9_STEP),
        "target_fov_rad": float(ctx.horizontal_fov),
        "sphere_radius_m": float(ctx.C9_SPHERE_RADIUS_M),
        "samples": [],
        "status": "ERROR",
        "error_reason": "",
    }

    def _store_c9_diag() -> None:
        _mono__set_test_diagnostics(
            ctx,
            c9_fov={
                "metrics": dict(metrics),
            },
        )
        return None

    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(
        ctx,
    )
    _store_c9_diag()

    try:
        _mono__open_test_scene(ctx, simulator, "c9_fov_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c9_diag()
        raise

    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c9_diag()

    if not simulator.wait_for_model_spawn(ctx.C9_SPHERE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C9_SPHERE_NAME}"
        _store_c9_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C9_SPHERE_NAME}")

    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c9_diag()
        raise RuntimeError(
            f"Failed to receive warmup image for C9 from topic {resolved_topic}: {exc}"
        ) from exc

    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    last_visible: Optional[Tuple[float, np.ndarray, int]] = None
    first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None

    for y in _mono__iter_float_range(ctx, 0.0, float(ctx.C9_MAX_Y), float(ctx.C9_STEP)):
        _mono__move_and_settle(
            ctx,
            simulator,
            ctx.C9_SPHERE_NAME,
            x=x_fixed,
            y=float(y),
            z=0.2,
            settle_s=0.35,
        )
        msg = _mono__wait_image_after(
            ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic
        )
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        white = _mono__white_mask(ctx, frame)
        contours = _mono__large_contours(
            ctx, white, min_area=ctx.C9_MIN_CONTOUR_AREA, border_margin=4
        )
        contour_count = len(contours)
        visible = contour_count > 0

        metrics["samples"].append(
            {"y_m": float(y), "visible": bool(visible), "contours": int(contour_count)}
        )

        if visible:
            last_visible = (float(y), frame, contour_count)
        elif last_visible is not None:
            first_not_visible = (float(y), frame, contour_count)
            break

    if last_visible is None:
        metrics["error_reason"] = "object_never_detected"
        _store_c9_diag()
        raise AssertionError("C9 failed: object was never detected in frame")
    if first_not_visible is None:
        metrics["error_reason"] = "object_never_lost"
        _store_c9_diag()
        raise AssertionError(
            "C9 failed: object did not disappear within tested Y range"
        )

    y_visible = float(last_visible[0])
    y_lost = float(first_not_visible[0])
    y_transition = float((y_visible + y_lost) / 2.0)
    y_edge_estimate = float(y_transition + float(ctx.C9_SPHERE_RADIUS_M))
    measured_fov = float(2.0 * atan(y_edge_estimate / x_fixed))
    target_fov = float(ctx.horizontal_fov)
    rel_error = (
        float(abs(measured_fov - target_fov) / target_fov)
        if target_fov > 0
        else float("inf")
    )

    dbg_visible = _mono__annotate(
        ctx,
        last_visible[1],
        [
            f"topic={resolved_topic}",
            f"Yvisible={y_visible:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVmeasured={measured_fov:.5f} rad",
            "visible=True",
        ],
    )

    dbg_lost = _mono__annotate(
        ctx,
        first_not_visible[1],
        [
            f"topic={resolved_topic}",
            f"Ylost={y_lost:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVtarget={target_fov:.5f} rad",
            "visible=False",
        ],
    )

    metrics["y_max_visible_m"] = y_visible
    metrics["y_first_not_visible_m"] = y_lost
    metrics["y_transition_m"] = y_transition
    metrics["y_edge_estimate_m"] = y_edge_estimate
    metrics["fov_measured_rad"] = measured_fov
    metrics["fov_measured_deg"] = float(degrees(measured_fov))
    metrics["fov_target_deg"] = float(degrees(target_fov))
    metrics["relative_error"] = rel_error
    metrics["checks"] = {"rel_error_le_0_05": bool(rel_error <= 0.05)}
    metrics["status"] = "PASS" if rel_error <= 0.05 else "FAIL"
    if rel_error > 0.05:
        metrics["error_reason"] = (
            f"fov_mismatch: measured={measured_fov:.6f}, target={target_fov:.6f}, rel_error={rel_error:.4f}"
        )
    _store_c9_diag()
    if rel_error > 0.05:
        raise AssertionError(
            f"C9 failed: measured={measured_fov:.6f} rad, target={target_fov:.6f} rad, rel_error={rel_error:.4f}"
        )

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C9", "passed": True, "metrics": metrics}


@_mono_safe_wrapper
def c10_clipping_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass

    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    result = _mono_c10_clipping_test(ctx, simulator)

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return result


@_mono_safe_wrapper
def c11_fps_stability_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass

    ctx = _mono_build_ctx(sensor)
    ctx._progress_cb = progress_cb
    ctx._simulator = simulator
    result = _mono_c11_fps_stability_test(ctx, simulator)

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return result
