"""
Depth camera (dcam) tests — C3, C5, C6, depth_perception.
"""

import copy
import logging
import math
import os
import time
from math import atan2, cos, pi, sin
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

from config import CONFIG
from ._common import (
    _camera_load_sensor_profile,
    _camera_worlds_root,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
#  Константы по умолчанию
# ---------------------------------------------------------------------------

IMAGE_WIDTH  = 1280
IMAGE_HEIGHT = 720
UPDATE_RATE  = 30

HORIZONTAL_FOV_RAD = 1.518
CLIP_NEAR = 0.28
CLIP_FAR  = 10.0

TEST_DISTANCES: Tuple[float, ...] = (1.0, 3.0, 5.0)
MAX_ABS_ERROR_M = 0.5

HALF_SIZE_H = 0.01
HALF_SIZE_V = 0.01

C3_TARGET_CUBE_NAME = "target_cube"
C5_RANGE_CUBE_NAME  = "range_cube"
C6_SHIFT_CUBE_NAME  = "shift_cube"

C3_RADIUS_M             = 2.0
C3_SAMPLES              = 30
C3_MAX_DEV_RATIO        = 0.05
C3_FALLBACK_HALF_ANGLE  = 0.6
C3_TARGET_SIZE_X_M      = 0.5
C3_MEAN_ABS_ERROR_M     = 1.5
C3_MAX_ABS_ERROR_M      = 2.0

C5_START_X           = 0.5
C5_END_X             = 10.0
C5_STEP              = 0.5
C5_DEPTH_TOLERANCE_M = 3.0
C5_CLIP_MARGIN_M     = 0.05
C5_TARGET_SIZE_X_M   = 0.5

C6_START_X              = 2.0
C6_END_X                = 1.0
C6_STEP                 = 0.01
C6_TARGET_SIZE_X_M      = 0.5
C6_DEPTH_CHANGE_EPS_M   = 0.001
C6_MIN_CHANGED_RATIO    = 0.0
C6_MONOTONIC_TOLERANCE  = 0.5
C6_DELTA_TOLERANCE_M    = 0.02
C6_ABS_ERROR_TOLERANCE  = 3.0

DCAM_OCCLUSION_FRONT_CUBE = "front_cube"
DCAM_OCCLUSION_BACK_CUBE = "back_cube"
DCAM_OCCLUSION_CASES = {"occ_large": 0.25, "occ_small": 0.05}
DCAM_OCCLUSION_MIN_PIXELS = 20

DEPTH_ROI_HALF_WINDOW           = 2
DEPTH_TOPIC_WARMUP_TIMEOUT_S    = 20.0
DEPTH_WAIT_PER_CANDIDATE_S      = 1.2
FRAME_FRESH_TIMEOUT_S           = 4.0
CLIP_SATURATION_EPS_M           = 0.02
DEPTH_POST_MOVE_CONFIRMATION    = 2


# ---------------------------------------------------------------------------
#  Построение контекста сенсора
# ---------------------------------------------------------------------------

def _build_ctx(sensor) -> Dict[str, Any]:
    """Собирает словарь ctx со всеми параметрами сенсора и топиками.

    Заменяет __init__ класса _DepthProfileTestContext.
    """
    sensor_name     = str(getattr(sensor, "sensor_name", ""))
    sensor_type     = str(getattr(sensor, "sensor_type", ""))
    sensor_sdf_path = str(getattr(sensor, "sdf_path", ""))

    logger.info(f"Инициализация контекста: sensor_name={sensor_name}, sdf={sensor_sdf_path}")

    profile = _camera_load_sensor_profile(sensor_sdf_path) if sensor_sdf_path else {}
    logger.info(f"Профиль SDF загружен: ключи={list(profile.keys()) if profile else 'пусто'}")

    depth_topic = str(profile.get("depth_topic", "") or "")
    image_topic = str(profile.get("image_topic", "") or getattr(sensor, "topic", "") or "")
    logger.info(f"Топики: depth={depth_topic!r}, image={image_topic!r}")

    image_width  = int(profile.get("image_width")  or IMAGE_WIDTH)
    image_height = int(profile.get("image_height") or IMAGE_HEIGHT)
    horizontal_fov = float(profile.get("horizontal_fov") or HORIZONTAL_FOV_RAD)
    clip_near    = float(profile.get("clip_near") or CLIP_NEAR)
    clip_far     = float(profile.get("clip_far")  or CLIP_FAR)
    update_rate  = int(profile.get("update_rate") or UPDATE_RATE)
    logger.info(f"Параметры камеры: {image_width}x{image_height}, "
                f"clip=[{clip_near}, {clip_far}], fov={horizontal_fov:.3f} рад, "
                f"rate={update_rate} Гц")

    worlds_root = _camera_worlds_root()
    test_to_world = {
        "dcam_occlusion_test":                 str(worlds_root / "camera_depth_occlusion.world"),
        "depth_perception_test":               str(worlds_root / "camera_depth_perception.world"),
        "c3_view_angle_stability_test":        str(worlds_root / "camera_c3_view_angle.world"),
        "c5_working_range_test":               str(worlds_root / "camera_c5_working_range.world"),
        "c6_small_displacement_sensitivity_test": str(worlds_root / "camera_c6_small_shifts.world"),
        "dcam_fov_test":                        str(worlds_root / "camera_depth_fov.world"),
    }
    for test_name, wpath in test_to_world.items():
        exists = os.path.exists(wpath)
        logger.info(f"Мир {test_name}: {wpath} ({'найден' if exists else 'НЕ НАЙДЕН'})")

    camera_model_name = _read_camera_model_name(sensor_sdf_path, sensor_name)
    logger.info(f"Имя модели камеры: {camera_model_name}")

    return {
        "sensor":               sensor,
        "sensor_name":          sensor_name,
        "sensor_type":          sensor_type,
        "sensor_sdf_path":      sensor_sdf_path,
        "DEPTH_TOPIC":          depth_topic,
        "IMAGE_TOPIC":          image_topic,
        "image_width":          image_width,
        "image_height":         image_height,
        "horizontal_fov":       horizontal_fov,
        "clip_near":            clip_near,
        "clip_far":             clip_far,
        "update_rate":          update_rate,
        "test_to_world":        test_to_world,
        "camera_model_name":    camera_model_name,
        "resolved_depth_topic": "",
        "resolved_image_topic": "",
        "last_test_diagnostics": {},
        "_simulator":           None,
        "_progress_cb":         None,
    }


# ---------------------------------------------------------------------------
#  Вспомогательные функции — работа с моделями
# ---------------------------------------------------------------------------

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


def _read_camera_model_name(sdf_path: str, sensor_name: str) -> str:
    model_name = _read_model_name_from_sdf(sdf_path)
    if model_name:
        return model_name
    return f"{sensor_name}_model"


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, sin(float(yaw) * 0.5), cos(float(yaw) * 0.5))


def _move_and_settle(simulator, model_name: str, x: float, y: float,
                     z: float, settle_s: float = 0.35) -> None:
    """Перемещает модель в (x, y, z) и ждёт settle_s секунд."""
    logger.info(f"Перемещение {model_name!r} → x={x:.4f}, y={y:.4f}, z={z:.4f} "
                f"(ожидание {settle_s:.2f}с)")
    simulator.set_pose(model_name, x=float(x), y=float(y), z=float(z))
    time.sleep(settle_s)


def _set_model_pose_6d(model_name: str, x: float, y: float, z: float,
                       yaw: float, settle_s: float = 0.35) -> None:
    """Устанавливает позу модели с рысканьем (yaw) через /gazebo/set_model_state."""
    logger.info(f"Установка позы 6D {model_name!r}: x={x:.4f}, y={y:.4f}, "
                f"z={z:.4f}, yaw={yaw:.4f} рад")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    state = ModelState()
    state.model_name     = model_name
    state.reference_frame = "world"
    state.pose = Pose(Point(float(x), float(y), float(z)), _yaw_to_quaternion(yaw))
    response = set_state(state)
    if not response.success:
        raise RuntimeError(
            f"Ошибка установки позы {model_name}: {response.status_message}"
        )
    time.sleep(settle_s)


def _ensure_render_display_env() -> Dict[str, str]:
    """Устанавливает переменные окружения DISPLAY/XAUTHORITY если не заданы."""
    applied: Dict[str, str] = {}
    if os.environ.get("DISPLAY"):
        return applied
    xauthority = os.environ.get("XAUTHORITY", "").strip()
    if not xauthority:
        default_xa = os.path.expanduser("~/.Xauthority")
        if os.path.exists(default_xa):
            os.environ["XAUTHORITY"] = default_xa
            applied["XAUTHORITY"] = default_xa
    for display in (":0",):
        socket_path = f"/tmp/.X11-unix/X{display.lstrip(':')}"
        if os.path.exists(socket_path):
            os.environ["DISPLAY"] = display
            applied["DISPLAY"] = display
            break
    return applied


# ---------------------------------------------------------------------------
#  Открытие сцены
# ---------------------------------------------------------------------------

def _open_test_scene(ctx: Dict[str, Any], simulator, test_name: str) -> None:
    """Открывает Gazebo-сцену для указанного теста и разрешает топики."""
    logger.info(f"Открытие сцены для теста: {test_name}")
    ctx["_simulator"]            = simulator
    ctx["last_test_diagnostics"] = {}
    ctx["resolved_depth_topic"]  = ""
    ctx["resolved_image_topic"]  = ""

    world = ctx["test_to_world"][test_name]
    sdf   = ctx["sensor_sdf_path"]
    logger.info(f"Файл мира: {world} ({'существует' if os.path.exists(world) else 'НЕ НАЙДЕН'})")
    logger.info(f"SDF сенсора: {sdf} ({'существует' if os.path.exists(sdf) else 'НЕ НАЙДЕН'})")

    display_env = _ensure_render_display_env()
    if display_env:
        logger.info(f"Переменные окружения дисплея: {display_env}")
        ctx["last_test_diagnostics"]["render_display_env"] = display_env

    logger.info("Вызов simulator.open_scene()...")
    if not simulator.open_scene(world, sdf):
        raise RuntimeError(f"Не удалось открыть сцену для {test_name}: {world}")
    logger.info("Сцена успешно открыта, разрешение топиков...")

    pcb = ctx.get("_progress_cb")
    if pcb:
        try: pcb(15)
        except Exception: pass

    _update_resolved_topics(ctx, simulator)
    logger.info(f"Топик глубины: {ctx['resolved_depth_topic']!r}, "
                f"топик изображения: {ctx['resolved_image_topic']!r}")

    ctx["last_test_diagnostics"].update({
        "scene_open_success":    True,
        "world_file":            str(world),
        "expected_depth_topic":  str(ctx["DEPTH_TOPIC"]),
        "expected_image_topic":  str(ctx["IMAGE_TOPIC"] or ""),
        "resolved_depth_topic":  str(ctx["resolved_depth_topic"] or ctx["DEPTH_TOPIC"]),
        "resolved_image_topic":  str(ctx["resolved_image_topic"] or ctx["IMAGE_TOPIC"] or ""),
    })

    logger.info("Ожидание Gazebo-сервисов...")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
    rospy.wait_for_service("/gazebo/set_model_state",      timeout=30.0)
    logger.info("Gazebo-сервисы готовы")

    if pcb:
        try: pcb(20)
        except Exception: pass


# ---------------------------------------------------------------------------
#  Разрешение топиков
# ---------------------------------------------------------------------------

def _list_image_topics() -> List[str]:
    try:
        published = rospy.get_published_topics()
    except Exception:
        return []
    return sorted([name for name, mt in published if mt == "sensor_msgs/Image"])


def _choose_depth_topic(candidates: List[str]) -> str:
    if not candidates:
        return ""
    return sorted(candidates,
                  key=lambda t: (0 if t.endswith("/depth/image_raw") else 1, len(t), t))[0]


def _resolve_depth_topic(ctx: Dict[str, Any],
                         warmup_timeout: float) -> Tuple[str, Dict[str, Any]]:
    """Ищет активный топик глубины среди известных кандидатов."""
    expected    = str(ctx["DEPTH_TOPIC"] or "")
    sensor_name = ctx["sensor_name"]
    preferred = [
        (expected,                                           "expected"),
        (f"/{sensor_name}/depth/image_raw",                  "name_namespace"),
        (f"/{sensor_name}_depth/image_raw",                  "name_underscore"),
        (f"/{sensor_name}_depth/depth/image_raw",            "name_underscore_nested"),
        (f"/{sensor_name}/depth_image_raw",                  "name_alt"),
    ]
    logger.info(f"Поиск топика глубины (таймаут={warmup_timeout:.1f}с), "
                f"ожидаемый: {expected!r}")

    deadline   = time.time() + float(warmup_timeout)
    last_topics: List[str] = []
    while time.time() < deadline:
        topics      = _list_image_topics()
        last_topics = topics
        for topic, source in preferred:
            if topic and topic in topics:
                logger.info(f"Топик глубины найден: {topic!r} (источник: {source})")
                diag = {
                    "expected_depth_topic": expected,
                    "selected_depth_topic": topic,
                    "selected_source":      source,
                    "topics_found":         topics,
                    "topic_mapping_changed": bool(topic != expected),
                }
                return topic, diag
        time.sleep(0.2)

    if expected and expected in last_topics:
        logger.info(f"Топик глубины (после ожидания): {expected!r}")
        return expected, {
            "expected_depth_topic": expected,
            "selected_depth_topic": expected,
            "selected_source":      "expected_after_warmup",
            "topics_found":         last_topics,
            "topic_mapping_changed": False,
        }

    generic     = [t for t in last_topics
                   if "depth" in t and ("image_raw" in t or t.endswith("/image"))]
    in_namespace = [t for t in generic if sensor_name in t]
    selected    = (_choose_depth_topic(in_namespace)
                   or _choose_depth_topic(generic)
                   or expected)
    logger.warning(f"Топик глубины не найден напрямую, эвристика: {selected!r}. "
                   f"Доступные топики изображений: {last_topics[:10]}")
    return selected, {
        "expected_depth_topic": expected,
        "selected_depth_topic": selected,
        "selected_source":      "heuristic_fallback",
        "topics_found":         last_topics,
        "topic_mapping_changed": bool(selected != expected),
    }


def _update_resolved_topics(ctx: Dict[str, Any], simulator) -> None:
    ctx["resolved_depth_topic"] = str(ctx["DEPTH_TOPIC"])
    ctx["resolved_image_topic"] = str(ctx["IMAGE_TOPIC"] or "")
    try:
        topics     = rospy.get_published_topics()
        img_topics = [t for t, _ in topics
                      if "image" in t.lower() or "depth" in t.lower() or "camera" in t.lower()]
        logger.info(f"Активные топики изображений/глубины: {img_topics[:20]}")
    except Exception:
        pass


def _resolved_color_topic(ctx: Dict[str, Any]) -> str:
    return str(ctx["resolved_image_topic"] or ctx["IMAGE_TOPIC"] or "").strip()


# ---------------------------------------------------------------------------
#  Получение сообщений с камеры
# ---------------------------------------------------------------------------

def _msg_stamp_s(msg: Image) -> float:
    stamp = float(msg.header.stamp.to_sec())
    return stamp if stamp > 0.0 else float(time.time())


def _wait_message_after(ctx: Dict[str, Any], prev_stamp_s: Optional[float],
                        topic: str, timeout: float, stage: str) -> Image:
    """Ожидает свежий кадр на топике после prev_stamp_s."""
    target_topic = str(topic or "").strip()
    if not target_topic:
        raise RuntimeError(f"Топик не задан для этапа={stage}")

    logger.info(f"Ожидание кадра: топик={target_topic!r}, этап={stage}, "
                f"prev_stamp={prev_stamp_s}, таймаут={timeout:.1f}с")
    sensor    = ctx["sensor"]
    simulator = ctx.get("_simulator")
    start     = time.time()
    saw_message = False
    attempt = 0

    while (time.time() - start) < float(timeout):
        remaining = max(0.2, float(timeout) - (time.time() - start))
        attempt  += 1
        try:
            msgs = sensor.capture_data(
                Image, topic=target_topic,
                window=min(1.0, remaining), timeout=0.25,
                simulator=simulator,
            )
            if not msgs:
                raise rospy.ROSException(f"Нет сообщений на {target_topic}")
            msg = msgs[-1]
        except rospy.ROSException:
            if attempt <= 3:
                logger.debug(f"Попытка {attempt}: таймаут на {target_topic!r}")
            continue

        saw_message = True
        stamp_s     = _msg_stamp_s(msg)
        if prev_stamp_s is None or stamp_s > (float(prev_stamp_s) + 1e-6):
            logger.info(f"Получен свежий кадр: {msg.width}x{msg.height}, "
                        f"enc={msg.encoding}, stamp={stamp_s:.6f}, этап={stage}")
            return msg
        elif attempt <= 3:
            logger.debug(f"Попытка {attempt}: stamp={stamp_s:.6f} не свежий "
                         f"(нужно > {prev_stamp_s:.6f})")

    if saw_message:
        raise RuntimeError(
            f"Свежий кадр не получен: топик={target_topic!r}, этап={stage}, "
            f"prev_stamp_s={prev_stamp_s}"
        )
    try:
        topics     = rospy.get_published_topics()
        img_topics = [t for t, _ in topics
                      if "image" in t.lower() or "depth" in t.lower()]
        logger.error(f"Кадр не получен вообще. Доступные топики: {img_topics[:15]}")
    except Exception:
        pass
    raise RuntimeError(f"Кадр не получен на топике={target_topic!r}, этап={stage}")


def _ensure_depth_topic(ctx: Dict[str, Any]) -> None:
    """Разрешает топик глубины если ещё не сделано."""
    if not ctx["resolved_depth_topic"]:
        selected, diag = _resolve_depth_topic(ctx, DEPTH_TOPIC_WARMUP_TIMEOUT_S)
        ctx["resolved_depth_topic"] = selected or ctx["DEPTH_TOPIC"]
        ctx["last_test_diagnostics"]["depth_topic_resolution"] = diag
        logger.info(f"Топик глубины установлен: {ctx['resolved_depth_topic']!r}")


def _wait_depth(ctx: Dict[str, Any], timeout: float = 3.0) -> Image:
    """Получает любой кадр глубины (без проверки свежести)."""
    if not ctx["DEPTH_TOPIC"]:
        raise RuntimeError("DEPTH_TOPIC не настроен для этого сенсора")
    _ensure_depth_topic(ctx)

    candidates = [ctx["resolved_depth_topic"]]
    if ctx["resolved_depth_topic"] != ctx["DEPTH_TOPIC"]:
        candidates.append(ctx["DEPTH_TOPIC"])

    errors: List[str] = []
    sensor    = ctx["sensor"]
    simulator = ctx.get("_simulator")
    for topic in candidates:
        try:
            wait_t = min(float(timeout), DEPTH_WAIT_PER_CANDIDATE_S)
            msgs   = sensor.capture_data(
                Image, topic=topic, window=wait_t, timeout=0.25,
                simulator=simulator,
            )
            if not msgs:
                raise RuntimeError(f"Нет сообщений на {topic}")
            logger.info(f"Получен кадр глубины с топика {topic!r}")
            return msgs[-1]
        except Exception as exc:
            errors.append(f"{topic}: {exc}")
    raise RuntimeError(
        f"Не удалось получить кадр глубины. Кандидаты={candidates}, ошибки={errors}"
    )


def _wait_depth_after(ctx: Dict[str, Any], prev_stamp_s: Optional[float],
                      timeout: Optional[float] = None, stage: str = "") -> Image:
    """Ждёт свежий кадр глубины после prev_stamp_s."""
    if not ctx["DEPTH_TOPIC"]:
        raise RuntimeError("DEPTH_TOPIC не настроен")
    _ensure_depth_topic(ctx)
    return _wait_message_after(
        ctx,
        prev_stamp_s=prev_stamp_s,
        topic=ctx["resolved_depth_topic"],
        timeout=float(timeout or FRAME_FRESH_TIMEOUT_S),
        stage=str(stage or "depth_wait_after"),
    )


def _wait_color_after(ctx: Dict[str, Any], prev_stamp_s: Optional[float],
                      timeout: Optional[float] = None,
                      stage: str = "") -> Optional[Image]:
    """Ждёт свежий кадр цвета. Возвращает None если топик не задан."""
    target_topic = _resolved_color_topic(ctx)
    if not target_topic:
        return None
    return _wait_message_after(
        ctx,
        prev_stamp_s=prev_stamp_s,
        topic=target_topic,
        timeout=float(timeout or FRAME_FRESH_TIMEOUT_S),
        stage=str(stage or "color_wait_after"),
    )


def _wait_confirmed_depth_after(ctx: Dict[str, Any], prev_stamp_s: Optional[float],
                                fresh_frames: Optional[int] = None,
                                stage: str = "") -> Tuple[Image, List[float]]:
    """Получает N свежих подтверждающих кадров глубины подряд."""
    count  = max(1, int(fresh_frames or DEPTH_POST_MOVE_CONFIRMATION))
    stamps: List[float] = []
    latest_prev = prev_stamp_s
    msg: Optional[Image] = None
    logger.info(f"Подтверждение кадра глубины: нужно {count} кадров, этап={stage}")
    for idx in range(count):
        msg = _wait_depth_after(
            ctx,
            prev_stamp_s=latest_prev,
            timeout=FRAME_FRESH_TIMEOUT_S,
            stage=f"{stage}_confirm_{idx + 1}",
        )
        latest_prev = _msg_stamp_s(msg)
        stamps.append(float(latest_prev))
    assert msg is not None
    return msg, stamps


def _try_wait_color(ctx: Dict[str, Any], timeout: float = 1.0) -> Optional[Image]:
    """Пытается получить цветной кадр, не бросает исключение."""
    target_topic = _resolved_color_topic(ctx)
    if not target_topic:
        return None
    try:
        sensor    = ctx["sensor"]
        simulator = ctx.get("_simulator")
        msgs = sensor.capture_data(
            Image, topic=target_topic, window=timeout, timeout=0.25,
            simulator=simulator,
        )
        return msgs[-1] if msgs else None
    except Exception:
        return None


# ---------------------------------------------------------------------------
#  Декодирование кадров
# ---------------------------------------------------------------------------

def _depth_msg_to_meters(msg: Image) -> np.ndarray:
    """Декодирует ROS Image глубины в массив float32 в метрах."""
    h, w   = int(msg.height), int(msg.width)
    enc    = (msg.encoding or "").lower()

    if enc == "32fc1":
        dtype, bytes_per_px = np.float32, 4
    elif enc == "16uc1":
        dtype, bytes_per_px = np.uint16, 2
    else:
        raise ValueError(f"Неподдерживаемая кодировка глубины: {msg.encoding}")

    row_stride = int(msg.step) if int(msg.step) > 0 else int(w * bytes_per_px)
    min_row    = int(w * bytes_per_px)
    if row_stride < min_row:
        raise ValueError(f"Неверный step для enc={msg.encoding}: "
                         f"step={row_stride}, минимум={min_row}")

    cols_with_stride = row_stride // bytes_per_px
    raw = np.frombuffer(msg.data, dtype=dtype)
    expected_size = int(h * cols_with_stride)
    if raw.size < expected_size:
        raise ValueError(f"Буфер глубины слишком мал: получено={raw.size}, "
                         f"ожидается={expected_size}")

    arr = raw[:expected_size].reshape(h, cols_with_stride)[:, :w]
    if enc == "16uc1":
        return arr.astype(np.uint16, copy=False).astype(np.float32) / 1000.0
    return arr.astype(np.float32, copy=False)


def _color_msg_to_bgr(msg: Image) -> np.ndarray:
    """Декодирует ROS Image цвета в BGR numpy-массив."""
    h, w = int(msg.height), int(msg.width)
    enc  = (msg.encoding or "").lower()
    if enc in ("rgb8", "r8g8b8"):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    if enc in ("mono8", "8uc1"):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Неподдерживаемая кодировка изображения: {msg.encoding}")


# ---------------------------------------------------------------------------
#  Обнаружение цветных объектов
# ---------------------------------------------------------------------------

def _clean_mask(mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel),
        cv2.MORPH_CLOSE, kernel,
    )


def _color_mask(bgr: np.ndarray, color: str) -> np.ndarray:
    """Возвращает бинарную маску для заданного цвета в HSV."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ranges = {
        "blue":   ((95,  60, 40), (140, 255, 255)),
        "green":  ((35,  60, 40), (90,  255, 255)),
        "yellow": ((15,  80, 80), (45,  255, 255)),
    }
    if color not in ranges:
        raise ValueError(f"Неподдерживаемый цвет: {color}")
    lower, upper = ranges[color]
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    return _clean_mask(mask)


def _find_centroid(bgr: np.ndarray, color: str,
                   min_area: float = 120.0) -> Optional[Tuple[int, int]]:
    """Находит центроид наибольшего контура заданного цвета."""
    mask      = _color_mask(bgr, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < float(min_area):
        return None
    m = cv2.moments(contour)
    if m["m00"] <= 1e-6:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])


# ---------------------------------------------------------------------------
#  Измерение глубины
# ---------------------------------------------------------------------------

def _depth_at_pixel_with_meta(depth_m: np.ndarray, x: int, y: int,
                               half_window: int = 2) -> Tuple[Optional[float], Dict[str, Any]]:
    """Среднее значение глубины в окне half_window вокруг пикселя (x, y)."""
    h, w   = depth_m.shape[:2]
    x0, y0 = max(0, int(x) - half_window), max(0, int(y) - half_window)
    x1, y1 = min(w, int(x) + half_window + 1), min(h, int(y) + half_window + 1)
    roi    = depth_m[y0:y1, x0:x1]
    valid  = roi[np.isfinite(roi) & (roi > 0.0)]
    meta   = {
        "pixel":       {"x": int(x), "y": int(y)},
        "roi_xyxy":    [int(x0), int(y0), int(x1), int(y1)],
        "roi_shape":   [int(max(0, y1 - y0)), int(max(0, x1 - x0))],
        "valid_count": int(valid.size),
        "aggregator":  "mean",
    }
    if valid.size == 0:
        return None, meta
    return float(np.mean(valid)), meta


def _fallback_point_from_finite_depth(
        depth_m: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Dict[str, Any]]:
    """Запасной поиск точки с минимальной конечной глубиной в центральной зоне."""
    h, w   = depth_m.shape[:2]
    y0, y1 = int(h * 0.2), int(h * 0.8)
    x0, x1 = int(w * 0.2), int(w * 0.8)
    central       = depth_m[y0:y1, x0:x1]
    central_valid = np.isfinite(central) & (central > 0.0)
    meta = {
        "strategy":              "finite_depth_min",
        "central_window_xyxy":   [int(x0), int(y0), int(x1), int(y1)],
        "central_valid_count":   int(np.count_nonzero(central_valid)),
        "fallback_scope":        "central",
    }
    if np.count_nonzero(central_valid) > 0:
        cd  = np.where(central_valid, central, np.inf)
        idx = np.unravel_index(int(np.argmin(cd)), cd.shape)
        py, px = int(y0 + idx[0]), int(x0 + idx[1])
        meta["chosen_depth_m"] = float(depth_m[py, px])
        return (px, py), meta

    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    meta["fallback_scope"]      = "global"
    meta["global_valid_count"]  = int(np.count_nonzero(valid))
    if not np.count_nonzero(valid):
        return None, meta
    masked = np.where(valid, depth_m, np.inf)
    idx    = np.unravel_index(int(np.argmin(masked)), masked.shape)
    py, px = int(idx[0]), int(idx[1])
    meta["chosen_depth_m"] = float(depth_m[py, px])
    return (px, py), meta


def _measure_depth_with_meta(
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
) -> Tuple[Optional[float], Tuple[int, int], Dict[str, Any]]:
    """Измеряет глубину в кадре. Приоритет: цветной центроид → запасная точка → центр."""
    h, w = depth_m.shape[:2]
    if bgr is not None and color_hint:
        centroid = _find_centroid(bgr, color_hint)
        if centroid is not None:
            z, roi_meta = _depth_at_pixel_with_meta(
                depth_m, centroid[0], centroid[1], half_window=DEPTH_ROI_HALF_WINDOW
            )
            if z is not None:
                roi_meta["source"]      = "color_centroid"
                roi_meta["color_hint"]  = str(color_hint)
                logger.debug(f"Глубина по центроиду {color_hint}: пиксель={centroid}, z={z:.4f}м")
                return z, centroid, roi_meta

    fallback_point, fallback_diag = _fallback_point_from_finite_depth(depth_m)
    if fallback_point is not None:
        z_fb, roi_meta = _depth_at_pixel_with_meta(
            depth_m, fallback_point[0], fallback_point[1], half_window=DEPTH_ROI_HALF_WINDOW
        )
        if z_fb is not None:
            roi_meta["source"]       = "finite_depth_fallback"
            roi_meta["color_hint"]   = str(color_hint) if color_hint else None
            roi_meta["fallback_meta"] = fallback_diag
            logger.debug(f"Глубина по запасной точке: пиксель={fallback_point}, z={z_fb:.4f}м")
            return z_fb, fallback_point, roi_meta

    cx, cy = w // 2, h // 2
    z_c, roi_meta = _depth_at_pixel_with_meta(
        depth_m, cx, cy, half_window=DEPTH_ROI_HALF_WINDOW
    )
    roi_meta["source"]     = "frame_center"
    roi_meta["color_hint"] = str(color_hint) if color_hint else None
    logger.debug(f"Глубина по центру кадра: пиксель=({cx},{cy}), z={z_c}")
    return z_c, (cx, cy), roi_meta


def _measure_depth(depth_m: np.ndarray, bgr: Optional[np.ndarray] = None,
                   color_hint: Optional[str] = None) -> Tuple[Optional[float], Tuple[int, int]]:
    z, point, _ = _measure_depth_with_meta(depth_m, bgr=bgr, color_hint=color_hint)
    return z, point


def _depth_frame_stats(depth_msg: Image, depth_m: np.ndarray) -> Dict[str, Any]:
    """Статистика кадра глубины для диагностики."""
    finite   = depth_m[np.isfinite(depth_m)]
    positive = finite[finite > 0.0] if finite.size > 0 else np.array([], np.float32)
    stats: Dict[str, Any] = {
        "encoding":       str(depth_msg.encoding),
        "dtype":          str(depth_m.dtype),
        "shape":          [int(depth_m.shape[0]), int(depth_m.shape[1])],
        "step_bytes":     int(depth_msg.step),
        "is_bigendian":   int(depth_msg.is_bigendian),
        "finite_count":   int(finite.size),
        "positive_count": int(positive.size),
        "finite_min_m":   None,
        "finite_max_m":   None,
    }
    if finite.size > 0:
        stats["finite_min_m"] = float(np.min(finite))
        stats["finite_max_m"] = float(np.max(finite))
    return stats


def _roi_depth_stats(depth_m: np.ndarray, roi_xyxy: List[int]) -> Dict[str, Any]:
    """Статистика глубины в заданной области интереса (ROI)."""
    empty = {"valid_count": 0, "min_m": None, "max_m": None,
             "mean_m": None, "median_m": None}
    if not roi_xyxy or len(roi_xyxy) != 4:
        return empty
    h, w   = depth_m.shape[:2]
    x0, y0, x1, y1 = [max(0, int(v)) for v in roi_xyxy]
    x1, y1 = min(w, x1), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return empty
    roi   = depth_m[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    if valid.size == 0:
        return empty
    return {
        "valid_count": int(valid.size),
        "min_m":       float(np.min(valid)),
        "max_m":       float(np.max(valid)),
        "mean_m":      float(np.mean(valid)),
        "median_m":    float(np.median(valid)),
    }


def _is_far_clip_saturated(ctx: Dict[str, Any], z: Optional[float],
                            roi_stats: Dict[str, Any]) -> bool:
    """Проверяет, насыщена ли глубина на дальнем клипе."""
    if z is None or not np.isfinite(z):
        return False
    far_threshold = float(ctx["clip_far"]) - CLIP_SATURATION_EPS_M
    if float(z) < far_threshold:
        return False
    roi_values = [roi_stats.get(k) for k in ("min_m", "max_m", "mean_m", "median_m")]
    finite_values = [float(v) for v in roi_values if v is not None and np.isfinite(v)]
    if not finite_values:
        return True
    return all(v >= far_threshold for v in finite_values)


# ---------------------------------------------------------------------------
#  Геометрические вспомогательные функции
# ---------------------------------------------------------------------------

def _iter_float_range(start: float, stop: float, step: float) -> List[float]:
    values: List[float] = []
    cur = float(start)
    while cur <= float(stop) + 1e-9:
        values.append(round(cur, 4))
        cur += float(step)
    return values


def _iter_float_range_desc(start: float, stop: float, step: float) -> List[float]:
    values: List[float] = []
    cur = float(start)
    while cur >= float(stop) - 1e-9:
        values.append(round(cur, 4))
        cur -= float(step)
    return values


def _front_face_depth(center_x_m: float, size_x_m: float) -> float:
    return float(center_x_m) - (float(size_x_m) * 0.5)


def _c5_expected_in_contract_range(ctx: Dict[str, Any], expected_depth: float) -> bool:
    near_limit = float(ctx["clip_near"]) + C5_CLIP_MARGIN_M
    far_limit  = float(ctx["clip_far"])  - C5_CLIP_MARGIN_M
    return bool(near_limit <= float(expected_depth) <= far_limit)


# ---------------------------------------------------------------------------
#  Вспомогательные функции описания результатов
# ---------------------------------------------------------------------------

def _camera_method_passed(result: dict) -> bool:
    if not isinstance(result, dict):
        return True
    if "passed" in result:
        return bool(result["passed"])
    metrics = result.get("metrics")
    if isinstance(metrics, dict):
        status = str(metrics.get("status", "") or "").strip().upper()
        if status:
            return status == "PASS"
    return True


def _depth_build_description(method_name: str, result: dict, passed: bool) -> str:
    metrics = result.get("metrics", {})
    prefix  = "" if passed else "Датчик не прошёл тест. "
    try:
        if method_name == "depth_perception_test":
            if passed:
                desc = (
                    f"Тест пройден: глубина корректно измеряется на всех дистанциях "
                    f"{metrics.get('distances_m', '?')}м. "
                    f"Монотонность соблюдена. Допуск: ≤{metrics.get('max_abs_error_m', '?')}м."
                )
            else:
                desc = (
                    f"Измерение глубины некорректно. "
                    f"Дистанции: {metrics.get('distances_m', '?')}м. "
                    f"Допуск: ≤{metrics.get('max_abs_error_m', '?')}м."
                )
        elif method_name == "c3_view_angle_stability_test":
            mean_e = float(metrics.get("mean_abs_error_m", 0))
            max_e  = float(metrics.get("max_abs_error_m", 0))
            if passed:
                desc = (
                    f"Тест пройден: глубина стабильна при разных углах обзора. "
                    f"Средняя ошибка: {mean_e:.4f}м (лимит {metrics.get('mean_abs_error_limit_m', '?')}м), "
                    f"максимальная: {max_e:.4f}м (лимит {metrics.get('max_abs_error_limit_m', '?')}м)."
                )
            else:
                desc = (
                    f"Глубина нестабильна при разных углах обзора. "
                    f"Средняя ошибка: {mean_e:.4f}м, максимальная: {max_e:.4f}м."
                )
        elif method_name == "c5_working_range_test":
            x_min   = float(metrics.get("x_min_ok_m", 0))
            x_max   = float(metrics.get("x_max_ok_m", 0))
            checks  = metrics.get("checks", {})
            width   = float(checks.get("interval_width_m", x_max - x_min))
            tol     = metrics.get("tolerance_m", "?")
            n_samp  = len(metrics.get("samples", []))
            ok_cnt  = sum(1 for s in metrics.get("samples", []) if s.get("ok"))
            if passed:
                desc = (
                    f"Тест пройден: стабильный рабочий диапазон [{x_min:.2f}, {x_max:.2f}]м "
                    f"(ширина {width:.2f}м, требовалось ≥{checks.get('min_interval_required_m', '?')}м). "
                    f"Из {n_samp} позиций {ok_cnt} прошли проверку глубины (допуск ≤{tol}м)."
                )
            else:
                desc = (
                    f"Рабочий диапазон слишком узкий: [{x_min:.2f}, {x_max:.2f}]м "
                    f"(ширина {width:.2f}м). Из {n_samp} позиций {ok_cnt} прошли "
                    f"проверку (допуск ≤{tol}м)."
                )
        elif method_name == "c6_small_displacement_sensitivity_test":
            min_step = metrics.get("min_detected_step_m")
            if passed and min_step is not None:
                desc = (
                    f"Тест пройден: датчик различает перемещения "
                    f"вплоть до {min_step * 1000:.1f} мм (порог {min_step:.4f} м)."
                )
            else:
                mm_val = min_step * 1000 if min_step is not None else None
                desc = (
                    "Датчик не различает малые перемещения. "
                    "Минимальный обнаруженный шаг: "
                    f"{mm_val:.1f} мм." if mm_val is not None
                    else "Датчик не различает малые перемещения. "
                         "Минимальный обнаруженный шаг: не определён."
                )
        elif method_name == "dcam_fov_test":
            h = float(metrics.get("measured_hfov_rad", 0))
            v = float(metrics.get("measured_vfov_rad", 0))
            th = float(metrics.get("target_hfov_rad", 0))
            tv = float(metrics.get("target_vfov_rad", 0))
            if passed:
                desc = (
                    f"Тест пройден: горизонтальный FOV {math.degrees(h):.2f}° "
                    f"(целевой {math.degrees(th):.2f}°), "
                    f"вертикальный FOV {math.degrees(v):.2f}° "
                    f"(целевой {math.degrees(tv):.2f}°)."
                )
            else:
                desc = "Измеренный FOV не соответствует целевому."
        elif method_name == "dcam_resolution_test":
            exp = metrics.get("expected_resolution", {})
            act = metrics.get("actual_resolution", {})
            if passed:
                desc = (
                    f"Тест пройден: разрешение {act.get('width')}x{act.get('height')} "
                    f"совпадает с ожидаемым {exp.get('width')}x{exp.get('height')}."
                )
            else:
                desc = (
                    f"Несовпадение разрешения: ожидалось {exp.get('width')}x{exp.get('height')}, "
                    f"получено {act.get('width')}x{act.get('height')}."
                )
        elif method_name == "dcam_occlusion_test":
            large = metrics.get("back_pixels", {}).get("occ_large", 0)
            small = metrics.get("back_pixels", {}).get("occ_small", 0)
            if passed:
                desc = (
                    f"Окклюзия работает корректно: пикселей заднего куба при большом смещении "
                    f"({large}) больше, чем при малом ({small})."
                )
            else:
                desc = (
                    f"Ошибка окклюзии: large={large}, small={small}."
                )        
        else:
            return None
    except Exception:
        return None
    return prefix + desc


# ---------------------------------------------------------------------------
#  Общий запуск теста
# ---------------------------------------------------------------------------

def _run_camera_test(test_fn, method_name: str, simulator, sensor,
                     progress_cb=None) -> dict:
    """Создаёт контекст и запускает тестовую функцию с обработкой ошибок."""
    sensor_name = getattr(sensor, "sensor_name", "?")
    logger.info(f"Запуск теста {method_name}: сенсор={sensor_name}")

    ctx = _build_ctx(sensor)
    if not ctx["DEPTH_TOPIC"]:
        msg = (
            f"Тест {method_name} требует камеру глубины, но сенсор "
            f"'{sensor_name}' не имеет топика глубины."
        )
        logger.error(msg)
        return {
            "passed":  False,
            "skipped": True,
            "error":   msg,
            "metrics": {"status": "SKIP", "error_reason": msg},
        }

    ctx["_progress_cb"] = progress_cb
    if progress_cb:
        try: progress_cb(5)
        except Exception: pass

    try:
        logger.info(f"Вызов тестовой функции {method_name}...")
        result = test_fn(ctx, simulator)
        logger.info(f"Тест {method_name} завершён: "
                    f"passed={result.get('passed') if isinstance(result, dict) else '?'}")
    except Exception as e:
        import traceback
        tb = traceback.format_exc()
        logger.error(f"Тест {method_name} завершился с ошибкой: {type(e).__name__}: {e}")
        logger.debug(tb)
        result  = {"passed": False, "error": f"{type(e).__name__}: {e}"}
        desc    = _depth_build_description(method_name, result, False)
        if desc:
            result["description"] = desc
        if progress_cb:
            try: progress_cb(100)
            except Exception: pass
        return result

    if not isinstance(result, dict):
        result = {"result": result}
    else:
        result = dict(result)
    result.setdefault("passed", _camera_method_passed(result))
    desc = _depth_build_description(method_name, result, result.get("passed", False))
    if desc:
        result["description"] = desc

    if progress_cb:
        try: progress_cb(100)
        except Exception: pass
    return result


# ---------------------------------------------------------------------------
#  depth_perception_test
# ---------------------------------------------------------------------------

def _depth_perception_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Проверка точности глубины без использования цвета.
    Куб перемещается на фиксированные дистанции, глубина измеряется
    как минимальная не‑фоновая величина в кадре.
    """
    logger.info("=" * 60)
    logger.info("  depth_perception_test: начало (упрощённая логика)")
    logger.info("=" * 60)

    _open_test_scene(ctx, simulator, "depth_perception_test")

    cube_name    = "close_green_cube"
    cube_z       = 0.25
    cube_half_x  = 0.25          # половина размера куба по X
    reset_x      = 50.0
    clip_far     = float(ctx["clip_far"])

    logger.info(f"Ожидание спауна куба {cube_name!r}...")
    if not simulator.wait_for_model_spawn(cube_name, 30):
        raise RuntimeError(f"Куб не появился в сцене: {cube_name}")
    logger.info(f"Куб {cube_name!r} успешно заспаунен")

    # --------------------------------------------------------------
    # Вспомогательная функция – минимальная не‑фоновая глубина
    # --------------------------------------------------------------
    def _measure_closest_depth(depth_map: np.ndarray) -> Optional[float]:
        """Минимальная положительная глубина, не насыщенная на far clip.
           Позволяет измерить куб, где бы он ни находился в кадре.
        """
        far_thresh = clip_far - 0.1           # всё, что дальше – фон
        valid = depth_map[(depth_map > 0) &
                          np.isfinite(depth_map) &
                          (depth_map < far_thresh)]
        if len(valid) == 0:
            return None
        return float(np.min(valid))

    results: List[Dict[str, Any]]    = []
    measured_values: List[float]     = []
    last_depth_stamp_s: Optional[float] = None

    metrics_payload: Dict[str, Any] = {
        "status":                "RUNNING",
        "error_reason":          "",
        "world_file":            str(ctx["test_to_world"]["depth_perception_test"]),
        "scene_open_success":    True,
        "expected_depth_topic":  str(ctx["DEPTH_TOPIC"]),
        "resolved_depth_topic":  str(ctx["resolved_depth_topic"] or ctx["DEPTH_TOPIC"]),
        "distances_m":           list(TEST_DISTANCES),
        "max_abs_error_m":       float(MAX_ABS_ERROR_M),
        "clip_near_m":           float(ctx["clip_near"]),
        "clip_far_m":            clip_far,
        "measurements":          results,
    }

    def _upd(status: str, error_reason: str = "") -> None:
        metrics_payload["status"]       = str(status)
        metrics_payload["error_reason"] = str(error_reason)

    for d in TEST_DISTANCES:
        logger.info(f"Измерение глубины на дистанции {d:.1f} м...")
        sample: Dict[str, Any] = {"distance_m": float(d), "status": "RUNNING"}

        try:
            # --- Сброс куба далеко ---
            logger.info(f"Сброс куба на x={reset_x} м (вне зоны видимости)")
            simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
            reset_msg = _wait_depth_after(ctx, prev_stamp_s=last_depth_stamp_s,
                                          timeout=3.0,
                                          stage=f"reset_{d:.1f}".replace('.', '_'))
            last_depth_stamp_s = _msg_stamp_s(reset_msg)

            # --- Перемещение на тестовую дистанцию ---
            target_x = float(d) + cube_half_x
            logger.info(f"Перемещение куба на x={target_x:.4f} м")
            simulator.set_pose(cube_name, target_x, 0.0, cube_z)
            depth_msg = _wait_depth_after(ctx, prev_stamp_s=last_depth_stamp_s,
                                          timeout=5.0,
                                          stage=f"target_{d:.1f}".replace('.', '_'))
            last_depth_stamp_s = _msg_stamp_s(depth_msg)
            sample["depth_stamp_s"] = float(last_depth_stamp_s)

            # Декодируем и измеряем минимальную не‑фоновую глубину
            depth_map = _depth_msg_to_meters(depth_msg)
            z_meas = _measure_closest_depth(depth_map)
            sample["measured_depth_m"] = z_meas
            logger.info(f"Минимальная не‑фоновая глубина: z={z_meas}")

            # --- Проверки ---
            if z_meas is None or z_meas <= 0.0:
                sample["status"] = "FAIL"
                sample["error_reason"] = "invalid_measurement"
                results.append(sample)
                _upd("FAIL", "invalid_measurement")
                raise RuntimeError(f"Некорректное измерение на дистанции {d} м: z={z_meas}")

            # Насыщение дальнего клипа
            if abs(z_meas - clip_far) < CLIP_SATURATION_EPS_M:
                sample["status"] = "FAIL"
                sample["error_reason"] = "far_clip_saturation"
                results.append(sample)
                _upd("FAIL", "far_clip_saturation")
                raise RuntimeError(f"Насыщение far_clip на {d} м: z={z_meas:.4f}")

            # Выход за границы клип‑диапазона
            if not (ctx["clip_near"] <= z_meas <= clip_far + 0.5):
                sample["status"] = "FAIL"
                sample["error_reason"] = "clip_range"
                results.append(sample)
                _upd("FAIL", "clip_range")
                raise RuntimeError(
                    f"Глубина {z_meas:.3f} вне [{ctx['clip_near']}, {clip_far+0.5}] на {d} м"
                )

            # Абсолютная ошибка
            abs_err = abs(z_meas - float(d))
            sample["abs_error_m"] = abs_err
            if abs_err > MAX_ABS_ERROR_M:
                sample["status"] = "FAIL"
                sample["error_reason"] = "abs_error"
                results.append(sample)
                _upd("FAIL", "abs_error")
                raise RuntimeError(f"Ошибка {abs_err:.4f} м > {MAX_ABS_ERROR_M} м на {d} м")

            sample["status"] = "PASS"
            results.append(sample)
            measured_values.append(z_meas)

        except Exception:
            if sample not in results:
                results.append(sample)
            raise

    # --- Монотонность ---
    logger.info(f"Проверка монотонности: {[round(v,4) for v in measured_values]}")
    if not all(measured_values[i] < measured_values[i+1] for i in range(len(measured_values)-1)):
        _upd("FAIL", "monotonicity")
        raise RuntimeError("Глубина не возрастает монотонно")

    # --- Успех ---
    _upd("PASS")
    return {
        "id": "DEPTH",
        "metrics": {
            "distances_m":          list(TEST_DISTANCES),
            "max_abs_error_m":      float(MAX_ABS_ERROR_M),
            "measurements":         results,
            "monotonic_increasing": True,
            "selected_depth_topic": ctx["resolved_depth_topic"],
        },
    }


# ---------------------------------------------------------------------------
#  c3_view_angle_stability_test
# ---------------------------------------------------------------------------

def _c3_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Реализация C3: стабильность глубины при разных углах обзора."""
    logger.info("=" * 60)
    logger.info("  C3: тест стабильности угла обзора — начало")
    logger.info("=" * 60)

    metrics: Dict[str, Any] = {
        "samples_target":        int(C3_SAMPLES),
        "radius_m":              float(C3_RADIUS_M),
        "expected_curve":        "front_face_depth = radius*cos(theta) - target_half_depth",
        "mean_abs_error_limit_m": float(C3_MEAN_ABS_ERROR_M),
        "max_abs_error_limit_m": float(C3_MAX_ABS_ERROR_M),
        "samples":               [],
        "mode":                  "",
    }

    _open_test_scene(ctx, simulator, "c3_view_angle_stability_test")

    logger.info(f"Ожидание спауна куба {C3_TARGET_CUBE_NAME!r}...")
    if not simulator.wait_for_model_spawn(C3_TARGET_CUBE_NAME, timeout=20):
        raise RuntimeError(f"Модель не появилась: {C3_TARGET_CUBE_NAME}")
    logger.info(f"Куб {C3_TARGET_CUBE_NAME!r} заспаунен")

    samples: List[Dict[str, Any]] = []
    cam_name = ctx["camera_model_name"]
    use_camera_orbit = simulator.wait_for_model_spawn(cam_name, timeout=10)
    logger.info(f"Режим C3: {'орбита камеры' if use_camera_orbit else 'орбита куба (запасной)'}")
    last_depth_stamp_s: Optional[float] = None

    if use_camera_orbit:
        metrics["mode"] = "camera_orbit"
        try:
            cube_x, cube_y, cube_z = 2.0, 0.0, 0.25
            cam_z = 0.25
            logger.info(f"C3: орбита камеры — {C3_SAMPLES} позиций, "
                        f"куб зафиксирован на x={cube_x}, y={cube_y}")
            for i in range(C3_SAMPLES):
                theta = 2.0 * pi * (float(i) / float(C3_SAMPLES))
                cam_x = cube_x + C3_RADIUS_M * cos(theta)
                cam_y = cube_y + C3_RADIUS_M * sin(theta)
                yaw   = atan2(cube_y - cam_y, cube_x - cam_x)
                logger.info(f"C3: образец {i+1}/{C3_SAMPLES} — "
                            f"theta={theta:.3f} рад, cam=({cam_x:.3f},{cam_y:.3f}), "
                            f"yaw={yaw:.3f}")
                _set_model_pose_6d(cam_name, x=cam_x, y=cam_y, z=cam_z,
                                   yaw=yaw, settle_s=0.35)

                depth_msg = _wait_depth_after(ctx, prev_stamp_s=last_depth_stamp_s,
                                              timeout=3.0,
                                              stage=f"c3_camera_orbit_{i}")
                last_depth_stamp_s = _msg_stamp_s(depth_msg)
                depth_m = _depth_msg_to_meters(depth_msg)
                color_msg = _wait_color_after(ctx,
                                              prev_stamp_s=last_depth_stamp_s - 1e-6,
                                              timeout=2.0,
                                              stage=f"c3_camera_orbit_color_{i}")
                bgr = _color_msg_to_bgr(color_msg) if color_msg is not None else None
                z, point = _measure_depth(depth_m, bgr, color_hint="blue")

                if z is None or np.isnan(z) or np.isinf(z):
                    logger.warning(f"C3: образец {i+1} — глубина не определена, пропуск")
                    continue
                expected_depth = max(0.0, float(C3_RADIUS_M) - (float(C3_TARGET_SIZE_X_M) * 0.5))
                abs_err = abs(float(z) - expected_depth)
                logger.info(f"C3: образец {i+1} — z={z:.4f}м, "
                            f"ожидается={expected_depth:.4f}м, ошибка={abs_err:.4f}м")
                samples.append({
                    "sample_index":    int(i),
                    "theta_rad":       float(theta),
                    "camera_x_m":      float(cam_x),
                    "camera_y_m":      float(cam_y),
                    "measured_depth_m": float(z),
                    "expected_depth_m": float(expected_depth),
                    "abs_error_m":     float(abs_err),
                })

            min_valid = max(10, int(0.6 * C3_SAMPLES))
            if len(samples) < min_valid:
                raise RuntimeError(
                    f"C3: слишком мало валидных измерений в режиме орбиты камеры: "
                    f"{len(samples)} (минимум {min_valid})"
                )
        except Exception as exc:
            logger.warning(f"C3: режим орбиты камеры завершился с ошибкой: {exc}. "
                           f"Переключение на запасной режим.")
            metrics["mode_error"] = str(exc)
            samples.clear()
            use_camera_orbit = False

    if not use_camera_orbit:
        metrics["mode"] = "target_cube_orbit_equivalent"
        metrics["implementation_note"] = (
            "Орбита камеры заменена эквивалентным движением куба "
            "относительно статичной камеры через /gazebo/set_model_state."
        )
        angles = np.linspace(-C3_FALLBACK_HALF_ANGLE, C3_FALLBACK_HALF_ANGLE, C3_SAMPLES)
        logger.info(f"C3: запасной режим — орбита куба, {C3_SAMPLES} позиций, "
                    f"диапазон углов=[{-C3_FALLBACK_HALF_ANGLE:.2f}, "
                    f"{C3_FALLBACK_HALF_ANGLE:.2f}] рад")

        for i, theta in enumerate(angles):
            x = C3_RADIUS_M * cos(float(theta))
            y = C3_RADIUS_M * sin(float(theta))
            logger.info(f"C3 (запасной): образец {i+1}/{C3_SAMPLES} — "
                        f"theta={theta:.3f} рад, куб → x={x:.4f}, y={y:.4f}")
            _move_and_settle(simulator, C3_TARGET_CUBE_NAME, x=x, y=y, z=0.25, settle_s=0.25)

            depth_msg = _wait_depth_after(ctx, prev_stamp_s=last_depth_stamp_s,
                                          timeout=3.0, stage=f"c3_target_orbit_{i}")
            last_depth_stamp_s = _msg_stamp_s(depth_msg)
            depth_m = _depth_msg_to_meters(depth_msg)
            color_msg = _wait_color_after(ctx,
                                          prev_stamp_s=last_depth_stamp_s - 1e-6,
                                          timeout=2.0,
                                          stage=f"c3_target_orbit_color_{i}")
            bgr = _color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point = _measure_depth(depth_m, bgr, color_hint="blue")

            if z is None or np.isnan(z) or np.isinf(z):
                logger.warning(f"C3 (запасной): образец {i+1} — глубина не определена, пропуск")
                continue
            expected_depth = _front_face_depth(center_x_m=float(x),
                                               size_x_m=float(C3_TARGET_SIZE_X_M))
            abs_err = abs(float(z) - expected_depth)
            logger.info(f"C3 (запасной): образец {i+1} — z={z:.4f}м, "
                        f"ожидается={expected_depth:.4f}м, ошибка={abs_err:.4f}м")
            samples.append({
                "sample_index":      int(i),
                "theta_rad":         float(theta),
                "target_center_x_m": float(x),
                "target_center_y_m": float(y),
                "expected_depth_m":  float(expected_depth),
                "measured_depth_m":  float(z),
                "abs_error_m":       float(abs_err),
            })

    if len(samples) < 10:
        metrics["samples"] = samples
        raise RuntimeError(f"C3: слишком мало валидных измерений: {len(samples)}")

    abs_errors      = [float(s["abs_error_m"]) for s in samples]
    measured_depths = [float(s["measured_depth_m"]) for s in samples]
    expected_depths = [float(s["expected_depth_m"]) for s in samples]
    mean_abs_error  = float(np.mean(abs_errors))
    max_abs_error   = float(np.max(abs_errors))

    metrics["samples"]               = samples
    metrics["mean_measured_depth_m"] = float(np.mean(measured_depths))
    metrics["mean_expected_depth_m"] = float(np.mean(expected_depths))
    metrics["mean_abs_error_m"]      = mean_abs_error
    metrics["max_abs_error_m"]       = max_abs_error
    metrics["checks"] = {
        "mean_abs_error_ok": bool(mean_abs_error <= C3_MEAN_ABS_ERROR_M),
        "max_abs_error_ok":  bool(max_abs_error  <= C3_MAX_ABS_ERROR_M),
    }

    logger.info(f"C3: итог — {len(samples)} образцов, "
                f"средняя ошибка={mean_abs_error:.4f}м (лимит={C3_MEAN_ABS_ERROR_M}м), "
                f"максимальная ошибка={max_abs_error:.4f}м (лимит={C3_MAX_ABS_ERROR_M}м)")

    if not all(metrics["checks"].values()):
        logger.error(f"C3 НЕ ПРОЙДЕН: mean={mean_abs_error:.4f} > {C3_MEAN_ABS_ERROR_M} "
                     f"или max={max_abs_error:.4f} > {C3_MAX_ABS_ERROR_M}")
        raise AssertionError(
            f"C3: mean_abs_error={mean_abs_error:.4f} (лимит={C3_MEAN_ABS_ERROR_M:.4f}), "
            f"max_abs_error={max_abs_error:.4f} (лимит={C3_MAX_ABS_ERROR_M:.4f})"
        )

    logger.info("C3: тест ПРОЙДЕН")
    return {"id": "C3", "passed": True, "metrics": metrics}


# ---------------------------------------------------------------------------
#  c5_working_range_test
# ---------------------------------------------------------------------------

def _c5_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Реализация C5: рабочий диапазон камеры глубины."""
    logger.info("=" * 60)
    logger.info("  C5: тест рабочего диапазона — начало")
    logger.info("=" * 60)

    x_values = _iter_float_range(C5_START_X, C5_END_X, C5_STEP)
    metrics: Dict[str, Any] = {
        "x_values_m":           x_values,
        "tolerance_m":          float(C5_DEPTH_TOLERANCE_M),
        "clip_margin_m":        float(C5_CLIP_MARGIN_M),
        "coverage_tolerance_m": float(C5_STEP),
        "target_size_x_m":      float(C5_TARGET_SIZE_X_M),
        "samples":              [],
        "first_frame_diagnostics": {},
    }
    logger.info(f"C5: диапазон x=[{C5_START_X}, {C5_END_X}]м, шаг={C5_STEP}м, "
                f"всего {len(x_values)} позиций, допуск={C5_DEPTH_TOLERANCE_M}м")

    _open_test_scene(ctx, simulator, "c5_working_range_test")

    logger.info(f"Ожидание спауна куба {C5_RANGE_CUBE_NAME!r}...")
    if not simulator.wait_for_model_spawn(C5_RANGE_CUBE_NAME, timeout=20):
        raise RuntimeError(f"Модель не появилась: {C5_RANGE_CUBE_NAME}")
    logger.info(f"Куб {C5_RANGE_CUBE_NAME!r} заспаунен")

    first_frame_diag: Optional[Dict] = None
    last_depth_stamp_s: Optional[float] = None

    for x in x_values:
        logger.info(f"C5: перемещение куба на x={x:.4f}м...")
        _move_and_settle(simulator, C5_RANGE_CUBE_NAME, x=float(x), y=0.0, z=0.25,
                         settle_s=0.3)

        depth_msg = _wait_depth_after(ctx, prev_stamp_s=last_depth_stamp_s,
                                      timeout=3.0,
                                      stage=f"c5_x_{str(x).replace('.','_')}")
        last_depth_stamp_s = _msg_stamp_s(depth_msg)
        depth_m = _depth_msg_to_meters(depth_msg)

        color_msg = _wait_color_after(ctx, prev_stamp_s=last_depth_stamp_s - 1e-6,
                                      timeout=2.0,
                                      stage=f"c5_color_x_{str(x).replace('.','_')}")
        bgr = _color_msg_to_bgr(color_msg) if color_msg is not None else None
        z, point, roi_meta = _measure_depth_with_meta(depth_m, bgr, color_hint="green")

        if first_frame_diag is None:
            first_frame_diag = _depth_frame_stats(depth_msg, depth_m)
            first_frame_diag["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
            first_frame_diag["measurement_roi"]   = roi_meta
            logger.info(f"C5: первый кадр — конечных={first_frame_diag.get('finite_count')}, "
                        f"мин={first_frame_diag.get('finite_min_m'):.4f}м, "
                        f"макс={first_frame_diag.get('finite_max_m'):.4f}м")

        roi_stats          = _roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))
        expected_depth     = _front_face_depth(center_x_m=float(x),
                                               size_x_m=float(C5_TARGET_SIZE_X_M))
        expected_in_range  = _c5_expected_in_contract_range(ctx, expected_depth)
        finite_ok          = bool(z is not None and np.isfinite(z))
        abs_err            = float(abs(float(z) - expected_depth)) if finite_ok else float("inf")
        sample_ok          = bool(finite_ok and expected_in_range
                                  and abs_err <= C5_DEPTH_TOLERANCE_M)

        logger.info(f"C5: x={x:.4f}м — ожидается={expected_depth:.4f}м, "
                    f"измерено={'None' if z is None else f'{z:.4f}м'}, "
                    f"ошибка={'inf' if not np.isfinite(abs_err) else f'{abs_err:.4f}м'}, "
                    f"в диапазоне={expected_in_range}, "
                    f"OK={sample_ok}")

        metrics["samples"].append({
            "x_m":                     float(x),
            "expected_front_face_depth_m": float(expected_depth),
            "expected_in_sensor_range": bool(expected_in_range),
            "depth_m":                 None if z is None else float(z),
            "measurement_roi":         roi_meta,
            "roi_depth_stats":         roi_stats,
            "finite_ok":               finite_ok,
            "abs_error_m":             abs_err if np.isfinite(abs_err) else None,
            "ok":                      sample_ok,
        })

    if first_frame_diag is not None:
        metrics["first_frame_diagnostics"] = first_frame_diag

    expected_ok_x = [
        float(x) for x in x_values
        if _c5_expected_in_contract_range(
            ctx,
            _front_face_depth(center_x_m=float(x), size_x_m=float(C5_TARGET_SIZE_X_M))
        )
    ]
    logger.info(f"C5: позиций в допустимом диапазоне сенсора: {len(expected_ok_x)}")

    best_start = best_end = cur_start = cur_end = None
    for sample in metrics["samples"]:
        if sample["ok"]:
            if cur_start is None:
                cur_start = float(sample["x_m"])
            cur_end = float(sample["x_m"])
        else:
            if cur_start is not None:
                if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                    best_start, best_end = cur_start, cur_end
                cur_start = cur_end = None
    if cur_start is not None:
        if best_start is None or (cur_end - cur_start) > (best_end - best_start):
            best_start, best_end = cur_start, cur_end

    metrics["expected_ok_x_values_m"] = expected_ok_x
    metrics["expected_x_min_ok_m"]    = expected_ok_x[0] if expected_ok_x else None
    metrics["expected_x_max_ok_m"]    = expected_ok_x[-1] if expected_ok_x else None

    if not expected_ok_x:
        raise AssertionError("C5: нет позиций в допустимом диапазоне клипа сенсора")
    if best_start is None or best_end is None:
        raise AssertionError("C5: стабильный диапазон глубины не найден")

    metrics["x_min_ok_m"]         = float(best_start)
    metrics["x_max_ok_m"]         = float(best_end)
    metrics["topic_diagnostics"]  = ctx["last_test_diagnostics"].get(
        "depth_topic_resolution", {}
    )
    metrics["selected_depth_topic"] = ctx["resolved_depth_topic"]
    metrics["selected_image_topic"] = ctx["resolved_image_topic"]

    interval_width   = float(best_end - best_start)
    min_interval_m   = min(1.0, float(C5_STEP) * 2)
    interval_ok      = interval_width >= min_interval_m
    metrics["checks"] = {
        "interval_width_m":        interval_width,
        "min_interval_required_m": min_interval_m,
        "interval_ok":             bool(interval_ok),
    }

    logger.info(f"C5: лучший стабильный интервал [{best_start:.2f}, {best_end:.2f}]м, "
                f"ширина={interval_width:.2f}м, минимум={min_interval_m:.2f}м — "
                f"{'OK' if interval_ok else 'НЕДОСТАТОЧНО'}")

    if not interval_ok:
        raise AssertionError(
            f"C5: стабильный интервал [{best_start:.2f}, {best_end:.2f}] "
            f"шириной {interval_width:.2f}м < минимума {min_interval_m:.2f}м"
        )

    logger.info("C5: тест ПРОЙДЕН")
    return {"id": "C5", "passed": True, "metrics": metrics}


# ---------------------------------------------------------------------------
#  c6_small_displacement_sensitivity_test
# ---------------------------------------------------------------------------

def _c6_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """C6: строгий бинарный поиск порога чувствительности с учётом кванта глубины."""
    logger.info("=" * 60)
    logger.info("  C6: тест чувствительности к малым перемещениям (бинарный поиск) — начало")
    logger.info("=" * 60)

    _open_test_scene(ctx, simulator, "c6_small_displacement_sensitivity_test")

    if not simulator.wait_for_model_spawn(C6_SHIFT_CUBE_NAME, timeout=20):
        raise RuntimeError(f"Модель не появилась: {C6_SHIFT_CUBE_NAME}")

    clip_far = float(ctx["clip_far"])

    def _measure_depth_fast(depth_map: np.ndarray) -> Optional[float]:
        """Минимальная не‑фоновая глубина."""
        far_thresh = clip_far - 0.1
        valid = depth_map[(depth_map > 0) & np.isfinite(depth_map) & (depth_map < far_thresh)]
        if len(valid) == 0:
            return None
        return float(np.min(valid))

    # ----- вспомогательная функция для захвата глубины в точке -----
    def _get_depth_at(x: float, prev_stamp: Optional[float], settle: float = 0.3) -> Tuple[Optional[float], float]:
        _move_and_settle(simulator, C6_SHIFT_CUBE_NAME, x=x, y=0.0, z=0.25, settle_s=settle)
        msg = _wait_depth_after(ctx, prev_stamp_s=prev_stamp, timeout=1.5,
                                stage=f"c6_x_{x:.5f}".replace('.', '_'))
        stamp = _msg_stamp_s(msg)
        depth_map = _depth_msg_to_meters(msg)
        depth = _measure_depth_fast(depth_map)
        return depth, stamp

    # ----- опорная позиция -----
    base_x = 2.0
    last_stamp: Optional[float] = None
    ref_depth, last_stamp = _get_depth_at(base_x, None)
    if ref_depth is None:
        raise RuntimeError("Не удалось получить опорную глубину")

    # ----- оценка кванта глубины (минимальное изменение, которое может выдать сенсор) -----
    logger.info("C6: оценка кванта глубины...")
    depths_same_spot: List[float] = []
    for _ in range(5):
        d, last_stamp = _get_depth_at(base_x, last_stamp, settle=0.2)
        if d is not None:
            depths_same_spot.append(d)
    depth_quantum = float('inf')
    if len(depths_same_spot) > 1:
        diffs = np.abs(np.diff(depths_same_spot))
        nonzero = diffs[diffs > 0]
        if len(nonzero) > 0:
            depth_quantum = float(np.min(nonzero))
    # Если все измерения идентичны, квант не определён — используем разумный минимум 0.1 мм
    if depth_quantum == float('inf') or depth_quantum <= 0.0:
        depth_quantum = 1e-4
    logger.info(f"C6: оценка кванта глубины = {depth_quantum:.6f} м")

    # Минимальный tolerance – половина кванта, но не меньше 0.1 мм
    tolerance = max(1e-4, depth_quantum * 0.5)
    logger.info(f"C6: точность бинарного поиска (tolerance) = {tolerance:.6f} м")

    # ----- определение верхней границы hi (заведомо различимый шаг) -----
    hi = 0.1
    # Небольшой запас: сначала вернёмся на опорную позицию и обновим ref_depth
    ref_depth, last_stamp = _get_depth_at(base_x, last_stamp, settle=0.3)
    while True:
        test_x = base_x - hi
        if test_x < 0.1:
            raise RuntimeError(f"Слишком большой шаг {hi} – куб уходит за начало координат")
        depth_at_hi, _ = _get_depth_at(test_x, last_stamp, settle=0.3)
        if depth_at_hi is None:
            raise RuntimeError(f"Не удалось измерить глубину при шаге {hi}")
        delta = abs(depth_at_hi - ref_depth)
        logger.info(f"C6: проверка hi={hi:.4f} м, дельта={delta:.6f} м")
        if delta > C6_DEPTH_CHANGE_EPS_M:
            break   # hi действительно различается
        hi *= 2.0
        if hi > 2.0:
            raise RuntimeError("Даже шаг 2 м не различается — проверьте сцену или сенсор")

    # ----- нижняя граница lo = 0 (шаг никогда не может быть меньше 0) -----
    lo = 0.0
    step_history: List[Dict[str, Any]] = []

    # ----- бинарный поиск -----
    while (hi - lo) > tolerance:
        mid = (lo + hi) / 2.0
        logger.info(f"C6: бинарный поиск: интервал [{lo:.6f}, {hi:.6f}], проверка mid={mid:.6f} м")
        # Возвращаем куб на опорную позицию и перемеряем (чтобы исключить дрейф)
        ref_depth, last_stamp = _get_depth_at(base_x, last_stamp, settle=0.2)
        # Измеряем на позиции base_x - mid
        test_depth, _ = _get_depth_at(base_x - mid, last_stamp)
        if test_depth is None:
            logger.warning("Пропуск из-за отсутствия измерения глубины")
            continue

        delta = abs(test_depth - ref_depth)
        detected = delta > C6_DEPTH_CHANGE_EPS_M
        logger.info(f"   дельта={delta:.6f} м, обнаружено: {detected}")
        step_history.append({"lo": lo, "hi": hi, "mid": mid, "delta": delta, "detected": detected})

        if detected:
            hi = mid   # различается → порог не больше mid
        else:
            lo = mid   # не различается → порог больше mid

    # После цикла hi — минимальный обнаруженный шаг с точностью не хуже tolerance
    min_step = hi
    logger.info(f"C6: бинарный поиск завершён: минимальный различимый шаг = {min_step:.6f} м ({min_step*1000:.2f} мм)")

    # ----- оценка прохождения -----
    passed = min_step <= C6_STEP * 2   # C6_STEP = 0.01 м, т.е. порог 2 см
    metrics = {
        "min_detected_step_m": min_step,
        "step_history": step_history,
        "ref_depth_m": ref_depth,
        "depth_quantum_m": depth_quantum,
        "tolerance_m": tolerance,
        "checks": {
            "min_step_detected": True,
            "min_step_le_2cm": passed,
        }
    }

    if not passed:
        raise AssertionError(f"C6: порог чувствительности слишком большой: {min_step:.4f} м")

    return {"id": "C6", "passed": True, "metrics": metrics}


# ---------------------------------------------------------------------------
#  FOV test
# ---------------------------------------------------------------------------

def _dcam_fov_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Быстрое измерение FOV камеры глубины бинарным поиском (без цвета)."""
    logger.info("=" * 60)
    logger.info("  DCAM FOV: измерение горизонтального и вертикального FOV")
    logger.info("=" * 60)

    _open_test_scene(ctx, simulator, "dcam_fov_test")

    # Проверка появления панелей
    for name in ("fov_cube_h", "fov_cube_v"):
        if not simulator.wait_for_model_spawn(name, timeout=20):
            raise RuntimeError(f"Модель не появилась: {name}")

    clip_far = float(ctx["clip_far"])

    # Параметры теста
    HALF_SIZE_H = 0.01   # половина размера панели по Y
    HALF_SIZE_V = 0.01   # половина размера панели по Z
    X_FIXED = 2.0
    MAX_Y = 4.0
    MAX_Z = 4.0
    TOLERANCE = 0.002    # 2 мм
    REL_ERROR_THRESHOLD = 0.05

    # Целевые FOV
    target_hfov = float(ctx["horizontal_fov"])
    img_w = ctx["image_width"]
    img_h = ctx["image_height"]
    target_vfov = 2.0 * math.atan(math.tan(target_hfov / 2.0) * (img_h / img_w))

    logger.info(f"D-FOV: целевой hFOV={math.degrees(target_hfov):.2f}°, vFOV={math.degrees(target_vfov):.2f}°")

    # Прогрев
    last_stamp = None
    msg = _wait_depth(ctx, timeout=5.0)
    last_stamp = _msg_stamp_s(msg)
    logger.info("D-FOV: прогрев ОК")

    # Функция захвата и проверки видимости обеих панелей одновременно
    def _capture_and_check(y: float, z: float, settle_s: float = 0.4) -> tuple:
        nonlocal last_stamp
        _move_and_settle(simulator, "fov_cube_h", x=X_FIXED, y=y, z=0.0, settle_s=settle_s)
        _move_and_settle(simulator, "fov_cube_v", x=X_FIXED, y=0.0, z=z, settle_s=settle_s)
        msg = _wait_depth_after(ctx, prev_stamp_s=last_stamp, timeout=2.0,
                                stage=f"fov_y{y:.4f}_z{z:.4f}")
        last_stamp = _msg_stamp_s(msg)
        depth = _depth_msg_to_meters(msg)

        # Видимость: есть ли хоть один пиксель с глубиной < far_clip - 0.1
        far_thresh = clip_far - 0.1
        valid = depth[(depth > 0) & np.isfinite(depth) & (depth < far_thresh)]
        # Обе панели видны, если есть хоть одна точка с глубиной меньше фона
        visible = len(valid) > 0
        return visible, visible

    # Начальная проверка в центре
    init_vis, _ = _capture_and_check(0.0, 0.0, settle_s=1.0)
    if not init_vis:
        raise RuntimeError("Панели не видны в центре кадра")
    logger.info("D-FOV: панели видны в центре")

    # Ожидаемые координаты перехода
    y_edge_target = X_FIXED * math.tan(target_hfov / 2.0)
    z_edge_target = X_FIXED * math.tan(target_vfov / 2.0)
    y_target = y_edge_target + HALF_SIZE_H
    z_target = z_edge_target + HALF_SIZE_V

    # 3‑точечная проверка
    offsets = [-0.1, 0.0, 0.1]
    points_h = [y_target + off for off in offsets]
    points_v = [z_target + off for off in offsets]
    vis = []
    for i in range(len(offsets)):
        v, _ = _capture_and_check(points_h[i], points_v[i],
                                  settle_s=0.8 if i == 0 else 0.5)
        vis.append(v)

    lo_h, hi_h = 0.0, MAX_Y
    lo_v, hi_v = 0.0, MAX_Z
    last_y, last_z = None, None
    active_h, active_v = True, True

    # Разбор 3‑точечных результатов
    if vis[0] and not vis[-1]:
        for off, v in zip(offsets, vis):
            if v: last_y = y_target + off
        active_h = False
        lo_h = hi_h = last_y
    elif not vis[0]:
        lo_h, hi_h = 0.0, points_h[0]
    elif vis[-1]:
        lo_h, hi_h = points_h[-1], MAX_Y

    if vis[0] and not vis[-1]:
        for off, v in zip(offsets, vis):
            if v: last_z = z_target + off
        active_v = False
        lo_v = hi_v = last_z
    elif not vis[0]:
        lo_v, hi_v = 0.0, points_v[0]
    elif vis[-1]:
        lo_v, hi_v = points_v[-1], MAX_Z

    # Бинарный поиск
    iteration = 0
    while active_h or active_v:
        iteration += 1
        mid_y = (lo_h + hi_h) / 2 if active_h else (last_y or lo_h)
        mid_z = (lo_v + hi_v) / 2 if active_v else (last_z or lo_v)
        settle = 1.0 if iteration == 1 else 0.4
        v, _ = _capture_and_check(mid_y, mid_z, settle_s=settle)

        if active_h:
            if v: lo_h = mid_y
            else: hi_h = mid_y
            if hi_h - lo_h <= TOLERANCE:
                active_h = False
                last_y = lo_h

        if active_v:
            if v: lo_v = mid_z
            else: hi_v = mid_z
            if hi_v - lo_v <= TOLERANCE:
                active_v = False
                last_z = lo_v

    last_y = last_y or lo_h
    last_z = last_z or lo_v
    logger.info(f"D-FOV: последняя видимая позиция: y={last_y:.4f} м, z={last_z:.4f} м")

    # Вычисление FOV
    y_edge = last_y - HALF_SIZE_H
    z_edge = last_z - HALF_SIZE_V
    measured_hfov = 2.0 * math.atan(y_edge / X_FIXED)
    measured_vfov = 2.0 * math.atan(z_edge / X_FIXED)

    rel_h = abs(measured_hfov - target_hfov) / target_hfov if target_hfov else 0.0
    rel_v = abs(measured_vfov - target_vfov) / target_vfov if target_vfov else 0.0

    h_ok = rel_h <= REL_ERROR_THRESHOLD
    v_ok = rel_v <= REL_ERROR_THRESHOLD
    passed = h_ok and v_ok

    metrics = {
        "target_hfov_rad": target_hfov,
        "target_vfov_rad": target_vfov,
        "measured_hfov_rad": measured_hfov,
        "measured_vfov_rad": measured_vfov,
        "rel_error_h": rel_h,
        "rel_error_v": rel_v,
        "y_last_visible_m": last_y,
        "z_last_visible_m": last_z,
        "checks": {"hfov_ok": h_ok, "vfov_ok": v_ok},
    }

    if not passed:
        raise AssertionError(
            f"D-FOV не пройден: hFOV error={rel_h:.2%}, vFOV error={rel_v:.2%}"
        )

    return {"id": "DCAM_FOV", "passed": True, "metrics": metrics}

# ---------------------------------------------------------------------------
#  Dcam test
# ---------------------------------------------------------------------------
def _dcam_resolution_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Проверка фактического разрешения глубинного кадра."""
    logger.info("=" * 60)
    logger.info("  DCAM Resolution: проверка разрешения глубины")
    logger.info("=" * 60)

    # Используем любую сцену с глубинным сенсором — например, depth_perception
    _open_test_scene(ctx, simulator, "depth_perception_test")

    expected_w = int(ctx["image_width"])
    expected_h = int(ctx["image_height"])
    depth_topic = ctx["resolved_depth_topic"] or ctx["DEPTH_TOPIC"]

    logger.info(f"Ожидаемое разрешение: {expected_w}x{expected_h}")
    logger.info(f"Топик глубины: {depth_topic}")

    # Получаем один глубинный кадр
    try:
        msg = _wait_depth(ctx, timeout=10.0)
    except Exception as e:
        raise RuntimeError(f"Не удалось получить глубинный кадр: {e}")

    actual_w = int(msg.width)
    actual_h = int(msg.height)
    encoding = str(msg.encoding or "")

    logger.info(f"Фактическое разрешение: {actual_w}x{actual_h}, кодировка: {encoding}")

    # Проверка валидности
    if actual_w <= 0 or actual_h <= 0:
        raise RuntimeError(f"Некорректные размеры кадра: {actual_w}x{actual_h}")

    # Сравнение
    matches = (actual_w == expected_w) and (actual_h == expected_h)
    metrics = {
        "expected_resolution": {"width": expected_w, "height": expected_h},
        "actual_resolution": {"width": actual_w, "height": actual_h},
        "encoding": encoding,
        "depth_topic": depth_topic,
        "checks": {"resolution_matches": matches},
    }

    if not matches:
        raise AssertionError(
            f"Разрешение не совпадает: ожидалось {expected_w}x{expected_h}, "
            f"получено {actual_w}x{actual_h}"
        )

    return {"id": "DCAM_RES", "passed": True, "metrics": metrics}

# ---------------------------------------------------------------------------
#  Occlusion test
# ---------------------------------------------------------------------------
def _dcam_occlusion_impl(ctx: Dict[str, Any], simulator) -> Dict[str, Any]:
    """Проверка окклюзии на глубинной камере без цвета."""
    logger.info("=" * 60)
    logger.info("  DCAM Occlusion: проверка перекрытия объектов")
    logger.info("=" * 60)

    _open_test_scene(ctx, simulator, "dcam_occlusion_test")

    front_name = DCAM_OCCLUSION_FRONT_CUBE
    back_name = DCAM_OCCLUSION_BACK_CUBE
    for name in (front_name, back_name):
        if not simulator.wait_for_model_spawn(name, timeout=20):
            raise RuntimeError(f"Модель не появилась: {name}")

    # Параметры сцены
    x_back = 3.6
    x_front = 3.0
    cube_z = 0.25
    half_size = 0.25                # полуразмер куба (0.5 / 2)
    depth_tolerance = half_size     # допустимое отклонение глубины для идентификации заднего куба
    min_pixels = DCAM_OCCLUSION_MIN_PIXELS

    clip_far = float(ctx["clip_far"])

    def _count_back_pixels(depth_map: np.ndarray) -> int:
        """Считает пиксели, глубина которых лежит в интервале заднего куба."""
        target = x_back
        lo = target - depth_tolerance
        hi = target + depth_tolerance
        # исключаем фон (far_clip) и невалидные значения
        valid = depth_map[(depth_map > 0) & np.isfinite(depth_map) & (depth_map < clip_far)]
        in_back = valid[(valid >= lo) & (valid <= hi)]
        return in_back.size

    # Прогрев
    last_stamp = None
    msg = _wait_depth(ctx, timeout=5.0)
    last_stamp = _msg_stamp_s(msg)
    logger.info("DCAM Occlusion: прогрев ОК")

    results = {}
    for case_name, y_offset in DCAM_OCCLUSION_CASES.items():
        logger.info(f"Случай '{case_name}': y = {y_offset:.2f} м")
        # Перемещаем передний куб
        _move_and_settle(simulator, front_name, x=x_front, y=y_offset, z=cube_z, settle_s=0.5)
        # Захватываем глубину
        msg = _wait_depth_after(ctx, prev_stamp_s=last_stamp, timeout=2.0,
                                stage=f"occlusion_{case_name}")
        last_stamp = _msg_stamp_s(msg)
        depth_map = _depth_msg_to_meters(msg)
        count = _count_back_pixels(depth_map)
        results[case_name] = count
        logger.info(f"   пикселей заднего куба: {count}")

    count_large = results.get("occ_large", 0)
    count_small = results.get("occ_small", 0)

    # Проверки
    relation_ok = count_large > count_small
    threshold_ok = count_large > min_pixels and count_small > min_pixels
    passed = relation_ok and threshold_ok

    metrics = {
        "back_pixels": results,
        "min_pixels": min_pixels,
        "checks": {
            "relation_ok": relation_ok,
            "threshold_ok": threshold_ok,
        }
    }

    if not passed:
        raise AssertionError(
            f"Окклюзия не пройдена: large={count_large}, small={count_small}, "
            f"relation_ok={relation_ok}, threshold_ok={threshold_ok}"
        )

    return {"id": "DCAM_OCCLUSION", "passed": True, "metrics": metrics}


# ---------------------------------------------------------------------------
#  Публичные функции-точки входа
# ---------------------------------------------------------------------------

def depth_perception_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"depth_perception_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_depth_perception_impl, "depth_perception_test",
                            simulator, sensor, progress_cb)

def c3_view_angle_stability_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"c3_view_angle_stability_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_c3_impl, "c3_view_angle_stability_test",
                            simulator, sensor, progress_cb)

def c5_working_range_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"c5_working_range_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_c5_impl, "c5_working_range_test",
                            simulator, sensor, progress_cb)

def c6_small_displacement_sensitivity_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"c6_small_displacement_sensitivity_test: "
                f"сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_c6_impl, "c6_small_displacement_sensitivity_test",
                            simulator, sensor, progress_cb)

def dcam_fov_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"dcam_fov_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_dcam_fov_impl, "dcam_fov_test",
                            simulator, sensor, progress_cb)

def dcam_resolution_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"dcam_resolution_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_dcam_resolution_impl, "dcam_resolution_test",
                            simulator, sensor, progress_cb)

def dcam_occlusion_test(simulator, sensor, progress_cb=None) -> dict:
    logger.info(f"dcam_occlusion_test: сенсор={getattr(sensor, 'sensor_name', '?')}")
    return _run_camera_test(_dcam_occlusion_impl, "dcam_occlusion_test",
                            simulator, sensor, progress_cb)
