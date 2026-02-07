#!/usr/bin/env python3
"""
Что это за файл:
- Общие утилиты для camera suite (C1/C4/C7): запуск launch, работа с ROS/Gazebo,
  базовая обработка кадров, сохранение артефактов и формирование структуры результата.

Как запускать:
- Файл не является самостоятельным тестом.
- Используется из test_camera_c1_size_order.py, test_camera_c4_geometries_presence.py,
  test_camera_c7_occlusion.py и run_camera_suite.py.

Где результаты:
- results/<camera_name>/captured_images/ (кадры)
- results/<camera_name>/report.json и report.csv (формирует run_camera_suite.py)

Зависимости:
- ROS Noetic (rospy, gazebo_msgs, geometry_msgs, sensor_msgs)
- OpenCV (cv2), numpy
"""

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[4]
RESULTS_ROOT = PROJECT_ROOT / "results"


def init_ros_node(node_name: str = "camera_suite_runner", timeout_s: float = 25.0) -> None:
    """
    Инициализирует ROS node с ретраями.

    Почему с ретраями:
    - roscore поднимается внутри roslaunch, и между стартом launch и готовностью master
      обычно есть небольшая задержка.
    """
    if rospy.core.is_initialized():
        return

    deadline = time.time() + timeout_s
    last_error: Optional[Exception] = None

    while time.time() < deadline:
        try:
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
            return
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            time.sleep(0.5)

    raise RuntimeError(f"Failed to init ROS node within {timeout_s}s: {last_error}")


def start_scene_launch(
    launch_file: str,
    camera_sdf_file: str,
    camera_model_name: str,
    image_topic: str,
    namespace: str = "",
) -> subprocess.Popen:
    """Запускает scene launch в отдельной process group, чтобы корректно завершать все дочерние процессы."""
    cmd = [
        "roslaunch",
        "scenario_test_pkg",
        launch_file,
        f"camera_sdf_file:={camera_sdf_file}",
        f"camera_model_name:={camera_model_name}",
        f"image_topic:={image_topic}",
        f"namespace:={namespace}",
    ]
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def stop_launch(proc: Optional[subprocess.Popen], timeout_s: float = 12.0) -> None:
    """Останавливает roslaunch и дочерние процессы Gazebo/ROS для текущего теста."""
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5.0)
    finally:
        # Пауза нужна, чтобы сервисы Gazebo гарантированно освободились перед следующим тестом.
        time.sleep(2.0)


def wait_for_gazebo_ready(timeout_s: float = 45.0) -> None:
    """Ждет готовности ключевых сервисов Gazebo для движения моделей и чтения сцены."""
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=timeout_s)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=timeout_s)


def wait_for_image(topic: str, timeout_s: float = 35.0) -> Image:
    """Ждет один кадр изображения с заданного topic до timeout_s."""
    deadline = time.time() + timeout_s
    last_error: Optional[Exception] = None

    while time.time() < deadline:
        if rospy.is_shutdown():
            break
        try:
            step_timeout = max(0.3, min(2.0, deadline - time.time()))
            return rospy.wait_for_message(topic, Image, timeout=step_timeout)
        except Exception as exc:  # noqa: BLE001
            last_error = exc

    raise TimeoutError(f"No image from {topic} within {timeout_s}s. Last error: {last_error}")


def ros_image_to_bgr(msg: Image) -> np.ndarray:
    """Конвертирует sensor_msgs/Image в BGR numpy-массив (удобно для OpenCV/HSV)."""
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()

    if enc in {"rgb8", "r8g8b8"}:
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if enc == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

    if enc in {"mono8", "8uc1"}:
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def set_model_pose(model_name: str, x: float, y: float, z: float) -> None:
    """
    Перемещает модель в world через /gazebo/set_model_state.

    Примечание:
    - Для тестов C1/C7 достаточно нулевой ориентации (quaternion = [0,0,0,1]).
    """
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    state.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))

    response = set_state(state)
    if not response.success:
        raise RuntimeError(f"set_model_state failed for {model_name}: {response.status_message}")


def move_model_and_wait(model_name: str, x: float, y: float, z: float, settle_s: float = 0.8) -> None:
    """Двигает модель и дает сцене стабилизироваться перед снятием кадра."""
    set_model_pose(model_name=model_name, x=x, y=y, z=z)
    time.sleep(settle_s)


def ensure_results_dirs(camera_name: str) -> Tuple[Path, Path]:
    """Создает директории результатов для камеры: results/<camera>/ и captured_images/."""
    suite_dir = RESULTS_ROOT / camera_name
    captured_dir = suite_dir / "captured_images"
    captured_dir.mkdir(parents=True, exist_ok=True)
    return suite_dir, captured_dir


def save_image(image_bgr: np.ndarray, captured_dir: Path, filename: str) -> Path:
    """Сохраняет изображение в results/<camera>/captured_images/ и возвращает путь к файлу."""
    out_path = captured_dir / filename
    cv2.imwrite(str(out_path), image_bgr)
    return out_path


def clean_mask(mask: np.ndarray) -> np.ndarray:
    """Чистит бинарную маску от мелкого шума (open + close)."""
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    return closed


def mask_red(hsv: np.ndarray) -> np.ndarray:
    """Сегментация красного цвета в HSV (два диапазона Hue)."""
    lower_1 = np.array([0, 90, 60], dtype=np.uint8)
    upper_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_2 = np.array([170, 90, 60], dtype=np.uint8)
    upper_2 = np.array([180, 255, 255], dtype=np.uint8)

    mask_1 = cv2.inRange(hsv, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv, lower_2, upper_2)
    return clean_mask(cv2.bitwise_or(mask_1, mask_2))


def mask_color(hsv: np.ndarray, color: str) -> np.ndarray:
    """Сегментация стандартных цветов, используемых в сценах C4/C7."""
    ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
        "green": ((40, 80, 60), (85, 255, 255)),
        "blue": ((100, 90, 60), (135, 255, 255)),
        "yellow": ((20, 90, 90), (40, 255, 255)),
    }

    if color == "red":
        return mask_red(hsv)

    if color not in ranges:
        raise ValueError(f"Unsupported color: {color}")

    lower, upper = ranges[color]
    mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    return clean_mask(mask)


def count_mask_pixels(mask: np.ndarray) -> int:
    """Считает количество ненулевых пикселей в маске."""
    return int(cv2.countNonZero(mask))


def largest_bbox_area(mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
    """Находит крупнейший контур и возвращает площадь bbox + координаты (x, y, w, h)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, (0, 0, 0, 0)

    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    return int(w * h), (int(x), int(y), int(w), int(h))


def build_result(
    test_id: str,
    status: str,
    metrics: Dict[str, Any],
    artifacts: List[str],
    error: Optional[str] = None,
) -> Dict[str, Any]:
    """Стандартизированный формат результата теста для suite report."""
    result: Dict[str, Any] = {
        "id": test_id,
        "status": status,
        "metrics": metrics,
        "artifacts": artifacts,
    }
    if error:
        result["error"] = error
    return result
