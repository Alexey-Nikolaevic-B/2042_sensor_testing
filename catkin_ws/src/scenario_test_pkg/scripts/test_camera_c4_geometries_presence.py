#!/usr/bin/env python3
"""
Что это за файл:
- Тест C4 (разная геометрия): проверяет присутствие 4 объектов по цвету
  в одном кадре сцены.

Как запускать:
- Как часть suite: run_camera_suite.py вызывает функцию run_test_c4(...)
- Отдельно:
  python3 catkin_ws/src/scenario_test_pkg/scripts/test_camera_c4_geometries_presence.py

Где результаты:
- results/<camera_name>/captured_images/

Топики:
- image_topic передается параметром
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import cv2

from camera_test_common import (
    build_result,
    count_mask_pixels,
    ensure_results_dirs,
    init_ros_node,
    mask_color,
    ros_image_to_bgr,
    save_image,
    start_scene_launch,
    stop_launch,
    wait_for_gazebo_ready,
    wait_for_image,
)


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SDF = PROJECT_ROOT / "resources" / "sensors" / "camera" / "uvc_profile_640x480_60deg.sdf"


def run_test_c4(
    image_topic: str,
    camera_sdf_file: str,
    camera_model_name: str,
    captured_dir: Path,
    namespace: str = "",
    pixel_threshold: int = 1500,
) -> Dict[str, Any]:
    """Запускает C4 и возвращает результат в формате suite-report."""
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {"pixel_counts": {}, "threshold": int(pixel_threshold)}

    proc = None
    try:
        proc = start_scene_launch(
            launch_file="scene_c4.launch",
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            image_topic=image_topic,
            namespace=namespace,
        )

        init_ros_node("camera_suite_runner")
        wait_for_gazebo_ready(timeout_s=45.0)

        msg = wait_for_image(topic=image_topic, timeout_s=35.0)
        frame_bgr = ros_image_to_bgr(msg)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        for color in ("red", "green", "blue", "yellow"):
            # Для каждого эталонного цвета строим маску и считаем число пикселей.
            mask = mask_color(hsv, color)
            metrics["pixel_counts"][color] = count_mask_pixels(mask)

        image_path = save_image(frame_bgr, captured_dir, "c4_geometries.png")
        artifacts.append(str(image_path))

        missing = {
            c: n for c, n in metrics["pixel_counts"].items() if int(n) <= int(pixel_threshold)
        }
        metrics["checks"] = {
            "all_colors_present": len(missing) == 0,
            "missing_or_low": missing,
        }

        status = "PASS" if len(missing) == 0 else "FAIL"
        return build_result(test_id="C4", status=status, metrics=metrics, artifacts=artifacts)

    except Exception as exc:  # noqa: BLE001
        return build_result(
            test_id="C4",
            status="FAIL",
            metrics=metrics,
            artifacts=artifacts,
            error=str(exc),
        )
    finally:
        stop_launch(proc)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run C4 camera test.")
    parser.add_argument("--camera-name", default="uvc_profile_640x480_60deg")
    parser.add_argument("--image-topic", default="/uvc_profile_640x480_60deg/image_raw")
    parser.add_argument("--camera-sdf-file", default=str(DEFAULT_SDF))
    parser.add_argument("--camera-model-name", default="uvc_profile_640x480_60deg_model")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--pixel-threshold", type=int, default=1500)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _, captured_dir = ensure_results_dirs(args.camera_name)

    result = run_test_c4(
        image_topic=args.image_topic,
        camera_sdf_file=args.camera_sdf_file,
        camera_model_name=args.camera_model_name,
        captured_dir=captured_dir,
        namespace=args.namespace,
        pixel_threshold=args.pixel_threshold,
    )

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
