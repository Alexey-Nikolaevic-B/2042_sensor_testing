#!/usr/bin/env python3
"""
Что это за файл:
- Тест C7 (частичное перекрытие): сравнивает видимую площадь синего back cube
  при двух смещениях по Y (условно 25% и 50% перекрытия).

Как запускать:
- Как часть suite: run_camera_suite.py вызывает функцию run_test_c7(...)
- Отдельно:
  python3 catkin_ws/src/scenario_test_pkg/scripts/test_camera_c7_occlusion.py

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
    move_model_and_wait,
    ros_image_to_bgr,
    save_image,
    start_scene_launch,
    stop_launch,
    wait_for_gazebo_ready,
    wait_for_image,
)


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SDF = PROJECT_ROOT / "resources" / "sensors" / "camera" / "uvc_profile_640x480_60deg.sdf"


def run_test_c7(
    image_topic: str,
    camera_sdf_file: str,
    camera_model_name: str,
    captured_dir: Path,
    namespace: str = "",
    settle_s: float = 0.8,
    pixel_threshold: int = 800,
) -> Dict[str, Any]:
    """Запускает C7 и возвращает результат в формате suite-report."""
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "blue_pixels": {},
        "cases": {"occ_25": 0.10, "occ_50": 0.20},
        "threshold": int(pixel_threshold),
    }

    proc = None
    try:
        proc = start_scene_launch(
            launch_file="scene_c7.launch",
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            image_topic=image_topic,
            namespace=namespace,
        )

        init_ros_node("camera_suite_runner")
        wait_for_gazebo_ready(timeout_s=45.0)

        # Обеспечиваем эталонную позицию фронт-куба перед серией измерений.
        move_model_and_wait("front_cube", x=3.0, y=0.0, z=0.25, settle_s=settle_s)

        for case_name, back_cube_y in (("occ_25", 0.10), ("occ_50", 0.20)):
            # Ключевой шаг C7: меняем перекрытие через смещение back cube по Y.
            move_model_and_wait("back_cube", x=3.0, y=back_cube_y, z=0.25, settle_s=settle_s)

            msg = wait_for_image(topic=image_topic, timeout_s=35.0)
            frame_bgr = ros_image_to_bgr(msg)
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

            blue_mask = mask_color(hsv, "blue")
            blue_count = count_mask_pixels(blue_mask)
            metrics["blue_pixels"][case_name] = int(blue_count)

            debug = frame_bgr.copy()
            cv2.putText(
                debug,
                f"{case_name}: blue_pixels={blue_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            image_path = save_image(debug, captured_dir, f"c7_{case_name}.png")
            artifacts.append(str(image_path))

        blue_25 = metrics["blue_pixels"].get("occ_25", 0)
        blue_50 = metrics["blue_pixels"].get("occ_50", 0)

        relation_ok = blue_25 > blue_50
        threshold_ok = blue_25 > pixel_threshold and blue_50 > pixel_threshold

        metrics["checks"] = {
            "occlusion_relation": bool(relation_ok),
            "threshold_ok": bool(threshold_ok),
        }

        status = "PASS" if (relation_ok and threshold_ok) else "FAIL"
        return build_result(test_id="C7", status=status, metrics=metrics, artifacts=artifacts)

    except Exception as exc:  # noqa: BLE001
        return build_result(
            test_id="C7",
            status="FAIL",
            metrics=metrics,
            artifacts=artifacts,
            error=str(exc),
        )
    finally:
        stop_launch(proc)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run C7 camera test.")
    parser.add_argument("--camera-name", default="uvc_profile_640x480_60deg")
    parser.add_argument("--image-topic", default="/uvc_profile_640x480_60deg/image_raw")
    parser.add_argument("--camera-sdf-file", default=str(DEFAULT_SDF))
    parser.add_argument("--camera-model-name", default="uvc_profile_640x480_60deg_model")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--pixel-threshold", type=int, default=800)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _, captured_dir = ensure_results_dirs(args.camera_name)

    result = run_test_c7(
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
