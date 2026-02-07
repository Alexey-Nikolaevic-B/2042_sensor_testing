#!/usr/bin/env python3
"""
Что это за файл:
- Тест C1 (одиночный объект): проверяет, что bbox красного куба
  уменьшается с ростом дистанции X до камеры.

Как запускать:
- Как часть suite: run_camera_suite.py вызывает функцию run_test_c1(...)
- Отдельно:
  python3 catkin_ws/src/scenario_test_pkg/scripts/test_camera_c1_size_order.py

Где результаты:
- При suite-запуске: results/<camera_name>/captured_images/
- При одиночном запуске: results/<camera_name>/captured_images/

Топики:
- image_topic передается параметром (без жесткой привязки к конкретной камере)
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List

import cv2

from camera_test_common import (
    build_result,
    ensure_results_dirs,
    init_ros_node,
    largest_bbox_area,
    mask_red,
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


def run_test_c1(
    image_topic: str,
    camera_sdf_file: str,
    camera_model_name: str,
    captured_dir: Path,
    namespace: str = "",
    move_cube_positions: Iterable[float] = (1.0, 3.0, 5.0),
    settle_s: float = 0.8,
    min_margin_ratio: float = 1.10,
) -> Dict[str, Any]:
    """Запускает C1 и возвращает результат в формате suite-report."""
    artifacts: List[str] = []
    positions = list(move_cube_positions)

    metrics: Dict[str, Any] = {
        "bbox_area_px": {},
        "move_cube_positions": positions,
        "min_margin_ratio": min_margin_ratio,
    }

    proc = None
    try:
        proc = start_scene_launch(
            launch_file="scene_c1.launch",
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            image_topic=image_topic,
            namespace=namespace,
        )

        init_ros_node("camera_suite_runner")
        wait_for_gazebo_ready(timeout_s=45.0)

        # Для C1 используем один и тот же world и двигаем один test_cube по X.
        for x in positions:
            label = f"x{int(x) if float(x).is_integer() else x}"

            # Ключевой шаг методики C1: изменить дистанцию объекта до камеры.
            move_model_and_wait("test_cube", x=x, y=0.0, z=0.25, settle_s=settle_s)

            # После стабилизации сцены снимаем кадр.
            msg = wait_for_image(topic=image_topic, timeout_s=35.0)
            frame_bgr = ros_image_to_bgr(msg)

            # Сегментация красного объекта и расчет bbox-площади в пикселях.
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            red_mask = mask_red(hsv)
            area, (bx, by, bw, bh) = largest_bbox_area(red_mask)
            metrics["bbox_area_px"][label] = int(area)

            debug_frame = frame_bgr.copy()
            if area > 0:
                cv2.rectangle(debug_frame, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
                cv2.putText(
                    debug_frame,
                    f"{label}: bbox={area}",
                    (max(8, bx), max(22, by - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            image_path = save_image(debug_frame, captured_dir, f"c1_{label}.png")
            artifacts.append(str(image_path))

        size_x1 = metrics["bbox_area_px"].get("x1", 0)
        size_x3 = metrics["bbox_area_px"].get("x3", 0)
        size_x5 = metrics["bbox_area_px"].get("x5", 0)

        order_ok = size_x1 > size_x3 > size_x5
        margin_ok = (size_x1 >= size_x3 * min_margin_ratio) and (size_x3 >= size_x5 * min_margin_ratio)

        metrics["checks"] = {
            "size_order": bool(order_ok),
            "size_margin": bool(margin_ok),
        }

        status = "PASS" if (order_ok and margin_ok) else "FAIL"
        return build_result(test_id="C1", status=status, metrics=metrics, artifacts=artifacts)

    except Exception as exc:  # noqa: BLE001
        return build_result(
            test_id="C1",
            status="FAIL",
            metrics=metrics,
            artifacts=artifacts,
            error=str(exc),
        )
    finally:
        stop_launch(proc)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run C1 camera test.")
    parser.add_argument("--camera-name", default="uvc_profile_640x480_60deg")
    parser.add_argument("--image-topic", default="/uvc_profile_640x480_60deg/image_raw")
    parser.add_argument("--camera-sdf-file", default=str(DEFAULT_SDF))
    parser.add_argument("--camera-model-name", default="uvc_profile_640x480_60deg_model")
    parser.add_argument("--namespace", default="")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _, captured_dir = ensure_results_dirs(args.camera_name)

    result = run_test_c1(
        image_topic=args.image_topic,
        camera_sdf_file=args.camera_sdf_file,
        camera_model_name=args.camera_model_name,
        captured_dir=captured_dir,
        namespace=args.namespace,
    )

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
