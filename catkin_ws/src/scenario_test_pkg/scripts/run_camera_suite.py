#!/usr/bin/env python3
"""
Что это за файл:
- Единая точка запуска camera test suite (C1, C4, C7) для выбранной камеры.
- Последовательно запускает тесты, собирает артефакты и сохраняет единый отчёт.

Как запускать:
- python3 catkin_ws/src/scenario_test_pkg/scripts/run_camera_suite.py --camera uvc_profile_640x480_60deg

Где результаты:
- results/<camera_name>/report.json
- results/<camera_name>/report.csv
- results/<camera_name>/captured_images/

Зависимости:
- Рабочая ROS/Gazebo среда (roslaunch, gazebo_ros, rospy)
- Зарегистрированный sensor profile типа `camera` в src/sensors/
"""

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple


SCRIPTS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[4]

# Добавляем корень проекта в sys.path, чтобы импортировать текущие модули проекта (config, src.sensors).
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from camera_test_common import ensure_results_dirs  # noqa: E402
from config import CONFIG  # noqa: E402
from src.sensors import make_sensor  # noqa: E402
from test_camera_c1_size_order import run_test_c1  # noqa: E402
from test_camera_c4_geometries_presence import run_test_c4  # noqa: E402
from test_camera_c7_occlusion import run_test_c7  # noqa: E402


def _resolve_camera_runtime(camera_name: str) -> Tuple[str, str, str]:
    """
    Получает runtime-параметры камеры из существующего реестра сенсоров проекта:
    - camera_sdf_file
    - camera_model_name
    - image_topic
    """
    sensor = make_sensor("camera", camera_name, CONFIG)
    if sensor is None:
        raise ValueError(f"Camera '{camera_name}' is not registered in sensor registry")

    sensor_sdf_path = Path(sensor.sensor_sdf_path)
    if not sensor_sdf_path.is_absolute():
        sensor_sdf_path = PROJECT_ROOT / sensor_sdf_path
    sensor_sdf_path = sensor_sdf_path.resolve()

    image_topic = getattr(sensor, "IMAGE_TOPIC", None)
    if not image_topic:
        raise ValueError(f"Camera '{camera_name}' does not define IMAGE_TOPIC")

    camera_model_name = getattr(sensor, "CAMERA_MODEL_NAME", f"{camera_name}_model")

    if not sensor_sdf_path.exists():
        raise FileNotFoundError(f"Camera SDF not found: {sensor_sdf_path}")

    return str(sensor_sdf_path), str(camera_model_name), str(image_topic)


def _write_report_json(report_path: Path, payload: Dict[str, Any]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")


def _write_report_csv(report_path: Path, tests: List[Dict[str, Any]]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["id", "status", "metrics_json", "artifacts_count", "error"],
        )
        writer.writeheader()

        for item in tests:
            writer.writerow(
                {
                    "id": item.get("id", ""),
                    "status": item.get("status", ""),
                    "metrics_json": json.dumps(item.get("metrics", {}), ensure_ascii=False),
                    "artifacts_count": len(item.get("artifacts", [])),
                    "error": item.get("error", ""),
                }
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run camera suite (C1/C4/C7) for one camera profile.")
    parser.add_argument("--camera", required=True, help="Registered camera sensor name, e.g. uvc_profile_640x480_60deg")
    parser.add_argument("--namespace", default="", help="Optional ROS namespace passed to scene launch files")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    camera_name = args.camera
    namespace = args.namespace

    try:
        camera_sdf_file, camera_model_name, image_topic = _resolve_camera_runtime(camera_name)
    except Exception as exc:  # noqa: BLE001
        print(f"[suite] failed to resolve camera '{camera_name}': {exc}")
        return 2
    suite_dir, captured_dir = ensure_results_dirs(camera_name)

    tests: List[Dict[str, Any]] = []

    tests.append(
        run_test_c1(
            image_topic=image_topic,
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            captured_dir=captured_dir,
            namespace=namespace,
        )
    )

    tests.append(
        run_test_c4(
            image_topic=image_topic,
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            captured_dir=captured_dir,
            namespace=namespace,
        )
    )

    tests.append(
        run_test_c7(
            image_topic=image_topic,
            camera_sdf_file=camera_sdf_file,
            camera_model_name=camera_model_name,
            captured_dir=captured_dir,
            namespace=namespace,
        )
    )

    overall = "PASS" if all(t.get("status") == "PASS" for t in tests) else "FAIL"
    payload: Dict[str, Any] = {
        "camera": camera_name,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "tests": tests,
        "overall": overall,
    }

    json_path = suite_dir / "report.json"
    csv_path = suite_dir / "report.csv"

    _write_report_json(json_path, payload)
    _write_report_csv(csv_path, tests)

    print(f"[suite] camera={camera_name}")
    print(f"[suite] image_topic={image_topic}")
    print(f"[suite] overall={overall}")
    print(f"[suite] report.json={json_path}")
    print(f"[suite] report.csv={csv_path}")
    print(f"[suite] captured_images={captured_dir}")

    return 0 if overall == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
