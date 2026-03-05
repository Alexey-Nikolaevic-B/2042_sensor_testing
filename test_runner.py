#!/usr/bin/env python3
"""
CLI runner тестов сенсоров по конвенции проекта.

Логика:
- создает сенсор через реестр (`make_sensor`),
- находит все callables с суффиксом `_test` у сенсора,
- запускает один тест (`--test <name>`) или полный suite (`--suite` / `--test suite`),
- сохраняет сводный JSON-отчет в results/<sensor_name>/report.json.
"""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from typing import Any, Dict, List, Tuple

from config import CONFIG
from src.core import Core
from src.sensors import make_sensor
from src.test_utils import load_test_functions


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run sensor tests discovered by *_test suffix")
    parser.add_argument("--sensor-type", required=True, help="Sensor type, e.g. camera, rfid")
    parser.add_argument(
        "--sensor-name",
        "--sensor",
        dest="sensor_name",
        default="",
        help="Registered sensor name (for cameras this is profile-id, e.g. uvc_profile_640x480_60deg)",
    )
    parser.add_argument(
        "--all-sensors",
        action="store_true",
        help="Run selected test/suite for all sensors of --sensor-type",
    )
    parser.add_argument(
        "--test",
        default="",
        help="Single test method name ending with _test, or 'suite' to run all discovered *_test",
    )
    parser.add_argument(
        "--suite",
        action="store_true",
        help="Run all discovered *_test for the selected sensor (alias of --test suite)",
    )
    args = parser.parse_args()
    if not args.all_sensors and not str(args.sensor_name).strip():
        parser.error("Specify --sensor-name/--sensor or use --all-sensors")
    return args


def _report_dir(sensor_name: str) -> str:
    path = os.path.join(CONFIG["ROOT_PATH"], "results", sensor_name)
    os.makedirs(path, exist_ok=True)
    return path


def _suite_report_dir(sensor_type: str) -> str:
    path = os.path.join(CONFIG["ROOT_PATH"], "results", f"{sensor_type}_suite")
    os.makedirs(path, exist_ok=True)
    return path


def _shutdown_core(core: Core) -> None:
    """Завершает процессы симулятора без sys.exit из Core.kill()."""
    sim = core.simulator
    try:
        sim.kill_gazebo()
    except Exception:
        pass

    # В проекте используются приватные методы для остановки ROS/node.
    # Здесь это безопасный best-effort shutdown для CLI runner.
    for method_name in ("_kill_node", "_kill_ros"):
        method = getattr(sim, method_name, None)
        if callable(method):
            try:
                method()
            except Exception:
                pass


def _safe_json(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_json(v) for v in value]
    return str(value)


def _run_sensor_tests(core: Core, sensor_type: str, sensor_name: str, args: argparse.Namespace) -> Tuple[str, str, Dict[str, Any]]:
    report_dir = _report_dir(sensor_name)
    sensor = make_sensor(sensor_type, sensor_name, CONFIG)
    if sensor is None:
        payload = {
            "sensor_type": sensor_type,
            "sensor_name": sensor_name,
            "run_mode": "suite" if (args.suite or args.test == "suite" or not args.test) else "single",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": [],
            "overall": "FAIL",
            "error": f"Sensor not found: type={sensor_type}, name={sensor_name}",
        }
        report_path = os.path.join(report_dir, "report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return "FAIL", report_path, payload

    discovered_tests = load_test_functions(sensor)
    if not discovered_tests:
        payload = {
            "sensor_type": sensor_type,
            "sensor_name": sensor_name,
            "run_mode": "suite",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": [],
            "overall": "FAIL",
            "error": "No tests found",
        }
        report_path = os.path.join(report_dir, "report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return "FAIL", report_path, payload

    run_suite = args.suite or args.test == "suite"
    if run_suite or not args.test:
        tests = dict(sorted(discovered_tests.items(), key=lambda item: item[0]))
        selected_mode = "suite"
    else:
        tests = {name: fn for name, fn in discovered_tests.items() if name == args.test}
        selected_mode = "single"

    if not tests:
        payload = {
            "sensor_type": sensor_type,
            "sensor_name": sensor_name,
            "run_mode": "single",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": [],
            "overall": "FAIL",
            "error": f"Test not found: {args.test}",
        }
        report_path = os.path.join(report_dir, "report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return "FAIL", report_path, payload

    test_results: List[Dict[str, Any]] = []
    for test_name, test_fn in tests.items():
        try:
            result = test_fn(core.simulator)
            test_results.append({"name": test_name, "status": "PASS", "result": result})
        except Exception as exc:  # noqa: BLE001
            entry: Dict[str, Any] = {"name": test_name, "status": "FAIL", "error": str(exc)}

            details: Dict[str, Any] = {}
            sim = core.simulator
            expected_topics: List[str] = []
            sensor_topics_getter = getattr(sensor, "get_expected_topics", None)
            if callable(sensor_topics_getter):
                try:
                    expected_topics = [str(topic) for topic in sensor_topics_getter() if str(topic).strip()]
                except Exception:  # noqa: BLE001
                    expected_topics = []

            if hasattr(sim, "get_last_scene_diagnostics") and callable(sim.get_last_scene_diagnostics):
                try:
                    diag = sim.get_last_scene_diagnostics()
                    if diag:
                        details["scene"] = diag
                except Exception:  # noqa: BLE001
                    pass

            runtime_diag_getter = getattr(sim, "collect_failure_diagnostics", None)
            if callable(runtime_diag_getter):
                try:
                    runtime_diag = runtime_diag_getter(expected_topics=expected_topics)
                    if runtime_diag:
                        details["runtime"] = runtime_diag
                except Exception:  # noqa: BLE001
                    pass

            if expected_topics and hasattr(sim, "capture_debug_frame") and callable(sim.capture_debug_frame):
                try:
                    debug_path = os.path.join(report_dir, f"{test_name}_debug.png")
                    details["debug_frame"] = sim.capture_debug_frame(expected_topics[0], debug_path, timeout_s=3.0)
                except Exception:  # noqa: BLE001
                    pass

            sensor_diag_getter = getattr(sensor, "get_last_test_diagnostics", None)
            if callable(sensor_diag_getter):
                try:
                    sensor_diag = sensor_diag_getter()
                    if sensor_diag:
                        details["sensor"] = sensor_diag
                except Exception:  # noqa: BLE001
                    pass

            if details:
                entry["error_details"] = _safe_json(details)
            test_results.append(entry)

    overall = "PASS" if all(t["status"] == "PASS" for t in test_results) else "FAIL"
    payload = {
        "sensor_type": sensor_type,
        "sensor_name": sensor_name,
        "run_mode": selected_mode,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "tests": test_results,
        "overall": overall,
    }

    report_path = os.path.join(report_dir, "report.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return overall, report_path, payload


def main() -> int:
    args = _parse_args()

    core = Core()
    core.prepare_simulator()

    try:
        if args.all_sensors:
            sensor_names = sorted(core.get_sensors_name_by_type(args.sensor_type))
            if not sensor_names:
                print(f"No sensors found for sensor-type={args.sensor_type}")
                return 2
        else:
            sensor_names = [str(args.sensor_name).strip()]

        suite_entries: List[Dict[str, Any]] = []
        suite_overall = "PASS"

        for sensor_name in sensor_names:
            overall, report_path, payload = _run_sensor_tests(core, args.sensor_type, sensor_name, args)
            suite_entries.append(
                {
                    "sensor_name": sensor_name,
                    "overall": overall,
                    "report": report_path,
                    "tests_count": len(payload.get("tests", [])),
                }
            )
            if overall != "PASS":
                suite_overall = "FAIL"

        if args.all_sensors:
            summary_payload = {
                "sensor_type": args.sensor_type,
                "run_mode": "all_sensors",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "sensors": suite_entries,
                "overall": suite_overall,
            }
            summary_dir = _suite_report_dir(args.sensor_type)
            summary_report_path = os.path.join(summary_dir, "report.json")
            with open(summary_report_path, "w", encoding="utf-8") as f:
                json.dump(summary_payload, f, ensure_ascii=False, indent=2)

            print(f"overall={suite_overall}")
            print(f"report={summary_report_path}")
            return 0 if suite_overall == "PASS" else 1

        single = suite_entries[0]
        print(f"overall={single['overall']}")
        print(f"report={single['report']}")
        return 0 if single["overall"] == "PASS" else 1

    finally:
        _shutdown_core(core)


if __name__ == "__main__":
    raise SystemExit(main())
