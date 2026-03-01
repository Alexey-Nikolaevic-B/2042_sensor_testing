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
from typing import Any, Dict, List

from config import CONFIG
from src.core import Core
from src.sensors import make_sensor
from src.test_utils import load_test_functions


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run sensor tests discovered by *_test suffix")
    parser.add_argument("--sensor-type", required=True, help="Sensor type, e.g. camera, rfid")
    parser.add_argument(
        "--sensor-name",
        required=True,
        help="Registered sensor name (for cameras this is profile-id, e.g. uvc_profile_640x480_60deg)",
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
    return parser.parse_args()


def _report_dir(sensor_name: str) -> str:
    path = os.path.join(CONFIG["ROOT_PATH"], "results", sensor_name)
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


def main() -> int:
    args = _parse_args()

    core = Core()
    core.prepare_simulator()

    try:
        sensor = make_sensor(args.sensor_type, args.sensor_name, CONFIG)
        if sensor is None:
            print(f"Sensor not found: type={args.sensor_type}, name={args.sensor_name}")
            return 2

        discovered_tests = load_test_functions(sensor)

        if not discovered_tests:
            print("No tests found")
            return 3

        # Suite semantics:
        # - no flags: run all discovered *_test
        # - --suite or --test suite: run all discovered *_test
        # - --test <name>: run only one test
        run_suite = args.suite or args.test == "suite"
        if run_suite or not args.test:
            tests = dict(sorted(discovered_tests.items(), key=lambda item: item[0]))
            selected_mode = "suite"
        else:
            tests = {name: fn for name, fn in discovered_tests.items() if name == args.test}
            selected_mode = "single"

        if not tests:
            print(f"Test not found: {args.test}")
            return 3

        test_results: List[Dict[str, Any]] = []
        for test_name, test_fn in tests.items():
            try:
                result = test_fn(core.simulator)
                test_results.append({"name": test_name, "status": "PASS", "result": result})
            except Exception as exc:  # noqa: BLE001
                entry: Dict[str, Any] = {"name": test_name, "status": "FAIL", "error": str(exc)}

                details: Dict[str, Any] = {}
                sim = core.simulator
                if hasattr(sim, "get_last_scene_diagnostics") and callable(sim.get_last_scene_diagnostics):
                    try:
                        diag = sim.get_last_scene_diagnostics()
                        if diag:
                            details["scene"] = diag
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
            "sensor_type": args.sensor_type,
            "sensor_name": args.sensor_name,
            "run_mode": selected_mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": test_results,
            "overall": overall,
        }

        report_path = os.path.join(_report_dir(args.sensor_name), "report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        print(f"overall={overall}")
        print(f"report={report_path}")

        return 0 if overall == "PASS" else 1

    finally:
        _shutdown_core(core)


if __name__ == "__main__":
    raise SystemExit(main())
