import json
import os
from datetime import datetime, timezone
from typing import Any, Dict, List, Tuple

from .gazebo_simulator import Simulator
from .sensor_library_utils import get_sensor_types, get_sensors_name_by_type
from .test_utils import load_test_functions
from .sensors import Sensor, make_sensor

from config import CONFIG


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)

    def prepare_simulator(self) -> None:
        self.simulator.launch()

    def get_sensor_types(self) -> List[str]:
        return get_sensor_types()

    def get_sensors_by_type(self, sensor_type: str) -> List[str]:
        return get_sensors_name_by_type(sensor_type)

    def get_sensors_name_by_type(self, sensor_type: str) -> List[str]:
        return get_sensors_name_by_type(sensor_type)

    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        return load_test_functions(sensor)

    def capture_data(self, sensor: Sensor, world_path, **kwargs) -> Any:
        return sensor.capture_data(self.simulator, world_path=world_path, **kwargs)

    def shutdown(self) -> None:
        try:
            self.simulator.kill_gazebo()
        except Exception:
            pass
        for method_name in ("_kill_node", "_kill_ros"):
            method = getattr(self.simulator, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass

    def kill(self) -> None:
        self.simulator.kill()

    @staticmethod
    def _report_dir(sensor_name: str) -> str:
        path = os.path.join(CONFIG["ROOT_PATH"], "results", sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    @staticmethod
    def _suite_report_dir(sensor_type: str) -> str:
        path = os.path.join(CONFIG["ROOT_PATH"], "results", f"{sensor_type}_suite")
        os.makedirs(path, exist_ok=True)
        return path

    @staticmethod
    def _safe_json(value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, dict):
            return {str(k): Core._safe_json(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [Core._safe_json(v) for v in value]
        return str(value)

    def run_sensor_tests(
        self,
        sensor_type: str,
        sensor_name: str,
        test_name: str = "",
        suite: bool = False,
    ) -> Tuple[str, str, Dict[str, Any]]:
        report_dir = self._report_dir(sensor_name)
        sensor = make_sensor(sensor_type, sensor_name, CONFIG)
        if sensor is None:
            payload = self._fail_payload(
                sensor_type,
                sensor_name,
                "suite" if suite else "single",
                f"Sensor not found: type={sensor_type}, name={sensor_name}",
            )
            return self._write_report(payload, report_dir)

        discovered = load_test_functions(sensor)
        if not discovered:
            payload = self._fail_payload(sensor_type, sensor_name, "suite", "No tests found")
            return self._write_report(payload, report_dir)

        run_suite = suite or test_name == "suite"
        if run_suite or not test_name:
            tests = dict(sorted(discovered.items()))
            selected_mode = "suite"
        else:
            tests = {name: fn for name, fn in discovered.items() if name == test_name}
            selected_mode = "single"

        if not tests:
            payload = self._fail_payload(sensor_type, sensor_name, "single", f"Test not found: {test_name}")
            return self._write_report(payload, report_dir)

        test_results: List[Dict[str, Any]] = []
        for test_case_name, test_case_fn in tests.items():
            try:
                result = test_case_fn(self.simulator)
                test_results.append({"name": test_case_name, "status": "PASS", "result": result})
            except Exception as exc:
                entry: Dict[str, Any] = {"name": test_case_name, "status": "FAIL", "error": str(exc)}
                details = self._collect_test_diagnostics(sensor, report_dir, test_case_name)
                if details:
                    entry["error_details"] = self._safe_json(details)
                test_results.append(entry)

        overall = "PASS" if all(item["status"] == "PASS" for item in test_results) else "FAIL"
        payload = {
            "sensor_type": sensor_type,
            "sensor_name": sensor_name,
            "run_mode": selected_mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": test_results,
            "overall": overall,
        }
        return self._write_report(payload, report_dir)

    def run_sensor_suite(
        self,
        sensor_type: str,
        sensor_name: str = "",
        test_name: str = "",
        suite: bool = False,
        all_sensors: bool = False,
    ) -> int:
        if all_sensors:
            sensor_names = sorted(self.get_sensors_name_by_type(sensor_type))
            if not sensor_names:
                print(f"No sensors found for sensor-type={sensor_type}")
                return 2
        else:
            sensor_names = [sensor_name]

        suite_entries: List[Dict[str, Any]] = []
        suite_overall = "PASS"

        for current_sensor_name in sensor_names:
            overall, report_path, payload = self.run_sensor_tests(
                sensor_type,
                current_sensor_name,
                test_name=test_name,
                suite=suite,
            )
            suite_entries.append(
                {
                    "sensor_name": current_sensor_name,
                    "overall": overall,
                    "report": report_path,
                    "tests_count": len(payload.get("tests", [])),
                }
            )
            if overall != "PASS":
                suite_overall = "FAIL"

        if all_sensors:
            summary = {
                "sensor_type": sensor_type,
                "run_mode": "all_sensors",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "sensors": suite_entries,
                "overall": suite_overall,
            }
            summary_dir = self._suite_report_dir(sensor_type)
            summary_path = os.path.join(summary_dir, "report.json")
            with open(summary_path, "w", encoding="utf-8") as output_file:
                json.dump(summary, output_file, ensure_ascii=False, indent=2)
            print(f"overall={suite_overall}")
            print(f"report={summary_path}")
            return 0 if suite_overall == "PASS" else 1

        single = suite_entries[0]
        print(f"overall={single['overall']}")
        print(f"report={single['report']}")
        return 0 if single["overall"] == "PASS" else 1

    @staticmethod
    def _fail_payload(sensor_type: str, sensor_name: str, mode: str, error: str) -> Dict[str, Any]:
        return {
            "sensor_type": sensor_type,
            "sensor_name": sensor_name,
            "run_mode": mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tests": [],
            "overall": "FAIL",
            "error": error,
        }

    @staticmethod
    def _write_report(payload: Dict[str, Any], report_dir: str) -> Tuple[str, str, Dict[str, Any]]:
        report_path = os.path.join(report_dir, "report.json")
        with open(report_path, "w", encoding="utf-8") as output_file:
            json.dump(payload, output_file, ensure_ascii=False, indent=2)
        return payload["overall"], report_path, payload

    def _collect_test_diagnostics(self, sensor: Sensor, report_dir: str, test_name: str) -> Dict[str, Any]:
        details: Dict[str, Any] = {}
        simulator = self.simulator

        expected_topics: List[str] = []
        getter = getattr(sensor, "get_expected_topics", None)
        if callable(getter):
            try:
                expected_topics = [str(topic) for topic in getter() if str(topic).strip()]
            except Exception:
                expected_topics = []

        scene_diag_getter = getattr(simulator, "get_last_scene_diagnostics", None)
        if callable(scene_diag_getter):
            try:
                diagnostics = scene_diag_getter()
                if diagnostics:
                    details["scene"] = diagnostics
            except Exception:
                pass

        runtime_getter = getattr(simulator, "collect_failure_diagnostics", None)
        if callable(runtime_getter):
            try:
                runtime = runtime_getter(expected_topics=expected_topics)
                if runtime:
                    details["runtime"] = runtime
            except Exception:
                pass

        if expected_topics:
            capture_fn = getattr(simulator, "capture_debug_frame", None)
            if callable(capture_fn):
                try:
                    debug_path = os.path.join(report_dir, f"{test_name}_debug.png")
                    details["debug_frame"] = capture_fn(expected_topics[0], debug_path, timeout_s=3.0)
                except Exception:
                    pass

        sensor_diag_getter = getattr(sensor, "get_last_test_diagnostics", None)
        if callable(sensor_diag_getter):
            try:
                sensor_diag = sensor_diag_getter()
                if sensor_diag:
                    details["sensor"] = sensor_diag
            except Exception:
                pass

        return details
