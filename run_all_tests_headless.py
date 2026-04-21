#!/usr/bin/env python3
"""
Headless test runner for all sensors in the database.

Usage:
    python run_all_tests_headless.py [--sensor NAME] [--test FUNC_NAME] [--verbose]
"""

import argparse
import logging
import logging.config
import socket
import sys
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).parent))

from config import CONFIG
from src.core import Core
from src.sensor import Sensor
import src.sensor_storage as db


# ----------------------------------------------------------------------
# Logging setup – capture EVERYTHING to file
# ----------------------------------------------------------------------
def setup_logging(verbose: bool = False) -> None:
    """
    Configure root logger to capture all log messages from all modules.
    Console: INFO (or DEBUG if --verbose). File: DEBUG always.
    """
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)  # Capture all levels

    # Remove all existing handlers (to avoid duplication)
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_level = logging.DEBUG if verbose else logging.INFO
    console_handler.setLevel(console_level)
    console_fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    console_handler.setFormatter(logging.Formatter(console_fmt, datefmt="%H:%M:%S"))
    root_logger.addHandler(console_handler)

    # File handler – detailed, always DEBUG
    file_handler = logging.FileHandler("headless_test_runner.log", mode="w")
    file_handler.setLevel(logging.DEBUG)
    file_fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    file_handler.setFormatter(logging.Formatter(file_fmt, datefmt="%Y-%m-%d %H:%M:%S"))
    root_logger.addHandler(file_handler)

    # Prevent other modules (e.g. rospy) from adding their own handlers
    logging.getLogger("rospy").propagate = True
    logging.getLogger("rosout").propagate = True


logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
# Helper: wait for roscore port
# ----------------------------------------------------------------------
def wait_for_roscore(timeout: float = 30.0) -> bool:
    """Wait until roscore is accepting connections on port 11311."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", 11311), timeout=1.0):
                logger.debug("roscore port is open")
                return True
        except (socket.error, OSError):
            time.sleep(0.5)
    return False


# ----------------------------------------------------------------------
# Main test execution
# ----------------------------------------------------------------------
def run_all_tests(
    sensor_filter: Optional[str] = None,
    test_filter: Optional[str] = None,
) -> Tuple[Dict[str, int], List[Dict[str, str]]]:
    """
    Execute tests and return summary counts plus detailed results list.
    """
    db.init_db()
    core = Core()

    # Fetch sensors
    all_sensors = db.get_all_sensors()
    if sensor_filter:
        all_sensors = [s for s in all_sensors if s["name"] == sensor_filter]
        if not all_sensors:
            logger.error(f"Sensor '{sensor_filter}' not found.")
            return {"total": 0, "passed": 0, "failed": 0}, []

    logger.info(f"Found {len(all_sensors)} sensor(s) to process.")

    # Start ROS core
    logger.info("Starting ROS core...")
    core.simulator.launch_ros()
    if not wait_for_roscore(timeout=30.0):
        logger.error("roscore did not start within 30 seconds.")
        return {"total": 0, "passed": 0, "failed": 0}, []
    logger.info("roscore ready.")

    # Initialize ROS node
    logger.info("Initializing ROS node...")
    core.simulator.launch_node()
    if not core.simulator.node_is_running:
        logger.error("ROS node failed to initialize.")
        return {"total": 0, "passed": 0, "failed": 0}, []
    logger.info("ROS node ready.")

    total_tests = 0
    passed = 0
    failed = 0
    results_list: List[Dict[str, str]] = []

    try:
        for sensor_dict in all_sensors:
            sensor_name = sensor_dict["name"]
            sensor_type = sensor_dict["type"]
            sdf_path = sensor_dict["sdf_path"]
            sensor_id = sensor_dict["id"]

            logger.info("=" * 60)
            logger.info(f"Sensor: {sensor_name} (type={sensor_type}, id={sensor_id})")
            logger.info(f"SDF path: {sdf_path}")

            sensor_obj = Sensor(
                sensor_type=sensor_type,
                sensor_name=sensor_name,
                sdf_path=sdf_path,
                topics=sensor_dict.get("topics", []),
                description=sensor_dict.get("description", ""),
                image_path=sensor_dict.get("image_path", ""),
                params=sensor_dict.get("params", {}),
            )

            tests = core.get_tests(sensor_obj)
            if not tests:
                logger.warning(f"No tests for type '{sensor_type}'")
                continue

            # Remove __basic__ test (not a real test)
            if "__basic__" in tests:
                del tests["__basic__"]
                logger.debug("Skipped __basic__ test")

            if test_filter:
                if test_filter in tests:
                    tests = {test_filter: tests[test_filter]}
                else:
                    logger.warning(
                        f"Test '{test_filter}' not found for type '{sensor_type}', skipping sensor."
                    )
                    continue

            if not tests:
                logger.info(f"No tests to run for sensor '{sensor_name}' after filtering.")
                continue

            logger.info(f"Will run {len(tests)} test(s): {', '.join(tests.keys())}")

            for func_name, test_func in tests.items():
                total_tests += 1
                logger.info("-" * 40)
                logger.info(f"Test: {func_name}")

                # IMPORTANT: Do NOT open a world here.
                # Each test function is responsible for calling open_scene()
                # with the appropriate world. Opening a fallback world first
                # only wastes time.

                t0 = time.time()
                try:
                    def progress_cb(value: int) -> None:
                        logger.debug(f"Progress: {value}%")

                    outcome = test_func(
                        core.simulator, sensor_obj, progress_cb=progress_cb
                    )
                    duration = time.time() - t0
                    passed_flag = (
                        outcome.get("passed", False)
                        if isinstance(outcome, dict)
                        else bool(outcome)
                    )
                    status = "Passed" if passed_flag else "Failed"
                    result_data = outcome if isinstance(outcome, dict) else {}

                    # Extract description for console output
                    description = ""
                    if isinstance(outcome, dict):
                        description = outcome.get("description", "")
                        if not description:
                            # Try to build summary from metrics
                            if "metrics" in outcome and isinstance(outcome["metrics"], dict):
                                parts = []
                                for k, v in outcome["metrics"].items():
                                    if isinstance(v, bool):
                                        parts.append(f"{k}: {'PASS' if v else 'FAIL'}")
                                if parts:
                                    description = "; ".join(parts)
                        if not description:
                            description = "No description provided."
                    else:
                        description = "Test returned non-dict result."

                    # Print concise result
                    symbol = "✅" if status == "Passed" else "❌"
                    print(f"\n{symbol} {func_name} - {status} ({duration:.2f}s)")
                    print(f"   {description}")
                    if status == "Failed" and isinstance(outcome, dict):
                        if "error" in outcome:
                            print(f"   Error: {outcome['error']}")

                    logger.info(f"Test finished: {status} (duration: {duration:.2f}s)")
                    if status == "Passed":
                        passed += 1
                    else:
                        failed += 1

                    results_list.append({
                        "name": func_name,
                        "status": status,
                        "description": description,
                    })

                except Exception as e:
                    duration = time.time() - t0
                    status = "Failed"
                    result_data = {
                        "error": str(e),
                        "traceback": traceback.format_exc(),
                    }
                    logger.error(f"Test raised exception: {e}\n{traceback.format_exc()}")
                    description = f"Exception: {e}"
                    print(f"\n❌ {func_name} - Failed ({duration:.2f}s)")
                    print(f"   {description}")
                    failed += 1
                    results_list.append({
                        "name": func_name,
                        "status": "Failed",
                        "description": description,
                    })

                # Save result to DB
                try:
                    db.save_test_result(
                        sensor_name=sensor_name,
                        test_name=func_name,
                        status=status,
                        result=result_data,
                        description="",
                        duration=duration,
                    )
                    logger.debug(f"Result saved to DB (status={status})")
                except Exception as e:
                    logger.error(f"Failed to save result: {e}")

                # After test, ensure Gazebo is killed (in case test didn't clean up)
                logger.info("Ensuring Gazebo is stopped...")
                core.simulator.kill_gazebo()
                time.sleep(1.0)

            logger.info(f"Finished sensor: {sensor_name}")

    finally:
        logger.info("Shutting down...")
        core.simulator.kill()
        logger.info("Done.")

    return {"total": total_tests, "passed": passed, "failed": failed}, results_list


# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor", help="Run only tests for this sensor name")
    parser.add_argument("--test", help="Run only this specific test function")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    setup_logging(args.verbose)
    logger.info("=== Headless Test Runner Started ===")
    summary, results = run_all_tests(sensor_filter=args.sensor, test_filter=args.test)

    # Print detailed summary
    print("\n" + "=" * 70)
    print("SUMMARY OF ALL TESTS")
    print("=" * 70)
    for res in results:
        symbol = "✅" if res["status"] == "Passed" else "❌"
        print(f"{symbol} {res['name']:<35} {res['description']}")
    print("=" * 70)
    print(f"Total tests run: {summary['total']}")
    print(f"Passed:          {summary['passed']}")
    print(f"Failed:          {summary['failed']}")
    print("\nDetailed log written to: headless_test_runner.log")
    print("=" * 70)

    sys.exit(0 if summary["failed"] == 0 else 1)


if __name__ == "__main__":
    main()