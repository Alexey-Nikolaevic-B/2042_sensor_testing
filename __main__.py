#!/usr/bin/env python3
"""
Единый entrypoint проекта 2042_sensor_testing.

С CLI-аргументами (--sensor-type и т.д.) → неинтерактивный запуск тестов
  (как RFID: Core.run_sensor_suite).
Без аргументов → интерактивное консольное меню (console.run).
"""
from __future__ import annotations

import argparse
import sys
import traceback

from src.core import Core
from src.console import run


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run sensor tests (camera / RFID) or start interactive console",
    )
    parser.add_argument(
        "--sensor-type",
        default="",
        help="Sensor type, e.g. camera, rfid",
    )
    parser.add_argument(
        "--sensor-name", "--sensor",
        dest="sensor_name",
        default="",
        help="Registered sensor name (e.g. d435_like, uvc_profile_640x480_60deg)",
    )
    parser.add_argument(
        "--all-sensors",
        action="store_true",
        help="Run selected test/suite for ALL sensors of --sensor-type",
    )
    parser.add_argument(
        "--test",
        default="",
        help="Single test method name (ending with _test), or 'suite'",
    )
    parser.add_argument(
        "--suite",
        action="store_true",
        help="Run all discovered *_test for the selected sensor(s)",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    # Если передан --sensor-type → CLI-режим (как RFID), без интерактива.
    if args.sensor_type:
        if not args.all_sensors and not str(args.sensor_name).strip():
            print("ERROR: specify --sensor-name or use --all-sensors")
            return 2

        core = Core()
        core.prepare_simulator()
        try:
            return core.run_sensor_suite(
                sensor_type=args.sensor_type,
                sensor_name=str(args.sensor_name).strip(),
                test_name=str(args.test).strip(),
                suite=args.suite,
                all_sensors=args.all_sensors,
            )
        finally:
            core.shutdown()

    # Без аргументов → интерактивное меню.
    core = Core()
    try:
        run(core)
    except Exception:
        print("\nUnexpected error while running.")
        print("\nDetails:")
        traceback.print_exc()
    finally:
        core.kill()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())