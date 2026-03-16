"""Tactile sensor tests."""

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ContactsState


def tactile_min_force_threshold(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T1 — minimum detectable force
    Scene: T1: 2.1
    Purpose: Determine minimum force that reliably triggers the sensor

    Steps:
    1. Increase force in 0.5 N steps
    2. Record minimum force that produces stable contact signal
    Pass criteria: Stable detection at force ≤ 2.0 N
    """
    result = {
        "passed": False,
        "test_name": "T1 - Minimum Force Threshold",
        "threshold_norm": 2.0,
        "force_step": 0.5,
        "forces_tested": [],
        "detected_forces": [],
        "min_detected_force": None,
        "detection_rate": {},
        "error": None
    }

    t0 = time.time()

    world_path = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/assets/worlds/tactile_force.world"

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    start_pos = [0.02, 0, 0.12]
    contact_pos = [0.02, 0, 0.095]

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    simulator.set_pose(probe_model, *start_pos)
    time.sleep(1)

    forces = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    total_steps = len(forces)

    for idx, force_norm in enumerate(forces):
        if progress_cb:
            progress_cb(int((idx / total_steps) * 100))

        result["forces_tested"].append(force_norm)

        penetration = force_norm * 0.001
        probe_z = contact_pos[2] - penetration
        simulator.set_pose(probe_model, contact_pos[0], contact_pos[1], probe_z)

        time.sleep(0.5)

        try:
            contacts = sensor.capture_data(ContactsState, window=1.0, timeout=0.2, simulator=simulator)

            detected = False
            force_magnitude = 0.0

            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                            force_magnitude = max(force_magnitude, mag)
                            detected = True

            result["detected_forces"].append(force_magnitude if detected else 0)

            if detected and result["min_detected_force"] is None:
                result["min_detected_force"] = force_norm

        except Exception as e:
            result["detected_forces"].append(0)

    simulator.set_pose(probe_model, *start_pos)

    for i, force in enumerate(result["forces_tested"]):
        result["detection_rate"][force] = 1.0 if result["detected_forces"][i] > 0 else 0.0

    min_detected = result["min_detected_force"]
    result["passed"] = min_detected is not None and min_detected <= result["threshold_norm"]
    result["duration"] = round(time.time() - t0, 2)

    return result


def tactile_response_uniformity(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T2 — surface response uniformity
    Scene: T2: 2.2
    Purpose: Create 3x3 grid response map over 50×50 mm sensor surface

    Specifications:
    - Sensor size: 0.05 × 0.05 m (50×50 mm)
    - Mounted horizontally
    - Load points: 3×3 grid on sensor surface
    - Applied force: 1.96 N (200 gf)

    Steps:
    1. Apply 1.96 N force at each grid point
    2. Measure response at each point
    3. Calculate deviation from mean
    Pass criteria: Deviation ≤ 10% from mean
    """
    result = {
        "passed": False,
        "test_name": "T2 - Response Uniformity",
        "sensor_size_m": 0.05,
        "applied_force_n": 1.96,
        "grid_size": [3, 3],
        "grid_points": [],
        "response_map": [],
        "mean_response": 0,
        "std_deviation": 0,
        "max_deviation": 0,
        "max_deviation_percent": 0,
        "deviation_threshold": 10.0,
        "error": None
    }

    t0 = time.time()
    world_path = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/assets/worlds/tactile_uniformity.world"

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "uniformity_probe"
    applied_force = result["applied_force_n"]
    penetration = applied_force * 0.001

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    sensor_size = result["sensor_size_m"]
    margin = sensor_size * 0.1
    grid_start = -sensor_size/2 + margin
    grid_end = sensor_size/2 - margin
    step = (grid_end - grid_start) / 2

    x_positions = [grid_start, grid_start + step, grid_end]
    y_positions = [grid_start, grid_start + step, grid_end]

    print(f"\n[DEBUG] Grid positions:")
    print(f"  X: {[round(x, 4) for x in x_positions]}")
    print(f"  Y: {[round(y, 4) for y in y_positions]}")

    total_points = len(x_positions) * len(y_positions)
    point_counter = 0

    response_grid = np.zeros((3, 3))

    for i, x in enumerate(x_positions):
        for j, y in enumerate(y_positions):
            point_counter += 1
            if progress_cb:
                progress_cb(int((point_counter / total_points) * 100))

            grid_point = {
                "x": round(x, 4),
                "y": round(y, 4),
                "row": i,
                "col": j
            }

            probe_z = 0.12
            simulator.set_pose(probe_model, x, y, probe_z)
            time.sleep(0.5)

            contact_z = 0.095 - penetration
            simulator.set_pose(probe_model, x, y, contact_z)
            time.sleep(0.5)

            try:
                contacts = sensor.capture_data(ContactsState, window=1.0, timeout=0.2, simulator=simulator)

                max_force = 0.0
                for msg in contacts:
                    if msg.states:
                        for state in msg.states:
                            if state.total_wrench.force:
                                f = state.total_wrench.force
                                mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                                max_force = max(max_force, mag)

                response_grid[i][j] = max_force
                grid_point["response"] = round(max_force, 4)
                grid_point["detected"] = max_force > 0

            except Exception as e:
                response_grid[i][j] = 0
                grid_point["response"] = 0
                grid_point["detected"] = False
                grid_point["error"] = str(e)

            result["grid_points"].append(grid_point)

            simulator.set_pose(probe_model, x, y, probe_z)

    valid_responses = response_grid[response_grid > 0]

    if len(valid_responses) > 0:
        result["mean_response"] = round(float(np.mean(valid_responses)), 4)
        result["std_deviation"] = round(float(np.std(valid_responses)), 4)
        result["response_map"] = response_grid.tolist()

        deviations = np.abs(response_grid - result["mean_response"])
        result["max_deviation"] = round(float(np.max(deviations)), 4)

        if result["mean_response"] > 0:
            result["max_deviation_percent"] = round(
                (result["max_deviation"] / result["mean_response"]) * 100, 2
            )

        result["passed"] = result["max_deviation_percent"] <= result["deviation_threshold"]

        # Additional uniformity metrics
        result["min_response"] = round(float(np.min(valid_responses)), 4)
        result["max_response"] = round(float(np.max(valid_responses)), 4)
        result["range"] = round(result["max_response"] - result["min_response"], 4)
        result["coeff_variation"] = round(
            (result["std_deviation"] / result["mean_response"]) * 100, 2
        )
    else:
        result["error"] = "No valid responses detected at any grid point"

    result["duration"] = round(time.time() - t0, 2)

    return result
