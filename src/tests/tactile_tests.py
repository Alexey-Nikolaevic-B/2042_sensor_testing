"""Tactile sensor tests."""

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ContactsState

from config import CONFIG


def tactile_min_force_threshold(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T1 — minimum detectable force
    Scene: T1: 2.1
    Purpose: Determine minimum force that reliably triggers the sensor

    Steps:
    1. Increase force in 0.5 N steps
    2. Record minimum force that produces stable contact signal
    Pass criteria: Stable detection at force ≤ 2.0 N
    Stable = at least 2 ContactsState messages with non-zero force in capture window
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

    world_path = f"{CONFIG['ROOT_PATH']}/assets/worlds/tactile_force.world"

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    start_z = 0.15
    contact_z = 0.095

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    simulator.set_pose(probe_model, 0.0, 0.0, start_z)
    time.sleep(1)

    forces = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    total_steps = len(forces)

    for idx, force_norm in enumerate(forces):
        if progress_cb:
            progress_cb(int((idx / total_steps) * 100))

        result["forces_tested"].append(force_norm)

        penetration = force_norm * 0.001
        probe_z = contact_z - penetration
        simulator.set_pose(probe_model, 0.0, 0.0, probe_z)

        time.sleep(0.5)

        try:
            contacts = sensor.capture_data(ContactsState, window=1.0, timeout=0.2, simulator=simulator)

            # Stable detection: at least 2 ContactsState messages with non-zero force
            nonzero_msg_count = 0
            force_magnitude = 0.0

            for msg in contacts:
                msg_has_force = False
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                            if mag > 0:
                                msg_has_force = True
                                force_magnitude = max(force_magnitude, mag)
                if msg_has_force:
                    nonzero_msg_count += 1

            detected = nonzero_msg_count >= 2
            result["detected_forces"].append(force_magnitude if detected else 0)

            if detected and result["min_detected_force"] is None:
                result["min_detected_force"] = force_norm

            print(f"[DEBUG T1] force_norm={force_norm}N  probe_z={probe_z:.4f}  "
                  f"nonzero_msgs={nonzero_msg_count}  detected={detected}  mag={force_magnitude:.4f}")

        except Exception as e:
            print(f"[DEBUG T1] force_norm={force_norm}N  exception: {e}")
            result["detected_forces"].append(0)

    simulator.set_pose(probe_model, 0.0, 0.0, start_z)

    for i, force in enumerate(result["forces_tested"]):
        result["detection_rate"][force] = 1.0 if result["detected_forces"][i] > 0 else 0.0

    min_detected = result["min_detected_force"]
    result["passed"] = min_detected is not None and min_detected <= result["threshold_norm"]
    result["duration"] = round(time.time() - t0, 2)

    print(f"[DEBUG T1] min_detected_force={min_detected}  passed={result['passed']}")
    return result


def tactile_response_uniformity(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T2 — surface response uniformity
    Scene: T2: 2.2
    Purpose: Create 3x3 grid response map over 20×20 mm sensor surface

    Specifications:
    - Sensor size: 0.02 × 0.02 m (20×20 mm)
    - Mounted horizontally
    - Load points: 3×3 grid on sensor surface
    - Applied force: 1.96 N (200 gf)

    Steps:
    1. Apply 1.96 N force at each grid point
    2. Measure response at each point
    3. Calculate deviation from mean
    Pass criteria: All 9 points must respond (no zeros) AND deviation ≤ 10% from mean
    """
    result = {
        "passed": False,
        "test_name": "T2 - Response Uniformity",
        "sensor_size_m": 0.02,
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
    world_path = f"{CONFIG['ROOT_PATH']}/assets/worlds/tactile_uniformity.world"

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
    grid_start = -sensor_size / 2 + margin
    grid_end = sensor_size / 2 - margin
    step = (grid_end - grid_start) / 2

    x_positions = [grid_start, grid_start + step, grid_end]
    y_positions = [grid_start, grid_start + step, grid_end]

    print(f"\n[DEBUG T2] sensor_size={sensor_size}m  grid_start={grid_start:.4f}  grid_end={grid_end:.4f}")
    print(f"[DEBUG T2] X: {[round(x, 4) for x in x_positions]}")
    print(f"[DEBUG T2] Y: {[round(y, 4) for y in y_positions]}")

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
                print(f"[DEBUG T2] point ({i},{j}) x={x:.4f} y={y:.4f}  force={max_force:.4f}")

            except Exception as e:
                response_grid[i][j] = 0
                grid_point["response"] = 0
                grid_point["detected"] = False
                grid_point["error"] = str(e)
                print(f"[DEBUG T2] point ({i},{j}) x={x:.4f} y={y:.4f}  exception: {e}")

            result["grid_points"].append(grid_point)

            simulator.set_pose(probe_model, x, y, probe_z)

    result["response_map"] = response_grid.tolist()

    # Fail immediately if ANY grid point produced zero response
    zero_points = [(i, j) for i in range(3) for j in range(3) if response_grid[i][j] == 0]
    if zero_points:
        result["error"] = f"Zero response at {len(zero_points)} grid point(s): {zero_points}"
        result["duration"] = round(time.time() - t0, 2)
        print(f"[DEBUG T2] FAILED — zero points: {zero_points}")
        return result

    all_responses = response_grid.flatten()
    result["mean_response"] = round(float(np.mean(all_responses)), 4)
    result["std_deviation"] = round(float(np.std(all_responses)), 4)

    deviations = np.abs(all_responses - result["mean_response"])
    result["max_deviation"] = round(float(np.max(deviations)), 4)

    if result["mean_response"] > 0:
        result["max_deviation_percent"] = round(
            (result["max_deviation"] / result["mean_response"]) * 100, 2
        )

    result["min_response"] = round(float(np.min(all_responses)), 4)
    result["max_response"] = round(float(np.max(all_responses)), 4)
    result["range"] = round(result["max_response"] - result["min_response"], 4)
    result["coeff_variation"] = round(
        (result["std_deviation"] / result["mean_response"]) * 100, 2
    ) if result["mean_response"] > 0 else 0.0

    result["passed"] = result["max_deviation_percent"] <= result["deviation_threshold"]
    result["duration"] = round(time.time() - t0, 2)

    print(f"[DEBUG T2] mean={result['mean_response']}  max_dev%={result['max_deviation_percent']}  passed={result['passed']}")
    return result


def tactile_time_stability(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T3 — temporal stability under constant load
    Scene: tactile_force.world
    Purpose: Verify that sensor output does not drift over 20 seconds under fixed force

    Steps:
    1. Press force_probe to fixed position at sensor center
    2. Sample ContactsState every second for 20 seconds
    3. Compute max drift from mean
    Pass criteria: max drift ≤ 15% of mean force
    """
    result = {
        "passed": False,
        "test_name": "T3 - Time Stability",
        "duration_s": 20,
        "sample_interval_s": 1.0,
        "drift_threshold_percent": 15.0,
        "time_series": [],
        "initial_force": None,
        "final_force": None,
        "max_drift_percent": None,
        "start_mean": None,
        "end_mean": None,
        "drift_percent": None,
        "valid_sample_count": 0,
        "recovery_ok": False,
        "error": None,
    }

    t0 = time.time()
    world_path = f"{CONFIG['ROOT_PATH']}/assets/worlds/tactile_force.world"

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    start_z = 0.15
    # Fixed 2 mm penetration for stability test
    contact_z = 0.093

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    simulator.set_pose(probe_model, 0.0, 0.0, start_z)
    time.sleep(1)
    simulator.set_pose(probe_model, 0.0, 0.0, contact_z)
    time.sleep(1)

    print(f"[DEBUG T3] Probe pressed at z={contact_z:.4f}, sampling for {result['duration_s']}s...")

    num_samples = int(result["duration_s"] / result["sample_interval_s"])

    for i in range(num_samples):
        if progress_cb:
            progress_cb(int((i / num_samples) * 100))

        sample_t0 = time.time()

        try:
            contacts = sensor.capture_data(ContactsState, window=0.5, timeout=0.1, simulator=simulator)

            max_force = 0.0
            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                            max_force = max(max_force, mag)

            result["time_series"].append({
                "t": round(time.time() - t0, 2),
                "force": round(max_force, 4),
            })
            print(f"[DEBUG T3] sample {i+1}/{num_samples}  t={result['time_series'][-1]['t']}s  force={max_force:.4f}")

        except Exception as e:
            result["time_series"].append({
                "t": round(time.time() - t0, 2),
                "force": 0.0,
                "error": str(e),
            })
            print(f"[DEBUG T3] sample {i+1}/{num_samples}  exception: {e}")

        # Sleep remaining time to maintain sample interval
        elapsed = time.time() - sample_t0
        sleep_time = max(0.0, result["sample_interval_s"] - elapsed)
        time.sleep(sleep_time)

    simulator.set_pose(probe_model, 0.0, 0.0, start_z)

    forces = [s["force"] for s in result["time_series"]]
    nonzero_forces = [f for f in forces if f > 0]
    result["valid_sample_count"] = len(nonzero_forces)

    if len(nonzero_forces) < 5:
        result["error"] = "Insufficient non-zero force samples during stability test"
        result["duration"] = round(time.time() - t0, 2)
        return result

    window_size = max(1, len(forces) // 5)
    start_window = forces[:window_size]
    end_window = forces[-window_size:]

    start_mean = float(np.mean(start_window))
    end_mean = float(np.mean(end_window))
    drift_percent = abs(end_mean - start_mean) / max(start_mean, 1e-6) * 100

    result["start_mean"] = round(start_mean, 4)
    result["end_mean"] = round(end_mean, 4)
    result["drift_percent"] = round(drift_percent, 2)
    result["initial_force"] = result["start_mean"]
    result["final_force"] = result["end_mean"]
    result["max_drift_percent"] = result["drift_percent"]

    try:
        recovery_contacts = sensor.capture_data(
            ContactsState,
            window=2.0,
            timeout=0.1,
            simulator=simulator,
        )
        recovery_nonzero_msg_count = 0
        for msg in recovery_contacts:
            msg_has_force = False
            if msg.states:
                for state in msg.states:
                    if state.total_wrench.force:
                        f = state.total_wrench.force
                        mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                        if mag > 0:
                            msg_has_force = True
                            break
            if msg_has_force:
                recovery_nonzero_msg_count += 1
        result["recovery_ok"] = recovery_nonzero_msg_count < 2
    except Exception as e:
        result["error"] = f"Recovery check failed: {e}"
        result["duration"] = round(time.time() - t0, 2)
        return result

    result["passed"] = (
        result["drift_percent"] <= result["drift_threshold_percent"]
        and result["valid_sample_count"] >= 5
        and result["recovery_ok"]
    )
    if not result["passed"] and result["error"] is None:
        if not result["recovery_ok"]:
            result["error"] = "Sensor did not recover after load removal"
        elif result["drift_percent"] > result["drift_threshold_percent"]:
            result["error"] = (
                f"Drift too high: {result['drift_percent']}% > "
                f"{result['drift_threshold_percent']}%"
            )

    result["duration"] = round(time.time() - t0, 2)
    print(f"[DEBUG T3] start_mean={result['start_mean']}  end_mean={result['end_mean']}  "
          f"drift%={result['drift_percent']}  valid_samples={result['valid_sample_count']}  "
          f"recovery_ok={result['recovery_ok']}  passed={result['passed']}")
    return result


def tactile_peak_load_response(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T4 — peak load response at multiple penetration levels
    Scene: tactile_force.world
    Purpose: Verify that sensor force response increases monotonically with penetration depth

    Penetration levels: 1 mm, 3 mm, 5 mm
    Steps:
    1. Apply each penetration level at sensor center
    2. Measure peak force at each level
    3. Verify forces are strictly monotonically increasing
    Pass criteria: All 3 levels produce non-zero force AND forces are monotonically increasing
    """
    penetration_levels = [0.001, 0.003, 0.005]

    result = {
        "passed": False,
        "test_name": "T4 - Peak Load Response",
        "penetration_levels_m": penetration_levels,
        "measured_forces": [],
        "is_monotonic": False,
        "error": None,
    }

    t0 = time.time()
    world_path = f"{CONFIG['ROOT_PATH']}/assets/worlds/tactile_force.world"

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    start_z = 0.15
    contact_z = 0.095

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    simulator.set_pose(probe_model, 0.0, 0.0, start_z)
    time.sleep(1)

    total_steps = len(penetration_levels)

    for idx, penetration in enumerate(penetration_levels):
        if progress_cb:
            progress_cb(int((idx / total_steps) * 100))

        probe_z = contact_z - penetration
        simulator.set_pose(probe_model, 0.0, 0.0, probe_z)
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

            result["measured_forces"].append(round(max_force, 4))
            print(f"[DEBUG T4] penetration={penetration*1000:.1f}mm  probe_z={probe_z:.4f}  force={max_force:.4f}")

        except Exception as e:
            result["measured_forces"].append(0.0)
            print(f"[DEBUG T4] penetration={penetration*1000:.1f}mm  exception: {e}")

        # Return to start between levels
        simulator.set_pose(probe_model, 0.0, 0.0, start_z)
        time.sleep(0.3)

    forces = result["measured_forces"]

    if len(forces) == len(penetration_levels) and all(f > 0 for f in forces):
        result["is_monotonic"] = all(forces[i] < forces[i + 1] for i in range(len(forces) - 1))
        result["passed"] = result["is_monotonic"]
        if not result["is_monotonic"]:
            result["error"] = f"Forces not monotonically increasing: {forces}"
    else:
        zero_levels = [penetration_levels[i] for i, f in enumerate(forces) if f == 0]
        result["error"] = f"Zero force at penetration levels: {[round(p*1000, 1) for p in zero_levels]} mm"

    result["duration"] = round(time.time() - t0, 2)
    print(f"[DEBUG T4] forces={forces}  monotonic={result['is_monotonic']}  passed={result['passed']}")
    return result
