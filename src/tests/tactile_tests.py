"""Tactile sensor tests."""

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ContactsState


from ._common import Worlds

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

    world_path = Worlds.TACTILE_FORCE

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
                        if state.total_wrench.force is not None:
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
    - Sensor size: read from sensor SDF params (key "size", [X, Y, Z])
    - Mounted horizontally
    - Load points: 3×3 grid on sensor surface
    - Applied force: 1.96 N (200 gf)

    Steps:
    1. Apply 1.96 N force at each grid point
    2. Fixed cycle per point: rise → move → lower → stabilise → collect → rise
    3. Compute median force per point (stable metric, not peak)
    4. Calculate deviation from mean across all points
    Pass criteria: Deviation ≤ 10% from mean, zero missed points
    """
    result = {
        "passed": False,
        "test_name": "T2 - Response Uniformity",
        "sensor_size_m": None,
        "applied_force_n": 1.96,
        "grid_size": [3, 3],
        "grid_points": [],
        "response_map": [],
        "mean_response": 0,
        "std_deviation": 0,
        "max_deviation": 0,
        "max_deviation_percent": 0,
        "deviation_threshold": 10.0,
        "missed_points": 0,
        "error": None,
    }

    t0 = time.time()

    # ── Read sensor dimensions from SDF params ────────────────────────────────
    raw_size = sensor.params.get("size")
    if not raw_size:
        result["error"] = "no sensor size"
        result["duration"] = round(time.time() - t0, 2)
        return result
    size_x = float(raw_size[0])
    size_y = float(raw_size[1])
    result["sensor_size_m"] = [round(size_x, 4), round(size_y, 4)]

    world_path = Worlds.TACTILE_UNIFORMITY

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "uniformity_probe"
    applied_force = result["applied_force_n"]
    penetration   = applied_force * 0.001
    rest_z        = 0.12
    contact_z     = 0.095 - penetration

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    # Build 3×3 grid using actual sensor dimensions from SDF
    x_margin = size_x * 0.1
    y_margin = size_y * 0.1
    x_start  = -size_x / 2 + x_margin
    x_end    =  size_x / 2 - x_margin
    y_start  = -size_y / 2 + y_margin
    y_end    =  size_y / 2 - y_margin
    x_step   = (x_end - x_start) / 2
    y_step   = (y_end - y_start) / 2

    x_positions = [x_start, x_start + x_step, x_end]
    y_positions = [y_start, y_start + y_step, y_end]

    print(f"\n[DEBUG] Grid positions:")
    print(f"  X: {[round(x, 4) for x in x_positions]}")
    print(f"  Y: {[round(y, 4) for y in y_positions]}")

    total_points  = len(x_positions) * len(y_positions)
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
                "col": j,
            }

            # ── Rise: position probe above this grid point ────────────────────
            simulator.set_pose(probe_model, x, y, rest_z)
            time.sleep(1.0)

            # ── Lower: make contact and wait for physics to stabilise ─────────
            simulator.set_pose(probe_model, x, y, contact_z)
            time.sleep(1.5)

            # ── Collect: gather all force magnitudes in the window → median ───
            try:
                contacts = sensor.capture_data(
                    ContactsState, window=3.0, timeout=2.0, simulator=simulator
                )

                magnitudes = []
                for msg in contacts:
                    if msg.states:
                        for state in msg.states:
                            if state.total_wrench.force is not None:
                                f = state.total_wrench.force
                                mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                                magnitudes.append(mag)

                stable_response = float(np.median(magnitudes)) if magnitudes else 0.0

                response_grid[i][j]    = stable_response
                grid_point["response"] = round(stable_response, 4)
                grid_point["detected"] = stable_response > 0
                grid_point["samples"]  = len(magnitudes)

            except Exception as e:
                response_grid[i][j]    = 0.0
                grid_point["response"] = 0.0
                grid_point["detected"] = False
                grid_point["samples"]  = 0
                grid_point["error"]    = str(e)

            # ── Rise: lift before moving to the next point ────────────────────
            simulator.set_pose(probe_model, x, y, rest_z)
            time.sleep(1.0)

            result["grid_points"].append(grid_point)

    # ── Analysis ──────────────────────────────────────────────────────────────
    all_responses = response_grid.flatten()               # all 9 points, zeros included
    missed        = int(np.sum(all_responses == 0))
    result["missed_points"] = missed

    valid_responses = all_responses[all_responses > 0]

    if len(valid_responses) > 0:
        mean_val = float(np.mean(valid_responses))

        result["mean_response"]   = round(mean_val, 4)
        result["std_deviation"]   = round(float(np.std(valid_responses)), 4)
        result["response_map"]    = response_grid.tolist()

        deviations = np.abs(valid_responses - mean_val)
        result["max_deviation"]   = round(float(np.max(deviations)), 4)

        if mean_val > 0:
            result["max_deviation_percent"] = round(
                (result["max_deviation"] / mean_val) * 100, 2
            )

        result["min_response"]    = round(float(np.min(valid_responses)), 4)
        result["max_response"]    = round(float(np.max(valid_responses)), 4)
        result["range"]           = round(result["max_response"] - result["min_response"], 4)
        result["coeff_variation"] = round(
            (result["std_deviation"] / mean_val) * 100, 2
        )

        # Pass only if deviation is within threshold AND every point responded
        result["passed"] = (
            result["max_deviation_percent"] <= result["deviation_threshold"]
            and missed == 0
        )
    else:
        result["error"] = "No valid responses detected at any grid point"

    result["duration"] = round(time.time() - t0, 2)
    return result


def tactile_temporal_stability(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T3 — temporal stability under constant load
    Scene: T3 (tactile_force.world)
    Purpose: Evaluate signal stability over time under constant load

    Conditions:
    - Constant force ~4.9 N applied at sensor center
    - Measurement duration: 30 seconds

    Steps:
    1. Apply constant load at center of sensor
    2. Record signal each second for 30 seconds
    3. Compare mean of first and last 20% of samples (drift)
    4. Compute jitter (coefficient of variation)
    5. Check signal recovery after load removal
    Pass criteria: Drift ≤ 5%
    """
    result = {
        "passed": False,
        "test_name": "T3 - Temporal Stability",
        "applied_force_n": 4.9,
        "measurement_duration_s": 30,
        "drift_threshold_percent": 5.0,
        "time_series": [],
        "start_mean": None,
        "end_mean": None,
        "drift_percent": None,
        "std_deviation": None,
        "jitter_percent": None,
        "recovery_ok": None,
        "error": None,
    }

    t0 = time.time()
    world_path = Worlds.TACTILE_STABILITY

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    center_x, center_y = 0.02, 0.0
    rest_z = 0.12
    applied_force = result["applied_force_n"]
    penetration = applied_force * 0.001
    contact_z = 0.095 - penetration

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    simulator.set_pose(probe_model, center_x, center_y, rest_z)
    time.sleep(1)

    # Apply constant load
    simulator.set_pose(probe_model, center_x, center_y, contact_z)
    time.sleep(1)

    # Record signal for 30 seconds, 1 sample/sec
    measurement_duration = result["measurement_duration_s"]
    sample_interval = 1.0
    total_samples = int(measurement_duration / sample_interval)
    force_values = []
    measure_start = time.time()

    for idx in range(total_samples):
        if progress_cb:
            progress_cb(int((idx / total_samples) * 85))

        try:
            contacts = sensor.capture_data(
                ContactsState, window=0.5, timeout=0.2, simulator=simulator
            )

            max_force = 0.0
            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force is not None:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                            max_force = max(max_force, mag)

            elapsed = time.time() - measure_start
            force_values.append(max_force)
            result["time_series"].append({
                "t": round(elapsed, 2),
                "force": round(max_force, 4),
            })

        except Exception:
            force_values.append(0.0)

        time.sleep(sample_interval)

    # Remove load and check recovery
    simulator.set_pose(probe_model, center_x, center_y, rest_z)
    time.sleep(2)

    if progress_cb:
        progress_cb(90)

    try:
        recovery_data = sensor.capture_data(
            ContactsState, window=1.0, timeout=0.2, simulator=simulator
        )
        recovery_force = 0.0
        for msg in recovery_data:
            if msg.states:
                for state in msg.states:
                    if state.total_wrench.force is not None:
                        f = state.total_wrench.force
                        mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                        recovery_force = max(recovery_force, mag)
        result["recovery_ok"] = recovery_force < 0.1
    except Exception:
        result["recovery_ok"] = None

    # Analyse drift and jitter
    valid = [v for v in force_values if v > 0]

    if len(valid) >= 4:
        window = max(1, len(valid) // 5)       # first / last 20 %
        start_mean = float(np.mean(valid[:window]))
        end_mean   = float(np.mean(valid[-window:]))

        result["start_mean"]    = round(start_mean, 4)
        result["end_mean"]      = round(end_mean, 4)
        result["std_deviation"] = round(float(np.std(valid)), 4)

        if start_mean > 0:
            drift  = abs(end_mean - start_mean) / start_mean * 100
            jitter = result["std_deviation"] / float(np.mean(valid)) * 100

            result["drift_percent"]  = round(drift, 2)
            result["jitter_percent"] = round(jitter, 2)
            result["passed"]         = drift <= result["drift_threshold_percent"]
        else:
            result["error"] = "No valid signal at start of measurement"
    else:
        result["error"] = "Insufficient valid readings collected"

    if progress_cb:
        progress_cb(100)

    result["duration"] = round(time.time() - t0, 2)
    return result


def tactile_peak_load_response(simulator, sensor, progress_cb=None) -> dict:
    """
    Method T4 — peak load (impact) response
    Scene: T4 (tactile_force.world)
    Purpose: Evaluate sensor response to a sudden impact load

    Conditions:
    - Probe dropped from ~0.5 m above sensor surface
    - Impact simulated by rapid probe teleport to deep penetration

    Steps:
    1. Position probe at drop height above sensor center
    2. Capture baseline (no contact)
    3. Rapidly lower probe to impact depth
    4. Record time-series signal (2 s, 100 ms steps) around impact
    5. Detect peak force and check for saturation (flat-top clipping)
    Pass criteria: Impulse detected, no saturation
    """
    result = {
        "passed": False,
        "test_name": "T4 - Peak Load Response",
        "drop_height_m": 0.5,
        "impact_time_series": [],
        "peak_force": None,
        "peak_time_s": None,
        "baseline_force": None,
        "saturation_detected": None,
        "saturation_ratio": None,
        "impulse_detected": False,
        "error": None,
    }

    t0 = time.time()
    world_path = Worlds.TACTILE_PEAK

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model  = "force_probe"
    center_x, center_y  = 0.02, 0.0
    sensor_surface_z     = 0.095
    rest_z   = sensor_surface_z + result["drop_height_m"]  # ~0.595
    impact_z = sensor_surface_z - 0.005                    # 5 mm penetration

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Force probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    # Position at drop height
    simulator.set_pose(probe_model, center_x, center_y, rest_z)
    time.sleep(1)

    if progress_cb:
        progress_cb(20)

    # Baseline: no contact
    try:
        baseline_data  = sensor.capture_data(
            ContactsState, window=1.0, timeout=0.2, simulator=simulator
        )
        baseline_force = 0.0
        for msg in baseline_data:
            if msg.states:
                for state in msg.states:
                    if state.total_wrench.force is not None:
                        f = state.total_wrench.force
                        mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                        baseline_force = max(baseline_force, mag)
        result["baseline_force"] = round(baseline_force, 4)
    except Exception:
        result["baseline_force"] = 0.0

    if progress_cb:
        progress_cb(35)

    # Simulate impact: teleport to deep penetration
    impact_start = time.time()
    simulator.set_pose(probe_model, center_x, center_y, impact_z)

    # High-frequency capture for 2 seconds (100 ms step)
    capture_duration = 2.0
    sample_interval  = 0.1
    total_samples    = int(capture_duration / sample_interval)
    force_series     = []

    for idx in range(total_samples):
        if progress_cb:
            progress_cb(35 + int((idx / total_samples) * 50))

        try:
            contacts = sensor.capture_data(
                ContactsState, window=0.08, timeout=0.1, simulator=simulator
            )
            max_force = 0.0
            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force is not None:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
                            max_force = max(max_force, mag)

            elapsed = time.time() - impact_start
            force_series.append(max_force)
            result["impact_time_series"].append({
                "t": round(elapsed, 3),
                "force": round(max_force, 4),
            })

        except Exception:
            force_series.append(0.0)

        time.sleep(sample_interval)

    # Return probe to rest
    simulator.set_pose(probe_model, center_x, center_y, rest_z)

    if progress_cb:
        progress_cb(90)

    # Analyse peak and saturation
    if force_series:
        peak_force = max(force_series)
        peak_idx   = force_series.index(peak_force)

        result["peak_force"] = round(peak_force, 4)
        if peak_idx < len(result["impact_time_series"]):
            result["peak_time_s"] = result["impact_time_series"][peak_idx]["t"]

        result["impulse_detected"] = peak_force > (result["baseline_force"] or 0) + 0.1

        # Saturation: if >50 % of nonzero samples sit at ≥95 % of peak → flat-top clipping
        nonzero = [v for v in force_series if v > 0]
        if nonzero and peak_force > 0:
            near_peak_count          = sum(1 for v in nonzero if v >= peak_force * 0.95)
            saturation_ratio         = near_peak_count / len(nonzero)
            result["saturation_ratio"]    = round(saturation_ratio, 3)
            result["saturation_detected"] = saturation_ratio > 0.5
        else:
            result["saturation_detected"] = False
            result["saturation_ratio"]    = 0.0

        result["passed"] = result["impulse_detected"] and not result["saturation_detected"]
    else:
        result["error"] = "No data captured during impact window"

    if progress_cb:
        progress_cb(100)

    result["duration"] = round(time.time() - t0, 2)
    return result
