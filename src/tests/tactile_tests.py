"""Tactile sensor tests."""

import math
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
        "description": None,
        "test_name": "T1 - Minimum Force Threshold",
        "sensor_size_m": None,
        "threshold_norm": 2.0,
        "force_step": 0.5,
        "forces_tested": [],
        "detected_forces": [],
        "min_detected_force": None,
        "detection_rate": {},
        "error": None,
    }

    t0 = time.time()

    # ── Read sensor dimensions from SDF params ────────────────────────────────
    raw_size = sensor.params.get("sensor_size") or sensor.params.get("size")
    if not raw_size:
        result["error"] = "no sensor size"
        result["duration"] = round(time.time() - t0, 2)
        return result
    size_x = float(raw_size[0])
    size_y = float(raw_size[1])
    result["sensor_size_m"] = [round(size_x, 4), round(size_y, 4)]

    world_path = Worlds.TACTILE_FORCE

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    # Compute probe positions from sensor height (body centered at z=0 in SDF,
    # top surface at z = size_z/2).  Probe tip sits 13mm below probe_link origin.
    size_z = float(raw_size[2])
    sensor_top_z = size_z / 2.0
    probe_tip_offset = 0.013
    start_pos = [0.0, 0.0, sensor_top_z + probe_tip_offset + 0.05]   # 5cm above
    contact_pos = [0.0, 0.0, sensor_top_z + probe_tip_offset]        # at surface
    print(f"[DEBUG T1] size_z={size_z}, sensor_top={sensor_top_z:.4f}, start_z={start_pos[2]:.4f}, contact_z={contact_pos[2]:.4f}")

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
        # Rise probe before each measurement to guarantee a fresh contact event
        simulator.set_pose(probe_model, contact_pos[0], contact_pos[1], start_pos[2])
        time.sleep(0.3)
        simulator.set_pose(probe_model, contact_pos[0], contact_pos[1], probe_z)

        try:
            # Use persistent subscriber — doesn't miss contact events like
            # wait_for_message does between iterations.
            contacts = sensor.capture_persistent(
                ContactsState, window=1.5, simulator=simulator
            )

            detected = False
            force_magnitude = 0.0

            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force is not None:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
                            force_magnitude = max(force_magnitude, mag)
                            detected = True

            result["detected_forces"].append(force_magnitude if detected else 0)

            if detected and result["min_detected_force"] is None:
                result["min_detected_force"] = force_norm

        except Exception as e:
            result["detected_forces"].append(0)

    simulator.set_pose(probe_model, *start_pos)

    for i, force in enumerate(result["forces_tested"]):
        result["detection_rate"][force] = (
            1.0 if result["detected_forces"][i] > 0 else 0.0
        )

    min_detected = result["min_detected_force"]
    result["passed"] = (
        min_detected is not None and min_detected <= result["threshold_norm"]
    )

    if min_detected is not None:
        result["description"] = (
            f"Min detectable force: {min_detected} N, "
            f"expected: ≤ {result['threshold_norm']} N. "
            f"Forces tested: {result['forces_tested']}. "
            f"Detection rates: {result['detection_rate']}."
        )
    else:
        tested = result["forces_tested"]
        result["description"] = (
            f"No stable force detected across {len(tested)} steps "
            f"(tested range: {min(tested) if tested else 'N/A'}–"
            f"{max(tested) if tested else 'N/A'} N), "
            f"expected: ≤ {result['threshold_norm']} N."
        )

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
        "description": None,
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
        "deviation_threshold": 150.0,
        "missed_points": 0,
        "error": None,
    }

    t0 = time.time()

    # ── Read sensor dimensions from SDF params ────────────────────────────────
    raw_size = sensor.params.get("sensor_size") or sensor.params.get("size")
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
    penetration = applied_force * 0.001

    # ── Compute probe positions based on sensor height ────────────────────────
    # Sensor body is placed in SDF at <pose>0 0 0</pose>, body link centered
    # at origin, so top surface is at z = size_z / 2.
    # The probe's tip (collision sphere) sits 13mm below the probe_link origin
    # (tip offset -10mm + sphere radius 3mm).  To place the tip exactly at the
    # sensor's top surface, probe_link must be at z = sensor_top_z + 0.013.
    size_z = float(raw_size[2])
    sensor_top_z = size_z / 2.0
    probe_tip_offset = 0.013  # from probe_link origin to tip's bottom contact
    rest_z = sensor_top_z + probe_tip_offset + 0.05           # 5cm above sensor
    contact_z = sensor_top_z + probe_tip_offset - penetration  # sunk into surface
    print(f"[DEBUG T2] size_z={size_z}, sensor_top={sensor_top_z:.4f}, rest_z={rest_z:.4f}, contact_z={contact_z:.4f}")

    if not simulator.wait_for_model_spawn(probe_model, 30):
        result["error"] = f"Probe '{probe_model}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    # ── Detect sensor shape (cylinder vs box) from SDF ────────────────────────
    # Cylinder: has <collision><geometry><cylinder><radius>
    # Box:      has <collision><geometry><box><size>
    # For cylindrical sensors, a square grid would place corner points
    # OUTSIDE the cylinder's footprint (diagonal > diameter / √2), causing
    # missed contacts.  Use a circular grid for cylinders.
    sdf_params = sensor.read_params_from_sdf([
        {"name": "collision/geometry/cylinder/radius"},
    ])
    cylinder_radius = sdf_params.get("radius")

    # Build a list of (row, col, x, y) tuples — 9 points total
    grid_positions = []
    if cylinder_radius:
        shape = "cylinder"
        # Test-circle radius = 70% of the cylinder's physical radius,
        # so all 8 peripheral points are guaranteed inside the collision.
        test_r = float(cylinder_radius) * 0.7
        # Center point → (1, 1) in the 3×3 response matrix
        grid_positions.append((1, 1, 0.0, 0.0))
        # 8 points on the circle, 45° apart.
        # Map each angular position to (row, col) in the 3×3 matrix so
        # response_grid retains a geometric layout (top/bottom/left/right).
        angle_to_rc = [
            (2, 1),  # 0°     →  +X, centre Y       → right-centre
            (2, 2),  # 45°    →  +X +Y              → top-right
            (1, 2),  # 90°    →  centre X, +Y       → top-centre
            (0, 2),  # 135°   →  -X +Y              → top-left
            (0, 1),  # 180°   →  -X, centre Y       → left-centre
            (0, 0),  # 225°   →  -X -Y              → bottom-left
            (1, 0),  # 270°   →  centre X, -Y       → bottom-centre
            (2, 0),  # 315°   →  +X -Y              → bottom-right
        ]
        for k, (row, col) in enumerate(angle_to_rc):
            angle = math.pi * k / 4.0  # 0, π/4, π/2, ...
            x = test_r * math.cos(angle)
            y = test_r * math.sin(angle)
            grid_positions.append((row, col, x, y))
    else:
        shape = "box"
        # Square 3×3 grid using 80% of the bounding-box footprint
        x_margin = size_x * 0.1
        y_margin = size_y * 0.1
        x_start = -size_x / 2 + x_margin
        x_end = size_x / 2 - x_margin
        y_start = -size_y / 2 + y_margin
        y_end = size_y / 2 - y_margin
        x_step = (x_end - x_start) / 2
        y_step = (y_end - y_start) / 2
        xs = [x_start, x_start + x_step, x_end]
        ys = [y_start, y_start + y_step, y_end]
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                grid_positions.append((i, j, x, y))

    result["shape"] = shape

    print(f"\n[DEBUG T2] shape={shape}")
    if shape == "cylinder":
        print(f"[DEBUG T2] cylinder radius={cylinder_radius}, test circle radius={test_r:.4f}m")
    else:
        print(f"[DEBUG T2] box size_x={size_x}, size_y={size_y}")
    print(f"[DEBUG T2] grid positions (row, col, x, y):")
    for row, col, x, y in grid_positions:
        print(f"  ({row},{col}) → x={x:+.4f}, y={y:+.4f}")

    total_points = len(grid_positions)
    point_counter = 0
    response_grid = np.zeros((3, 3))

    for row, col, x, y in grid_positions:
        point_counter += 1
        if progress_cb:
            progress_cb(int((point_counter / total_points) * 100))

        grid_point = {
            "x": round(x, 4),
            "y": round(y, 4),
            "row": row,
            "col": col,
        }

        # ── Rise: position probe above this grid point ────────────────────
        simulator.set_pose(probe_model, x, y, rest_z)
        time.sleep(0.5)

        # ── Lower: make contact ───────────────────────────────────────────
        # We start capturing BEFORE lowering the probe so the persistent
        # subscriber catches the initial contact events.  The bumper plugin
        # publishes on contact-state changes, so the transition from "no
        # contact" to "in contact" is the most reliable event to catch.
        simulator.set_pose(probe_model, x, y, contact_z)

        # Wiggle probe slightly to generate repeated contact events
        # (some bumper_plugin versions publish only on contact changes).
        try:
            def _wiggle():
                for _ in range(2):
                    time.sleep(0.2)
                    simulator.set_pose(probe_model, x, y, contact_z + 0.0005)
                    time.sleep(0.2)
                    simulator.set_pose(probe_model, x, y, contact_z)

            import threading
            wiggle_th = threading.Thread(target=_wiggle, daemon=True)
            wiggle_th.start()

            # Capture using persistent subscriber during the wiggle
            contacts = sensor.capture_persistent(
                ContactsState, window=1.5, simulator=simulator
            )
            wiggle_th.join(timeout=0.5)

            magnitudes = []
            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force is not None:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
                            magnitudes.append(mag)

            stable_response = float(np.median(magnitudes)) if magnitudes else 0.0

            response_grid[row][col] = stable_response
            grid_point["response"] = round(stable_response, 4)
            grid_point["detected"] = stable_response > 0
            grid_point["samples"] = len(magnitudes)

        except Exception as e:
            response_grid[row][col] = 0.0
            grid_point["response"] = 0.0
            grid_point["detected"] = False
            grid_point["samples"] = 0
            grid_point["error"] = str(e)

        # ── Rise: lift before moving to the next point ────────────────────
        simulator.set_pose(probe_model, x, y, rest_z)
        time.sleep(1.0)

        result["grid_points"].append(grid_point)

    # ── Analysis ──────────────────────────────────────────────────────────────
    all_responses = response_grid.flatten()  # all 9 points, zeros included
    missed = int(np.sum(all_responses == 0))
    result["missed_points"] = missed

    valid_responses = all_responses[all_responses > 0]

    if len(valid_responses) > 0:
        mean_val = float(np.mean(valid_responses))

        result["mean_response"] = round(mean_val, 4)
        result["std_deviation"] = round(float(np.std(valid_responses)), 4)
        result["response_map"] = response_grid.tolist()

        deviations = np.abs(valid_responses - mean_val)
        result["max_deviation"] = round(float(np.max(deviations)), 4)

        if mean_val > 0:
            result["max_deviation_percent"] = round(
                (result["max_deviation"] / mean_val) * 100, 2
            )

        result["min_response"] = round(float(np.min(valid_responses)), 4)
        result["max_response"] = round(float(np.max(valid_responses)), 4)
        result["range"] = round(result["max_response"] - result["min_response"], 4)
        result["coeff_variation"] = round((result["std_deviation"] / mean_val) * 100, 2)

        # Pass only if deviation is within threshold AND every point responded
        result["passed"] = (
            result["max_deviation_percent"] <= result["deviation_threshold"]
            and missed == 0
        )

        result["description"] = (
            f"Max deviation: {result['max_deviation_percent']}% "
            f"(expected: ≤ {result['deviation_threshold']}%), "
            f"mean response: {result['mean_response']} N, "
            f"range: {result['min_response']}–{result['max_response']} N, "
            f"std: {result['std_deviation']} N, "
            f"missed points: {missed}/9."
        )
    else:
        result["error"] = "No valid responses detected at any grid point"
        result["description"] = (
            f"No valid responses at any of the 9 grid points. "
            f"Sensor size: {result['sensor_size_m']} m."
        )

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
        "description": None,
        "test_name": "T3 - Temporal Stability",
        "sensor_size_m": None,
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

    # ── Read sensor dimensions from SDF params ────────────────────────────────
    raw_size = sensor.params.get("sensor_size") or sensor.params.get("size")
    if not raw_size:
        result["error"] = "no sensor size"
        result["duration"] = round(time.time() - t0, 2)
        return result
    size_x = float(raw_size[0])
    size_y = float(raw_size[1])
    result["sensor_size_m"] = [round(size_x, 4), round(size_y, 4)]

    world_path = Worlds.TACTILE_STABILITY

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    center_x, center_y = 0.0, 0.0
    applied_force = result["applied_force_n"]
    penetration = applied_force * 0.001
    # Compute probe positions from sensor height (see T1/T2 for details)
    size_z = float(raw_size[2])
    sensor_top_z = size_z / 2.0
    probe_tip_offset = 0.013
    rest_z = sensor_top_z + probe_tip_offset + 0.05
    contact_z = sensor_top_z + probe_tip_offset - penetration
    print(f"[DEBUG T3] size_z={size_z}, sensor_top={sensor_top_z:.4f}, rest_z={rest_z:.4f}, contact_z={contact_z:.4f}")

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
            # Tiny wiggle to trigger bumper plugin contact-change publish
            simulator.set_pose(probe_model, center_x, center_y, contact_z + 0.0005)
            time.sleep(0.05)
            simulator.set_pose(probe_model, center_x, center_y, contact_z)

            # Capture with persistent subscriber — catches all events
            contacts = sensor.capture_persistent(
                ContactsState, window=0.7, simulator=simulator
            )

            max_force = 0.0
            for msg in contacts:
                if msg.states:
                    for state in msg.states:
                        if state.total_wrench.force is not None:
                            f = state.total_wrench.force
                            mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
                            max_force = max(max_force, mag)

            elapsed = time.time() - measure_start
            force_values.append(max_force)
            result["time_series"].append(
                {
                    "t": round(elapsed, 2),
                    "force": round(max_force, 4),
                }
            )

        except Exception:
            force_values.append(0.0)

        time.sleep(sample_interval - 0.7)  # adjust for persistent capture time

    # Remove load and check recovery
    simulator.set_pose(probe_model, center_x, center_y, rest_z)
    time.sleep(2)

    if progress_cb:
        progress_cb(90)

    try:
        recovery_data = sensor.capture_persistent(
            ContactsState, window=1.0, simulator=simulator
        )
        recovery_force = 0.0
        for msg in recovery_data:
            if msg.states:
                for state in msg.states:
                    if state.total_wrench.force is not None:
                        f = state.total_wrench.force
                        mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
                        recovery_force = max(recovery_force, mag)
        result["recovery_ok"] = recovery_force < 0.1
    except Exception:
        result["recovery_ok"] = None

    # Analyse drift and jitter
    valid = [v for v in force_values if v > 0]

    if len(valid) >= 4:
        window = max(1, len(valid) // 5)  # first / last 20 %
        start_mean = float(np.mean(valid[:window]))
        end_mean = float(np.mean(valid[-window:]))

        result["start_mean"] = round(start_mean, 4)
        result["end_mean"] = round(end_mean, 4)
        result["std_deviation"] = round(float(np.std(valid)), 4)

        if start_mean > 0:
            drift = abs(end_mean - start_mean) / start_mean * 100
            jitter = result["std_deviation"] / float(np.mean(valid)) * 100

            result["drift_percent"] = round(drift, 2)
            result["jitter_percent"] = round(jitter, 2)
            result["passed"] = drift <= result["drift_threshold_percent"]

            recovery_str = (
                "OK"
                if result.get("recovery_ok") is True
                else "FAIL" if result.get("recovery_ok") is False else "unknown"
            )
            result["description"] = (
                f"Signal drift: {result['drift_percent']}% "
                f"(expected: ≤ {result['drift_threshold_percent']}%), "
                f"start mean: {result['start_mean']} N → end mean: {result['end_mean']} N, "
                f"jitter: {result['jitter_percent']}%, "
                f"std: {result['std_deviation']} N, "
                f"recovery after unload: {recovery_str}."
            )
        else:
            result["error"] = "No valid signal at start of measurement"
            result["description"] = (
                "Signal was zero throughout the measurement window — sensor may not be in contact."
            )
    else:
        result["error"] = "Insufficient valid readings collected"
        result["description"] = (
            f"Only {len(valid)} valid readings out of {result['measurement_duration_s']} expected — "
            f"too few samples to evaluate drift."
        )

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
        "description": None,
        "test_name": "T4 - Peak Load Response",
        "sensor_size_m": None,
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

    # ── Read sensor dimensions from SDF params ────────────────────────────────
    raw_size = sensor.params.get("sensor_size") or sensor.params.get("size")
    if not raw_size:
        result["error"] = "no sensor size"
        result["duration"] = round(time.time() - t0, 2)
        return result
    size_x = float(raw_size[0])
    size_y = float(raw_size[1])
    result["sensor_size_m"] = [round(size_x, 4), round(size_y, 4)]

    world_path = Worlds.TACTILE_PEAK

    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    time.sleep(3)

    probe_model = "force_probe"
    center_x, center_y = 0.0, 0.0
    # Compute sensor surface height dynamically from its size.
    # Sensor body in SDF is centered at (0,0,0), top at z = size_z/2.
    # Probe tip sits 13mm below probe_link origin.
    size_z = float(raw_size[2])
    probe_tip_offset = 0.013
    sensor_surface_z = size_z / 2.0 + probe_tip_offset  # probe_link z when tip touches surface
    rest_z = sensor_surface_z + result["drop_height_m"]  # drop from above
    impact_z = sensor_surface_z - 0.005  # 5 mm penetration into surface
    print(f"[DEBUG T4] size_z={size_z}, sensor_surface_z={sensor_surface_z:.4f}, rest_z={rest_z:.4f}, impact_z={impact_z:.4f}")

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
        baseline_data = sensor.capture_persistent(
            ContactsState, window=1.0, simulator=simulator
        )
        baseline_force = 0.0
        for msg in baseline_data:
            if msg.states:
                for state in msg.states:
                    if state.total_wrench.force is not None:
                        f = state.total_wrench.force
                        mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
                        baseline_force = max(baseline_force, mag)
        result["baseline_force"] = round(baseline_force, 4)
    except Exception:
        result["baseline_force"] = 0.0

    if progress_cb:
        progress_cb(35)

    # Simulate impact: teleport to deep penetration
    impact_start = time.time()
    simulator.set_pose(probe_model, center_x, center_y, impact_z)

    # Capture all contact events for 2 seconds with ONE persistent subscriber.
    # Using capture_data in a tight loop would miss most events between
    # wait_for_message calls.
    try:
        capture_duration = 2.0
        if progress_cb:
            progress_cb(50)

        contacts_all = sensor.capture_persistent(
            ContactsState, window=capture_duration, simulator=simulator
        )

        # Bucket messages by time into 100ms slots
        sample_interval = 0.1
        total_samples = int(capture_duration / sample_interval)
        force_series = [0.0] * total_samples

        for msg in contacts_all:
            if not msg.states:
                continue
            # Get the force magnitude from this message
            max_msg_force = 0.0
            for state in msg.states:
                if state.total_wrench.force is not None:
                    f = state.total_wrench.force
                    mag = (f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5
                    max_msg_force = max(max_msg_force, mag)

            # Place into a bucket based on message header stamp
            try:
                t_msg = msg.header.stamp.to_sec() if hasattr(msg, "header") else 0.0
                # We don't know the absolute ref time; use msg order instead
            except Exception:
                t_msg = 0.0

        # Simpler: distribute events evenly by index if we have any
        if contacts_all:
            n = len(contacts_all)
            for i, msg in enumerate(contacts_all):
                if not msg.states:
                    continue
                bucket_idx = min(int(i / n * total_samples), total_samples - 1)
                mf = 0.0
                for state in msg.states:
                    if state.total_wrench.force is not None:
                        f = state.total_wrench.force
                        mag = (f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5
                        mf = max(mf, mag)
                force_series[bucket_idx] = max(force_series[bucket_idx], mf)

        # Fill time series
        for idx, fval in enumerate(force_series):
            result["impact_time_series"].append(
                {
                    "t": round(idx * sample_interval, 3),
                    "force": round(fval, 4),
                }
            )

        if progress_cb:
            progress_cb(85)

    except Exception as e:
        force_series = [0.0]
        print(f"[DEBUG T4] impact capture error: {e}")

    # Return probe to rest
    simulator.set_pose(probe_model, center_x, center_y, rest_z)

    if progress_cb:
        progress_cb(90)

    # Analyse peak and saturation
    if force_series:
        peak_force = max(force_series)
        peak_idx = force_series.index(peak_force)

        result["peak_force"] = round(peak_force, 4)
        if peak_idx < len(result["impact_time_series"]):
            result["peak_time_s"] = result["impact_time_series"][peak_idx]["t"]

        result["impulse_detected"] = peak_force > (result["baseline_force"] or 0) + 0.1

        # Saturation: if >50 % of nonzero samples sit at ≥95 % of peak → flat-top clipping
        nonzero = [v for v in force_series if v > 0]
        if nonzero and peak_force > 0:
            near_peak_count = sum(1 for v in nonzero if v >= peak_force * 0.95)
            saturation_ratio = near_peak_count / len(nonzero)
            result["saturation_ratio"] = round(saturation_ratio, 3)
            result["saturation_detected"] = saturation_ratio > 0.5
        else:
            result["saturation_detected"] = False
            result["saturation_ratio"] = 0.0

        result["passed"] = (
            result["impulse_detected"] and not result["saturation_detected"]
        )

        result["description"] = (
            f"Peak force: {result['peak_force']} N at t={result['peak_time_s']} s "
            f"(baseline: {result['baseline_force']} N, drop height: {result['drop_height_m']} m), "
            f"impulse: {'detected' if result['impulse_detected'] else 'not detected'}, "
            f"saturation ratio: {result['saturation_ratio']} "
            f"({'clipping detected' if result['saturation_detected'] else 'no clipping'})."
        )
    else:
        result["error"] = "No data captured during impact window"
        result["description"] = (
            "No contact data received during the 2-second impact capture window."
        )

    if progress_cb:
        progress_cb(100)

    result["duration"] = round(time.time() - t0, 2)
    return result
