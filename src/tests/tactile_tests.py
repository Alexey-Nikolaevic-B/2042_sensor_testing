"""
Tactile sensor tests.

Geometry assumptions (sensor NOT static, world_anchor fixed joint, body pose z=0.013):
  - body cylinder centre  z = +0.013  (body link pose in model.sdf)
  - sensor top surface    z = +0.026  (centre + half of 0.026 length)
  - sensor bottom surface z =  0.000  (sits flush on ground plane)

  NOTE: Previously the body had no pose (centre at z=0) so the bottom half was
  underground, causing constant ground-plane contact noise (~10 N) that leaked
  into every bumper reading regardless of probe depth.

Force probe (flat disc, no stick):
  - disc radius 8 mm, thickness 2 mm, centred at link origin
  - bottom face Z = link_origin_Z - 0.001  (half thickness)
  - PROBE_TIP_OFFSET = 0.001
  - set_model_state moves probe_link directly in world frame
  - probe bottom face Z = probe_link Z - PROBE_TIP_OFFSET
  - to place face above sensor top: probe_link Z = sensor_top + gap + PROBE_TIP_OFFSET

Contact detection uses FILTERED force (probe-only contacts).
With the sensor no longer static, ground-plane contacts no longer produce
spurious constant forces in the bumper output.
"""

import time
import logging
import numpy as np
import rospy
import math
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import (GetModelState, GetModelStateRequest,
                              SetModelState, SetModelStateRequest)

from ._common import Worlds

logger = logging.getLogger(__name__)

PROBE_TIP_OFFSET  = 0.001   # metres — half disc thickness (2mm disc, centre at origin,
                            # bottom face = origin - 0.001)
CONTACT_THRESHOLD = 0.5     # N — above teleport-transient noise
CONTACT_DEBOUNCE  = 2       # consecutive readings to confirm contact

# Penetration depths for T1 force characterisation
PROBE_DEPTHS_M = [0.0001, 0.0002, 0.0005, 0.001, 0.002, 0.003, 0.005]


# ─────────────────────────────────────────────────────────────────────────────
# Gazebo service helpers
# ─────────────────────────────────────────────────────────────────────────────

def _get_pose(model_name: str, timeout: float = 3.0):
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=timeout)
        svc = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        req = GetModelStateRequest()
        req.model_name = model_name
        req.relative_entity_name = 'world'
        resp = svc(req)
        if resp.success:
            p = resp.pose.position
            return p.x, p.y, p.z
        logger.debug(f"_get_pose({model_name}): {resp.status_message}")
    except Exception as e:
        logger.debug(f"_get_pose({model_name}): {e}")
    return None, None, None


def _set_pose(model_name: str, x: float, y: float, z: float) -> bool:
    """Teleport model to (x,y,z) and zero all velocities.

    Zeroing twist is critical for gravity=false kinematic bodies: Gazebo
    computes an implicit velocity from consecutive set_model_state positions
    and applies it between calls. Without zeroing, a probe stepped from
    z=0.04 to z=0.03 accumulates downward velocity and continues drifting
    during the capture_data window, making contact_z unreliable.
    """
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
        svc = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        # Zero all velocities so the body stays exactly where placed
        state.twist.linear.x  = 0.0
        state.twist.linear.y  = 0.0
        state.twist.linear.z  = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = 'world'
        resp = svc(state)
        if not resp.success:
            logger.warning(f"_set_pose({model_name}): {resp.status_message}")
        return resp.success
    except Exception as e:
        logger.error(f"_set_pose({model_name}): {e}")
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Force measurement helpers
# ─────────────────────────────────────────────────────────────────────────────

def _get_probe_z(probe_model: str) -> float:
    """Read actual world-frame Z of the probe after physics has settled.
    Falls back to None if the service fails."""
    _, _, z = _get_pose(probe_model)
    return z


def _measure_force(sensor, window: float = 0.4, probe_only: bool = True,
                   simulator=None) -> float:
    """Collect ContactsState for `window` seconds. Returns max force in N.

    Design notes:
    - timeout=2.0: the bumper topic can stall briefly after a set_model_state
      teleport. 0.3s was too short — capture_data returned an empty list and
      reported 0.0 N even while the probe was visibly pressing the sensor.
    - max not median: during a window some frames have contact, some don't
      (bumper publishes empty states between contacts). Median over a mix of
      contact + no-contact frames gives ~0. Max gives the actual peak force
      during the window, which is what we want for contact detection.
    - probe_only filter: collision names look like
        "force_probe::probe_link::probe_tip"
        "leptrino_cfs018ca101u::body::tactile_collision"
      Both contain 'probe' on at least one side when probe touches sensor.
      All seen pairs are logged at DEBUG so name mismatches are visible.
    """
    from gazebo_msgs.msg import ContactsState as CS
    topic = f"/{sensor.sensor_name}/bumper_states"
    msgs = sensor.capture_data(CS, topic=topic, window=window, timeout=2.0)
    magnitudes = []
    for msg in msgs:
        if not msg.states:
            continue
        for state in msg.states:
            c1 = state.collision1_name.lower()
            c2 = state.collision2_name.lower()
            logger.debug(f"  contact pair: [{c1}] x [{c2}]")
            if probe_only and 'probe' not in c1 and 'probe' not in c2:
                continue
            f = state.total_wrench.force
            magnitudes.append((f.x**2 + f.y**2 + f.z**2) ** 0.5)
    return float(max(magnitudes)) if magnitudes else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Contact search
# ─────────────────────────────────────────────────────────────────────────────

def _find_contact_surface(sensor, probe_model: str,
                          sx: float, sy: float, sensor_top_z: float,
                          simulator=None):
    """Lower probe from 20 mm above sensor_top until filtered force confirms contact.
    Returns probe Z at contact surface, or None. Probe left at contact Z.

    Timing: _measure_force blocks for window+timeout seconds per step.
    No extra sleep needed — the measure call itself paces the descent.
    Steps logged at INFO so the approach is visible without --verbose.
    """
    start_z = sensor_top_z + 0.020 + PROBE_TIP_OFFSET
    logger.info(f"  Contact search from z={start_z:.4f} "
                f"(tip {(start_z-PROBE_TIP_OFFSET-sensor_top_z)*1000:.1f} mm above surface)")
    _set_pose(probe_model, sx, sy, start_z)
    time.sleep(1.0)  # let physics settle after initial placement

    step = 0.001     # 1 mm steps — coarser but reliable; fine depths in Phase 2
    n_steps = int(0.040 / step)  # 40 mm total search range
    z = start_z
    consec = 0
    first_z = None

    for _ in range(n_steps):
        z -= step
        _set_pose(probe_model, sx, sy, z)
        f = _measure_force(sensor, window=0.3, probe_only=True, simulator=simulator)

        # Read actual position — velocity zeroing in _set_pose should keep
        # this close to z, but we log the real value so drift is visible.
        actual_z = _get_probe_z(probe_model)
        if actual_z is None:
            actual_z = z
        drift_mm = (actual_z - z) * 1000
        tip_z = actual_z - PROBE_TIP_OFFSET
        logger.info(f"  step cmd={z:.4f} actual={actual_z:.4f} (drift={drift_mm:+.2f}mm) "
                    f"tip={tip_z:.4f} gap={(tip_z-sensor_top_z)*1000:+.1f}mm f={f:.4f}N")
        if f > CONTACT_THRESHOLD:
            consec += 1
            if first_z is None:
                first_z = actual_z   # use real position, not commanded
            if consec >= CONTACT_DEBOUNCE:
                logger.info(f"  Contact confirmed: probe_z={first_z:.4f} "
                            f"tip_z={first_z-PROBE_TIP_OFFSET:.4f} f={f:.3f}N")
                _set_pose(probe_model, sx, sy, first_z)
                time.sleep(0.5)
                return first_z
        else:
            consec = 0
            first_z = None

    logger.warning(f"  No contact found ({n_steps} steps, searched to z={z:.4f})")
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Sensor geometry
# ─────────────────────────────────────────────────────────────────────────────

def _sensor_geo(sensor):
    """Return sensor geometry dict or None.

    With the new model.sdf the body link has <pose>0 0 0.013 0 0 0</pose>,
    so _get_pose('leptrino_cfs018ca101u') returns the MODEL origin (0,0,0)
    not the body link centre. We query the body link directly via
    get_link_state, or fall back to model pose + known body offset.
    """
    raw = sensor.params.get("sensor_size") or sensor.params.get("size")
    if not raw:
        logger.error("No sensor_size in params")
        return None
    sz = [float(v) for v in raw]

    # Try to read body link pose directly (most accurate)
    body_z = None
    try:
        rospy.wait_for_service('/gazebo/get_link_state', timeout=2.0)
        from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
        svc = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        req = GetLinkStateRequest()
        req.link_name = f"{sensor.sensor_name}::body"
        req.reference_frame = 'world'
        resp = svc(req)
        if resp.success:
            p = resp.link_state.pose.position
            x, y, body_z = p.x, p.y, p.z
            logger.info(f"  Body link pose: ({x:.4f},{y:.4f},{body_z:.4f})")
    except Exception as e:
        logger.debug(f"  get_link_state failed: {e}")

    if body_z is None:
        # Fallback: model origin + body offset from SDF (0.013 m)
        ax, ay, az = _get_pose(sensor.sensor_name)
        x  = ax if ax is not None else 0.0
        y  = ay if ay is not None else 0.0
        body_z = (az if az is not None else 0.0) + sz[2] / 2.0
        logger.info(f"  Body pose (fallback, model+offset): ({x:.4f},{y:.4f},{body_z:.4f})")

    top_z = body_z + sz[2] / 2.0
    logger.info(f"  Sensor centre z={body_z:.4f} top_z={top_z:.4f} "
                f"size={[round(v*1000,1) for v in sz]}mm")
    return {"x": x, "y": y, "z": body_z, "top_z": top_z, "size": sz}


# ─────────────────────────────────────────────────────────────────────────────
# T1 — Contact detection + multi-depth force characterisation
# ─────────────────────────────────────────────────────────────────────────────

def tactile_min_force_threshold(simulator, sensor, progress_cb=None) -> dict:
    """
    Phase 1: Find the sensor surface by lowering the probe in 0.5 mm steps
             until bumper reports probe contact (filtered, debounced).

    Phase 2: Apply 7 penetration depths (0.1 mm … 5 mm). At each depth,
             measure stable filtered force, lift back to rest, repeat.
             Reports full force-displacement table.

    Passes if contact surface is found AND minimum detected force ≤ 2.0 N.
    """
    logger.info("=" * 60)
    logger.info("  T1: Contact Detection + Force-Depth Characterisation")
    logger.info("=" * 60)
    result = {
        "passed": False, "test_name": "T1 – Contact Detection + Force-Depth",
        "description": None, "error": None,
        "contact_found": False, "contact_z": None, "sensor_top_z": None,
        "noise_n": None, "force_depth_table": [],
        "min_detected_n": None, "threshold_n": 2.0,
    }
    t0 = time.time()

    if progress_cb: progress_cb(5)
    if not simulator.open_scene(Worlds.TACTILE_FORCE, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time()-t0, 2); return result
    time.sleep(3)

    probe = "force_probe"
    if not simulator.wait_for_model_spawn(probe, 30):
        result["error"] = f"Probe '{probe}' not spawned"
        result["duration"] = round(time.time()-t0, 2); return result

    if progress_cb: progress_cb(10)
    geo = _sensor_geo(sensor)
    if geo is None:
        result["error"] = "No sensor_size in params"
        result["duration"] = round(time.time()-t0, 2); return result
    sx, sy, sensor_top = geo["x"], geo["y"], geo["top_z"]
    result["sensor_top_z"] = round(sensor_top, 4)

    # Baseline noise
    if progress_cb: progress_cb(12)
    logger.info("  Baseline noise (probe at z=0.30) ...")
    _set_pose(probe, sx, sy, 0.30)
    time.sleep(1.5)
    noise = _measure_force(sensor, window=0.6, probe_only=True, simulator=simulator)
    result["noise_n"] = round(noise, 4)
    logger.info(f"  Filtered noise = {noise:.4f} N  "
                f"({'OK' if noise < 0.3 else 'HIGH — check collision names'})")

    # Phase 1: find surface
    if progress_cb: progress_cb(15)
    logger.info("  Phase 1: finding contact surface ...")
    contact_z = _find_contact_surface(sensor, probe, sx, sy, sensor_top, simulator=simulator)
    if contact_z is None:
        result["error"] = "Contact surface not found in 30 mm sweep"
        result["description"] = ("Probe swept 30 mm and detected no sensor contact. "
                                 "Check lateral alignment and probe tip offset.")
        result["duration"] = round(time.time()-t0, 2); return result

    result["contact_found"] = True
    result["contact_z"]     = round(contact_z, 4)
    diff_mm = (contact_z - PROBE_TIP_OFFSET - sensor_top) * 1000
    logger.info(f"  Contact z={contact_z:.4f}, tip_z={contact_z-PROBE_TIP_OFFSET:.4f}, "
                f"offset from expected surface: {diff_mm:+.2f} mm")

    # Phase 2: force-displacement characterisation
    if progress_cb: progress_cb(25)
    logger.info(f"  Phase 2: {len(PROBE_DEPTHS_M)} penetration depths ...")
    logger.info(f"  {'Depth mm':>10}  {'Probe Z':>9}  {'Force N':>9}")
    logger.info(f"  {'--------':>10}  {'-------':>9}  {'-------':>9}")

    table     = []
    min_force = None
    rest_z    = contact_z + 0.005  # lift 5 mm between depths

    for i, depth in enumerate(PROBE_DEPTHS_M):
        probe_z = contact_z - depth
        _set_pose(probe, sx, sy, probe_z)
        force = _measure_force(sensor, window=0.5, probe_only=True, simulator=simulator)

        # Read actual Z to confirm probe did not drift during measurement
        actual_probe_z = _get_probe_z(probe)
        if actual_probe_z is None:
            actual_probe_z = probe_z
        actual_depth = (contact_z - actual_probe_z) * 1000  # mm, positive = into sensor

        if force > 0.0 and min_force is None:
            min_force = force

        table.append({"depth_mm": round(actual_depth, 3), "force_n": round(force, 4)})
        tag = "  <- first detection" if (force > 0.0 and len([r for r in table if r["force_n"] > 0]) == 1) else ""
        drift_mm = (actual_probe_z - probe_z) * 1000
        logger.info(f"  {actual_depth:>10.3f}mm  cmd={probe_z:.4f} actual={actual_probe_z:.4f} "
                    f"(drift={drift_mm:+.2f}mm)  f={force:.4f}N{tag}")

        _set_pose(probe, sx, sy, rest_z)
        if progress_cb:
            progress_cb(25 + int((i+1)/len(PROBE_DEPTHS_M)*65))

    result["force_depth_table"] = table
    result["min_detected_n"]    = round(min_force, 4) if min_force is not None else None

    if min_force is None:
        result["passed"]      = False
        result["description"] = ("Contact found but all force readings = 0 N. "
                                 "Bumper may not report probe-sensor contacts.")
    elif min_force <= result["threshold_n"]:
        result["passed"]      = True
        result["description"] = (f"Min detectable force {min_force:.4f} N "
                                 f"<= {result['threshold_n']} N threshold.")
    else:
        result["passed"]      = False
        result["description"] = (f"Min detectable force {min_force:.4f} N "
                                 f"> {result['threshold_n']} N threshold.")

    logger.info(f"  {'PASSED' if result['passed'] else 'FAILED'}: {result['description']}")
    result["duration"] = round(time.time()-t0, 2)
    if progress_cb: progress_cb(100)
    return result


# ─────────────────────────────────────────────────────────────────────────────
# T2, T3, T4 — stubbed (restore after T1 is confirmed working)
# ─────────────────────────────────────────────────────────────────────────────

def tactile_response_uniformity(simulator, sensor, progress_cb=None) -> dict:
    """
    T2: Response Uniformity – contact detection across a grid of points.
    """
    logger.info("=" * 60)
    logger.info("  T2: Response Uniformity")
    logger.info("=" * 60)

    result = {
        "passed": False,
        "test_name": "T2 – Response Uniformity",
        "description": None,
        "error": None,
        "points_tested": 0,
        "points_passed": 0,
        "point_results": [],
    }
    t0 = time.time()

    if progress_cb:
        progress_cb(5)

    if not simulator.open_scene(Worlds.TACTILE_UNIFORMITY, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result
    time.sleep(3)

    probe = "force_probe"
    if not simulator.wait_for_model_spawn(probe, 30):
        result["error"] = f"Probe '{probe}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        progress_cb(10)

    geo = _sensor_geo(sensor)
    if geo is None:
        result["error"] = "No sensor_size in params"
        result["duration"] = round(time.time() - t0, 2)
        return result
    sx, sy, sensor_top = geo["x"], geo["y"], geo["top_z"]

    # Uniformity grid for a cylindrical sensor (radius ~9 mm)
    # Centre + 6 points on a 5 mm radius ring (well within active area)
    ring_radius_m = 0.005
    angles = [0, 60, 120, 180, 240, 300]
    points = [(0.0, 0.0)]  # centre
    for ang in angles:
        rad = math.radians(ang)
        points.append((ring_radius_m * math.cos(rad), ring_radius_m * math.sin(rad)))

    logger.info(f"  Testing {len(points)} grid points:")
    for i, (dx, dy) in enumerate(points):
        logger.info(f"    Point {i+1}: x={sx+dx:.4f}, y={sy+dy:.4f}")

    # For uniformity test we use a lower threshold (sphere contact area smaller)
    UNIFORMITY_THRESHOLD = 0.3  # N
    PROBE_TIP_OFFSET_SPHERE = 0.002  # sphere radius

    for i, (dx, dy) in enumerate(points):
        px = sx + dx
        py = sy + dy
        logger.info(f"--- Point {i+1}/{len(points)}: ({px:.4f}, {py:.4f}) ---")

        if progress_cb:
            progress_cb(10 + int((i / len(points)) * 80))

        # Contact search (similar to T1 but with finer step)
        start_z = sensor_top + 0.020 + PROBE_TIP_OFFSET_SPHERE
        _set_pose(probe, px, py, start_z)
        time.sleep(0.5)

        contact_z = None
        step = 0.0005  # 0.5 mm steps
        n_steps = int(0.040 / step)
        z = start_z
        consec = 0
        for _ in range(n_steps):
            z -= step
            _set_pose(probe, px, py, z)
            f = _measure_force(sensor, window=0.2, probe_only=True, simulator=simulator)
            actual_z = _get_probe_z(probe)
            if actual_z is None:
                actual_z = z
            tip_z = actual_z - PROBE_TIP_OFFSET_SPHERE
            logger.debug(f"    z={z:.4f} actual={actual_z:.4f} tip={tip_z:.4f} f={f:.4f}N")
            if f > UNIFORMITY_THRESHOLD:
                consec += 1
                if contact_z is None:
                    contact_z = actual_z
                if consec >= 2:
                    logger.info(f"    Contact found at z={contact_z:.4f} (f={f:.4f}N)")
                    break
            else:
                consec = 0
                contact_z = None
        else:
            contact_z = None

        point_passed = contact_z is not None
        result["points_tested"] += 1
        if point_passed:
            result["points_passed"] += 1

        result["point_results"].append({
            "index": i+1,
            "x": px,
            "y": py,
            "passed": point_passed,
            "contact_z": round(contact_z, 5) if contact_z else None,
        })

        # Lift probe high before moving to next point
        _set_pose(probe, px, py, 0.30)
        time.sleep(0.3)

    # Overall pass/fail
    if result["points_passed"] == result["points_tested"]:
        result["passed"] = True
        result["description"] = f"Contact detected at all {result['points_tested']} points."
    else:
        result["passed"] = False
        result["description"] = (
            f"Contact detected at {result['points_passed']}/{result['points_tested']} points. "
            f"Failed points: {[r['index'] for r in result['point_results'] if not r['passed']]}"
        )

    logger.info(f"  {'PASSED' if result['passed'] else 'FAILED'}: {result['description']}")
    result["duration"] = round(time.time() - t0, 2)
    if progress_cb:
        progress_cb(100)
    return result


def tactile_temporal_stability(simulator, sensor, progress_cb=None) -> dict:
    """
    T3: Temporal Stability – constant load over time.
    """
    logger.info("=" * 60)
    logger.info("  T3: Temporal Stability")
    logger.info("=" * 60)

    result = {
        "passed": False,
        "test_name": "T3 – Temporal Stability",
        "description": None,
        "error": None,
        "hold_duration_s": 10.0,
        "target_force_n": 0.5,            # stop stepping when force reaches this
        "max_penetration_mm": 5.0,        # safety limit
        "step_size_mm": 0.2,
        "force_samples": [],
        "mean_force_n": None,
        "std_force_n": None,
        "cv_percent": None,
        "contact_lost": False,
    }
    t0 = time.time()

    if progress_cb:
        progress_cb(5)

    if not simulator.open_scene(Worlds.TACTILE_FORCE, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result
    time.sleep(3)

    probe = "force_probe"
    if not simulator.wait_for_model_spawn(probe, 30):
        result["error"] = f"Probe '{probe}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        progress_cb(10)

    # Accurate sensor geometry
    geo = _sensor_geo(sensor)
    if geo is None:
        result["error"] = "No sensor_size in params"
        result["duration"] = round(time.time() - t0, 2)
        return result
    sx, sy, sensor_top = geo["x"], geo["y"], geo["top_z"]
    logger.info(f"  Sensor geometry: centre X={sx:.4f} Y={sy:.4f} Z={geo['z']:.4f}")
    logger.info(f"  Sensor top surface Z = {sensor_top:.4f}")
    logger.info(f"  PROBE_TIP_OFFSET = {PROBE_TIP_OFFSET*1000:.1f} mm")

    # Phase 1: Position probe at rest (5 mm above sensor top)
    rest_z = sensor_top + PROBE_TIP_OFFSET + 0.005
    logger.info(f"  Phase 1: Moving probe to rest Z = {rest_z:.4f} (tip {sensor_top+0.005:.4f})")
    _set_pose(probe, sx, sy, rest_z)
    time.sleep(1.0)

    if progress_cb:
        progress_cb(20)

    # Phase 2: Step down gradually until target force is reached
    step_m = result["step_size_mm"] / 1000.0
    max_penetration_m = result["max_penetration_mm"] / 1000.0
    target_force = result["target_force_n"]
    current_z = rest_z
    hold_z = None
    final_force = 0.0

    logger.info(f"  Phase 2: Stepping down until force >= {target_force} N (max {result['max_penetration_mm']} mm)")
    logger.info(f"  {'Step':>5} {'Cmd Z':>9} {'Actual Z':>9} {'Tip Z':>9} {'Pen (mm)':>9} {'Force (N)':>10}")
    logger.info(f"  {'----':>5} {'-----':>9} {'--------':>9} {'-----':>9} {'-------':>9} {'--------':>10}")

    step_count = 0
    while True:
        step_count += 1
        # Move down by one step
        next_z = current_z - step_m
        _set_pose(probe, sx, sy, next_z)
        time.sleep(0.3)  # physics settle

        actual_z = _get_probe_z(probe)
        tip_z = actual_z - PROBE_TIP_OFFSET if actual_z else None
        penetration_mm = (sensor_top - tip_z) * 1000 if tip_z else 0.0
        f = _measure_force(sensor, window=0.2, probe_only=True, simulator=simulator)

        logger.info(f"  {step_count:>5} {next_z:>9.4f} {actual_z if actual_z else 'N/A':>9} "
                    f"{tip_z if tip_z else 'N/A':>9} {penetration_mm:>9.2f} {f:>10.4f}")

        # Stop conditions
        if f >= target_force:
            hold_z = next_z
            final_force = f
            logger.info(f"  Target force reached at step {step_count}")
            break
        if penetration_mm >= result["max_penetration_mm"]:
            hold_z = next_z
            final_force = f
            logger.warning(f"  Max penetration reached before target force")
            break

        current_z = next_z

    if hold_z is None:
        result["error"] = "Failed to establish contact"
        result["duration"] = round(time.time() - t0, 2)
        return result

    # Final settle at hold position
    time.sleep(1.5)
    f_initial = _measure_force(sensor, window=1.0, probe_only=True, simulator=simulator)
    actual_hold_z = _get_probe_z(probe)
    logger.info(f"  Settled at hold: actual Z = {actual_hold_z:.4f}, force = {f_initial:.4f} N")

    if f_initial < CONTACT_THRESHOLD:
        result["error"] = f"Force dropped below threshold after settle: {f_initial:.4f} N"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        progress_cb(30)

    # Phase 3: Hold and monitor
    hold_duration = result["hold_duration_s"]
    sample_interval = 0.5
    num_samples = int(hold_duration / sample_interval)
    force_samples = []
    contact_lost = False

    logger.info(f"  Phase 3: Holding for {hold_duration} s (sampling every {sample_interval} s)")
    logger.info(f"  {'Time (s)':>10} {'Force (N)':>12} {'Status':>10}")
    logger.info(f"  {'--------':>10} {'---------':>12} {'------':>10}")

    for i in range(num_samples):
        elapsed = (i + 1) * sample_interval
        if progress_cb:
            progress_cb(30 + int((elapsed / hold_duration) * 40))

        time.sleep(sample_interval)
        f = _measure_force(sensor, window=0.3, probe_only=True, simulator=simulator)
        force_samples.append(f)

        if f < CONTACT_THRESHOLD:
            contact_lost = True
            status = "LOST"
        else:
            status = "OK"

        logger.info(f"  {elapsed:>10.1f} {f:>12.4f} {status:>10}")

    result["force_samples"] = [round(f, 4) for f in force_samples]
    result["contact_lost"] = contact_lost

    if force_samples:
        mean_f = np.mean(force_samples)
        std_f = np.std(force_samples)
        cv = (std_f / mean_f) * 100 if mean_f > 0 else float('inf')
        result["mean_force_n"] = round(mean_f, 4)
        result["std_force_n"] = round(std_f, 4)
        result["cv_percent"] = round(cv, 2)
        logger.info(f"  Statistics: mean={mean_f:.4f} N, std={std_f:.4f} N, CV={cv:.2f}%")
    else:
        result["error"] = "No force samples collected"
        result["duration"] = round(time.time() - t0, 2)
        return result

    # Phase 4: Lift and verify return to baseline
    logger.info("  Phase 4: Lifting probe to rest...")
    _set_pose(probe, sx, sy, rest_z)
    time.sleep(1.0)
    f_final = _measure_force(sensor, window=0.5, probe_only=True, simulator=simulator)
    logger.info(f"  Force after lift: {f_final:.4f} N")

    MAX_CV_PERCENT = 10.0
    if contact_lost:
        result["passed"] = False
        result["description"] = "Contact lost during hold period."
    elif cv > MAX_CV_PERCENT:
        result["passed"] = False
        result["description"] = f"Force variation {cv:.2f}% exceeds {MAX_CV_PERCENT}% limit."
    elif f_final > CONTACT_THRESHOLD:
        result["passed"] = False
        result["description"] = f"Force did not return to zero after lift ({f_final:.4f} N)."
    else:
        result["passed"] = True
        result["description"] = (
            f"Force stable: mean {mean_f:.4f} N, CV {cv:.2f}%, "
            f"contact maintained, returned to zero."
        )

    logger.info(f"  {'PASSED' if result['passed'] else 'FAILED'}: {result['description']}")
    result["duration"] = round(time.time() - t0, 2)
    if progress_cb:
        progress_cb(100)
    return result

def tactile_peak_load_response(simulator, sensor, progress_cb=None) -> dict:
    """
    T4: Peak Load Response – sudden impact (simulated drop).
    """
    logger.info("=" * 60)
    logger.info("  T4: Peak Load Response")
    logger.info("=" * 60)

    result = {
        "passed": False,
        "test_name": "T4 – Peak Load Response",
        "description": None,
        "error": None,
        "drop_height_m": 0.030,          # above contact surface
        "impact_velocity_m_s": -1.0,      # downward
        "peak_force_n": None,
        "saturation_limit_n": 100.0,
        "saturated": False,
        "force_time_series": [],
        "post_impact_settled": False,
    }
    t0 = time.time()

    if progress_cb:
        progress_cb(5)

    if not simulator.open_scene(Worlds.TACTILE_FORCE, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result
    time.sleep(3)

    probe = "force_probe"
    if not simulator.wait_for_model_spawn(probe, 30):
        result["error"] = f"Probe '{probe}' not spawned"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        progress_cb(10)

    geo = _sensor_geo(sensor)
    if geo is None:
        result["error"] = "No sensor_size in params"
        result["duration"] = round(time.time() - t0, 2)
        return result
    sx, sy, sensor_top = geo["x"], geo["y"], geo["top_z"]

    # Phase 1: Find contact surface
    logger.info("  Phase 1: Finding contact surface...")
    contact_z = _find_contact_surface(sensor, probe, sx, sy, sensor_top, simulator=simulator)
    if contact_z is None:
        result["error"] = "Contact surface not found"
        result["duration"] = round(time.time() - t0, 2)
        return result
    logger.info(f"  Contact surface z = {contact_z:.4f}")

    if progress_cb:
        progress_cb(30)

    # Phase 2: Position probe at drop height and set velocity
    drop_height = result["drop_height_m"]
    start_z = contact_z + drop_height
    logger.info(f"  Phase 2: Moving probe to drop height z={start_z:.4f} ({drop_height*1000:.1f} mm above surface)")
    _set_pose(probe, sx, sy, start_z)
    time.sleep(0.5)

    # Apply downward velocity via set_model_state (gravity is false on probe)
    logger.info(f"  Applying initial velocity: {result['impact_velocity_m_s']} m/s downward")
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
        svc = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = probe
        state.pose.position.x = sx
        state.pose.position.y = sy
        state.pose.position.z = start_z
        state.pose.orientation.w = 1.0
        state.twist.linear.z = result["impact_velocity_m_s"]
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = 'world'
        resp = svc(state)
        if not resp.success:
            logger.warning(f"set_model_state with velocity failed: {resp.status_message}")
    except Exception as e:
        result["error"] = f"Failed to set velocity: {e}"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        progress_cb(40)

    # Phase 3: Capture high‑rate force data during impact
    logger.info("  Phase 3: Capturing impact force data for 1.0 second at ~50 Hz...")
    capture_duration = 1.0
    sample_interval = 0.02  # 50 Hz
    num_samples = int(capture_duration / sample_interval)
    force_series = []
    timestamps = []

    start_capture = time.time()
    for i in range(num_samples):
        # We cannot use sensor.capture_data for real‑time streaming; we'll poll the last message
        # by using a short window capture each time.
        f = _measure_force(sensor, window=0.02, probe_only=True, simulator=simulator)
        force_series.append(f)
        timestamps.append(time.time() - start_capture)
        time.sleep(sample_interval)

    result["force_time_series"] = [[round(t, 3), round(f, 4)] for t, f in zip(timestamps, force_series)]

    # Phase 4: Analyze results
    peak_force = max(force_series) if force_series else 0.0
    result["peak_force_n"] = round(peak_force, 4)
    result["saturated"] = peak_force >= result["saturation_limit_n"]

    # Check post‑impact settling (last 0.2 s average)
    settle_window = 0.2
    settle_samples = int(settle_window / sample_interval)
    if len(force_series) >= settle_samples:
        post_impact_avg = np.mean(force_series[-settle_samples:])
        result["post_impact_avg_n"] = round(post_impact_avg, 4)
        result["post_impact_settled"] = post_impact_avg < CONTACT_THRESHOLD
    else:
        result["post_impact_avg_n"] = None
        result["post_impact_settled"] = False

    logger.info(f"  Peak force: {peak_force:.4f} N")
    logger.info(f"  Saturation limit: {result['saturation_limit_n']} N, saturated: {result['saturated']}")
    logger.info(f"  Post‑impact settled: {result['post_impact_settled']} (avg {result.get('post_impact_avg_n', 'N/A')} N)")

    # Pass criteria
    if peak_force < CONTACT_THRESHOLD:
        result["passed"] = False
        result["description"] = f"Impact not detected (peak force {peak_force:.4f} N)."
    elif result["saturated"]:
        result["passed"] = False
        result["description"] = f"Sensor saturated (peak {peak_force:.4f} N >= {result['saturation_limit_n']} N)."
    elif not result["post_impact_settled"]:
        result["passed"] = False
        result["description"] = "Force did not return to zero after impact."
    else:
        result["passed"] = True
        result["description"] = f"Impact detected (peak {peak_force:.4f} N), no saturation, returned to baseline."

    logger.info(f"  {'PASSED' if result['passed'] else 'FAILED'}: {result['description']}")
    result["duration"] = round(time.time() - t0, 2)
    if progress_cb:
        progress_cb(100)
    return result