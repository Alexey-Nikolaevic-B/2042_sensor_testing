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
# Sensor geometry — shape-agnostic
# ─────────────────────────────────────────────────────────────────────────────

def _parse_sensor_shape(sensor):
    """Extract shape geometry from sensor.params. Returns dict or None.

    Priority order for shape determination:
      1. Explicit radius+length params           → cylinder
      2. Explicit shape="cylinder"/"box" param   → use that, get dims from sensor_size/size
      3. sensor_size/size = [w, d, h] fallback:
           w != d (>5% difference)               → box
           w == d but radius param also exists   → cylinder (diameter check)
           w == d, no radius param               → AMBIGUOUS: default to box with warning.
             A square sensor_size like [0.152,0.152,0.029] is almost certainly a box
             force plate, not a cylinder. Only classify as cylinder if explicitly told.

    Returned dict keys:
      shape           "box" | "cylinder"
      height_m        total body height (metres)
      safe_xy_radius  largest circle fully inside active area × 0.80 margin
      width_m         (box) full x size
      depth_m         (box) full y size
      radius_m        (cylinder) outer radius
    """
    # ── 1. explicit radius + length params ───────────────────────────
    r_raw = sensor.params.get("radius")
    l_raw = sensor.params.get("length")
    if r_raw is not None and l_raw is not None:
        r, l = float(r_raw), float(l_raw)
        logger.info(f"  Shape: cylinder  r={r*1000:.1f}mm  l={l*1000:.1f}mm  "
                    f"[from radius+length params]")
        return {"shape": "cylinder", "height_m": l,
                "safe_xy_radius": r * 0.80, "radius_m": r}

    # ── 2. explicit shape string param ───────────────────────────────
    explicit_shape = (sensor.params.get("shape") or "").strip().lower()

    # ── 3. sensor_size / size list ───────────────────────────────────
    raw = sensor.params.get("sensor_size") or sensor.params.get("size")
    if raw is None:
        logger.error("  No shape params found. Sensor SDF needs sensor_size, size, "
                     "or radius+length params.")
        return None
    sz = [float(v) for v in raw]
    if len(sz) < 3:
        logger.error(f"  sensor_size has {len(sz)} values, need at least 3: {sz}")
        return None
    w, d, h = sz[0], sz[1], sz[2]

    # Explicit shape param overrides heuristic
    if explicit_shape == "cylinder":
        r = w / 2.0
        logger.info(f"  Shape: cylinder  r={r*1000:.1f}mm  h={h*1000:.1f}mm  "
                    f"[explicit shape=cylinder, sensor_size={[round(v*1000,1) for v in sz]}mm]")
        return {"shape": "cylinder", "height_m": h,
                "safe_xy_radius": r * 0.80, "radius_m": r}

    if explicit_shape == "box":
        logger.info(f"  Shape: box  w={w*1000:.1f}mm  d={d*1000:.1f}mm  h={h*1000:.1f}mm  "
                    f"[explicit shape=box]")
        return {"shape": "box", "height_m": h,
                "safe_xy_radius": min(w, d) / 2.0 * 0.80,
                "width_m": w, "depth_m": d}

    # Fallback heuristic — only used if shape param is missing from SDF
    w_d_ratio = abs(w - d) / max(w, d)
    if w_d_ratio > 0.05:
        # Clearly rectangular
        logger.info(f"  Shape: box (w≠d, ratio={w_d_ratio:.2f})  "
                    f"w={w*1000:.1f}mm d={d*1000:.1f}mm h={h*1000:.1f}mm")
        return {"shape": "box", "height_m": h,
                "safe_xy_radius": min(w, d) / 2.0 * 0.80,
                "width_m": w, "depth_m": d}
    else:
        # w ≈ d — could be square box or cylinder.
        # Default to BOX. A cylinder should have shape=cylinder in its SDF.
        logger.warning(f"  sensor_size has w≈d ({w*1000:.1f}mm) but no shape param — "
                       f"defaulting to BOX. Add <shape>cylinder</shape> to sensor SDF "
                       f"if this is a cylindrical sensor.")
        return {"shape": "box", "height_m": h,
                "safe_xy_radius": min(w, d) / 2.0 * 0.80,
                "width_m": w, "depth_m": d}


def _sensor_geo(sensor):
    """Return sensor geometry dict or None.

    Queries /gazebo/get_link_state for the body link (most accurate).
    Falls back to model pose + half-height offset if unavailable.

    Returned dict keys:
      x, y, z    — body link centre in world frame
      top_z      — z of the top contact surface
      shape_info — dict from _parse_sensor_shape()
    """
    shape = _parse_sensor_shape(sensor)
    if shape is None:
        return None
    h = shape["height_m"]

    # ── query body link pose ──────────────────────────────────────────
    body_z = None
    x, y = 0.0, 0.0
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
            logger.info(f"  Body link pose (get_link_state): "
                        f"({x:.4f},{y:.4f},{body_z:.4f})")
    except Exception as e:
        logger.debug(f"  get_link_state failed: {e}")

    if body_z is None:
        ax, ay, az = _get_pose(sensor.sensor_name)
        x      = ax if ax is not None else 0.0
        y      = ay if ay is not None else 0.0
        body_z = (az if az is not None else 0.0) + h / 2.0
        logger.info(f"  Body link pose (fallback model+h/2): "
                    f"({x:.4f},{y:.4f},{body_z:.4f})")

    top_z = body_z + h / 2.0
    logger.info(f"  top_z={top_z:.4f}  height={h*1000:.1f}mm  "
                f"safe_xy_radius={shape['safe_xy_radius']*1000:.1f}mm")
    return {"x": x, "y": y, "z": body_z, "top_z": top_z, "shape_info": shape}


def _grid_points(geo: dict, probe_radius_m: float) -> list:
    """Generate test points covering the sensor active area.

    For both box and cylinder sensors, produces:
      - 1 centre point
      - For cylinder: 6 points on a ring at 65% of safe_xy_radius
      - For box: 8 points on a ring + centre, or a 3x3 grid fitting inside
        the box with the probe radius as margin

    Each point is (label, world_x, world_y).
    Ring radius is chosen so the probe (radius probe_radius_m) stays fully
    inside the sensor boundary with margin.
    """
    sx, sy = geo["x"], geo["y"]
    shape  = geo["shape_info"]
    safe_r = shape["safe_xy_radius"] - probe_radius_m

    if safe_r <= 0:
        logger.warning(f"  Probe radius {probe_radius_m*1000:.1f}mm >= "
                       f"safe_xy_radius {shape['safe_xy_radius']*1000:.1f}mm — "
                       f"using centre only")
        return [("C", sx, sy)]

    points = [("C", sx, sy)]

    if shape["shape"] == "cylinder":
        # 6 points on a hexagonal ring at 65% of safe_r
        ring_r = safe_r * 0.65
        for i in range(6):
            a = math.radians(i * 60)
            points.append((f"P{i+1}",
                           sx + ring_r * math.cos(a),
                           sy + ring_r * math.sin(a)))
    else:
        # Box: 3×3 grid with probe_radius margin from edges
        # Grid spans ±grid_r in both axes
        w_half = shape["width_m"]  / 2.0 - probe_radius_m
        d_half = shape["depth_m"]  / 2.0 - probe_radius_m
        # Use 60% of half-extents for inner ring to stay safely away from edges
        gw = w_half * 0.60
        gd = d_half * 0.60
        offsets = [(-gw, -gd), (0, -gd), (gw, -gd),
                   (-gw,  0 ),            (gw,  0 ),
                   (-gw,  gd), (0,  gd), (gw,  gd)]
        for i, (dx, dy) in enumerate(offsets):
            points.append((f"P{i+1}", sx + dx, sy + dy))

    logger.info(f"  Grid: {len(points)} points  "
                f"(shape={shape['shape']}  safe_r={safe_r*1000:.1f}mm  "
                f"probe_r={probe_radius_m*1000:.1f}mm)")
    for label, px, py in points:
        r = math.hypot(px - sx, py - sy)
        logger.info(f"    {label}: world=({px:.4f},{py:.4f})  "
                    f"r={r*1000:.1f}mm from centre")
    return points


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
    safe_height = sensor_top + 0.050 + PROBE_TIP_OFFSET   # 50mm above surface

    # Baseline noise
    if progress_cb: progress_cb(12)
    logger.info(f"  Baseline noise (probe at safe height z={safe_height:.3f}) ...")
    _set_pose(probe, sx, sy, safe_height)
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

    # Sphere probe tip offset (sphere radius in tactile_uniformity.world)
    T2_TIP_OFFSET = 0.002  # metres — sphere radius

    # Generate grid from actual sensor shape/size
    grid = _grid_points(geo, probe_radius_m=T2_TIP_OFFSET)
    safe_height = sensor_top + 0.050 + T2_TIP_OFFSET

    logger.info(f"  Grid: {len(grid)} points  sensor={geo['shape_info']['shape']}  "
                f"safe_xy_radius={geo['shape_info']['safe_xy_radius']*1000:.1f}mm")

    UNIFORMITY_THRESHOLD = 0.3  # N — lower threshold for small sphere probe

    for i, (label, px, py) in enumerate(grid):
        r_from_centre = math.hypot(px - sx, py - sy)
        logger.info(f"  ── Point {i+1}/{len(grid)}: {label} "
                    f"({px:.4f},{py:.4f})  r={r_from_centre*1000:.1f}mm ──")

        if progress_cb:
            progress_cb(10 + int((i / len(grid)) * 80))

        # Move to safe height at this XY first
        _set_pose(probe, px, py, safe_height)
        time.sleep(0.5)

        # Contact search with 1 mm steps, 50 mm range, debounced x2
        contact_z = None
        start_z = sensor_top + 0.020 + T2_TIP_OFFSET
        step = 0.001
        n_steps = int(0.050 / step)
        z = start_z
        consec = 0
        _set_pose(probe, px, py, start_z)
        time.sleep(0.5)
        for _ in range(n_steps):
            z -= step
            _set_pose(probe, px, py, z)
            f = _measure_force(sensor, window=0.3, probe_only=True, simulator=simulator)
            actual_z = _get_probe_z(probe) or z
            drift_mm = (actual_z - z) * 1000
            tip_z    = actual_z - T2_TIP_OFFSET
            logger.info(f"    cmd={z:.4f} actual={actual_z:.4f} "
                        f"(drift={drift_mm:+.2f}mm) tip={tip_z:.4f} f={f:.4f}N")
            if f > UNIFORMITY_THRESHOLD:
                consec += 1
                if contact_z is None:
                    contact_z = actual_z
                if consec >= 2:
                    logger.info(f"    Contact confirmed: z={contact_z:.4f} f={f:.4f}N")
                    break
            else:
                consec = 0
                contact_z = None
        else:
            logger.warning(f"    No contact found after {n_steps} steps")
            contact_z = None

        point_passed = contact_z is not None
        result["points_tested"] += 1
        if point_passed:
            result["points_passed"] += 1

        result["point_results"].append({
            "index": i + 1,
            "label": label,
            "x": round(px, 5),
            "y": round(py, 5),
            "r_mm": round(r_from_centre * 1000, 2),
            "passed": point_passed,
            "contact_z": round(contact_z, 5) if contact_z else None,
        })

        # Lift to safe height before next point
        _set_pose(probe, px, py, safe_height)
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
    T3: Temporal Stability – exact copy of T1 Phase 2, repeating a single depth.
    """
    logger.info("=" * 60)
    logger.info("  T3: Temporal Stability")
    logger.info("=" * 60)
    result = {
        "passed": False,
        "test_name": "T3 – Temporal Stability",
        "description": None,
        "error": None,
        "force_samples": [],
        "mean_force_n": None,
        "std_force_n": None,
        "cv_percent": None,
        "failed_cycles": 0,
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
    safe_height = sensor_top + 0.050 + PROBE_TIP_OFFSET

    # Baseline noise
    if progress_cb: progress_cb(12)
    logger.info(f"  Baseline noise (probe at safe height z={safe_height:.3f}) ...")
    _set_pose(probe, sx, sy, safe_height)
    time.sleep(1.5)
    noise = _measure_force(sensor, window=0.6, probe_only=True, simulator=simulator)
    logger.info(f"  Filtered noise = {noise:.4f} N  "
                f"({'OK' if noise < 0.3 else 'HIGH — check collision names'})")

    # Phase 1: find surface (exactly as T1)
    if progress_cb: progress_cb(15)
    logger.info("  Phase 1: finding contact surface ...")
    contact_z = _find_contact_surface(sensor, probe, sx, sy, sensor_top, simulator=simulator)
    if contact_z is None:
        result["error"] = "Contact surface not found"
        result["duration"] = round(time.time()-t0, 2); return result

    logger.info(f"  Contact z={contact_z:.4f}, tip_z={contact_z-PROBE_TIP_OFFSET:.4f}")

    # Phase 2: repeated constant-depth measurements (exact T1 loop)
    if progress_cb: progress_cb(25)
    rest_z = contact_z + 0.005   # lift 5 mm between measurements

    # Use the depth that produced force in T1: 3.0 mm
    tap_depth_m = 0.003           # 3.0 mm
    num_cycles = 10               # for 10 seconds at ~1 Hz

    logger.info(f"  Phase 2: {num_cycles} cycles at {tap_depth_m*1000:.1f} mm depth ...")
    logger.info(f"  {'Cycle':>6}  {'Depth mm':>10}  {'Probe Z':>9}  {'Force N':>9}")
    logger.info(f"  {'-----':>6}  {'--------':>10}  {'-------':>9}  {'-------':>9}")

    force_samples = []
    failed_cycles = 0

    for i in range(num_cycles):
        probe_z = contact_z - tap_depth_m
        _set_pose(probe, sx, sy, probe_z)
        force = _measure_force(sensor, window=0.5, probe_only=True, simulator=simulator)

        actual_probe_z = _get_probe_z(probe)
        if actual_probe_z is None:
            actual_probe_z = probe_z
        actual_depth = (contact_z - actual_probe_z) * 1000

        if force > 0.0:
            force_samples.append(force)
        else:
            failed_cycles += 1

        drift_mm = (actual_probe_z - probe_z) * 1000
        logger.info(f"  {i+1:>6}  {actual_depth:>10.3f}  {actual_probe_z:>9.4f}  {force:>9.4f}")

        # Lift to rest after measurement (exactly as T1)
        _set_pose(probe, sx, sy, rest_z)
        time.sleep(0.3)

        if progress_cb:
            progress_cb(25 + int((i+1)/num_cycles * 65))

    if len(force_samples) == 0:
        result["error"] = "No force detected in any cycle"
        result["duration"] = round(time.time()-t0, 2); return result

    mean_f = np.mean(force_samples)
    std_f = np.std(force_samples)
    cv = (std_f / mean_f) * 100 if mean_f > 0 else float('inf')

    result["mean_force_n"] = round(mean_f, 4)
    result["std_force_n"] = round(std_f, 4)
    result["cv_percent"] = round(cv, 2)
    result["failed_cycles"] = failed_cycles

    # Relaxed CV threshold to account for simulation transients
    MAX_CV_PERCENT = 25.0
    if failed_cycles > 0:
        result["passed"] = False
        result["description"] = f"{failed_cycles} cycle(s) failed."
    elif cv > MAX_CV_PERCENT:
        result["passed"] = False
        result["description"] = f"Force variation {cv:.2f}% exceeds {MAX_CV_PERCENT}% limit."
    else:
        result["passed"] = True
        result["description"] = f"Stable response: mean {mean_f:.4f} N, CV {cv:.2f}%"

    logger.info(f"  {'PASSED' if result['passed'] else 'FAILED'}: {result['description']}")
    result["duration"] = round(time.time()-t0, 2)
    if progress_cb: progress_cb(100)
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
    safe_height = sensor_top + 0.050 + PROBE_TIP_OFFSET

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