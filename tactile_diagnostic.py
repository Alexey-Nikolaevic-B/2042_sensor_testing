#!/usr/bin/env python3
"""
Tactile sensor diagnostic — fast, verbose, single-purpose.

Usage:
    python tactile_diagnostic.py --sensor leptrino_cfs018ca101u
"""

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

from config import CONFIG
from src.core import Core
from src.sensor import Sensor
import src.sensor_storage as db
from run_all_tests_headless import setup_logging, wait_for_roscore

logger = logging.getLogger("tactile_diag")


def _get_model_pose(model_name):
    try:
        import rospy as _rospy
        from gazebo_msgs.srv import GetModelState, GetModelStateRequest
        _rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        svc = _rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        req = GetModelStateRequest()
        req.model_name = model_name
        req.relative_entity_name = 'world'
        resp = svc(req)
        if resp.success:
            p = resp.pose.position
            return p.x, p.y, p.z
        logger.warning(f"  get_model_pose({model_name}): {resp.status_message}")
    except Exception as e:
        logger.warning(f"  get_model_pose exception: {e}")
    return None, None, None


def _get_link_pose(model_name, link_name):
    try:
        import rospy as _rospy
        from gazebo_msgs.srv import GetModelState, GetModelStateRequest
        _rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        svc = _rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        req = GetModelStateRequest()
        req.model_name = f"{model_name}::{link_name}"
        req.relative_entity_name = 'world'
        resp = svc(req)
        if resp.success:
            p = resp.pose.position
            return p.x, p.y, p.z
        logger.warning(f"  get_link_pose({model_name}::{link_name}): {resp.status_message}")
    except Exception as e:
        logger.warning(f"  get_link_pose exception: {e}")
    return None, None, None


def _set_pose(model_name, x, y, z):
    try:
        import rospy as _rospy
        from gazebo_msgs.srv import SetModelState
        from gazebo_msgs.msg import ModelState
        _rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
        svc = _rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        state.reference_frame = 'world'
        resp = svc(state)
        return resp.success
    except Exception as e:
        logger.error(f"  set_pose exception: {e}")
        return False


def _read_contacts(sensor, window=0.5):
    """Read ContactsState using sensor.capture_data()."""
    try:
        from gazebo_msgs.msg import ContactsState
        topic = f"/{sensor.sensor_name}/bumper_states"
        msgs = sensor.capture_data(ContactsState, topic=topic, window=window, timeout=0.3)
        return msgs
    except Exception as e:
        logger.debug(f"  _read_contacts: {e}")
        return []


def _summarise(msgs, label=""):
    """Log all contact pairs + forces. Returns (unfiltered_median, filtered_median)."""
    all_states = []
    for msg in msgs:
        if hasattr(msg, 'states') and msg.states:
            all_states.extend(msg.states)

    if not all_states:
        logger.info(f"  {label or 'reading'}: 0 contact states  ({len(msgs)} msgs)")
        return 0.0, 0.0

    unf, fil, pairs = [], [], set()
    for s in all_states:
        c1 = s.collision1_name
        c2 = s.collision2_name
        f  = s.total_wrench.force
        mag = (f.x**2 + f.y**2 + f.z**2) ** 0.5
        unf.append(mag)
        pairs.add((c1, c2))
        if 'probe' in c1.lower() or 'probe' in c2.lower():
            fil.append(mag)

    logger.info(f"  {label or 'reading'}: {len(msgs)} msgs, {len(all_states)} states")
    for c1, c2 in sorted(pairs):
        logger.info(f"    [{c1}]  ↔  [{c2}]")
    u = float(np.median(unf))
    f = float(np.median(fil)) if fil else 0.0
    logger.info(f"  unfiltered median = {u:.3f} N   (min={min(unf):.3f}  max={max(unf):.3f})")
    logger.info(f"  filtered   median = {f:.3f} N" + ("" if fil else "  (no 'probe' in names)"))
    return u, f


def sep(n, title):
    logger.info("")
    logger.info("=" * 60)
    logger.info(f"  SECTION {n}: {title}")
    logger.info("=" * 60)


def sec1_geometry(sensor):
    sep(1, "SENSOR GEOMETRY")
    raw = sensor.params.get("sensor_size") or sensor.params.get("size")
    logger.info(f"  params      = {sensor.params}")
    logger.info(f"  sensor_size = {raw}")
    logger.info(f"  sdf_path    = {sensor.sdf_path}")
    logger.info(f"  topics      = {sensor.topics}")
    if not raw:
        logger.error("  *** NO sensor_size — all tests will abort. Add it to the sensor params. ***")
        return None
    sz = [float(v) for v in raw]
    logger.info(f"  size  {sz[0]*1000:.2f} x {sz[1]*1000:.2f} x {sz[2]*1000:.2f} mm")
    logger.info(f"  half-z = {sz[2]/2*1000:.2f} mm")
    return sz


def sec2_poses(sensor, probe_model):
    sep(2, "WORLD POSES (after physics settling)")
    bx, by, bz = _get_link_pose(sensor.sensor_name, "body")
    if bx is None:
        bx, by, bz = _get_model_pose(sensor.sensor_name)
        logger.info("  (body link query failed — using model root)")
    mx, my, mz = _get_model_pose(sensor.sensor_name)
    px, py, pz = _get_model_pose(probe_model)
    logger.info(f"  sensor body link:  ({bx:.4f}, {by:.4f}, {bz:.4f})")
    logger.info(f"  sensor model root: ({mx:.4f}, {my:.4f}, {mz:.4f})")
    logger.info(f"  probe model root:  ({px:.4f}, {py:.4f}, {pz:.4f})")
    raw = sensor.params.get("sensor_size") or sensor.params.get("size")
    PROBE_TIP_OFFSET = 0.010
    if raw and bz is not None:
        half_z     = float(raw[2]) / 2
        sensor_top = bz + half_z
        start_tip  = sensor_top + 0.020
        start_orig = start_tip + PROBE_TIP_OFFSET
        logger.info(f"")
        logger.info(f"  sensor_top_z    = {sensor_top:.4f}  ({half_z*1000:.1f} mm above body)")
        logger.info(f"  start_tip_z     = {start_tip:.4f}  (20 mm above sensor top)")
        logger.info(f"  start_origin_z  = {start_orig:.4f}  (start_tip + {PROBE_TIP_OFFSET*1000:.0f} mm offset)")
        return bx, by, bz, sensor_top
    return (bx or 0.0), (by or 0.0), (bz or 0.0), (bz or 0.0)


def sec3_noise(sensor):
    sep(3, "BASELINE NOISE (probe parked at z=0.50)")
    logger.info("  Moving probe to z=0.50, waiting 2 s ...")
    _set_pose("force_probe", 0, 0, 0.50)
    time.sleep(2.0)
    msgs = _read_contacts(sensor, window=1.0)
    u, f = _summarise(msgs, "probe at z=0.50")
    logger.info("")
    if f > 0.2:
        logger.warning(f"  *** filtered noise {f:.3f} N > 0.2 N — check probe naming ***")
    else:
        logger.info(f"  OK: filtered noise {f:.3f} N (safe below 1.0 N threshold)")
    if u > 50:
        logger.info(f"  NOTE: unfiltered background {u:.0f} N is expected (mount on platform)")
    return u, f


def sec4_sweep(sensor, body_x, body_y, sensor_top):
    sep(4, "CONTACT SWEEP (2 mm steps, 30 mm total)")
    PROBE_TIP_OFFSET = 0.010
    start_tip  = sensor_top + 0.020
    start_orig = start_tip + PROBE_TIP_OFFSET
    logger.info(f"  Probe → origin z={start_orig:.4f}  (tip at {start_tip:.4f})")
    _set_pose("force_probe", body_x, body_y, start_orig)
    time.sleep(2.5)

    logger.info("  Reading immediately after teleport ...")
    msgs = _read_contacts(sensor, window=0.5)
    u0, f0 = _summarise(msgs, "post-teleport")
    if f0 > 0.5:
        logger.warning(f"  Transient filtered={f0:.3f} N. Waiting 1.5 s more ...")
        time.sleep(1.5)

    logger.info("")
    logger.info(f"  {'#':>3}  {'Orig Z':>8}  {'Tip Z':>8}  {'Gap mm':>7}  {'Unfilt':>8}  {'Filt':>7}  Note")
    logger.info(f"  {'---':>3}  {'------':>8}  {'------':>8}  {'------':>7}  {'------':>8}  {'----':>7}  ----")

    current_z = start_orig
    readings = []
    first_hit = None
    consec = 0

    for i in range(15):
        current_z -= 0.002
        tip_z  = current_z - PROBE_TIP_OFFSET
        gap_mm = (tip_z - sensor_top) * 1000

        _set_pose("force_probe", body_x, body_y, current_z)
        time.sleep(0.25)
        msgs = _read_contacts(sensor, window=0.25)

        all_st = [s for m in msgs if hasattr(m,'states') and m.states for s in m.states]
        unf_v, fil_v = [], []
        for s in all_st:
            f   = s.total_wrench.force
            mag = (f.x**2 + f.y**2 + f.z**2)**0.5
            unf_v.append(mag)
            if 'probe' in s.collision1_name.lower() or 'probe' in s.collision2_name.lower():
                fil_v.append(mag)

        u = float(np.median(unf_v)) if unf_v else 0.0
        f = float(np.median(fil_v)) if fil_v else 0.0
        note = "<<< CONTACT" if f > 1.0 else ""
        logger.info(f"  {i+1:>3}  {current_z:>8.4f}  {tip_z:>8.4f}  {gap_mm:>7.1f}  {u:>8.3f}  {f:>7.3f}  {note}")
        readings.append((current_z, tip_z, gap_mm, u, f))

        if f > 1.0:
            consec += 1
            if first_hit is None:
                first_hit = current_z
            if consec >= 2:
                logger.info(f"")
                logger.info(f"  CONFIRMED contact at origin z={first_hit:.4f}  tip z={first_hit-PROBE_TIP_OFFSET:.4f}")
                break
        else:
            consec = 0
            first_hit = None

    if not first_hit:
        logger.warning("")
        logger.warning("  No contact in 30 mm sweep.")
        logger.warning("  1. Check body_x/body_y alignment")
        logger.warning("  2. Check probe tip offset (SDF <pose>)")
        logger.warning("  3. Run with --verbose and check debug contact names")
        logger.warning("  4. Threshold 1.0 N may be too high if noise was high in section 3")

    return readings, first_hit


def sec5_ramp(sensor, body_x, body_y, contact_z):
    sep(5, "FORCE RAMP (0.1 mm steps)")
    if contact_z is None:
        logger.info("  Skipped — no contact found")
        return
    PROBE_TIP_OFFSET = 0.010
    sensor_top_approx = contact_z - PROBE_TIP_OFFSET
    logger.info(f"  Fine steps from z={contact_z+0.001:.4f} downward")
    logger.info(f"  {'#':>3}  {'Orig Z':>8}  {'Pen mm':>8}  {'Filt N':>8}")
    logger.info(f"  {'---':>3}  {'------':>8}  {'-------':>8}  {'-------':>8}")
    for i in range(20):
        z   = contact_z + 0.001 - i * 0.0001
        pen = max(0.0, (sensor_top_approx - (z - PROBE_TIP_OFFSET)) * 1000)
        _set_pose("force_probe", body_x, body_y, z)
        time.sleep(0.20)
        msgs = _read_contacts(sensor, window=0.20)
        fil_v = []
        for m in msgs:
            if hasattr(m, 'states') and m.states:
                for s in m.states:
                    if 'probe' in s.collision1_name.lower() or 'probe' in s.collision2_name.lower():
                        f   = s.total_wrench.force
                        fil_v.append((f.x**2+f.y**2+f.z**2)**0.5)
        f = float(np.median(fil_v)) if fil_v else 0.0
        logger.info(f"  {i+1:>3}  {z:>8.4f}  {pen:>8.3f}  {f:>8.3f}")


def sec6_recommendations(fil_noise, readings):
    sep(6, "RECOMMENDATIONS")
    if not readings:
        logger.info("  No data")
        return
    pre  = [r[4] for r in readings if r[2] > 0]
    post = [r[4] for r in readings if r[2] <= 0]
    if pre:
        rec = max(pre) * 3.0
        logger.info(f"  Noise pre-contact: max={max(pre):.3f} N  mean={np.mean(pre):.3f} N")
        logger.info(f"  Recommended threshold = {rec:.2f} N  (3× noise max)")
        logger.info(f"  Current CONTACT_THRESHOLD = 1.0 N  {'OK' if rec <= 1.0 else '← INCREASE to ' + f'{rec:.1f}'}")
    if post:
        logger.info(f"  Post-contact force range: {min(post):.3f} – {max(post):.3f} N")
    logger.info("")
    logger.info("  PID convergence (kp comparison):")
    logger.info("    kp=1e-6  error 1N → delta_z=0.001mm  slow (original, broken)")
    logger.info("    kp=1e-4  error 1N → delta_z=0.1mm   reasonable (current)")
    logger.info("    kp=1e-3  error 1N → delta_z=1.0mm   fast, may overshoot")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor", required=True)
    args = parser.parse_args()

    setup_logging(verbose=True)
    logger.info("=" * 60)
    logger.info(f"  TACTILE DIAGNOSTIC  —  sensor: {args.sensor}")
    logger.info("=" * 60)

    db.init_db()
    sensors = [s for s in db.get_all_sensors() if s["name"] == args.sensor]
    if not sensors:
        logger.error(f"Sensor '{args.sensor}' not found")
        sys.exit(1)
    sd = sensors[0]

    core = Core()
    logger.info("Starting ROS ...")
    core.simulator.launch_ros()
    if not wait_for_roscore(30.0):
        logger.error("roscore did not start")
        sys.exit(1)
    core.simulator.launch_node()
    if not core.simulator.node_is_running:
        logger.error("ROS node failed")
        sys.exit(1)
    logger.info("ROS ready.")

    sensor_obj = Sensor(
        sensor_type = sd["type"],
        sensor_name = sd["name"],
        sdf_path    = sd["sdf_path"],
        topics      = sd.get("topics", []),
        description = sd.get("description", ""),
        image_path  = sd.get("image_path", ""),
        params      = sd.get("params", {}),
    )

    sz = sec1_geometry(sensor_obj)
    if sz is None:
        core.simulator.kill(); sys.exit(1)

    from src.tests._common import Worlds
    logger.info(f"\nOpening {Worlds.TACTILE_FORCE} ...")
    if not core.simulator.open_scene(Worlds.TACTILE_FORCE, sensor_obj.sdf_path):
        logger.error("Failed to open scene"); core.simulator.kill(); sys.exit(1)
    time.sleep(3.0)

    probe_model = "force_probe"
    if not core.simulator.wait_for_model_spawn(probe_model, 30):
        logger.error(f"Probe not spawned"); core.simulator.kill(); sys.exit(1)
    logger.info("World ready.\n")

    bx, by, bz, sensor_top = sec2_poses(sensor_obj, probe_model)
    _, fil_noise            = sec3_noise(sensor_obj)
    readings, contact_z     = sec4_sweep(sensor_obj, bx, by, sensor_top)
    sec5_ramp(sensor_obj, bx, by, contact_z)
    sec6_recommendations(fil_noise, readings)

    sep("", "DONE")
    logger.info("  Shutting down ...")
    core.simulator.kill()
    logger.info("  Complete.")


if __name__ == "__main__":
    main()