import copy
import json
import logging
import math
import os
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from enum import Enum
from math import atan, atan2, cos, degrees, pi, sin, tan
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from types import SimpleNamespace

import cv2
import numpy as np
import rospy
import rostopic
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

from config import CONFIG

logger = logging.getLogger(__name__)


# ── World registry ────────────────────────────────────────────────────────────
#
# Every world file lives under assets/worlds/.
# To add a new world:
#   1. Drop the .world file into assets/worlds/
#   2. Add an entry below:  MY_WORLD = "assets/worlds/my_world.world"
#   3. Reference it in your test:  world = Worlds.MY_WORLD.value
#
class Worlds(str, Enum):
    path = f"{CONFIG['ROOT_PATH']}/assets/worlds"

    # RFID
    RFID_CHANGE_DISTANCE  = f"{path}/rfid_change_distance.world"
    RFID_MASS_READ        = f"{path}/rfid_mass_read.world"
    RFID_OVERLAP_TAGS     = f"{path}/rfid_overlap_tags.world"
    RFID_ANGLE_DEPENDENCE = f"{path}/rfid_angle_dependence.world"
    RFID_MOVE_TAGS        = f"{path}/rfid_move_tags.world"
    RFID_ANTENNA_ROTATION = f"{path}/rfid_antenna_rotation.world"
    # Camera
    CAMERA_SMOKE = f"{path}/camera_c1_single_cube.world"
    CAMERA_DEPTH_ACCURACY = f"{path}/camera_depth_perception.world"
    CAMERA_RESOLUTION = f"{path}/camera_c2_resolution.world"
    c1 = f"{path}/camera_c1_single_cube.world"
    
# ── Helpers ───────────────────────────────────────────────────────────────────


def _PoseStamped():
    from geometry_msgs.msg import PoseStamped
    return PoseStamped


def _world_from_db(sensor, func_name: str) -> str:
    """
    Primary world lookup: reads world_path from the DB entry for this test.
    Falls back to the Worlds enum value whose name matches func_name (upper-cased).
    """
    import src.database.sensor_storage as db
    tests = {t["func_name"]: t for t in db.get_type_tests(sensor.sensor_type)}
    path  = tests.get(func_name, {}).get("world_path", "")
    if path:
        return path
    # Fallback: try to match by name convention
    key = func_name.upper()
    if key in Worlds.__members__:
        return Worlds[key].value
    return ""


def get_test(func_name: str) -> callable:
    fn = TESTS.get(func_name)
    if fn is None:
        raise KeyError(f"No test registered under name {func_name!r}")
    return fn


def get_tests_for_type(sensor_type: str) -> dict[str, callable]:
    try:
        import src.database.sensor_storage as db
        result = {}
        for t in db.get_type_tests(sensor_type):
            fn = TESTS.get(t["func_name"])
            if fn is not None:
                result[t["func_name"]] = fn
            else:
                logger.warning(
                    "get_tests_for_type: %r assigned to type %r but not in TESTS",
                    t["func_name"], sensor_type,
                )
        return result
    except Exception as e:
        logger.error("get_tests_for_type failed: %s", e)
        return {}


# ── How to add a new test ─────────────────────────────────────────────────────
#
# 1. Write a plain function:
#        def my_test(simulator, sensor, progress_cb=None) -> dict
#
# 2. Add it to the TESTS dict at the bottom of this file:
#        TESTS = { ..., "my_test": my_test }
#
# 3. Add a Worlds entry if you need a dedicated world:
#        MY_WORLD = "assets/worlds/my_world.world"
#    then reference it as Worlds.MY_WORLD.value in your test,
#    OR just set the world path in the UI (Add Sensor Type → Tests).
#    The DB value always wins over the enum fallback.
#
#
# ── simulator API ─────────────────────────────────────────────────────────────
#
#   simulator.open_scene(world_path, sdf_path) -> bool
#   simulator.wait_for_model_spawn(model_name, timeout_sec) -> bool
#   simulator.set_pose(model_name, x, y, z,
#                      quaternion=None, linear_velocity=None)
#   simulator.kill()
#
#
# ── sensor fields ─────────────────────────────────────────────────────────────
#
#   sensor.sensor_type  str
#   sensor.sensor_name  str
#   sensor.sdf_path     str
#   sensor.topic        str    primary ROS topic
#   sensor.params       dict
#
#   sensor.capture_data(msg_type, window, timeout, simulator) -> list[msg]
#       Raw message list — use for any sensor type.
#
#   sensor.capture_frames(msg_type, window, timeout, simulator) -> {frame_id: pose}
#       RFID-style dict — use when you need to check tag IDs.
#
#
# ── return value ──────────────────────────────────────────────────────────────
#
#   Must return a dict with at least {"passed": bool}.
#   Every extra key is logged to the UI results panel automatically.
#


# ── RFID tests ────────────────────────────────────────────────────────────────

def rfid_max_stable_read_distance(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    read_distance = float(sensor.params.get("rzero", 3))

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        f.write("fix1 1 0.5 0 0\n")

    if not simulator.open_scene(Worlds.RFID_CHANGE_DISTANCE.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    tag = "rfid_tag1"
    if not simulator.wait_for_model_spawn(tag, 30):
        raise RuntimeError("tag not spawned")

    current  = 0.5
    reset    = read_distance * 5
    max_dist = 0.0
    steps    = max(1, round((read_distance - 0.5) / 0.5) + 1)
    step     = 0
    t0       = time.time()

    while current <= read_distance + 1e-9:
        try:
            simulator.set_pose(tag, reset, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag, current, 0, 0)
            if tag in sensor.capture_frames(_PoseStamped(), window=3, simulator=simulator):
                max_dist = current
        except Exception:
            pass
        step += 1
        if progress_cb:
            progress_cb(int(step / steps * 100))
        current = round(current + 0.5, 1)

    return {
        "passed":            abs(max_dist - read_distance) <= 0.5,
        "duration":          time.time() - t0,
        "max_read_distance": max_dist,
    }


def rfid_min_stable_read_distance(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        f.write("fix1 1 0.5 0 0\n")

    if not simulator.open_scene(Worlds.RFID_CHANGE_DISTANCE.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    tag = "rfid_tag1"
    if not simulator.wait_for_model_spawn(tag, 30):
        raise RuntimeError("tag not spawned")

    min_dist = current = 0.25
    reset    = 25.0
    steps    = max(1, round(0.25 / 0.01))
    step     = 0
    t0       = time.time()

    while current >= 0:
        try:
            simulator.set_pose(tag, reset, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag, current, 0, 0)
            if tag in sensor.capture_frames(_PoseStamped(), window=2, simulator=simulator):
                min_dist = current
        except Exception:
            pass
        step += 1
        if progress_cb:
            progress_cb(int(step / steps * 100))
        current = round(current - 0.01, 5)

    return {
        "passed":            min_dist <= 0.05,
        "duration":          time.time() - t0,
        "min_read_distance": min_dist,
    }


def rfid_mass_read(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    read_distance = float(sensor.params.get("rzero", 3))
    tags_count    = 75
    radius        = read_distance / 2

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        for i in range(tags_count):
            x = radius * math.cos(2 * math.pi / tags_count * i)
            y = radius * math.sin(2 * math.pi / tags_count * i)
            f.write(f"fix{i+1} {i+1} {x} {y} 0\n")

    if not simulator.open_scene(Worlds.RFID_MASS_READ.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    for i in range(tags_count):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")
        if progress_cb:
            progress_cb(int((i + 1) / tags_count * 50))

    t0   = time.time()
    data = sensor.capture_frames(_PoseStamped(), window=20, simulator=simulator)
    if progress_cb:
        progress_cb(100)

    return {
        "passed":              len(data) / tags_count >= 0.75,
        "duration":            time.time() - t0,
        "tags_detected_count": len(data),
    }


def rfid_overlap_tags(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    read_distance = float(sensor.params.get("rzero", 3))
    tags_count    = 5
    radius        = read_distance / 2
    distances     = [0.2, 0.1, 0.05, 0.02]
    dist_result   = None
    t0            = time.time()

    for step, distance in enumerate(distances):
        with open(CONFIG["RFID_MAP_PATH"], "w") as f:
            for i in range(tags_count):
                x = radius * math.cos(distance / radius * i)
                y = radius * math.sin(distance / radius * i)
                f.write(f"fix{i+1} {i+1} {x} {y} 0\n")

        if not simulator.open_scene(Worlds.RFID_OVERLAP_TAGS.value, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
                raise RuntimeError(f"tag {i+1} not spawned")

        data = sensor.capture_frames(_PoseStamped(), window=5, simulator=simulator)
        if len(data) == tags_count:
            dist_result = distance
        if progress_cb:
            progress_cb(int((step + 1) / len(distances) * 100))

    return {
        "passed":      dist_result is not None,
        "duration":    time.time() - t0,
        "dist_result": dist_result,
    }


def rfid_angle_dependence(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    read_distance = float(sensor.params.get("rzero", 3))
    angles        = [0, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
    radius        = read_distance / 2

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        for i, angle in enumerate(angles):
            x = radius * math.sin(angle)
            z = radius * math.cos(angle)
            f.write(f"fix{i+1} {i+1} {x} 0 {z}\n")

    if not simulator.open_scene(Worlds.RFID_ANGLE_DEPENDENCE.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    for i in range(len(angles)):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")
        if progress_cb:
            progress_cb(int((i + 1) / len(angles) * 50))

    t0   = time.time()
    data = sensor.capture_frames(_PoseStamped(), window=20, simulator=simulator)
    if progress_cb:
        progress_cb(100)

    return {
        "passed":              len(data) == len(angles),
        "duration":            time.time() - t0,
        "tags_detected_count": len(data),
    }


def rfid_move_tags(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    from geometry_msgs.msg import Vector3
    read_distance = float(sensor.params.get("rzero", 3))
    velocities    = [Vector3(0.5, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)]
    factor        = 1.5
    start_dist    = -1 * read_distance * factor
    result_vel    = None
    t0            = time.time()

    for step, velocity in enumerate(velocities):
        with open(CONFIG["RFID_MAP_PATH"], "w") as f:
            f.write(f"fix1 1 {start_dist} 0 0\n")
        if not simulator.open_scene(Worlds.RFID_MOVE_TAGS.value, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        if not simulator.wait_for_model_spawn("rfid_tag1", 30):
            raise RuntimeError("tag not spawned")

        simulator.set_pose("rfid_tag1", x=start_dist, y=0, z=0, linear_velocity=velocity)
        data = sensor.capture_frames(
            _PoseStamped(),
            window=factor * read_distance / velocity.x * 2,
            simulator=simulator,
        )
        if len(data) == 1:
            result_vel = velocity
        if progress_cb:
            progress_cb(int((step + 1) / len(velocities) * 100))
        if result_vel is None:
            break

    return {
        "passed":                result_vel is not None,
        "duration":              time.time() - t0,
        "max_detected_velocity": result_vel.x if result_vel else None,
    }


def rfid_antenna_rotation(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    from geometry_msgs.msg import Quaternion
    read_distance = float(sensor.params.get("rzero", 3))
    angles        = [0, math.pi / 6, math.pi / 3, math.pi / 2]
    tags_count    = 10
    radius        = read_distance / 2
    passed        = False
    angle2count   = {}

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        for i in range(tags_count):
            x = radius * math.cos(2 * math.pi / tags_count * i)
            y = radius * math.sin(2 * math.pi / tags_count * i)
            f.write(f"fix{i+1} {i+1} {x} {y} 0\n")

    if not simulator.open_scene(Worlds.RFID_ANTENNA_ROTATION.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    for i in range(tags_count):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")

    t0 = time.time()
    for step, angle in enumerate(angles):
        q = Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
        simulator.set_pose("rfid_antenna", x=0, y=0, z=0, quaternion=q)
        time.sleep(0.01)
        data = sensor.capture_frames(_PoseStamped(), window=7, simulator=simulator)
        if len(data) == tags_count and angle == 0:
            passed = True
        angle2count[round(math.degrees(angle), 1)] = len(data)
        if progress_cb:
            progress_cb(int((step + 1) / len(angles) * 100))

    return {
        "passed":           passed,
        "duration":         time.time() - t0,
        "angle2tags_count": angle2count,
    }


# ── Camera helpers ────────────────────────────────────────────────────────────

def _img_to_numpy(msg):
    """Convert a sensor_msgs/Image to a numpy array."""
    import numpy as np
    dtype = np.float32 if "32FC" in msg.encoding else np.uint8
    ch    = 1 if msg.encoding in ("32FC1", "mono8", "8UC1") else 3
    return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, ch)


def _save_image(arr, save_dir: str, name: str) -> str:
    import os, cv2
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, f"{name}.jpg")
    if arr.dtype != "uint8":
        # false-colour depth
        import math
        flat  = arr[:, :, 0]
        fin   = flat[~((flat == float("inf")) | (flat == float("-inf")))]
        norm  = flat.copy()
        if len(fin) and fin.max() > fin.min():
            norm = ((flat - fin.min()) / (fin.max() - fin.min()) * 255).clip(0, 255).astype("uint8")
        else:
            norm = (norm * 0).astype("uint8")
        cv2.imwrite(path, cv2.applyColorMap(norm, cv2.COLORMAP_JET))
    else:
        bgr = arr[:, :, ::-1].copy() if arr.shape[2] >= 3 else arr[:, :, 0]
        cv2.imwrite(path, bgr)
    return path


def _get_save_dir(sensor, func_name: str) -> str:
    import os
    from config import CONFIG
    return os.path.join(CONFIG.get("SAVE_DIR", "results"), sensor.sensor_name, func_name)


# ── Camera tests ──────────────────────────────────────────────────────────────

def camera_data_received(simulator, sensor, progress_cb=None) -> dict:
    """
    Smoke test: verify the camera publishes a valid image.
    Pass criterion: at least one non-empty frame received.
    """
    from sensor_msgs.msg import Image
    import numpy as np

    world = _world_from_db(sensor, "camera_data_received") or Worlds.CAMERA_SMOKE.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    if progress_cb:
        progress_cb(20)

    t0   = time.time()
    msgs = sensor.capture_data(Image, window=15.0, timeout=2.0, warmup=3.0, simulator=simulator)

    if not msgs:
        return {
            "passed":   False,
            "duration": round(time.time() - t0, 2),
            "detail":   "No image received within 15 s",
        }

    arr       = _img_to_numpy(msgs[0])
    non_zero  = int(np.count_nonzero(arr))
    save_path = _save_image(arr, _get_save_dir(sensor, "camera_data_received"), "frame_0")

    if progress_cb:
        progress_cb(100)

    return {
        "passed":      non_zero > 0,
        "duration":    round(time.time() - t0, 2),
        "resolution":  f"{msgs[0].width}x{msgs[0].height}",
        "encoding":    msgs[0].encoding,
        "frames_recv": len(msgs),
        "non_zero_px": non_zero,
        "saved_to":    save_path,
    }


def camera_depth_accuracy(simulator, sensor, progress_cb=None) -> dict:
    """
    Method C1 — Depth Accuracy.
    Pass criterion: relative error δ ≤ 2 % at Z_true = 3.0 m.
    """
    from sensor_msgs.msg import Image
    import numpy as np

    Z_TRUE    = 3.0
    PASS_PCT  = 2.0

    world = _world_from_db(sensor, "camera_depth_accuracy") or Worlds.CAMERA_DEPTH_ACCURACY.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    if progress_cb:
        progress_cb(20)

    t0   = time.time()
    msgs = sensor.capture_data(Image, window=5.0, timeout=2.0, warmup=3.0, simulator=simulator)

    if not msgs:
        raise RuntimeError("No depth frames received")

    if progress_cb:
        progress_cb(60)

    frames     = [_img_to_numpy(m)[:, :, 0] for m in msgs]
    depth_map  = float("nan")
    import numpy as np
    depth_map  = np.nanmedian(np.stack(frames, axis=0), axis=0)
    cy, cx     = depth_map.shape[0] // 2, depth_map.shape[1] // 2
    patch      = depth_map[cy - 5:cy + 5, cx - 5:cx + 5]
    z_measured = float(np.nanmedian(patch))
    abs_err    = abs(z_measured - Z_TRUE)
    rel_err    = abs_err / Z_TRUE * 100.0

    save_path  = _save_image(
        depth_map[:, :, None] if depth_map.ndim == 2 else depth_map,
        _get_save_dir(sensor, "camera_depth_accuracy"), "depth_frame"
    )

    if progress_cb:
        progress_cb(100)

    return {
        "passed":        rel_err <= PASS_PCT,
        "duration":      round(time.time() - t0, 2),
        "z_true_m":      Z_TRUE,
        "z_measured_m":  round(z_measured, 4),
        "abs_error_m":   round(abs_err, 4),
        "rel_error_pct": round(rel_err, 2),
        "frames_used":   len(msgs),
        "saved_to":      save_path,
    }


def camera_resolution(simulator, sensor, progress_cb=None) -> dict:
    """
    Method C2 — Resolution.
    Pass criterion: two contours distinguishable at separation ≥ 0.05 m.
    """
    from sensor_msgs.msg import Image
    import cv2, numpy as np

    SEPARATIONS  = [0.20, 0.10, 0.05, 0.02]
    PASS_MIN_SEP = 0.05
    SPHERE_A     = "sphere_a"
    SPHERE_B     = "sphere_b"

    world = _world_from_db(sensor, "camera_resolution") or Worlds.CAMERA_RESOLUTION.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    if not simulator.wait_for_model_spawn(SPHERE_A, 30):
        raise RuntimeError(f"{SPHERE_A} did not spawn")
    if not simulator.wait_for_model_spawn(SPHERE_B, 30):
        raise RuntimeError(f"{SPHERE_B} did not spawn")

    if progress_cb:
        progress_cb(20)

    t0          = time.time()
    save_dir    = _get_save_dir(sensor, "camera_resolution")
    min_sep_ok  = None
    sep_results = {}

    for i, sep in enumerate(SEPARATIONS):
        half   = sep / 2.0
        warmup = 3.0 if i == 0 else 0.0   # plugin already up after first frame
        simulator.set_pose(SPHERE_A, x=3.0, y=-half, z=0.0)
        simulator.set_pose(SPHERE_B, x=3.0, y= half, z=0.0)
        time.sleep(0.1)

        msgs = sensor.capture_data(Image, window=3.0, timeout=2.0, warmup=warmup, simulator=simulator)
        if not msgs:
            sep_results[f"{int(sep*1000)}mm"] = 0
            continue

        arr  = _img_to_numpy(msgs[-1])
        _save_image(arr, save_dir, f"sep_{int(sep*1000):04d}mm")

        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY) if arr.shape[2] == 3 else arr[:, :, 0]
        _, bw = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        n = len([c for c in contours if cv2.contourArea(c) >= 50])
        sep_results[f"{int(sep*1000)}mm"] = n
        if n >= 2:
            min_sep_ok = sep

        if progress_cb:
            progress_cb(20 + int((i + 1) / len(SEPARATIONS) * 75))

    if progress_cb:
        progress_cb(100)

    return {
        "passed":           min_sep_ok is not None and min_sep_ok <= PASS_MIN_SEP,
        "duration":         round(time.time() - t0, 2),
        "min_sep_passed_m": min_sep_ok,
        "pass_threshold_m": PASS_MIN_SEP,
        "sep_contours":     sep_results,
        "saved_to":         save_dir,
    }

# ███╗░░░███╗░█████╗░███╗░░██╗░█████╗░░█████╗░░█████╗░███╗░░░███╗███████╗██████╗░░█████╗░
# ████╗░████║██╔══██╗████╗░██║██╔══██╗██╔══██╗██╔══██╗████╗░████║██╔════╝██╔══██╗██╔══██╗
# ██╔████╔██║██║░░██║██╔██╗██║██║░░██║██║░░╚═╝███████║██╔████╔██║█████╗░░██████╔╝███████║
# ██║╚██╔╝██║██║░░██║██║╚████║██║░░██║██║░░██╗██╔══██║██║╚██╔╝██║██╔══╝░░██╔══██╗██╔══██║
# ██║░╚═╝░██║╚█████╔╝██║░╚███║╚█████╔╝╚█████╔╝██║░░██║██║░╚═╝░██║███████╗██║░░██║██║░░██║
# ╚═╝░░░░░╚═╝░╚════╝░╚═╝░░╚══╝░╚════╝░░╚════╝░╚═╝░░╚═╝╚═╝░░░░░╚═╝╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝


def _camera_worlds_root() -> Path:
    root = Path(str(CONFIG["ROOT_PATH"]))
    assets_worlds = root / "assets" / "worlds"
    if assets_worlds.exists():
        return assets_worlds

    configured = Path(str(CONFIG.get("WORLDS_PATH", "") or ""))
    if configured.is_absolute():
        return configured
    return root / configured





def _text(node: Optional[ET.Element], path: str, default: str = "") -> str:
    if node is None:
        return default
    child = node.find(path)
    if child is None or child.text is None:
        return default
    return str(child.text).strip()


def _to_float(value: str, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def _to_int(value: str, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(round(float(str(value).strip())))
    except Exception:
        return default


def _normalize_topic(value: str) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    parts = [part for part in token.split("/") if part]
    return "/" + "/".join(parts)


def _join_topic(namespace: str, topic: str) -> str:
    topic_name = str(topic or "").strip()
    if not topic_name:
        return ""
    if topic_name.startswith("/"):
        return _normalize_topic(topic_name)
    ns = _normalize_topic(namespace)
    if not ns:
        return _normalize_topic(topic_name)
    return _normalize_topic(f"{ns}/{topic_name}")


def _parse_pose(value: str) -> Tuple[float, float, float, float, float, float]:
    items = [item for item in str(value or "").split() if item]
    if len(items) != 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    floats = [_to_float(item, 0.0) for item in items]
    return tuple(float(item or 0.0) for item in floats)  # type: ignore[return-value]


def _plugin_namespace(plugin: ET.Element) -> str:
    return (
        _text(plugin, "ros/namespace")
        or _text(plugin, "robotNamespace")
        or _text(plugin, "robot_namespace")
    )


def _plugin_remaps(plugin: ET.Element) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for arg in plugin.findall("ros/argument"):
        text = str(arg.text or "").strip()
        if ":=" not in text:
            continue
        lhs, rhs = text.split(":=", 1)
        lhs = lhs.strip()
        rhs = rhs.strip()
        if lhs:
            mapping[lhs] = rhs or lhs
    return mapping


def _plugin_update_rate(plugin: ET.Element, sensor_node: ET.Element) -> Optional[int]:
    return (
        _to_int(_text(plugin, "update_rate"), None)
        or _to_int(_text(plugin, "updateRate"), None)
        or _to_int(_text(sensor_node, "update_rate"), None)
    )


def _plugin_topic(
    plugin: Optional[ET.Element],
    sensor_node: ET.Element,
    remap_keys: List[str],
    plugin_tags: List[str],
    default_topic: str = "",
) -> str:
    if plugin is None:
        return ""

    namespace = _plugin_namespace(plugin)
    remaps = _plugin_remaps(plugin)
    for key in remap_keys:
        value = str(remaps.get(key, "")).strip()
        if value:
            return _join_topic(namespace, value)

    for tag in plugin_tags:
        value = _text(plugin, tag)
        if value:
            return _join_topic(namespace, value)

    default_value = str(default_topic or "").strip()
    if default_value:
        return _join_topic(namespace, default_value)

    sensor_name = _text(sensor_node, "camera/name") or sensor_node.get("name", "")
    if sensor_name:
        return _join_topic(namespace, f"{sensor_name}/image_raw")
    return ""


def _sensor_payload(sensor_node: ET.Element) -> Dict[str, Any]:
    plugin = sensor_node.find("plugin")
    pose = _parse_pose(_text(sensor_node, "pose"))
    return {
        "sensor_type": str(sensor_node.get("type", "")).strip(),
        "name": str(sensor_node.get("name", "")).strip(),
        "pose": pose,
        "image_width": _to_int(_text(sensor_node, "camera/image/width"), None),
        "image_height": _to_int(_text(sensor_node, "camera/image/height"), None),
        "image_format": _text(sensor_node, "camera/image/format"),
        "horizontal_fov": _to_float(_text(sensor_node, "camera/horizontal_fov"), None),
        "clip_near": _to_float(_text(sensor_node, "camera/clip/near"), None),
        "clip_far": _to_float(_text(sensor_node, "camera/clip/far"), None),
        "noise_type": _text(sensor_node, "camera/noise/type"),
        "noise_mean": _to_float(_text(sensor_node, "camera/noise/mean"), None),
        "noise_stddev": _to_float(_text(sensor_node, "camera/noise/stddev"), None),
        "update_rate": _plugin_update_rate(plugin, sensor_node) if plugin is not None else _to_int(_text(sensor_node, "update_rate"), None),
        "plugin_filename": str(plugin.get("filename", "")).strip() if plugin is not None else "",
        "namespace": _plugin_namespace(plugin) if plugin is not None else "",
        "image_topic": _plugin_topic(
            plugin,
            sensor_node,
            remap_keys=["image_raw"],
            plugin_tags=["imageTopicName"],
            default_topic="image_raw",
        ),
        "depth_topic": _plugin_topic(
            plugin,
            sensor_node,
            remap_keys=["depth/image_raw"],
            plugin_tags=["depthImageTopicName"],
            default_topic="",
        ),
    }


def _camera_load_sensor_profile(sensor_sdf_path: str) -> Dict[str, Any]:
    path = Path(sensor_sdf_path)
    tree = ET.parse(path)
    root = tree.getroot()
    model = root.find("model")
    model_name = str(model.get("name", "")).strip() if model is not None else ""

    sensors = [_sensor_payload(node) for node in root.findall(".//sensor")]
    camera_sensors = [item for item in sensors if item["sensor_type"] == "camera"]
    depth_sensors = [item for item in sensors if item["sensor_type"] == "depth"]

    family = "mono"
    if depth_sensors:
        family = "depth"
    elif len(camera_sensors) >= 2:
        names = " ".join(
            f"{item.get('name', '')} {item.get('namespace', '')} {item.get('image_topic', '')}"
            for item in camera_sensors
        ).lower()
        if "left" in names and "right" in names:
            family = "stereo"

    profile: Dict[str, Any] = {
        "sensor_name": path.stem,
        "model_name": model_name,
        "family": family,
        "image_topic": "",
        "depth_topic": "",
        "left_topic": "",
        "right_topic": "",
        "image_width": None,
        "image_height": None,
        "image_format": "",
        "horizontal_fov": None,
        "clip_near": None,
        "clip_far": None,
        "noise_type": "",
        "noise_mean": None,
        "noise_stddev": None,
        "update_rate": None,
        "baseline": None,
    }

    if family == "depth":
        primary = depth_sensors[0] if depth_sensors else (camera_sensors[0] if camera_sensors else {})
        color = camera_sensors[0] if camera_sensors else primary
        profile.update(
            {
                "image_topic": str(color.get("image_topic", "") or primary.get("image_topic", "") or "").strip(),
                "depth_topic": str(primary.get("depth_topic", "") or "").strip(),
                "image_width": primary.get("image_width"),
                "image_height": primary.get("image_height"),
                "image_format": str(primary.get("image_format", "")),
                "horizontal_fov": primary.get("horizontal_fov"),
                "clip_near": primary.get("clip_near"),
                "clip_far": primary.get("clip_far"),
                "noise_type": str(primary.get("noise_type", "")),
                "noise_mean": primary.get("noise_mean"),
                "noise_stddev": primary.get("noise_stddev"),
                "update_rate": primary.get("update_rate"),
            }
        )
        return profile

    if family == "stereo":
        left = next(
            (
                item for item in camera_sensors
                if "left" in str(item.get("name", "")).lower()
                or "left" in str(item.get("namespace", "")).lower()
                or "left" in str(item.get("image_topic", "")).lower()
            ),
            camera_sensors[0] if camera_sensors else {},
        )
        right = next(
            (
                item for item in camera_sensors
                if item is not left and (
                    "right" in str(item.get("name", "")).lower()
                    or "right" in str(item.get("namespace", "")).lower()
                    or "right" in str(item.get("image_topic", "")).lower()
                )
            ),
            camera_sensors[1] if len(camera_sensors) > 1 else {},
        )
        left_pose = left.get("pose", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        right_pose = right.get("pose", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        baseline = math.dist(left_pose[:3], right_pose[:3]) if left and right else None
        profile.update(
            {
                "left_topic": str(left.get("image_topic", "") or "").strip(),
                "right_topic": str(right.get("image_topic", "") or "").strip(),
                "image_width": left.get("image_width"),
                "image_height": left.get("image_height"),
                "image_format": str(left.get("image_format", "")),
                "horizontal_fov": left.get("horizontal_fov"),
                "clip_near": left.get("clip_near"),
                "clip_far": left.get("clip_far"),
                "noise_type": str(left.get("noise_type", "")),
                "noise_mean": left.get("noise_mean"),
                "noise_stddev": left.get("noise_stddev"),
                "update_rate": left.get("update_rate"),
                "baseline": baseline,
            }
        )
        return profile

    primary = camera_sensors[0] if camera_sensors else (depth_sensors[0] if depth_sensors else {})
    profile.update(
        {
            "image_topic": str(primary.get("image_topic", "") or "").strip(),
            "image_width": primary.get("image_width"),
            "image_height": primary.get("image_height"),
            "image_format": str(primary.get("image_format", "")),
            "horizontal_fov": primary.get("horizontal_fov"),
            "clip_near": primary.get("clip_near"),
            "clip_far": primary.get("clip_far"),
            "noise_type": str(primary.get("noise_type", "")),
            "noise_mean": primary.get("noise_mean"),
            "noise_stddev": primary.get("noise_stddev"),
            "update_rate": primary.get("update_rate"),
        }
    )
    return profile


def _camera_classify_sensor_profile(sensor_sdf_path: str) -> str:
    return str(_camera_load_sensor_profile(sensor_sdf_path).get("family", "mono"))

def _mono_build_ctx(sensor):
    ctx = SimpleNamespace()
    ctx.IMAGE_TOPIC = ""
    ctx.IMAGE_WIDTH = 640
    ctx.IMAGE_HEIGHT = 480
    ctx.UPDATE_RATE = 30
    ctx.HORIZONTAL_FOV_RAD = 1.0471975512
    ctx.CLIP_NEAR = 0.1
    ctx.CLIP_FAR = 50.0
    ctx.C1_CUBE_NAME = "test_cube"
    ctx.C2_SPHERE_A_NAME = "sphere_a"
    ctx.C2_SPHERE_B_NAME = "sphere_b"
    ctx.C7_FRONT_CUBE_NAME = "front_cube"
    ctx.C7_BACK_CUBE_NAME = "back_cube"
    ctx.C9_SPHERE_NAME = "fov_sphere"
    ctx.C10_CUBE_NAME = "clip_cube"
    ctx.C1_POSITIONS = (1.0, 3.0, 5.0)
    ctx.C1_TRACK_Y = 0.35
    ctx.C1_TRACK_Z = 0.25
    ctx.C2_DISTANCES = (0.20, 0.15, 0.10, 0.05, 0.02)
    ctx.C2_MIN_CONTOUR_AREA = 80
    ctx.C1_MIN_MARGIN_RATIO = 1.10
    ctx.C4_MIN_PIXELS = 1500
    ctx.C7_CASES = {"occ_25": 0.20, "occ_50": 0.10}
    ctx.C7_MIN_PIXELS = 800
    ctx.C9_STEP = 0.05
    ctx.C9_MAX_Y = 4.0
    ctx.C9_MIN_CONTOUR_AREA = 120
    ctx.C9_SPHERE_RADIUS_M = 0.1
    ctx.C10_NEAR_START_X = 0.05
    ctx.C10_NEAR_SEARCH_END_X = 2.0
    ctx.C10_NEAR_STEP = 0.01
    ctx.C10_FAR_COARSE_STEP = 0.5
    ctx.C10_FAR_FINE_STEP = 0.01
    ctx.C10_MIN_RED_PIXELS = 20
    ctx.C11_DURATION_S = 60.0
    ctx.C11_WARMUP_SECONDS = 2.0
    ctx.C11_JITTER_PERCENTILE = 95
    ctx.C11_MAX_JITTER_S = 0.015
    ctx.sensor = sensor
    ctx.sensor_name = str(getattr(sensor, "sensor_name", ""))
    ctx.sensor_type = str(getattr(sensor, "sensor_type", ""))
    ctx.sensor_sdf_path = str(getattr(sensor, "sdf_path", ""))
    ctx.CONFIG = {"ROOT_PATH": str(CONFIG["ROOT_PATH"])}
    profile = _camera_load_sensor_profile(ctx.sensor_sdf_path) if ctx.sensor_sdf_path else {}
    ctx.IMAGE_TOPIC = str(profile.get("image_topic", "") or getattr(sensor, "topic", "") or ctx.IMAGE_TOPIC)
    ctx.image_width = int(profile.get("image_width") or ctx.IMAGE_WIDTH)
    ctx.image_height = int(profile.get("image_height") or ctx.IMAGE_HEIGHT)
    ctx.horizontal_fov = float(profile.get("horizontal_fov") or ctx.HORIZONTAL_FOV_RAD)
    ctx.clip_near = float(profile.get("clip_near") or ctx.CLIP_NEAR)
    ctx.clip_far = float(profile.get("clip_far") or ctx.CLIP_FAR)
    ctx.update_rate = int(profile.get("update_rate") or ctx.UPDATE_RATE)
    worlds_root = _camera_worlds_root()
    ctx.test_to_world = {
        "c1_size_order_test": str(worlds_root / "camera_c1_single_cube.world"),
        "c2_resolution_test": str(worlds_root / "camera_c2_resolution.world"),
        "c4_geometries_presence_test": str(worlds_root / "camera_c4_geometries.world"),
        "c7_occlusion_test": str(worlds_root / "camera_c7_occlusion.world"),
        "c9_fov_test": str(worlds_root / "camera_c9_fov.world"),
        "c10_clipping_test": str(worlds_root / "camera_c10_clipping.world"),
        "c11_fps_stability_test": str(worlds_root / "camera_c11_fps_static_load.world"),
    }
    ctx._last_test_diagnostics = {}
    return ctx

def _mono__set_test_diagnostics(ctx, **kwargs) -> None:
    ctx._last_test_diagnostics.update(kwargs)
    

def _mono_get_last_test_diagnostics(ctx) -> Dict[str, Any]:
    # JSON-serializable payload, shallow-copy is enough here.
    return dict(ctx._last_test_diagnostics)
    

def _mono__results_dir(ctx) -> str:
    path = os.path.join(ctx.CONFIG["ROOT_PATH"], "results", ctx.sensor_name)
    os.makedirs(path, exist_ok=True)
    return path
    

def _mono__captured_dir(ctx) -> str:
    path = os.path.join(_mono__results_dir(ctx, ), "captured_images")
    os.makedirs(path, exist_ok=True)
    return path
    

def _mono__metrics_dir(ctx) -> str:
    path = os.path.join(_mono__results_dir(ctx, ), "metrics")
    os.makedirs(path, exist_ok=True)
    return path
    

def _mono__save_metrics_json(ctx, name: str, payload: Dict[str, Any]) -> str:
    out = os.path.join(_mono__metrics_dir(ctx, ), name)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return out
    

def _mono__wait_image(ctx, timeout: float = 35.0, topic: Optional[str] = None) -> Image:
    target_topic = str(topic or ctx.IMAGE_TOPIC)
    return rospy.wait_for_message(target_topic, Image, timeout=timeout)
    

def _mono__scene_diag(ctx, simulator) -> Dict[str, Any]:
    return {}
    

def _mono__resolved_image_topic(ctx, simulator) -> Tuple[str, Dict[str, Any]]:
    scene_diag = _mono__scene_diag(ctx, simulator)
    return str(ctx.IMAGE_TOPIC), scene_diag
    

def _mono__ensure_render_display_env(ctx) -> Dict[str, str]:
    applied: Dict[str, str] = {}
    if os.environ.get("DISPLAY"):
        return applied
    
    xauthority = os.environ.get("XAUTHORITY", "").strip()
    if not xauthority:
        default_xauthority = os.path.expanduser("~/.Xauthority")
        if os.path.exists(default_xauthority):
            os.environ["XAUTHORITY"] = default_xauthority
            applied["XAUTHORITY"] = default_xauthority
    
    for display in (":0",):
        display_socket = f"/tmp/.X11-unix/X{display.lstrip(':')}"
        if os.path.exists(display_socket):
            os.environ["DISPLAY"] = display
            applied["DISPLAY"] = display
            break
    
    return applied
    

def _mono__msg_stamp_s(ctx, msg: Image) -> float:
    stamp = float(msg.header.stamp.to_sec())
    if stamp <= 0.0:
        return float(time.time())
    return stamp
    

def _mono__wait_image_after(
    ctx,
    prev_stamp_s: Optional[float],
    timeout: float = 35.0,
    topic: Optional[str] = None,
) -> Image:
    start = time.time()
    target_topic = str(topic or ctx.IMAGE_TOPIC)
    saw_message = False
    
    while (time.time() - start) < float(timeout):
        remaining = max(0.2, float(timeout) - (time.time() - start))
        try:
            msg = _mono__wait_image(ctx, timeout=min(remaining, 5.0), topic=target_topic)
        except rospy.ROSException:
            continue
        saw_message = True
    
        if prev_stamp_s is None:
            return msg
    
        stamp = _mono__msg_stamp_s(ctx, msg)
        if stamp > float(prev_stamp_s) + 1e-6:
            return msg
    
    if not saw_message:
        raise RuntimeError(f"No image received on {target_topic} within {timeout:.1f}s")
    raise RuntimeError(
        f"No fresh image received on {target_topic} after stamp {float(prev_stamp_s):.6f} "
        f"within {timeout:.1f}s"
    )
    

def _mono__msg_to_bgr(ctx, msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    
    if enc in ("rgb8", "r8g8b8"):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    
    if enc == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    
    if enc in ("mono8", "8uc1"):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    

def _mono__clean_mask(ctx, mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    return closed
    

def _mono__red_mask(ctx, hsv: np.ndarray) -> np.ndarray:
    lower_1 = np.array([0, 90, 60], dtype=np.uint8)
    upper_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_2 = np.array([170, 90, 60], dtype=np.uint8)
    upper_2 = np.array([180, 255, 255], dtype=np.uint8)
    mask_1 = cv2.inRange(hsv, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv, lower_2, upper_2)
    return _mono__clean_mask(ctx, cv2.bitwise_or(mask_1, mask_2))
    

def _mono__color_mask(ctx, hsv: np.ndarray, color: str) -> np.ndarray:
    if color == "red":
        return _mono__red_mask(ctx, hsv)
    
    ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
        "green": ((40, 80, 60), (85, 255, 255)),
        "blue": ((100, 90, 60), (135, 255, 255)),
        "yellow": ((20, 90, 90), (40, 255, 255)),
    }
    if color not in ranges:
        raise ValueError(f"Unsupported color: {color}")
    
    lower, upper = ranges[color]
    mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    return _mono__clean_mask(ctx, mask)
    

def _mono__bbox_area(ctx, mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, (0, 0, 0, 0)
    
    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    return int(w * h), (int(x), int(y), int(w), int(h))
    

def _mono__count_pixels(ctx, mask: np.ndarray) -> int:
    return int(cv2.countNonZero(mask))
    

def _mono__iter_float_range(ctx, start: float, stop: float, step: float) -> List[float]:
    values: List[float] = []
    cur = float(start)
    while cur <= float(stop) + 1e-9:
        values.append(round(cur, 4))
        cur += float(step)
    return values
    

def _mono__white_mask(ctx, frame_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([0, 0, 175], dtype=np.uint8),
        np.array([180, 85, 255], dtype=np.uint8),
    )
    return _mono__clean_mask(ctx, mask)
    

def _mono__large_contours(
    ctx,
    mask: np.ndarray,
    min_area: float,
    border_margin: int = 3,
) -> List[Tuple[float, Tuple[int, int, int, int]]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = mask.shape[:2]
    
    result: List[Tuple[float, Tuple[int, int, int, int]]] = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < float(min_area):
            continue
    
        x, y, cw, ch = cv2.boundingRect(cnt)
        if x <= border_margin or y <= border_margin:
            continue
        if x + cw >= (w - border_margin) or y + ch >= (h - border_margin):
            continue
    
        result.append((area, (int(x), int(y), int(cw), int(ch))))
    return result
    

def _mono__red_stats(ctx, frame_bgr: np.ndarray) -> Dict[str, float]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    red = _mono__red_mask(ctx, hsv)
    red_pixels = float(_mono__count_pixels(ctx, red))
    
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour_area = 0.0
    if contours:
        max_contour_area = float(max(cv2.contourArea(c) for c in contours))
    
    h, w = frame_bgr.shape[:2]
    frame_area = float(max(1, h * w))
    pixel_ratio = float(red_pixels / frame_area)
    contour_ratio = float(max_contour_area / frame_area)
    
    return {
        "red_pixels": float(red_pixels),
        "pixel_ratio": pixel_ratio,
        "max_contour_area": max_contour_area,
        "contour_ratio": contour_ratio,
        "visible_by_pixels": bool(red_pixels >= int(ctx.C10_MIN_RED_PIXELS)),
    }
    

def _mono__annotate(ctx, frame: np.ndarray, lines: List[str]) -> np.ndarray:
    debug = frame.copy()
    y = 30
    for line in lines:
        cv2.putText(debug, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        y += 28
    return debug
    

def _mono__read_clip_from_sdf(ctx, camera_model_path_abs: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "camera_model_path_abs": str(camera_model_path_abs),
        "near_m": None,
        "far_m": None,
        "source": "",
        "error": "",
    }
    if not camera_model_path_abs:
        result["error"] = "camera_model_path_abs is empty"
        return result
    
    try:
        tree = ET.parse(camera_model_path_abs)
        root = tree.getroot()
        clip = root.find(".//sensor/camera/clip")
        if clip is None:
            clip = root.find(".//camera/clip")
        if clip is None:
            result["error"] = "No <clip> section found in SDF"
            return result
    
        near_node = clip.find("near")
        far_node = clip.find("far")
        if near_node is None or far_node is None:
            result["error"] = "No <near>/<far> in <clip>"
            return result
    
        near_v = float((near_node.text or "").strip())
        far_v = float((far_node.text or "").strip())
        result["near_m"] = float(near_v)
        result["far_m"] = float(far_v)
        result["source"] = "sdf_clip"
        return result
    except Exception as exc:
        result["error"] = f"Failed to parse SDF clip: {exc}"
        return result
    

def _mono__save_frame(ctx, name: str, frame: np.ndarray) -> str:
    out = os.path.join(_mono__captured_dir(ctx, ), name)
    cv2.imwrite(out, frame)
    return out
    

def _mono__open_test_scene(ctx, simulator, test_name: str) -> None:
    ctx._last_test_diagnostics = {}
    world = ctx.test_to_world[test_name]
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        diag = _mono__scene_diag(ctx, simulator)
        reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
        raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")
    
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
    

def _mono__move_and_settle(ctx, simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.8) -> None:
    simulator.set_pose(model_name, x=x, y=y, z=z)
    time.sleep(settle_s)
    

def _mono__c4_run_shell_capture(ctx, cmd: str, timeout_s: float = 5.0, max_lines: int = 200) -> Dict[str, Any]:
    try:
        res = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=float(timeout_s),
            check=False,
        )
        return {
            "cmd": str(cmd),
            "returncode": int(res.returncode),
            "stdout_tail": (res.stdout or "").splitlines()[-int(max_lines):],
            "stderr_tail": (res.stderr or "").splitlines()[-int(max_lines):],
        }
    except Exception as exc:
        return {
            "cmd": str(cmd),
            "returncode": None,
            "stdout_tail": [],
            "stderr_tail": [str(exc)],
        }
    

def _mono__c4_extract_log_path_from_lines(ctx, lines: List[str]) -> str:
    # Typical token inside pgrep/command line: __log:=/abs/path/gazebo-1.log
    for line in lines:
        s = str(line)
        marker = "__log:="
        idx = s.find(marker)
        if idx < 0:
            continue
        tail = s[idx + len(marker):].strip()
        if not tail:
            continue
        token = tail.split()[0].strip().strip("'").strip('"')
        if token.endswith(".log"):
            return token
        if ".log" in token:
            return token.split(".log", 1)[0] + ".log"
    return ""
    

def _mono__c4_collect_log_candidates_from_dir(ctx, base: Path, recursive: bool = False) -> List[Path]:
    if not base.exists() or not base.is_dir():
        return []
    patterns = ("gazebo-*.log", "gzserver-*.log")
    found: List[Path] = []
    for pattern in patterns:
        found.extend(base.rglob(pattern) if recursive else base.glob(pattern))
    return found
    

def _mono__c4_read_latest_gzserver_log_tail(
    ctx,
    cls,
    log_path_hint: Optional[str] = None,
    ros_log_dir_hint: Optional[str] = None,
    max_lines: int = 200,
) -> Dict[str, Any]:
    searched_paths: List[str] = []
    hint_file_exists = False
    hint_dir_exists = False
    hint_file_size_bytes: Optional[int] = None
    waited_ms = 0
    
    def _read_tail(path: Path) -> Dict[str, Any]:
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f_in:
                lines = f_in.read().splitlines()
            return {
                "path": str(path),
                "tail": lines[-int(max_lines):],
                "searched_paths": searched_paths,
                "hint_file_exists": bool(hint_file_exists),
                "hint_dir_exists": bool(hint_dir_exists),
                "hint_file_size_bytes": hint_file_size_bytes,
                "waited_ms": int(waited_ms),
            }
        except Exception as exc:
            return {
                "path": str(path),
                "tail": [f"<gazebo-log-read-error> {exc}"],
                "searched_paths": searched_paths,
                "hint_file_exists": bool(hint_file_exists),
                "hint_dir_exists": bool(hint_dir_exists),
                "hint_file_size_bytes": hint_file_size_bytes,
                "waited_ms": int(waited_ms),
            }
    
    if log_path_hint:
        hint = Path(log_path_hint).expanduser()
        searched_paths.append(str(hint))
        hint_dir_exists = bool(hint.parent.exists() and hint.parent.is_dir())
        t_start = time.perf_counter()
        for _ in range(10):
            exists_now = bool(hint.exists() and hint.is_file())
            if exists_now:
                hint_file_exists = True
                try:
                    hint_file_size_bytes = int(hint.stat().st_size)
                except Exception:
                    hint_file_size_bytes = None
                if hint_file_size_bytes is None or hint_file_size_bytes > 0:
                    waited_ms = int((time.perf_counter() - t_start) * 1000.0)
                    return _read_tail(hint)
            time.sleep(0.2)
        waited_ms = int((time.perf_counter() - t_start) * 1000.0)
        hint_file_exists = bool(hint.exists() and hint.is_file())
        if hint_file_exists:
            try:
                hint_file_size_bytes = int(hint.stat().st_size)
            except Exception:
                hint_file_size_bytes = None
    
    candidates: List[Path] = []
    
    if ros_log_dir_hint:
        ros_dir = Path(str(ros_log_dir_hint)).expanduser()
        searched_paths.append(str(ros_dir))
        candidates.extend(cls._c4_collect_log_candidates_from_dir(ros_dir, recursive=True))
    
    env_ros_log_dir = os.environ.get("ROS_LOG_DIR", "")
    if env_ros_log_dir:
        env_dir = Path(env_ros_log_dir).expanduser()
        searched_paths.append(str(env_dir))
        candidates.extend(cls._c4_collect_log_candidates_from_dir(env_dir, recursive=True))
    
    latest_dir = Path.home() / ".ros" / "log" / "latest"
    searched_paths.append(str(latest_dir))
    candidates.extend(cls._c4_collect_log_candidates_from_dir(latest_dir, recursive=False))
    
    ros_log_root = Path.home() / ".ros" / "log"
    searched_paths.append(str(ros_log_root))
    candidates.extend(cls._c4_collect_log_candidates_from_dir(ros_log_root, recursive=True))
    
    unique_existing: List[Path] = []
    for path in candidates:
        try:
            if path.exists() and path.is_file() and path not in unique_existing:
                unique_existing.append(path)
        except Exception:
            continue
    
    if not unique_existing:
        return {
            "path": "",
            "tail": ["<gazebo-log-not-found>"],
            "searched_paths": searched_paths,
            "hint_file_exists": bool(hint_file_exists),
            "hint_dir_exists": bool(hint_dir_exists),
            "hint_file_size_bytes": hint_file_size_bytes,
            "waited_ms": int(waited_ms),
        }
    
    chosen = sorted(unique_existing, key=lambda p: p.stat().st_mtime)[-1]
    return _read_tail(chosen)
    

def _mono__c4_collect_scene_open_diagnostics(ctx, simulator, scene_diag: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    if not isinstance(scene_diag, dict):
        scene_diag = {}
    
    rosservice_full = _mono__c4_run_shell_capture(ctx, 
        "rosservice list | grep '^/gazebo/' || true",
        timeout_s=6.0,
        max_lines=400,
    )
    rosnode_info = _mono__c4_run_shell_capture(ctx, 
        "rosnode info /gazebo || true",
        timeout_s=6.0,
        max_lines=200,
    )
    rostopic_gazebo_clock = _mono__c4_run_shell_capture(ctx, 
        "rostopic list | grep -E '^/clock$|^/gazebo/' || true",
        timeout_s=6.0,
        max_lines=300,
    )
    gz_pid_info = _mono__c4_run_shell_capture(ctx, 
        "pgrep -f '(^|/)gzserver([[:space:]]|$)' | head -n 1 || true",
        timeout_s=3.0,
        max_lines=10,
    )
    gz_pid = ""
    for line in gz_pid_info.get("stdout_tail", []):
        s = str(line).strip()
        if s.isdigit():
            gz_pid = s
            break
    if gz_pid:
        gzserver_ps = _mono__c4_run_shell_capture(ctx, 
            f"ps -fp {gz_pid} || true",
            timeout_s=3.0,
            max_lines=80,
        )
    else:
        gzserver_ps = {
            "cmd": "ps -fp <gzserver_pid>",
            "returncode": 0,
            "stdout_tail": [],
            "stderr_tail": ["gzserver pid not found"],
        }
    
    gz_log_hint = ""
    gz_state = scene_diag.get("gzserver_state", {})
    if isinstance(gz_state, dict):
        pgrep_tail = gz_state.get("pgrep_tail", [])
        if isinstance(pgrep_tail, list):
            gz_log_hint = _mono__c4_extract_log_path_from_lines(ctx, [str(x) for x in pgrep_tail])
    
    ros_log_dir_hint = ""
    launch_env = scene_diag.get("launch_env", {})
    if isinstance(launch_env, dict):
        ros_log_dir_hint = str(launch_env.get("ROS_LOG_DIR", "") or "")
    
    launch_stdout_tail = scene_diag.get("launch_stdout_tail", [])
    launch_stderr_tail = scene_diag.get("launch_stderr_tail", [])
    
    return {
        "rosservice_gazebo_full": rosservice_full,
        "rosnode_info_gazebo": rosnode_info,
        "rostopic_gazebo_clock": rostopic_gazebo_clock,
        "gzserver_ps": gzserver_ps,
        "roslaunch_stdout_tail": list(launch_stdout_tail)[-50:] if isinstance(launch_stdout_tail, list) else [],
        "roslaunch_stderr_tail": list(launch_stderr_tail)[-50:] if isinstance(launch_stderr_tail, list) else [],
        "gazebo_log_path_hint": str(gz_log_hint),
        "gazebo_log_tail": _mono__c4_read_latest_gzserver_log_tail(ctx, 
            log_path_hint=gz_log_hint,
            ros_log_dir_hint=ros_log_dir_hint,
            max_lines=200,
        ),
    }
    

def _mono__c4_classify_scene_reason(ctx, last_reason: str, diagnostics: Dict[str, Any]) -> Tuple[str, str]:
    reason = str(last_reason or "gazebo_api_timeout")
    if reason in ("topics_not_ready", "roslaunch_exited_while_waiting_topics"):
        return (
            "camera_topics_not_ready",
            "Expected camera topics were not resolved or did not publish messages in time",
        )
    
    rosservice = diagnostics.get("rosservice_gazebo_full", {})
    services = [
        str(line).strip()
        for line in rosservice.get("stdout_tail", [])
        if str(line).strip().startswith("/gazebo/")
    ]
    service_set = set(services)
    logger_services = {"/gazebo/get_loggers", "/gazebo/set_logger_level"}
    only_logger_services = bool(service_set) and service_set.issubset(logger_services)
    if only_logger_services:
        return (
            "gazebo_only_logger_services",
            "gazebo_ros_api_plugin likely not initialized (only logger services present)",
        )
    gz_ps = diagnostics.get("gzserver_ps", {})
    gz_alive = bool(gz_ps.get("stdout_tail"))
    if reason in ("gzserver_died", "roslaunch_exited") or not gz_alive:
        return ("gzserver_died_early", "gzserver died before Gazebo API services became available")
    return ("gazebo_api_timeout", "Gazebo API services timeout; see diagnostics: rosservice_gazebo_full / gazebo_log_tail")
    

def _mono__open_c4_scene_with_retry(ctx, simulator) -> None:
    world = ctx.test_to_world["c4_geometries_presence_test"]
    attempts: List[Dict[str, Any]] = []
    last_reason = "unknown"
    
    for attempt in (1, 2):
        if attempt == 2:
            simulator.kill_gazebo()
            time.sleep(1.0)

        scene_diag: Dict[str, Any] = {}
        try:
            opened = simulator.open_scene(world, ctx.sensor_sdf_path)
        except Exception as exc:
            opened = None
            if not isinstance(scene_diag, dict):
                scene_diag = {}
            scene_diag.setdefault("open_scene_exception", str(exc))
        last_reason = str(scene_diag.get("reason", "unknown")) if isinstance(scene_diag, dict) else "unknown"
        attempts.append({"attempt": int(attempt), "opened": bool(opened), "reason": last_reason})
        if opened:
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
            return

        retryable_reasons = {
            "gazebo_services_not_available",
            "gazebo_services_timeout",
            "roslaunch_exited",
        }
        if attempt == 1 and last_reason in retryable_reasons:
            continue
        break
    
    extra_diag = _mono__c4_collect_scene_open_diagnostics(ctx, simulator, scene_diag={})
    classified_reason, classified_msg = _mono__c4_classify_scene_reason(ctx, last_reason, extra_diag)
    _mono__set_test_diagnostics(ctx, 
        c4_scene_open={
            "reason": classified_reason,
            "message": classified_msg,
            "attempts": attempts,
            "diagnostics": extra_diag,
        }
    )
    raise RuntimeError(
        f"Failed to open scene for c4_geometries_presence_test (reason={classified_reason}). "
        f"See diagnostics: rosservice_gazebo_full / gazebo_log_tail"
    )
    

def _mono_c1_size_order_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c1_size_order_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "positions": list(ctx.C1_POSITIONS),
        "cube_pose": {"y": float(ctx.C1_TRACK_Y), "z": float(ctx.C1_TRACK_Z)},
        "bbox_area_px": {},
        "bbox_px": {},
        "red_pixels": {},
        "frame_stamp_s": {},
        "min_margin_ratio": float(ctx.C1_MIN_MARGIN_RATIO),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c1_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c1_size_order_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c1_size_order={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c1_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c1_size_order_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c1_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c1_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C1_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C1_CUBE_NAME}"
        _store_c1_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C1_CUBE_NAME}")
    
    prev_stamp_s: Optional[float] = None
    for x in ctx.C1_POSITIONS:
        label = f"x{int(x)}"
        _mono__move_and_settle(ctx, 
            simulator,
            ctx.C1_CUBE_NAME,
            x=float(x),
            y=float(ctx.C1_TRACK_Y),
            z=float(ctx.C1_TRACK_Z),
        )
    
        try:
            msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        except Exception as exc:
            metrics["error_reason"] = f"image_receive_failed:{exc}"
            _store_c1_diag()
            raise RuntimeError(f"Failed to receive fresh image for C1 from topic {resolved_topic}: {exc}") from exc
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        red = _mono__red_mask(ctx, hsv)
        area, (bx, by, bw, bh) = _mono__bbox_area(ctx, red)
        metrics["bbox_area_px"][label] = int(area)
        metrics["bbox_px"][label] = {"x": int(bx), "y": int(by), "w": int(bw), "h": int(bh)}
        metrics["red_pixels"][label] = int(_mono__count_pixels(ctx, red))
        metrics["frame_stamp_s"][label] = float(prev_stamp_s)
    
        artifacts.append(_mono__save_frame(ctx, f"c1_{label}_raw.png", frame))
    
        debug = frame.copy()
        if area > 0:
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
        cv2.putText(
            debug,
            f"{label}: area={area} ts={prev_stamp_s:.6f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        artifacts.append(_mono__save_frame(ctx, f"c1_{label}.png", debug))
    
    x1 = metrics["bbox_area_px"].get("x1", 0)
    x3 = metrics["bbox_area_px"].get("x3", 0)
    x5 = metrics["bbox_area_px"].get("x5", 0)
    order_ok = x1 > x3 > x5
    margin_ok = (x1 >= x3 * ctx.C1_MIN_MARGIN_RATIO) and (x3 >= x5 * ctx.C1_MIN_MARGIN_RATIO)
    metrics["checks"] = {"size_order": bool(order_ok), "size_margin": bool(margin_ok)}
    metrics["status"] = "PASS" if (order_ok and margin_ok) else "FAIL"
    if not (order_ok and margin_ok):
        metrics["error_reason"] = (
            f"size_order_failed: checks={metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )
    
    metrics_path = _store_c1_diag()
    if not (order_ok and margin_ok):
        raise AssertionError(f"C1 checks failed: {metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}")
    
    return {"id": "C1", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c4_geometries_presence_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c4_geometries_presence_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "pixel_counts": {},
        "threshold": int(ctx.C4_MIN_PIXELS),
        "display_env": {},
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c4_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c4_geometries_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c4_geometries_presence={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c4_diag()
    
    try:
        _mono__open_c4_scene_with_retry(ctx, simulator)
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c4_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c4_diag()
    
    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _store_c4_diag()
        raise RuntimeError(f"Failed to receive image for C4 from topic {resolved_topic}: {exc}") from exc
    
    frame = _mono__msg_to_bgr(ctx, msg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    for color in ("red", "green", "blue", "yellow"):
        metrics["pixel_counts"][color] = _mono__count_pixels(ctx, _mono__color_mask(ctx, hsv, color))
    
    missing = {c: n for c, n in metrics["pixel_counts"].items() if int(n) <= int(metrics["threshold"])}
    metrics["checks"] = {"all_present": len(missing) == 0, "missing_or_low": missing}
    metrics["status"] = "PASS" if len(missing) == 0 else "FAIL"
    if missing:
        metrics["error_reason"] = f"missing_or_low:{missing}"
    
    debug = _mono__annotate(ctx, 
        frame,
        [
            f"topic={resolved_topic}",
            f"threshold={metrics['threshold']}",
            f"red={metrics['pixel_counts'].get('red', 0)}",
            f"green={metrics['pixel_counts'].get('green', 0)}",
            f"blue={metrics['pixel_counts'].get('blue', 0)}",
            f"yellow={metrics['pixel_counts'].get('yellow', 0)}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c4_geometries.png", debug))
    metrics_path = _store_c4_diag()
    
    if missing:
        raise AssertionError(f"C4 checks failed: missing_or_low={missing}, threshold={metrics['threshold']}")
    
    return {"id": "C4", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c7_occlusion_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c7_occlusion_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "cases": dict(ctx.C7_CASES),
        "blue_pixels": {},
        "threshold": int(ctx.C7_MIN_PIXELS),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c7_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c7_occlusion_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c7_occlusion={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c7_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c7_occlusion_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c7_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c7_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C7_FRONT_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_FRONT_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_FRONT_CUBE_NAME}")
    if not simulator.wait_for_model_spawn(ctx.C7_BACK_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_BACK_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_BACK_CUBE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c7_diag()
        raise RuntimeError(f"Failed to receive warmup image for C7 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    
    def _move_and_capture(model_name: str, x: float, y: float, z: float, settle_s: float = 0.35) -> np.ndarray:
        nonlocal prev_stamp_s
    
        _mono__move_and_settle(ctx, simulator, model_name, x=float(x), y=float(y), z=float(z), settle_s=settle_s)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        return _mono__msg_to_bgr(ctx, msg)
    
    _move_and_capture(ctx.C7_BACK_CUBE_NAME, x=3.6, y=0.0, z=0.25)
    
    for case_name, y in ctx.C7_CASES.items():
        frame = _move_and_capture(ctx.C7_FRONT_CUBE_NAME, x=3.0, y=float(y), z=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        blue = _mono__color_mask(ctx, hsv, "blue")
        blue_count = _mono__count_pixels(ctx, blue)
        metrics["blue_pixels"][case_name] = int(blue_count)
    
        debug = _mono__annotate(ctx, 
            frame,
            [
                f"topic={resolved_topic}",
                f"{case_name}: blue={blue_count}",
                f"front_y={float(y):.2f}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, f"c7_{case_name}.png", debug))
    
    blue_25 = metrics["blue_pixels"].get("occ_25", 0)
    blue_50 = metrics["blue_pixels"].get("occ_50", 0)
    relation_ok = blue_25 > blue_50
    threshold_ok = blue_25 > ctx.C7_MIN_PIXELS and blue_50 > ctx.C7_MIN_PIXELS
    metrics["checks"] = {"occlusion_relation": bool(relation_ok), "threshold_ok": bool(threshold_ok)}
    metrics["status"] = "PASS" if (relation_ok and threshold_ok) else "FAIL"
    if not (relation_ok and threshold_ok):
        metrics["error_reason"] = (
            f"occlusion_mismatch: occ_25={blue_25}, occ_50={blue_50}, threshold={ctx.C7_MIN_PIXELS}"
        )
    
    metrics_path = _store_c7_diag()
    
    if not (relation_ok and threshold_ok):
        raise AssertionError(
            f"C7 checks failed: {metrics['checks']}, blue_pixels={metrics['blue_pixels']}, "
            f"threshold={ctx.C7_MIN_PIXELS}"
        )
    
    return {"id": "C7", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c2_resolution_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c2_resolution_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "expected_resolution": {
            "width": int(ctx.image_width),
            "height": int(ctx.image_height),
        },
        "actual_resolution": None,
        "encoding": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "status": "ERROR",
        "error_reason": "",
        "display_env": {},
    }
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    
    world = ctx.test_to_world["c2_resolution_test"]
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        scene_diag = _mono__scene_diag(ctx, simulator)
        metrics["resolved_topic"] = str(ctx.IMAGE_TOPIC)
        metrics["topic_mapping_changed"] = False
        metrics["scene_open_success"] = False
        metrics["error_reason"] = f"scene_open_failed:{scene_diag.get('reason', 'unknown')}" if scene_diag else "scene_open_failed"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to open scene for c2_resolution_test: {world} "
            f"(reason={scene_diag.get('reason', 'unknown') if scene_diag else 'unknown'})"
        )
    
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
    metrics["scene_open_success"] = True
    
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    
    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(f"Failed to receive image for C2 from topic {resolved_topic}: {exc}") from exc
    
    actual_width = int(getattr(msg, "width", 0) or 0)
    actual_height = int(getattr(msg, "height", 0) or 0)
    metrics["actual_resolution"] = {
        "width": actual_width,
        "height": actual_height,
    }
    metrics["encoding"] = str(getattr(msg, "encoding", "") or "")
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    
    if actual_width <= 0 or actual_height <= 0:
        metrics["error_reason"] = "invalid_image_resolution"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"C2 received invalid image dimensions from topic {resolved_topic}: "
            f"width={actual_width}, height={actual_height}"
        )
    
    try:
        frame = _mono__msg_to_bgr(ctx, msg)
    except Exception:
        frame = None
    
    if frame is not None:
        debug = _mono__annotate(ctx, 
            frame,
            [
                f"topic={resolved_topic}",
                f"expected={ctx.image_width}x{ctx.image_height}",
                f"actual={actual_width}x{actual_height}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, "c2_resolution_frame.png", debug))
    
    resolution_matches = (
        actual_width == int(ctx.image_width) and actual_height == int(ctx.image_height)
    )
    metrics["checks"] = {"resolution_matches": bool(resolution_matches)}
    metrics["status"] = "PASS" if resolution_matches else "FAIL"
    if not resolution_matches:
        metrics["error_reason"] = (
            f"resolution_mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}"
        )
    
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    metrics_path = _mono__save_metrics_json(ctx, "c2_resolution_metrics.json", metrics)
    if not resolution_matches:
        raise AssertionError(
            f"C2 resolution mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}, topic={resolved_topic}"
        )
    
    return {"id": "C2", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c9_fov_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    x_fixed = 2.0
    
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c9_fov_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "x_fixed_m": float(x_fixed),
        "step_y_m": float(ctx.C9_STEP),
        "target_fov_rad": float(ctx.horizontal_fov),
        "sphere_radius_m": float(ctx.C9_SPHERE_RADIUS_M),
        "samples": [],
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c9_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c9_fov_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c9_fov={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c9_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c9_fov_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c9_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c9_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C9_SPHERE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C9_SPHERE_NAME}"
        _store_c9_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C9_SPHERE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c9_diag()
        raise RuntimeError(f"Failed to receive warmup image for C9 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    last_visible: Optional[Tuple[float, np.ndarray, int]] = None
    first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None
    
    for y in _mono__iter_float_range(ctx, 0.0, float(ctx.C9_MAX_Y), float(ctx.C9_STEP)):
        _mono__move_and_settle(ctx, simulator, ctx.C9_SPHERE_NAME, x=x_fixed, y=float(y), z=0.2, settle_s=0.35)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        white = _mono__white_mask(ctx, frame)
        contours = _mono__large_contours(ctx, white, min_area=ctx.C9_MIN_CONTOUR_AREA, border_margin=4)
        contour_count = len(contours)
        visible = contour_count > 0
    
        metrics["samples"].append({"y_m": float(y), "visible": bool(visible), "contours": int(contour_count)})
    
        if visible:
            last_visible = (float(y), frame, contour_count)
        elif last_visible is not None:
            first_not_visible = (float(y), frame, contour_count)
            break
    
    if last_visible is None:
        metrics["error_reason"] = "object_never_detected"
        _store_c9_diag()
        raise AssertionError("C9 failed: object was never detected in frame")
    if first_not_visible is None:
        metrics["error_reason"] = "object_never_lost"
        _store_c9_diag()
        raise AssertionError("C9 failed: object did not disappear within tested Y range")
    
    y_visible = float(last_visible[0])
    y_lost = float(first_not_visible[0])
    y_transition = float((y_visible + y_lost) / 2.0)
    y_edge_estimate = float(y_transition + float(ctx.C9_SPHERE_RADIUS_M))
    measured_fov = float(2.0 * atan(y_edge_estimate / x_fixed))
    target_fov = float(ctx.horizontal_fov)
    rel_error = float(abs(measured_fov - target_fov) / target_fov) if target_fov > 0 else float("inf")
    
    dbg_visible = _mono__annotate(ctx, 
        last_visible[1],
        [
            f"topic={resolved_topic}",
            f"Yvisible={y_visible:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVmeasured={measured_fov:.5f} rad",
            "visible=True",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c9_ymax_visible.png", dbg_visible))
    
    dbg_lost = _mono__annotate(ctx, 
        first_not_visible[1],
        [
            f"topic={resolved_topic}",
            f"Ylost={y_lost:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVtarget={target_fov:.5f} rad",
            "visible=False",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c9_after_ymax_not_visible.png", dbg_lost))
    
    metrics["y_max_visible_m"] = y_visible
    metrics["y_first_not_visible_m"] = y_lost
    metrics["y_transition_m"] = y_transition
    metrics["y_edge_estimate_m"] = y_edge_estimate
    metrics["fov_measured_rad"] = measured_fov
    metrics["fov_measured_deg"] = float(degrees(measured_fov))
    metrics["fov_target_deg"] = float(degrees(target_fov))
    metrics["relative_error"] = rel_error
    metrics["checks"] = {"rel_error_le_0_02": bool(rel_error <= 0.02)}
    metrics["status"] = "PASS" if rel_error <= 0.02 else "FAIL"
    if rel_error > 0.02:
        metrics["error_reason"] = (
            f"fov_mismatch: measured={measured_fov:.6f}, target={target_fov:.6f}, rel_error={rel_error:.4f}"
        )
    
    metrics_path = _store_c9_diag()
    if rel_error > 0.02:
        raise AssertionError(
            f"C9 failed: measured={measured_fov:.6f} rad, target={target_fov:.6f} rad, rel_error={rel_error:.4f}"
        )
    
    return {"id": "C9", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c10_clipping_test(ctx, simulator) -> Dict[str, Any]:
    clip_cfg = _mono__read_clip_from_sdf(ctx, ctx.sensor_sdf_path)
    near_target = float(clip_cfg["near_m"]) if clip_cfg.get("near_m") is not None else float(ctx.clip_near)
    far_target = float(clip_cfg["far_m"]) if clip_cfg.get("far_m") is not None else float(ctx.clip_far)
    near_tol = max(0.05, abs(near_target) * 0.05)
    far_tol = max(0.05, abs(far_target) * 0.05)
    near_start = min(float(ctx.C10_NEAR_START_X), max(0.01, near_target * 0.3))
    near_end = max(float(ctx.C10_NEAR_SEARCH_END_X), near_target * 8.0)
    
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c10_clipping_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "near_clip_target_m": float(near_target),
        "far_clip_target_m": float(far_target),
        "clip_source": clip_cfg,
        "near_tolerance_m": float(near_tol),
        "far_tolerance_m": float(far_tol),
        "near_search": {"start_x": float(near_start), "end_x": float(near_end)},
        "far_search": {"coarse_step": float(ctx.C10_FAR_COARSE_STEP), "fine_step": float(ctx.C10_FAR_FINE_STEP)},
        "min_red_pixels": int(ctx.C10_MIN_RED_PIXELS),
        "visibility_rule": f"red_pixels>={int(ctx.C10_MIN_RED_PIXELS)}",
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c10_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c10_clipping_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c10_clipping={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c10_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c10_clipping_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c10_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c10_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C10_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C10_CUBE_NAME}"
        _store_c10_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C10_CUBE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c10_diag()
        raise RuntimeError(f"Failed to receive warmup image for C10 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    
    def _move_and_capture(x: float, settle_s: float) -> Tuple[np.ndarray, Dict[str, float], int]:
        nonlocal prev_stamp_s
    
        _mono__move_and_settle(ctx, simulator, ctx.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=settle_s)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        red_stats = _mono__red_stats(ctx, frame)
        red_pixels = int(red_stats["red_pixels"])
        return frame, red_stats, red_pixels
    
    near_before: Optional[Tuple[float, np.ndarray, int]] = None
    near_after: Optional[Tuple[float, np.ndarray, int]] = None
    
    for x in _mono__iter_float_range(ctx, near_start, near_end, ctx.C10_NEAR_STEP):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.2)
        if bool(red_stats["visible_by_pixels"]):
            near_after = (float(x), frame, red_pixels)
            break
        near_before = (float(x), frame, red_pixels)
    
    if near_after is None:
        metrics["error_reason"] = "near_boundary_not_found"
        _store_c10_diag()
        raise AssertionError("C10 failed: clip_cube did not appear in near search range")
    
    near_x = float(near_after[0])
    metrics["x_near_m"] = near_x
    metrics["near_boundary"] = {
        "before_x_m": float(near_before[0]) if near_before is not None else None,
        "after_x_m": float(near_after[0]),
    }
    
    far_last_visible = near_after
    far_first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None
    
    far_search_stop = float(far_target) * 1.2 + max(2.0, 0.2 * float(far_target))
    metrics["far_search_stop_m"] = float(far_search_stop)
    for x in _mono__iter_float_range(ctx, 
        near_x + float(ctx.C10_FAR_COARSE_STEP),
        far_search_stop,
        float(ctx.C10_FAR_COARSE_STEP),
    ):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.2)
        visible = bool(red_stats["visible_by_pixels"])
        if visible:
            far_last_visible = (float(x), frame, red_pixels)
        else:
            far_first_not_visible = (float(x), frame, red_pixels)
            break
    
    if far_first_not_visible is None:
        metrics["error_reason"] = "far_boundary_not_found"
        _store_c10_diag()
        raise AssertionError("C10 failed: clip_cube did not disappear in far search range")
    
    fine_start = max(float(near_x), float(far_last_visible[0]) - float(ctx.C10_FAR_COARSE_STEP))
    fine_end = float(far_first_not_visible[0])
    far_last_visible_fine = far_last_visible
    far_first_not_visible_fine = far_first_not_visible
    
    for x in _mono__iter_float_range(ctx, fine_start, fine_end, float(ctx.C10_FAR_FINE_STEP)):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.15)
        visible = bool(red_stats["visible_by_pixels"])
        if visible:
            far_last_visible_fine = (float(x), frame, red_pixels)
        else:
            far_first_not_visible_fine = (float(x), frame, red_pixels)
            break
    
    far_x = float(far_last_visible_fine[0])
    metrics["x_far_m"] = far_x
    metrics["x_far_first_not_visible_m"] = float(far_first_not_visible_fine[0])
    metrics["far_boundary"] = {
        "last_visible_x_m": float(far_last_visible_fine[0]),
        "first_not_visible_x_m": float(far_first_not_visible_fine[0]),
    }
    
    near_ok = abs(near_x - float(near_target)) <= float(near_tol)
    far_ok = abs(far_x - float(far_target)) <= float(far_tol)
    metrics["checks"] = {
        "near_abs_error_m": float(abs(near_x - float(near_target))),
        "far_abs_error_m": float(abs(far_x - float(far_target))),
        "near_ok": bool(near_ok),
        "far_ok": bool(far_ok),
    }
    
    near_after_stats = _mono__red_stats(ctx, near_after[1])
    far_before_stats = _mono__red_stats(ctx, far_last_visible_fine[1])
    far_after_stats = _mono__red_stats(ctx, far_first_not_visible_fine[1])
    metrics["visibility_debug"] = {
        "near_before_red_pixels": int(near_before[2]) if near_before is not None else None,
        "near_before_pixel_ratio": float(_mono__red_stats(ctx, near_before[1])["pixel_ratio"]) if near_before is not None else None,
        "near_after_red_pixels": int(near_after[2]),
        "near_after_pixel_ratio": float(near_after_stats["pixel_ratio"]),
        "far_before_red_pixels": int(far_last_visible_fine[2]),
        "far_before_pixel_ratio": float(far_before_stats["pixel_ratio"]),
        "far_after_red_pixels": int(far_first_not_visible_fine[2]),
        "far_after_pixel_ratio": float(far_after_stats["pixel_ratio"]),
    }
    
    if near_before is not None:
        dbg = _mono__annotate(ctx, 
            near_before[1],
            [
                f"near_before x={near_before[0]:.2f}",
                f"red_px={near_before[2]}",
                f"ratio={_mono__red_stats(ctx, near_before[1])['pixel_ratio']:.4f}",
                "visible=False",
                f"topic={resolved_topic}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, "c10_near_before.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        near_after[1],
        [
            f"near_after x={near_after[0]:.2f}",
            f"red_px={near_after[2]}",
            f"ratio={near_after_stats['pixel_ratio']:.4f}",
            "visible=True",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_near_after.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        far_last_visible_fine[1],
        [
            f"far_before x={far_last_visible_fine[0]:.2f}",
            f"red_px={far_last_visible_fine[2]}",
            f"ratio={far_before_stats['pixel_ratio']:.4f}",
            "visible=True",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_far_before.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        far_first_not_visible_fine[1],
        [
            f"far_after x={far_first_not_visible_fine[0]:.2f}",
            f"red_px={far_first_not_visible_fine[2]}",
            f"ratio={far_after_stats['pixel_ratio']:.4f}",
            "visible=False",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_far_after.png", dbg))
    
    metrics["status"] = "PASS" if (near_ok and far_ok) else "FAIL"
    if not (near_ok and far_ok):
        metrics["error_reason"] = (
            f"clipping_mismatch: near={near_x:.3f}/{near_target:.3f}, "
            f"far={far_x:.3f}/{far_target:.3f}"
        )
    
    metrics_path = _store_c10_diag()
    if not (near_ok and far_ok):
        raise AssertionError(
            f"C10 failed: near={near_x:.3f} (target {near_target:.3f}, tol={near_tol:.3f}), "
            f"far={far_x:.3f} (target {far_target:.3f}, tol={far_tol:.3f})"
        )
    
    return {"id": "C10", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def _mono_c11_fps_stability_test(ctx, simulator) -> Dict[str, Any]:
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c11_fps_stability_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "duration_target_s": float(ctx.C11_DURATION_S),
        "update_rate_target_hz": float(ctx.update_rate),
        "warmup_seconds": float(ctx.C11_WARMUP_SECONDS),
        "jitter_percentile": int(ctx.C11_JITTER_PERCENTILE),
        "jitter_limit_s": float(ctx.C11_MAX_JITTER_S),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c11_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c11_fps_stability_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c11_fps_stability={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c11_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c11_fps_stability_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c11_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c11_diag()
    
    try:
        _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_receive_failed:{exc}"
        _store_c11_diag()
        raise RuntimeError(f"Failed to receive warmup image for C11 from topic {resolved_topic}: {exc}") from exc
    
    timestamps: List[float] = []
    first_msg: Dict[str, Optional[Image]] = {"msg": None}
    last_msg: Dict[str, Optional[Image]] = {"msg": None}
    header_stamp_missing_count = 0
    
    def _on_image(msg: Image) -> None:
        nonlocal header_stamp_missing_count
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            header_stamp_missing_count += 1
            stamp = float(rospy.Time.now().to_sec())
        timestamps.append(stamp)
        if first_msg["msg"] is None:
            first_msg["msg"] = msg
        last_msg["msg"] = msg
    
    sub = rospy.Subscriber(resolved_topic, Image, _on_image, queue_size=2000)
    started_wall = time.perf_counter()
    try:
        while (time.perf_counter() - started_wall) < float(ctx.C11_DURATION_S):
            time.sleep(0.1)
    finally:
        sub.unregister()
    
    metrics["duration_actual_s"] = round(time.perf_counter() - started_wall, 4)
    metrics["raw_frames_captured"] = int(len(timestamps))
    metrics["header_stamp_missing_count"] = int(header_stamp_missing_count)
    metrics["first_raw_stamp_s"] = float(timestamps[0]) if timestamps else None
    metrics["last_raw_stamp_s"] = float(timestamps[-1]) if timestamps else None
    if len(timestamps) < 2:
        metrics["error_reason"] = f"not_enough_frames_captured:{len(timestamps)}"
        _store_c11_diag()
        raise AssertionError(f"C11 failed: not enough frames captured ({len(timestamps)})")
    
    monotonic_stamps: List[float] = []
    for ts in timestamps:
        if not monotonic_stamps or ts > monotonic_stamps[-1]:
            monotonic_stamps.append(float(ts))
    
    metrics["monotonic_frames_captured"] = int(len(monotonic_stamps))
    metrics["non_monotonic_dropped"] = int(len(timestamps) - len(monotonic_stamps))
    if len(monotonic_stamps) < 2:
        metrics["error_reason"] = "no_monotonic_timestamp_sequence"
        _store_c11_diag()
        raise AssertionError("C11 failed: no monotonic timestamp sequence")
    
    warmup_frames = int(max(1, round(float(ctx.update_rate) * float(ctx.C11_WARMUP_SECONDS))))
    if len(monotonic_stamps) <= (warmup_frames + 1):
        metrics["error_reason"] = (
            f"not_enough_frames_after_warmup:{len(monotonic_stamps)} total,warmup={warmup_frames}"
        )
        _store_c11_diag()
        raise AssertionError(
            f"C11 failed: not enough frames after warmup ({len(monotonic_stamps)} total, warmup={warmup_frames})"
        )
    
    eval_stamps = monotonic_stamps[warmup_frames:]
    total_dt = float(eval_stamps[-1] - eval_stamps[0])
    if total_dt <= 0.0:
        metrics["error_reason"] = f"invalid_timestamps_interval:{total_dt}"
        _store_c11_diag()
        raise AssertionError(f"C11 failed: invalid timestamps interval ({total_dt})")
    
    n_frames = len(eval_stamps)
    fps_actual = float((n_frames - 1) / total_dt)
    ideal_dt = float(1.0 / float(ctx.update_rate))
    deltas = np.diff(np.array(eval_stamps, dtype=np.float64))
    
    abs_jitter = np.abs(deltas - ideal_dt) if deltas.size > 0 else np.array([], dtype=np.float64)
    # Было раньше: jitter = max(|dt - ideal_dt|), что слишком чувствительно к единичным пикам.
    # Теперь: устойчивый jitter = P95(|dt - ideal_dt|), max оставляем как диагностическую метрику.
    jitter = float(np.percentile(abs_jitter, ctx.C11_JITTER_PERCENTILE)) if abs_jitter.size > 0 else 0.0
    jitter_max_abs = float(np.max(abs_jitter)) if abs_jitter.size > 0 else 0.0
    max_dt = float(np.max(deltas)) if deltas.size > 0 else 0.0
    dropouts = int(np.sum(deltas > (2.0 * ideal_dt))) if deltas.size > 0 else 0
    
    fps_ok = fps_actual >= (0.95 * float(ctx.update_rate))
    jitter_ok = jitter <= float(ctx.C11_MAX_JITTER_S)
    dropouts_ok = dropouts == 0
    
    metrics.update(
        {
            "frames_captured": int(n_frames),
            "frames_skipped_warmup": int(warmup_frames),
            "first_eval_stamp_s": float(eval_stamps[0]),
            "last_eval_stamp_s": float(eval_stamps[-1]),
            "timestamps_interval_s": total_dt,
            "fps_actual_hz": fps_actual,
            "ideal_dt_s": ideal_dt,
            "jitter_s": jitter,
            "jitter_old_max_abs_s": jitter_max_abs,
            "max_dt_s": max_dt,
            "dropouts_count": dropouts,
            "checks": {
                "fps_ok": bool(fps_ok),
                "jitter_ok": bool(jitter_ok),
                "dropouts_ok": bool(dropouts_ok),
            },
        }
    )
    
    if first_msg["msg"] is not None:
        frame_first = _mono__msg_to_bgr(ctx, first_msg["msg"])
        debug_first = _mono__annotate(ctx, frame_first, ["C11 first frame", f"fps={fps_actual:.2f}", f"jitter={jitter:.4f}s"])
        artifacts.append(_mono__save_frame(ctx, "c11_first_frame.png", debug_first))
    if last_msg["msg"] is not None:
        frame_last = _mono__msg_to_bgr(ctx, last_msg["msg"])
        debug_last = _mono__annotate(ctx, frame_last, ["C11 last frame", f"dropouts={dropouts}", f"max_dt={max_dt:.4f}s"])
        artifacts.append(_mono__save_frame(ctx, "c11_last_frame.png", debug_last))
    
    metrics["status"] = "PASS" if (fps_ok and jitter_ok and dropouts_ok) else "FAIL"
    if not (fps_ok and jitter_ok and dropouts_ok):
        metrics["error_reason"] = (
            f"fps_jitter_or_dropout_failed: fps={fps_actual:.3f}, jitter={jitter:.4f}, dropouts={dropouts}"
        )
    
    metrics_path = _store_c11_diag()
    if not (fps_ok and jitter_ok and dropouts_ok):
        raise AssertionError(
            f"C11 failed: fps={fps_actual:.3f} (target>={0.95 * ctx.update_rate:.3f}), "
            f"jitter={jitter:.4f}s (limit<={ctx.C11_MAX_JITTER_S:.4f}s), dropouts={dropouts}"
        )
    
    return {"id": "C11", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}


class _DepthProfileTestContext:
    DEPTH_TOPIC = ""
    IMAGE_TOPIC = ""

    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 720
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.518
    CLIP_NEAR = 0.28
    CLIP_FAR = 10.0

    TEST_DISTANCES: Tuple[float, ...] = (1.0, 3.0, 5.0)
    MAX_ABS_ERROR_M = 0.8
    C3_TARGET_CUBE_NAME = "target_cube"
    C5_RANGE_CUBE_NAME = "range_cube"
    C6_SHIFT_CUBE_NAME = "shift_cube"

    C3_RADIUS_M = 2.0
    C3_SAMPLES = 30
    C3_MAX_DEV_RATIO = 0.05
    C3_FALLBACK_HALF_ANGLE_RAD = 0.6

    C5_START_X = 0.5
    C5_END_X = 10.0
    C5_STEP = 0.5
    C5_DEPTH_TOLERANCE_M = 0.10
    C5_CLIP_MARGIN_M = 0.05
    C5_TARGET_SIZE_X_M = 0.5
    DEPTH_ROI_HALF_WINDOW = 2  # 5x5 ROI

    C6_START_X = 2.0
    C6_END_X = 1.0
    C6_STEP = 0.01
    C6_TARGET_SIZE_X_M = 0.5
    C6_DEPTH_CHANGE_EPS_M = 0.004
    C6_MIN_CHANGED_RATIO = 0.80
    C6_MONOTONIC_TOLERANCE_M = 0.002
    C6_DELTA_TOLERANCE_M = 0.003
    C6_ABS_ERROR_TOLERANCE_M = 0.03
    DEPTH_TOPIC_WARMUP_TIMEOUT_S = 20.0
    DEPTH_WAIT_PER_CANDIDATE_S = 1.2
    FRAME_FRESH_TIMEOUT_S = 4.0
    CLIP_SATURATION_EPS_M = 0.02
    DEPTH_POST_MOVE_CONFIRMATION_FRAMES = 2
    C3_TARGET_SIZE_X_M = 0.5
    C3_MEAN_ABS_ERROR_M = 0.08
    C3_MAX_ABS_ERROR_M = 0.15

    def __init__(self, sensor):
        self.sensor = sensor
        self.sensor_name = str(getattr(sensor, "sensor_name", ""))
        self.sensor_type = str(getattr(sensor, "sensor_type", ""))
        self.sensor_sdf_path = str(getattr(sensor, "sdf_path", ""))
        self.CONFIG = {"ROOT_PATH": str(CONFIG["ROOT_PATH"])}
        self._last_test_diagnostics: Dict[str, Any] = {}

        profile = _camera_load_sensor_profile(self.sensor_sdf_path) if self.sensor_sdf_path else {}
        self.DEPTH_TOPIC = str(profile.get("depth_topic", "") or self.DEPTH_TOPIC)
        self.IMAGE_TOPIC = str(profile.get("image_topic", "") or getattr(sensor, "topic", "") or self.IMAGE_TOPIC)
        self.image_width = int(profile.get("image_width") or self.IMAGE_WIDTH)
        self.image_height = int(profile.get("image_height") or self.IMAGE_HEIGHT)
        self.horizontal_fov = float(profile.get("horizontal_fov") or self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(profile.get("clip_near") or self.CLIP_NEAR)
        self.clip_far = float(profile.get("clip_far") or self.CLIP_FAR)
        self.update_rate = int(profile.get("update_rate") or self.UPDATE_RATE)

        worlds_root = _camera_worlds_root()
        self.test_to_world = {
            "depth_perception_test": str(worlds_root / "camera_depth_perception.world"),
            "c3_view_angle_stability_test": str(worlds_root / "camera_c3_view_angle.world"),
            "c5_working_range_test": str(worlds_root / "camera_c5_working_range.world"),
            "c6_small_displacement_sensitivity_test": str(worlds_root / "camera_c6_small_shifts.world"),
        }
        self.camera_model_name = self._read_camera_model_name()
        self._resolved_depth_topic = ""
        self._resolved_image_topic = ""

    def _set_test_diagnostics(self, **kwargs) -> None:
        self._last_test_diagnostics.update(kwargs)

    def get_last_test_diagnostics(self) -> Dict[str, Any]:
        return copy.deepcopy(self._last_test_diagnostics)

    def _results_dir(self) -> str:
        path = os.path.join(self.CONFIG["ROOT_PATH"], "results", self.sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    def _captured_dir(self) -> str:
        path = os.path.join(self._results_dir(), "captured_images")
        os.makedirs(path, exist_ok=True)
        return path

    def _metrics_dir(self) -> str:
        path = os.path.join(self._results_dir(), "metrics")
        os.makedirs(path, exist_ok=True)
        return path

    def _save_frame(self, name: str, frame: np.ndarray) -> str:
        out = os.path.join(self._captured_dir(), name)
        cv2.imwrite(out, frame)
        return out

    def _save_metrics_json(self, name: str, payload: Dict[str, Any]) -> str:
        out = os.path.join(self._metrics_dir(), name)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return out

    def _open_test_scene(self, simulator, test_name: str) -> None:
        self._last_test_diagnostics = {}
        self._resolved_depth_topic = ""
        self._resolved_image_topic = ""
        world = self.test_to_world[test_name]
        display_env = self._ensure_render_display_env()
        if display_env:
            self._set_test_diagnostics(render_display_env=display_env)
        if not simulator.open_scene(world, self.sensor_sdf_path):
            diag = self._scene_diag(simulator)
            reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
            raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")
        self._update_resolved_topics(simulator)
        self._set_test_diagnostics(
            scene_open_success=True,
            world_file=str(world),
            expected_depth_topic=str(self.DEPTH_TOPIC),
            expected_image_topic=str(self.IMAGE_TOPIC or ""),
            resolved_depth_topic=str(self._resolved_depth_topic or self.DEPTH_TOPIC),
            resolved_image_topic=str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
        )
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=30.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)

    @staticmethod
    def _iter_float_range(start: float, stop: float, step: float) -> List[float]:
        values: List[float] = []
        cur = float(start)
        while cur <= float(stop) + 1e-9:
            values.append(round(cur, 4))
            cur += float(step)
        return values

    @staticmethod
    def _iter_float_range_desc(start: float, stop: float, step: float) -> List[float]:
        values: List[float] = []
        cur = float(start)
        while cur >= float(stop) - 1e-9:
            values.append(round(cur, 4))
            cur -= float(step)
        return values

    @staticmethod
    def _depth_msg_to_meters(msg: Image) -> np.ndarray:
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "").lower()

        if enc == "32fc1":
            dtype = np.float32
            bytes_per_px = 4
        elif enc == "16uc1":
            dtype = np.uint16
            bytes_per_px = 2
        else:
            raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

        row_stride_bytes = int(msg.step) if int(msg.step) > 0 else int(w * bytes_per_px)
        min_row_bytes = int(w * bytes_per_px)
        if row_stride_bytes < min_row_bytes:
            raise ValueError(
                f"Invalid depth step for encoding={msg.encoding}: step={row_stride_bytes}, min_required={min_row_bytes}"
            )

        cols_with_stride = row_stride_bytes // bytes_per_px
        raw = np.frombuffer(msg.data, dtype=dtype)
        expected_size = int(h * cols_with_stride)
        if raw.size < expected_size:
            raise ValueError(
                f"Depth buffer too small for encoding={msg.encoding}: got={raw.size}, expected={expected_size}"
            )

        arr = raw[:expected_size].reshape(h, cols_with_stride)[:, :w]
        if enc == "16uc1":
            arr_mm = arr.astype(np.uint16, copy=False)
            return arr_mm.astype(np.float32) / 1000.0

        return arr.astype(np.float32, copy=False)

    @staticmethod
    def _color_msg_to_bgr(msg: Image) -> np.ndarray:
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "").lower()

        if enc in ("rgb8", "r8g8b8"):
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if enc == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

        if enc in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        return closed

    def _color_mask(self, bgr: np.ndarray, color: str) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
            "blue": ((95, 60, 40), (140, 255, 255)),
            "green": ((35, 60, 40), (90, 255, 255)),
            "yellow": ((15, 80, 80), (45, 255, 255)),
        }
        if color not in ranges:
            raise ValueError(f"Unsupported color: {color}")
        lower, upper = ranges[color]
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        return self._clean_mask(mask)

    def _find_centroid(self, bgr: np.ndarray, color: str, min_area: float = 120.0) -> Optional[Tuple[int, int]]:
        mask = self._color_mask(bgr, color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < float(min_area):
            return None
        moments = cv2.moments(contour)
        if moments["m00"] <= 1e-6:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return cx, cy

    @staticmethod
    def _depth_at_pixel_with_meta(depth_m: np.ndarray, x: int, y: int, half_window: int = 2) -> Tuple[Optional[float], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        x0 = max(0, int(x) - int(half_window))
        y0 = max(0, int(y) - int(half_window))
        x1 = min(w, int(x) + int(half_window) + 1)
        y1 = min(h, int(y) + int(half_window) + 1)
        roi = depth_m[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        meta = {
            "pixel": {"x": int(x), "y": int(y)},
            "roi_xyxy": [int(x0), int(y0), int(x1), int(y1)],
            "roi_shape": [int(max(0, y1 - y0)), int(max(0, x1 - x0))],
            "valid_count": int(valid.size),
            "aggregator": "mean",
        }
        if valid.size == 0:
            return None, meta
        return float(np.mean(valid)), meta

    def _depth_at_pixel(self, depth_m: np.ndarray, x: int, y: int, window: int = 2) -> Optional[float]:
        z, _ = self._depth_at_pixel_with_meta(depth_m, x=x, y=y, half_window=window)
        return z

    def _depth_frame_stats(self, depth_msg: Image, depth_m: np.ndarray) -> Dict[str, Any]:
        finite = depth_m[np.isfinite(depth_m)]
        positive = finite[finite > 0.0] if finite.size > 0 else np.array([], dtype=np.float32)
        stats: Dict[str, Any] = {
            "encoding": str(depth_msg.encoding),
            "dtype": str(depth_m.dtype),
            "shape": [int(depth_m.shape[0]), int(depth_m.shape[1])],
            "step_bytes": int(depth_msg.step),
            "is_bigendian": int(depth_msg.is_bigendian),
            "finite_count": int(finite.size),
            "positive_count": int(positive.size),
            "finite_min_m": None,
            "finite_max_m": None,
        }
        if finite.size > 0:
            stats["finite_min_m"] = float(np.min(finite))
            stats["finite_max_m"] = float(np.max(finite))
        return stats

    @staticmethod
    def _depth_preview(depth_m: np.ndarray) -> np.ndarray:
        finite = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
        if finite.size == 0:
            return np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)
        dmin = float(np.percentile(finite, 5))
        dmax = float(np.percentile(finite, 95))
        if dmax <= dmin:
            dmax = dmin + 1e-3
        norm = np.clip((depth_m - dmin) / (dmax - dmin), 0.0, 1.0)
        u8 = (norm * 255.0).astype(np.uint8)
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

    @staticmethod
    def _roi_depth_stats(depth_m: np.ndarray, roi_xyxy: List[int]) -> Dict[str, Any]:
        if not roi_xyxy or len(roi_xyxy) != 4:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        x0, y0, x1, y1 = [int(v) for v in roi_xyxy]
        h, w = depth_m.shape[:2]
        x0 = max(0, min(w, x0))
        x1 = max(0, min(w, x1))
        y0 = max(0, min(h, y0))
        y1 = max(0, min(h, y1))
        if x1 <= x0 or y1 <= y0:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        roi = depth_m[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            return {"valid_count": 0, "min_m": None, "max_m": None, "mean_m": None, "median_m": None}
        return {
            "valid_count": int(valid.size),
            "min_m": float(np.min(valid)),
            "max_m": float(np.max(valid)),
            "mean_m": float(np.mean(valid)),
            "median_m": float(np.median(valid)),
        }

    @staticmethod
    def _fallback_point_from_finite_depth(depth_m: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        y0 = int(h * 0.2)
        y1 = int(h * 0.8)
        x0 = int(w * 0.2)
        x1 = int(w * 0.8)
        central = depth_m[y0:y1, x0:x1]
        central_valid = np.isfinite(central) & (central > 0.0)

        meta: Dict[str, Any] = {
            "strategy": "finite_depth_min",
            "central_window_xyxy": [int(x0), int(y0), int(x1), int(y1)],
            "central_valid_count": int(np.count_nonzero(central_valid)),
            "fallback_scope": "central",
        }

        if np.count_nonzero(central_valid) > 0:
            central_depth = np.where(central_valid, central, np.inf)
            idx = np.unravel_index(int(np.argmin(central_depth)), central_depth.shape)
            py = int(y0 + idx[0])
            px = int(x0 + idx[1])
            meta["chosen_depth_m"] = float(depth_m[py, px])
            return (px, py), meta

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        meta["fallback_scope"] = "global"
        meta["global_valid_count"] = int(np.count_nonzero(valid))
        if np.count_nonzero(valid) == 0:
            return None, meta

        masked = np.where(valid, depth_m, np.inf)
        idx = np.unravel_index(int(np.argmin(masked)), masked.shape)
        py = int(idx[0])
        px = int(idx[1])
        meta["chosen_depth_m"] = float(depth_m[py, px])
        return (px, py), meta

    def _measure_depth_with_meta(
        self,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
    ) -> Tuple[Optional[float], Tuple[int, int], Dict[str, Any]]:
        h, w = depth_m.shape[:2]
        if bgr is not None and color_hint:
            centroid = self._find_centroid(bgr, color_hint)
            if centroid is not None:
                z, roi_meta = self._depth_at_pixel_with_meta(
                    depth_m,
                    centroid[0],
                    centroid[1],
                    half_window=int(self.DEPTH_ROI_HALF_WINDOW),
                )
                if z is not None:
                    roi_meta["source"] = "color_centroid"
                    roi_meta["color_hint"] = str(color_hint)
                    return z, centroid, roi_meta

        fallback_point, fallback_diag = self._fallback_point_from_finite_depth(depth_m)
        if fallback_point is not None:
            z_fallback, roi_meta = self._depth_at_pixel_with_meta(
                depth_m,
                fallback_point[0],
                fallback_point[1],
                half_window=int(self.DEPTH_ROI_HALF_WINDOW),
            )
            if z_fallback is not None:
                roi_meta["source"] = "finite_depth_fallback"
                roi_meta["color_hint"] = str(color_hint) if color_hint else None
                roi_meta["fallback_meta"] = fallback_diag
                return z_fallback, fallback_point, roi_meta

        cx, cy = w // 2, h // 2
        z_center, roi_meta = self._depth_at_pixel_with_meta(
            depth_m,
            cx,
            cy,
            half_window=int(self.DEPTH_ROI_HALF_WINDOW),
        )
        roi_meta["source"] = "frame_center"
        roi_meta["color_hint"] = str(color_hint) if color_hint else None
        return z_center, (cx, cy), roi_meta

    def _measure_depth(
        self,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray] = None,
        color_hint: Optional[str] = None,
    ) -> Tuple[Optional[float], Tuple[int, int]]:
        z, point, _ = self._measure_depth_with_meta(depth_m=depth_m, bgr=bgr, color_hint=color_hint)
        return z, point

    @staticmethod
    def _draw_debug(
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray],
        point: Tuple[int, int],
        lines: List[str],
    ) -> np.ndarray:
        if bgr is not None:
            debug = bgr.copy()
        else:
            dm = depth_m.copy()
            finite = dm[np.isfinite(dm) & (dm > 0)]
            if finite.size == 0:
                debug = np.zeros((depth_m.shape[0], depth_m.shape[1], 3), dtype=np.uint8)
            else:
                dmin = float(np.percentile(finite, 5))
                dmax = float(np.percentile(finite, 95))
                if dmax <= dmin:
                    dmax = dmin + 1e-3
                norm = np.clip((dm - dmin) / (dmax - dmin), 0.0, 1.0)
                u8 = (norm * 255.0).astype(np.uint8)
                debug = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

        cv2.circle(debug, point, 5, (255, 255, 255), 2)
        y = 30
        for line in lines:
            cv2.putText(debug, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            y += 28
        return debug

    def _save_failure_artifacts(
        self,
        prefix: str,
        depth_m: np.ndarray,
        bgr: Optional[np.ndarray],
        point: Tuple[int, int],
        roi_meta: Dict[str, Any],
        extra_lines: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
        roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))

        if bgr is not None:
            rgb_dbg = bgr.copy()
        else:
            rgb_dbg = self._depth_preview(depth_m)
        cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
        lines = [
            f"source={roi_meta.get('source', 'unknown')}",
            f"valid={roi_stats.get('valid_count', 0)}",
            f"mean={roi_stats.get('mean_m') if roi_stats.get('mean_m') is not None else float('nan'):.3f}",
            f"median={roi_stats.get('median_m') if roi_stats.get('median_m') is not None else float('nan'):.3f}",
        ]
        for line in (extra_lines or []):
            lines.append(str(line))
        y = 30
        for line in lines:
            cv2.putText(rgb_dbg, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            y += 24
        rgb_path = self._save_frame(f"{prefix}_rgb.png", rgb_dbg)

        depth_dbg = self._depth_preview(depth_m)
        cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
        depth_path = self._save_frame(f"{prefix}_depth.png", depth_dbg)

        return {
            "rgb_artifact": rgb_path,
            "depth_artifact": depth_path,
            "roi_stats": roi_stats,
            "roi_meta": roi_meta,
        }

    @staticmethod
    def _read_model_name_from_sdf(path: str) -> str:
        if not path:
            return ""
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            model = root.find("model")
            if model is not None and model.get("name"):
                return str(model.get("name"))
        except Exception:
            return ""
        return ""

    def _read_camera_model_name(self) -> str:
        model_name = self._read_model_name_from_sdf(self.sensor_sdf_path)
        if model_name:
            return model_name
        return f"{self.sensor_name}_model"

    @staticmethod
    def _move_and_settle(simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.35) -> None:
        simulator.set_pose(model_name, x=float(x), y=float(y), z=float(z))
        time.sleep(settle_s)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        return Quaternion(0.0, 0.0, sin(float(yaw) * 0.5), cos(float(yaw) * 0.5))

    def _set_model_pose_6d(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        settle_s: float = 0.35,
    ) -> None:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose = Pose(
            Point(float(x), float(y), float(z)),
            self._yaw_to_quaternion(yaw),
        )
        response = set_state(state)
        if not response.success:
            raise RuntimeError(f"Failed to set model state for {model_name}: {response.status_message}")
        time.sleep(settle_s)

    @staticmethod
    def _list_image_topics() -> List[str]:
        try:
            published = rospy.get_published_topics()
        except Exception:
            return []
        return sorted([name for name, msg_type in published if msg_type == "sensor_msgs/Image"])

    @staticmethod
    def _choose_depth_topic(candidates: List[str]) -> str:
        if not candidates:
            return ""
        prioritized = sorted(
            candidates,
            key=lambda t: (0 if t.endswith("/depth/image_raw") else 1, len(t), t),
        )
        return prioritized[0]

    def _resolve_depth_topic(self, warmup_timeout: float) -> Tuple[str, Dict[str, Any]]:
        expected = str(self.DEPTH_TOPIC or "")
        sensor_name = str(self.sensor_name)
        preferred = [
            (expected, "expected"),
            (f"/{sensor_name}/depth/image_raw", "name_namespace"),
            (f"/{sensor_name}_depth/image_raw", "name_underscore"),
            (f"/{sensor_name}_depth/depth/image_raw", "name_underscore_nested"),
            (f"/{sensor_name}/depth_image_raw", "name_alt"),
        ]

        deadline = time.time() + float(warmup_timeout)
        last_topics: List[str] = []
        while time.time() < deadline:
            topics = self._list_image_topics()
            last_topics = topics
            for topic, source in preferred:
                if topic and topic in topics:
                    diag = {
                        "expected_depth_topic": expected,
                        "selected_depth_topic": topic,
                        "selected_source": source,
                        "topics_found": topics,
                        "topic_mapping_changed": bool(topic != expected),
                    }
                    return topic, diag
            time.sleep(0.2)

        if expected and expected in last_topics:
            diag = {
                "expected_depth_topic": expected,
                "selected_depth_topic": expected,
                "selected_source": "expected_after_warmup",
                "topics_found": last_topics,
                "topic_mapping_changed": False,
            }
            return expected, diag

        token = sensor_name
        generic = [
            t for t in last_topics
            if ("depth" in t and ("image_raw" in t or t.endswith("/image")))
        ]
        in_namespace = [t for t in generic if token in t]
        selected = self._choose_depth_topic(in_namespace) or self._choose_depth_topic(generic) or expected
        diag = {
            "expected_depth_topic": expected,
            "selected_depth_topic": selected,
            "selected_source": "heuristic_fallback",
            "topics_found": last_topics,
            "topic_mapping_changed": bool(selected != expected),
        }
        return selected, diag

    @staticmethod
    def _scene_diag(simulator) -> Dict[str, Any]:
        return {}

    @staticmethod
    def _ensure_render_display_env() -> Dict[str, str]:
        applied: Dict[str, str] = {}
        if os.environ.get("DISPLAY"):
            return applied

        xauthority = os.environ.get("XAUTHORITY", "").strip()
        if not xauthority:
            default_xauthority = os.path.expanduser("~/.Xauthority")
            if os.path.exists(default_xauthority):
                os.environ["XAUTHORITY"] = default_xauthority
                applied["XAUTHORITY"] = default_xauthority

        for display in (":0",):
            display_socket = f"/tmp/.X11-unix/X{display.lstrip(':')}"
            if os.path.exists(display_socket):
                os.environ["DISPLAY"] = display
                applied["DISPLAY"] = display
                break

        return applied

    @staticmethod
    def _msg_stamp_s(msg: Image) -> float:
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            return float(time.time())
        return stamp

    def _update_resolved_topics(self, simulator) -> Dict[str, Any]:
        scene_diag = self._scene_diag(simulator)
        self._resolved_depth_topic = str(self.DEPTH_TOPIC)
        self._resolved_image_topic = str(self.IMAGE_TOPIC or "")
        return scene_diag

    def _resolved_color_topic(self) -> str:
        return str(self._resolved_image_topic or self.IMAGE_TOPIC or "").strip()

    def _wait_message_after(
        self,
        prev_stamp_s: Optional[float],
        topic: str,
        timeout: float,
        stage: str,
    ) -> Image:
        target_topic = str(topic or "").strip()
        if not target_topic:
            raise RuntimeError(f"No topic configured for stage={stage}")

        start = time.time()
        saw_message = False
        while (time.time() - start) < float(timeout):
            remaining = max(0.2, float(timeout) - (time.time() - start))
            try:
                msg = rospy.wait_for_message(target_topic, Image, timeout=min(1.0, remaining))
            except rospy.ROSException:
                continue

            saw_message = True
            stamp_s = self._msg_stamp_s(msg)
            if prev_stamp_s is None or stamp_s > (float(prev_stamp_s) + 1e-6):
                return msg

        if saw_message:
            raise RuntimeError(
                f"No fresh frame on topic={target_topic} after stage={stage}; prev_stamp_s={prev_stamp_s}"
            )
        raise RuntimeError(f"No frame received on topic={target_topic} during stage={stage}")

    def _wait_depth_after(self, prev_stamp_s: Optional[float], timeout: Optional[float] = None, stage: str = "") -> Image:
        if not self.DEPTH_TOPIC:
            raise RuntimeError("DEPTH_TOPIC is not configured for this depth profile")
        if not self._resolved_depth_topic:
            selected, diag = self._resolve_depth_topic(self.DEPTH_TOPIC_WARMUP_TIMEOUT_S)
            self._resolved_depth_topic = selected or self.DEPTH_TOPIC
            self._set_test_diagnostics(depth_topic_resolution=diag)
        return self._wait_message_after(
            prev_stamp_s=prev_stamp_s,
            topic=self._resolved_depth_topic,
            timeout=float(timeout or self.FRAME_FRESH_TIMEOUT_S),
            stage=str(stage or "depth_wait_after"),
        )

    def _wait_color_after(self, prev_stamp_s: Optional[float], timeout: Optional[float] = None, stage: str = "") -> Optional[Image]:
        target_topic = self._resolved_color_topic()
        if not target_topic:
            return None
        return self._wait_message_after(
            prev_stamp_s=prev_stamp_s,
            topic=target_topic,
            timeout=float(timeout or self.FRAME_FRESH_TIMEOUT_S),
            stage=str(stage or "color_wait_after"),
        )

    def _is_far_clip_saturated(self, z: Optional[float], roi_stats: Dict[str, Any]) -> bool:
        if z is None or not np.isfinite(z):
            return False
        far_threshold = float(self.clip_far) - float(self.CLIP_SATURATION_EPS_M)
        if float(z) < far_threshold:
            return False

        roi_values = [
            roi_stats.get("min_m"),
            roi_stats.get("max_m"),
            roi_stats.get("mean_m"),
            roi_stats.get("median_m"),
        ]
        finite_values = [float(v) for v in roi_values if v is not None and np.isfinite(v)]
        if not finite_values:
            return True
        return all(value >= far_threshold for value in finite_values)

    def _wait_confirmed_depth_after(
        self,
        prev_stamp_s: Optional[float],
        fresh_frames: Optional[int] = None,
        stage: str = "",
    ) -> Tuple[Image, List[float]]:
        count = max(1, int(fresh_frames or self.DEPTH_POST_MOVE_CONFIRMATION_FRAMES))
        stamps: List[float] = []
        latest_prev = prev_stamp_s
        msg: Optional[Image] = None
        for idx in range(count):
            msg = self._wait_depth_after(
                prev_stamp_s=latest_prev,
                timeout=self.FRAME_FRESH_TIMEOUT_S,
                stage=f"{stage}_confirm_{idx + 1}",
            )
            latest_prev = self._msg_stamp_s(msg)
            stamps.append(float(latest_prev))
        assert msg is not None
        return msg, stamps

    @staticmethod
    def _front_face_depth(center_x_m: float, size_x_m: float) -> float:
        return float(center_x_m) - (float(size_x_m) * 0.5)

    def _c5_expected_in_contract_range(self, expected_depth: float) -> bool:
        near_limit = float(self.clip_near) + float(self.C5_CLIP_MARGIN_M)
        far_limit = float(self.clip_far) - float(self.C5_CLIP_MARGIN_M)
        return bool(near_limit <= float(expected_depth) <= far_limit)

    def _wait_depth(self, timeout: float = 3.0) -> Image:
        if not self.DEPTH_TOPIC:
            raise RuntimeError("DEPTH_TOPIC is not configured for this depth profile")
        if not self._resolved_depth_topic:
            selected, diag = self._resolve_depth_topic(self.DEPTH_TOPIC_WARMUP_TIMEOUT_S)
            self._resolved_depth_topic = selected or self.DEPTH_TOPIC
            self._set_test_diagnostics(depth_topic_resolution=diag)

        candidates = [self._resolved_depth_topic]
        if self._resolved_depth_topic != self.DEPTH_TOPIC:
            candidates.append(self.DEPTH_TOPIC)

        errors: List[str] = []
        for topic in candidates:
            try:
                wait_t = min(float(timeout), float(self.DEPTH_WAIT_PER_CANDIDATE_S))
                return rospy.wait_for_message(topic, Image, timeout=wait_t)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{topic}: {exc}")
                continue
        raise RuntimeError(f"Failed to receive depth frame. candidates={candidates}, errors={errors}")

    def _try_wait_color(self, timeout: float = 1.0) -> Optional[Image]:
        target_topic = self._resolved_color_topic()
        if not target_topic:
            return None
        try:
            return rospy.wait_for_message(target_topic, Image, timeout=timeout)
        except rospy.ROSException:
            return None

    def depth_perception_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "depth_perception_test")

        cube_name = "close_green_cube"
        cube_z = 0.25
        reset_x = 50.0

        if not simulator.wait_for_model_spawn(cube_name, 30):
            raise RuntimeError(f"cube not spawned: {cube_name}")

        results = []
        measured_values = []
        first_frame_diagnostics: Optional[Dict[str, Any]] = None
        last_depth_stamp_s: Optional[float] = None
        metrics_payload: Dict[str, Any] = {
            "status": "RUNNING",
            "error_reason": "",
            "world_file": str(self.test_to_world["depth_perception_test"]),
            "scene_open_success": True,
            "expected_depth_topic": str(self.DEPTH_TOPIC),
            "expected_image_topic": str(self.IMAGE_TOPIC or ""),
            "resolved_depth_topic": str(self._resolved_depth_topic or self.DEPTH_TOPIC),
            "resolved_image_topic": str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
            "distances_m": list(self.TEST_DISTANCES),
            "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
            "clip_near_m": float(self.clip_near),
            "clip_far_m": float(self.clip_far),
            "measurements": results,
        }

        def _persist_depth_metrics(status: str, error_reason: str = "") -> str:
            metrics_payload["status"] = str(status)
            metrics_payload["error_reason"] = str(error_reason)
            metrics_payload["first_frame_diagnostics"] = first_frame_diagnostics or {}
            metrics_payload["topic_diagnostics"] = self.get_last_test_diagnostics().get("depth_topic_resolution", {})
            metrics_payload["selected_depth_topic"] = str(self._resolved_depth_topic or self.DEPTH_TOPIC)
            metrics_payload["selected_image_topic"] = str(self._resolved_image_topic or self.IMAGE_TOPIC or "")
            path = self._save_metrics_json("depth_perception_metrics.json", metrics_payload)
            self._set_test_diagnostics(depth_perception_metrics_json=path)
            return path

        for d in self.TEST_DISTANCES:
            sample: Dict[str, Any] = {
                "distance_m": float(d),
                "status": "RUNNING",
                "resolved_depth_topic": str(self._resolved_depth_topic or self.DEPTH_TOPIC),
                "resolved_image_topic": str(self._resolved_image_topic or self.IMAGE_TOPIC or ""),
            }
            try:
                if last_depth_stamp_s is None:
                    baseline_msg = self._wait_depth(timeout=2.0)
                    last_depth_stamp_s = self._msg_stamp_s(baseline_msg)
                    sample["baseline_stamp_s"] = float(last_depth_stamp_s)

                simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
                reset_depth_msg = self._wait_depth_after(
                    prev_stamp_s=last_depth_stamp_s,
                    timeout=3.0,
                    stage=f"depth_perception_reset_d_{str(d).replace('.', '_')}",
                )
                reset_stamp_s = self._msg_stamp_s(reset_depth_msg)
                sample["reset_frame_stamp_s"] = float(reset_stamp_s)

                simulator.set_pose(cube_name, float(d) + 0.25, 0.0, cube_z)
                depth_msg, confirmation_stamps = self._wait_confirmed_depth_after(
                    prev_stamp_s=reset_stamp_s,
                    stage=f"depth_perception_target_d_{str(d).replace('.', '_')}",
                )
                depth_stamp_s = self._msg_stamp_s(depth_msg)
                last_depth_stamp_s = depth_stamp_s
                sample["depth_frame_stamp_s"] = float(depth_stamp_s)
                sample["post_move_depth_frame_stamps_s"] = [float(v) for v in confirmation_stamps]

                self._set_test_diagnostics(depth_topic_selected=self._resolved_depth_topic)
                depth_m = self._depth_msg_to_meters(depth_msg)
                color_msg = self._wait_color_after(
                    prev_stamp_s=depth_stamp_s - 1e-6,
                    timeout=2.0,
                    stage=f"depth_perception_color_d_{str(d).replace('.', '_')}",
                )
                bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                if color_msg is not None:
                    sample["color_frame_stamp_s"] = float(self._msg_stamp_s(color_msg))

                z, point, roi_meta = self._measure_depth_with_meta(depth_m, bgr=bgr, color_hint="green")
                sample["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                sample["measurement_roi"] = roi_meta
                sample["measurement_source"] = str(roi_meta.get("source", ""))
                if z is None or np.isnan(z) or z <= 0.0:
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[f"distance={float(d):.2f}", "reason=invalid_measurement"],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "invalid_measurement"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="invalid_measurement")
                    raise RuntimeError(f"Invalid depth measurement at distance={d}: {z}")

                if first_frame_diagnostics is None:
                    first_frame_diagnostics = self._depth_frame_stats(depth_msg, depth_m)
                    first_frame_diagnostics["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                    first_frame_diagnostics["measurement_roi"] = roi_meta
                    self._set_test_diagnostics(depth_perception_first_frame=first_frame_diagnostics)

                roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))
                far_clip_saturated = self._is_far_clip_saturated(z, roi_stats)
                sample["roi_depth_stats"] = roi_stats
                sample["measured_depth_m"] = float(z)
                sample["far_clip_saturated"] = bool(far_clip_saturated)

                if far_clip_saturated:
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_saturation_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[
                            f"distance={float(d):.2f}",
                            f"z={float(z):.3f}",
                            f"clip_far={float(self.clip_far):.3f}",
                            "reason=far_clip_saturation",
                        ],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "far_clip_saturation"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="far_clip_saturation")
                    raise AssertionError(
                        f"Depth measurement saturated at far clip at distance={d}: z={z}, clip_far={self.clip_far}"
                    )

                if not (self.clip_near <= float(z) <= (self.clip_far + 0.5)):
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_clip_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[f"distance={float(d):.2f}", f"z={float(z):.3f}", "reason=clip_range"],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "clip_range"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="clip_range")
                    raise AssertionError(
                        f"Depth out of clip range at distance={d}: z={z}, clip=({self.clip_near}, {self.clip_far})"
                    )

                abs_err = abs(float(z) - float(d))
                rel_err = abs_err / float(d) * 100.0
                sample["abs_error_m"] = float(abs_err)
                sample["rel_error_pct"] = float(rel_err)

                if abs_err > float(self.MAX_ABS_ERROR_M):
                    fail_artifacts = self._save_failure_artifacts(
                        prefix=f"depth_perception_fail_abs_err_d_{str(d).replace('.', '_')}",
                        depth_m=depth_m,
                        bgr=bgr,
                        point=point,
                        roi_meta=roi_meta,
                        extra_lines=[
                            f"distance={float(d):.2f}",
                            f"z={float(z):.3f}",
                            f"abs_err={float(abs_err):.3f}",
                            "reason=abs_error",
                        ],
                    )
                    sample["status"] = "FAIL"
                    sample["error_reason"] = "abs_error"
                    sample["artifacts"] = fail_artifacts
                    self._set_test_diagnostics(depth_perception_failure=fail_artifacts)
                    _persist_depth_metrics(status="FAIL", error_reason="abs_error")
                    raise AssertionError(
                        f"Depth absolute error too high at distance={d}: abs_err={abs_err:.4f}, max={self.MAX_ABS_ERROR_M}"
                    )

                sample["status"] = "PASS"
                results.append(sample)
                measured_values.append(float(z))
            except Exception:
                if sample not in results:
                    results.append(sample)
                raise

            x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
            if bgr is not None:
                rgb_dbg = bgr.copy()
            else:
                rgb_dbg = self._depth_preview(depth_m)
            cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                rgb_dbg,
                f"d={float(d):.2f} z={float(z):.3f}m",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._save_frame(f"depth_perception_rgb_d_{str(d).replace('.', '_')}.png", rgb_dbg)

            depth_dbg = self._depth_preview(depth_m)
            cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                depth_dbg,
                f"roi_min={roi_stats['min_m'] if roi_stats['min_m'] is not None else float('nan'):.3f} "
                f"roi_max={roi_stats['max_m'] if roi_stats['max_m'] is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self._save_frame(f"depth_perception_depth_d_{str(d).replace('.', '_')}.png", depth_dbg)

        monotonic_ok = all(measured_values[i] < measured_values[i + 1] for i in range(len(measured_values) - 1))
        if not monotonic_ok:
            _persist_depth_metrics(status="FAIL", error_reason="monotonicity")
            raise AssertionError(f"Depth monotonicity failed: {measured_values}")

        metrics_json = _persist_depth_metrics(status="PASS")
        return {
            "id": "DEPTH",
            "metrics": {
                "distances_m": list(self.TEST_DISTANCES),
                "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
                "measurements": results,
                "monotonic_increasing": True,
                "first_frame_diagnostics": first_frame_diagnostics or {},
                "topic_diagnostics": self.get_last_test_diagnostics().get("depth_topic_resolution", {}),
                "selected_depth_topic": self._resolved_depth_topic,
                "selected_image_topic": self._resolved_image_topic,
            },
            "metrics_json": metrics_json,
        }

    def c3_view_angle_stability_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "samples_target": int(self.C3_SAMPLES),
            "radius_m": float(self.C3_RADIUS_M),
            "expected_curve": "front_face_depth = radius*cos(theta) - target_half_depth",
            "mean_abs_error_limit_m": float(self.C3_MEAN_ABS_ERROR_M),
            "max_abs_error_limit_m": float(self.C3_MAX_ABS_ERROR_M),
            "samples": [],
            "mode": "",
        }

        self._open_test_scene(simulator, "c3_view_angle_stability_test")
        if not simulator.wait_for_model_spawn(self.C3_TARGET_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C3_TARGET_CUBE_NAME}")

        samples: List[Dict[str, Any]] = []
        use_camera_orbit = simulator.wait_for_model_spawn(self.camera_model_name, timeout=10)
        last_depth_stamp_s: Optional[float] = None

        if use_camera_orbit:
            metrics["mode"] = "camera_orbit"
            try:
                cube_x, cube_y, cube_z = 2.0, 0.0, 0.25
                cam_z = 0.25
                for i in range(self.C3_SAMPLES):
                    theta = 2.0 * pi * (float(i) / float(self.C3_SAMPLES))
                    cam_x = cube_x + self.C3_RADIUS_M * cos(theta)
                    cam_y = cube_y + self.C3_RADIUS_M * sin(theta)
                    yaw = atan2(cube_y - cam_y, cube_x - cam_x)
                    self._set_model_pose_6d(self.camera_model_name, x=cam_x, y=cam_y, z=cam_z, yaw=yaw, settle_s=0.35)

                    depth_msg = self._wait_depth_after(
                        prev_stamp_s=last_depth_stamp_s,
                        timeout=3.0,
                        stage=f"c3_camera_orbit_{i}",
                    )
                    last_depth_stamp_s = self._msg_stamp_s(depth_msg)
                    depth_m = self._depth_msg_to_meters(depth_msg)
                    color_msg = self._wait_color_after(
                        prev_stamp_s=last_depth_stamp_s - 1e-6,
                        timeout=2.0,
                        stage=f"c3_camera_orbit_color_{i}",
                    )
                    bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                    z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                    if z is None or np.isnan(z) or np.isinf(z):
                        continue
                    expected_depth = max(
                        0.0,
                        float(self.C3_RADIUS_M) - (float(self.C3_TARGET_SIZE_X_M) * 0.5),
                    )
                    abs_err = abs(float(z) - expected_depth)
                    samples.append(
                        {
                            "sample_index": int(i),
                            "theta_rad": float(theta),
                            "camera_x_m": float(cam_x),
                            "camera_y_m": float(cam_y),
                            "measured_depth_m": float(z),
                            "expected_depth_m": float(expected_depth),
                            "abs_error_m": float(abs_err),
                        }
                    )

                    if i in (0, self.C3_SAMPLES // 2):
                        dbg = self._draw_debug(depth_m, bgr, point, [f"C3 camera orbit", f"sample={i}", f"depth={z:.3f}m"])
                        artifacts.append(self._save_frame(f"c3_camera_orbit_{i}.png", dbg))

                if len(samples) < max(10, int(0.6 * self.C3_SAMPLES)):
                    raise RuntimeError(f"Too few valid measurements in camera orbit mode: {len(samples)}")
            except Exception as exc:
                metrics["mode_error"] = str(exc)
                samples.clear()
                use_camera_orbit = False

        if not use_camera_orbit:
            # Эквивалентная реализация: двигаем target_cube относительно неподвижной камеры.
            metrics["mode"] = "target_cube_orbit_equivalent"
            metrics["implementation_note"] = (
                "Camera orbit is replaced with equivalent target_cube motion relative to static camera "
                "using /gazebo/set_model_state."
            )
            angles = np.linspace(-self.C3_FALLBACK_HALF_ANGLE_RAD, self.C3_FALLBACK_HALF_ANGLE_RAD, self.C3_SAMPLES)
            for i, theta in enumerate(angles):
                x = self.C3_RADIUS_M * cos(float(theta))
                y = self.C3_RADIUS_M * sin(float(theta))
                self._move_and_settle(simulator, self.C3_TARGET_CUBE_NAME, x=x, y=y, z=0.25, settle_s=0.25)

                depth_msg = self._wait_depth_after(
                    prev_stamp_s=last_depth_stamp_s,
                    timeout=3.0,
                    stage=f"c3_target_orbit_{i}",
                )
                last_depth_stamp_s = self._msg_stamp_s(depth_msg)
                depth_m = self._depth_msg_to_meters(depth_msg)
                color_msg = self._wait_color_after(
                    prev_stamp_s=last_depth_stamp_s - 1e-6,
                    timeout=2.0,
                    stage=f"c3_target_orbit_color_{i}",
                )
                bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
                z, point = self._measure_depth(depth_m, bgr, color_hint="blue")
                if z is None or np.isnan(z) or np.isinf(z):
                    continue
                expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C3_TARGET_SIZE_X_M))
                abs_err = abs(float(z) - expected_depth)
                samples.append(
                    {
                        "sample_index": int(i),
                        "theta_rad": float(theta),
                        "target_center_x_m": float(x),
                        "target_center_y_m": float(y),
                        "expected_depth_m": float(expected_depth),
                        "measured_depth_m": float(z),
                        "abs_error_m": float(abs_err),
                    }
                )

                if i in (0, self.C3_SAMPLES // 2):
                    dbg = self._draw_debug(
                        depth_m,
                        bgr,
                        point,
                        [f"C3 equivalent", f"sample={i}", f"depth={z:.3f}m", f"expected={expected_depth:.3f}m"],
                    )
                    artifacts.append(self._save_frame(f"c3_equivalent_{i}.png", dbg))

        if len(samples) < 10:
            metrics["samples"] = samples
            metrics_path = self._save_metrics_json("c3_view_angle_stability_metrics.json", metrics)
            raise RuntimeError(f"C3 failed: too few valid depth samples ({len(samples)})")

        abs_errors = [float(sample["abs_error_m"]) for sample in samples]
        measured_depths = [float(sample["measured_depth_m"]) for sample in samples]
        expected_depths = [float(sample["expected_depth_m"]) for sample in samples]

        mean_abs_error = float(np.mean(abs_errors))
        max_abs_error = float(np.max(abs_errors))
        metrics["samples"] = samples
        metrics["mean_measured_depth_m"] = float(np.mean(measured_depths))
        metrics["mean_expected_depth_m"] = float(np.mean(expected_depths))
        metrics["mean_abs_error_m"] = mean_abs_error
        metrics["max_abs_error_m"] = max_abs_error
        metrics["checks"] = {
            "mean_abs_error_ok": bool(mean_abs_error <= self.C3_MEAN_ABS_ERROR_M),
            "max_abs_error_ok": bool(max_abs_error <= self.C3_MAX_ABS_ERROR_M),
        }

        metrics_path = self._save_metrics_json("c3_view_angle_stability_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C3 failed: mean_abs_error={mean_abs_error:.4f} (limit={self.C3_MEAN_ABS_ERROR_M:.4f}), "
                f"max_abs_error={max_abs_error:.4f} (limit={self.C3_MAX_ABS_ERROR_M:.4f})"
            )

        return {"id": "C3", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c5_working_range_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        metrics: Dict[str, Any] = {
            "x_values_m": self._iter_float_range(self.C5_START_X, self.C5_END_X, self.C5_STEP),
            "tolerance_m": float(self.C5_DEPTH_TOLERANCE_M),
            "clip_margin_m": float(self.C5_CLIP_MARGIN_M),
            "target_size_x_m": float(self.C5_TARGET_SIZE_X_M),
            "samples": [],
            "first_frame_diagnostics": {},
        }

        self._open_test_scene(simulator, "c5_working_range_test")
        if not simulator.wait_for_model_spawn(self.C5_RANGE_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C5_RANGE_CUBE_NAME}")

        first_frame_diagnostics: Optional[Dict[str, Any]] = None
        last_depth_stamp_s: Optional[float] = None
        for x in metrics["x_values_m"]:
            self._move_and_settle(simulator, self.C5_RANGE_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.3)
            depth_msg = self._wait_depth_after(
                prev_stamp_s=last_depth_stamp_s,
                timeout=3.0,
                stage=f"c5_x_{str(x).replace('.', '_')}",
            )
            last_depth_stamp_s = self._msg_stamp_s(depth_msg)
            self._set_test_diagnostics(depth_topic_selected=self._resolved_depth_topic)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._wait_color_after(
                prev_stamp_s=last_depth_stamp_s - 1e-6,
                timeout=2.0,
                stage=f"c5_color_x_{str(x).replace('.', '_')}",
            )
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point, roi_meta = self._measure_depth_with_meta(depth_m, bgr, color_hint="green")

            if first_frame_diagnostics is None:
                first_frame_diagnostics = self._depth_frame_stats(depth_msg, depth_m)
                first_frame_diagnostics["measurement_pixel"] = {"x": int(point[0]), "y": int(point[1])}
                first_frame_diagnostics["measurement_roi"] = roi_meta
                self._set_test_diagnostics(c5_first_frame=first_frame_diagnostics)

            roi_stats = self._roi_depth_stats(depth_m, roi_meta.get("roi_xyxy", []))
            expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C5_TARGET_SIZE_X_M))
            expected_in_sensor_range = self._c5_expected_in_contract_range(expected_depth)

            finite_ok = bool(z is not None and np.isfinite(z))
            abs_err = float(abs(float(z) - float(expected_depth))) if finite_ok else float("inf")
            sample_ok = bool(finite_ok and expected_in_sensor_range and abs_err <= self.C5_DEPTH_TOLERANCE_M)

            metrics["samples"].append(
                {
                    "x_m": float(x),
                    "expected_front_face_depth_m": float(expected_depth),
                    "expected_in_sensor_range": bool(expected_in_sensor_range),
                    "depth_m": None if z is None else float(z),
                    "measurement_roi": roi_meta,
                    "roi_depth_stats": roi_stats,
                    "finite_ok": finite_ok,
                    "abs_error_m": abs_err if np.isfinite(abs_err) else None,
                    "ok": sample_ok,
                }
            )

            x0, y0, x1, y1 = [int(v) for v in roi_meta.get("roi_xyxy", [0, 0, 0, 0])]
            if bgr is not None:
                rgb_dbg = bgr.copy()
            else:
                rgb_dbg = self._depth_preview(depth_m)
            cv2.rectangle(rgb_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(rgb_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                rgb_dbg,
                f"x={float(x):.2f} z={float(z) if z is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            artifacts.append(self._save_frame(f"c5_rgb_x_{str(x).replace('.', '_')}.png", rgb_dbg))

            depth_dbg = self._depth_preview(depth_m)
            cv2.rectangle(depth_dbg, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.circle(depth_dbg, (int(point[0]), int(point[1])), 4, (255, 255, 255), 2)
            cv2.putText(
                depth_dbg,
                f"roi_min={roi_stats['min_m'] if roi_stats['min_m'] is not None else float('nan'):.3f} "
                f"roi_max={roi_stats['max_m'] if roi_stats['max_m'] is not None else float('nan'):.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            artifacts.append(self._save_frame(f"c5_depth_x_{str(x).replace('.', '_')}.png", depth_dbg))

            if abs(float(x) - 0.5) < 1e-9 or abs(float(x) - 5.0) < 1e-9 or abs(float(x) - 10.0) < 1e-9:
                dbg = self._draw_debug(
                    depth_m,
                    bgr,
                    point,
                    [f"C5 x={x:.2f}m", f"depth={z if z is not None else float('nan'):.3f}m", f"ok={sample_ok}"],
                )
                artifacts.append(self._save_frame(f"c5_x_{str(x).replace('.', '_')}.png", dbg))

        if first_frame_diagnostics is not None:
            metrics["first_frame_diagnostics"] = first_frame_diagnostics

        expected_ok_x_values = [
            float(x)
            for x in metrics["x_values_m"]
            if self._c5_expected_in_contract_range(
                self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C5_TARGET_SIZE_X_M))
            )
        ]
        best_start = None
        best_end = None
        cur_start = None
        cur_end = None

        for sample in metrics["samples"]:
            if sample["ok"]:
                if cur_start is None:
                    cur_start = float(sample["x_m"])
                cur_end = float(sample["x_m"])
            else:
                if cur_start is not None:
                    if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                        best_start, best_end = cur_start, cur_end
                    cur_start, cur_end = None, None

        if cur_start is not None:
            if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                best_start, best_end = cur_start, cur_end

        metrics["expected_ok_x_values_m"] = expected_ok_x_values
        metrics["expected_x_min_ok_m"] = expected_ok_x_values[0] if expected_ok_x_values else None
        metrics["expected_x_max_ok_m"] = expected_ok_x_values[-1] if expected_ok_x_values else None

        if not expected_ok_x_values:
            metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
            raise AssertionError("C5 failed: no sampled positions fall inside the sensor clip range")

        if best_start is None or best_end is None:
            metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
            raise AssertionError("C5 failed: no stable depth interval found")

        metrics["x_min_ok_m"] = float(best_start)
        metrics["x_max_ok_m"] = float(best_end)
        metrics["topic_diagnostics"] = self.get_last_test_diagnostics().get("depth_topic_resolution", {})
        metrics["selected_depth_topic"] = self._resolved_depth_topic
        metrics["selected_image_topic"] = self._resolved_image_topic
        metrics["checks"] = {
            "x_min_ok_covers_expected": bool(float(best_start) <= float(expected_ok_x_values[0]) + 1e-6),
            "x_max_ok_covers_expected": bool(float(best_end) >= float(expected_ok_x_values[-1]) - 1e-6),
        }

        metrics_path = self._save_metrics_json("c5_working_range_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C5 failed: stable interval [{best_start:.2f}, {best_end:.2f}] does not cover "
                f"[{expected_ok_x_values[0]:.2f}, {expected_ok_x_values[-1]:.2f}]"
            )

        return {"id": "C5", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

    def c6_small_displacement_sensitivity_test(self, simulator) -> Dict[str, Any]:
        artifacts: List[str] = []
        x_values = self._iter_float_range_desc(self.C6_START_X, self.C6_END_X, self.C6_STEP)
        metrics: Dict[str, Any] = {
            "x_values_m": x_values,
            "eps_m": float(self.C6_DEPTH_CHANGE_EPS_M),
            "required_ratio": float(self.C6_MIN_CHANGED_RATIO),
            "target_size_x_m": float(self.C6_TARGET_SIZE_X_M),
            "monotonic_tolerance_m": float(self.C6_MONOTONIC_TOLERANCE_M),
            "delta_tolerance_m": float(self.C6_DELTA_TOLERANCE_M),
            "abs_error_tolerance_m": float(self.C6_ABS_ERROR_TOLERANCE_M),
            "samples": [],
        }

        self._open_test_scene(simulator, "c6_small_displacement_sensitivity_test")
        if not simulator.wait_for_model_spawn(self.C6_SHIFT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C6_SHIFT_CUBE_NAME}")

        depths: List[Optional[float]] = []
        last_depth_stamp_s: Optional[float] = None
        for x in x_values:
            self._move_and_settle(simulator, self.C6_SHIFT_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=0.16)
            depth_msg = self._wait_depth_after(
                prev_stamp_s=last_depth_stamp_s,
                timeout=3.0,
                stage=f"c6_x_{str(x).replace('.', '_')}",
            )
            last_depth_stamp_s = self._msg_stamp_s(depth_msg)
            depth_m = self._depth_msg_to_meters(depth_msg)
            color_msg = self._wait_color_after(
                prev_stamp_s=last_depth_stamp_s - 1e-6,
                timeout=2.0,
                stage=f"c6_color_x_{str(x).replace('.', '_')}",
            )
            bgr = self._color_msg_to_bgr(color_msg) if color_msg is not None else None
            z, point = self._measure_depth(depth_m, bgr, color_hint="yellow")
            z_valid = bool(z is not None and np.isfinite(z))
            depths.append(float(z) if z_valid else None)
            expected_depth = self._front_face_depth(center_x_m=float(x), size_x_m=float(self.C6_TARGET_SIZE_X_M))
            abs_err = abs(float(z) - expected_depth) if z_valid else None
            metrics["samples"].append(
                {
                    "x_m": float(x),
                    "expected_front_face_depth_m": float(expected_depth),
                    "depth_m": None if not z_valid else float(z),
                    "abs_error_m": None if abs_err is None else float(abs_err),
                }
            )

            if abs(float(x) - self.C6_START_X) < 1e-9 or abs(float(x) - self.C6_END_X) < 1e-9:
                dbg = self._draw_debug(
                    depth_m,
                    bgr,
                    point,
                    [f"C6 x={x:.2f}m", f"depth={z if z is not None else float('nan'):.3f}m"],
                )
                artifacts.append(self._save_frame(f"c6_x_{str(x).replace('.', '_')}.png", dbg))

        deltas: List[float] = []
        for i in range(1, len(depths)):
            prev = depths[i - 1]
            cur = depths[i]
            if prev is None or cur is None:
                continue
            deltas.append(float(abs(cur - prev)))

        if not deltas:
            raise AssertionError("C6 failed: no valid consecutive depth pairs")

        changed = [d for d in deltas if d > self.C6_DEPTH_CHANGE_EPS_M]
        changed_ratio = float(len(changed) / len(deltas))
        median_delta = float(np.median(np.array(deltas, dtype=np.float64)))
        monotonic_violations = 0
        for i in range(1, len(depths)):
            prev = depths[i - 1]
            cur = depths[i]
            if prev is None or cur is None:
                continue
            if float(cur) > float(prev) + float(self.C6_MONOTONIC_TOLERANCE_M):
                monotonic_violations += 1

        expected_step_m = float(self.C6_STEP)
        abs_errors = [
            float(sample["abs_error_m"])
            for sample in metrics["samples"]
            if sample.get("abs_error_m") is not None
        ]
        mean_abs_error = float(np.mean(abs_errors)) if abs_errors else float("inf")

        metrics["delta_abs_m"] = [float(d) for d in deltas]
        metrics["median_delta_m"] = median_delta
        metrics["changed_pairs"] = int(len(changed))
        metrics["pairs_total"] = int(len(deltas))
        metrics["changed_ratio"] = changed_ratio
        metrics["expected_step_m"] = expected_step_m
        metrics["monotonic_violations"] = int(monotonic_violations)
        metrics["mean_abs_error_m"] = mean_abs_error
        metrics["checks"] = {
            "changed_ratio_ge_0_8": bool(changed_ratio >= self.C6_MIN_CHANGED_RATIO),
            "monotonic_nonincreasing": bool(monotonic_violations == 0),
            "median_delta_matches_step": bool(abs(median_delta - expected_step_m) <= self.C6_DELTA_TOLERANCE_M),
            "mean_abs_error_ok": bool(mean_abs_error <= self.C6_ABS_ERROR_TOLERANCE_M),
        }

        metrics_path = self._save_metrics_json("c6_small_displacement_sensitivity_metrics.json", metrics)
        if not all(metrics["checks"].values()):
            raise AssertionError(
                f"C6 failed: changed_ratio={changed_ratio:.3f}, monotonic_violations={monotonic_violations}, "
                f"median_delta={median_delta:.4f}, expected_step={expected_step_m:.4f}, mean_abs_error={mean_abs_error:.4f}"
            )

        return {"id": "C6", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

class _StereoProfileTestContext:
    LEFT_IMAGE_TOPIC = ""
    RIGHT_IMAGE_TOPIC = ""

    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.92
    CLIP_NEAR = 0.3
    CLIP_FAR = 20.0
    BASELINE_M = 0.12

    C1_CUBE_NAME = "test_cube"
    C7_FRONT_CUBE_NAME = "front_cube"
    C7_BACK_CUBE_NAME = "back_cube"

    C4_MIN_PIXELS = 1500
    # Смещения подобраны так, чтобы occ_25 имел меньшую окклюзию (больше blue-пикселей),
    # а occ_50 — большую окклюзию (меньше blue-пикселей).
    C7_CASES = {"occ_25": 0.20, "occ_50": 0.10}
    C7_MIN_PIXELS = 800
    MIN_DISPARITY_PX = 2
    C8_OBJECTS = ("obj_near_cube", "obj_near_sphere", "obj_far_cube", "obj_far_sphere", "wall_left", "wall_right")
    S1_MAX_REL_ERROR = 0.10
    S1_MIN_PASS_OBJECTS = 3
    S2_MIN_VALID_GAIN = 0.05
    S2_CROP_BLOCK_SIZE = 21
    S2_CROP_TEXTURE_THRESHOLD = 10
    S2_CROP_UNIQUENESS_RATIO = 10
    S2_ROI_Y0_RATIO = 0.26
    S2_ROI_Y1_RATIO = 0.58
    S2_ROI_X0_RATIO = 0.24
    S2_ROI_X1_RATIO = 0.76
    # Первичный прогрев топиков/кадров для stereo делаем длиннее,
    # иначе на "холодном" запуске часто прилетают таймауты.
    PAIR_TIMEOUT_S = 25.0
    PAIR_RETRIES = 3
    PAIR_QUEUE_SIZE = 20
    PAIR_SLOP_S = 0.08
    PAIR_MAX_SKEW_S = 0.08
    TOPIC_WARMUP_TIMEOUT_S = 25.0

    def __init__(self, sensor):
        self.sensor = sensor
        self.sensor_name = str(getattr(sensor, "sensor_name", ""))
        self.sensor_type = str(getattr(sensor, "sensor_type", ""))
        self.sensor_sdf_path = str(getattr(sensor, "sdf_path", ""))
        self.CONFIG = {"ROOT_PATH": str(CONFIG["ROOT_PATH"])}

        worlds_root = _camera_worlds_root()
        profile = _camera_load_sensor_profile(self.sensor_sdf_path) if self.sensor_sdf_path else {}

        self.test_to_world = {
            "stereo_topics_presence_test": str(worlds_root / "camera_c4_geometries.world"),
            "stereo_disparity_test": str(worlds_root / "camera_c1_single_cube.world"),
            "stereo_occlusion_test": str(worlds_root / "camera_c7_occlusion.world"),
            "s1_stereo_accuracy_test": str(worlds_root / "camera_c8_stereo_complex.world"),
            "s2_texture_vs_smooth_stability_test": str(worlds_root / "camera_c8_stereo_complex.world"),
        }

        sensor_topics = list(getattr(sensor, "topics", []) or [])
        self.LEFT_IMAGE_TOPIC = str(profile.get("left_topic", "") or (sensor_topics[0] if len(sensor_topics) > 0 else self.LEFT_IMAGE_TOPIC))
        self.RIGHT_IMAGE_TOPIC = str(profile.get("right_topic", "") or (sensor_topics[1] if len(sensor_topics) > 1 else self.RIGHT_IMAGE_TOPIC))
        self.image_width = int(profile.get("image_width") or self.IMAGE_WIDTH)
        self.image_height = int(profile.get("image_height") or self.IMAGE_HEIGHT)
        self.horizontal_fov = float(profile.get("horizontal_fov") or self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(profile.get("clip_near") or self.CLIP_NEAR)
        self.clip_far = float(profile.get("clip_far") or self.CLIP_FAR)
        self.update_rate = int(profile.get("update_rate") or self.UPDATE_RATE)
        self.baseline = float(profile.get("baseline") or self.BASELINE_M)
        self._last_test_diagnostics: Dict[str, Any] = {}
        self._last_scene_diag: Dict[str, Any] = {}
        self._resolved_left_topic = str(self.LEFT_IMAGE_TOPIC)
        self._resolved_right_topic = str(self.RIGHT_IMAGE_TOPIC)

    def _results_dir(self) -> str:
        path = os.path.join(self.CONFIG["ROOT_PATH"], "results", self.sensor_name)
        os.makedirs(path, exist_ok=True)
        return path

    def _captured_dir(self) -> str:
        path = os.path.join(self._results_dir(), "captured_images")
        os.makedirs(path, exist_ok=True)
        return path

    def _save_frame(self, name: str, frame: np.ndarray) -> str:
        out = os.path.join(self._captured_dir(), name)
        cv2.imwrite(out, frame)
        return out

    def _set_test_diagnostics(self, **kwargs) -> None:
        self._last_test_diagnostics.update(kwargs)

    def get_last_test_diagnostics(self) -> Dict[str, Any]:
        return copy.deepcopy(self._last_test_diagnostics)

    @staticmethod
    def _scene_diag(simulator) -> Dict[str, Any]:
        return {}

    @staticmethod
    def _ensure_render_display_env() -> Dict[str, str]:
        applied: Dict[str, str] = {}
        if os.environ.get("DISPLAY"):
            return applied

        xauthority = os.environ.get("XAUTHORITY", "").strip()
        if not xauthority:
            default_xauthority = os.path.expanduser("~/.Xauthority")
            if os.path.exists(default_xauthority):
                os.environ["XAUTHORITY"] = default_xauthority
                applied["XAUTHORITY"] = default_xauthority

        for display in (":0",):
            display_socket = f"/tmp/.X11-unix/X{display.lstrip(':')}"
            if os.path.exists(display_socket):
                os.environ["DISPLAY"] = display
                applied["DISPLAY"] = display
                break

        return applied

    def _reset_resolved_stereo_topics(self) -> None:
        self._last_scene_diag = {}
        self._resolved_left_topic = str(self.LEFT_IMAGE_TOPIC)
        self._resolved_right_topic = str(self.RIGHT_IMAGE_TOPIC)

    def _update_resolved_stereo_topics(self, simulator) -> Dict[str, Any]:
        scene_diag = self._scene_diag(simulator)
        self._last_scene_diag = copy.deepcopy(scene_diag) if isinstance(scene_diag, dict) else {}
        self._resolved_left_topic = str(self.LEFT_IMAGE_TOPIC)
        self._resolved_right_topic = str(self.RIGHT_IMAGE_TOPIC)
        return scene_diag

    def _open_test_scene(self, simulator, test_name: str) -> None:
        self._last_test_diagnostics = {}
        self._reset_resolved_stereo_topics()
        display_env = self._ensure_render_display_env()
        if display_env:
            self._set_test_diagnostics(stereo_render_env={"display_env": dict(display_env)})
        world = self.test_to_world[test_name]
        if not simulator.open_scene(world, self.sensor_sdf_path):
            diag = self._scene_diag(simulator)
            self._last_scene_diag = copy.deepcopy(diag) if isinstance(diag, dict) else {}
            reason = diag.get("reason", "unknown") if isinstance(diag, dict) else "unknown"
            raise RuntimeError(f"Failed to open scene for {test_name}: {world} (reason={reason})")

        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
        scene_diag = self._update_resolved_stereo_topics(simulator)
        self._set_test_diagnostics(
            stereo_scene={
                "display_env": dict(display_env),
                "scene_reason": str(scene_diag.get("reason", "")) if scene_diag else "",
                "expected_left": str(self.LEFT_IMAGE_TOPIC),
                "expected_right": str(self.RIGHT_IMAGE_TOPIC),
                "resolved_left": str(self._resolved_left_topic),
                "resolved_right": str(self._resolved_right_topic),
                "topic_mapping_changed": bool(
                    str(self._resolved_left_topic) != str(self.LEFT_IMAGE_TOPIC)
                    or str(self._resolved_right_topic) != str(self.RIGHT_IMAGE_TOPIC)
                ),
            }
        )

    @staticmethod
    def _msg_to_bgr(msg: Image) -> np.ndarray:
        h, w = msg.height, msg.width
        enc = (msg.encoding or "").lower()

        if enc in ("rgb8", "r8g8b8"):
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if enc == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

        if enc in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def _wait_pair(self, timeout: float = 35.0) -> Tuple[Image, Image]:
        left, right, _ = self._wait_pair_closest(timeout=timeout, retries=1, max_skew_s=float(self.PAIR_MAX_SKEW_S))
        return left, right

    @staticmethod
    def _msg_stamp(msg: Image) -> float:
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            return float(time.time())
        return stamp

    @staticmethod
    def _pair_stamp(left_msg: Image, right_msg: Image) -> float:
        return float(max(_StereoProfileTestContext._msg_stamp(left_msg), _StereoProfileTestContext._msg_stamp(right_msg)))

    @staticmethod
    def _list_image_topics() -> List[str]:
        try:
            published = rospy.get_published_topics()
        except Exception:
            return []
        return sorted([name for name, msg_type in published if msg_type == "sensor_msgs/Image"])

    @staticmethod
    def _looks_like_side_topic(topic: str, side: str) -> bool:
        normalized = str(topic or "").strip()
        if not normalized:
            return False
        if side == "left":
            return ("/left/" in normalized) or ("_left/" in normalized) or normalized.endswith("_left/image_raw")
        if side == "right":
            return ("/right/" in normalized) or ("_right/" in normalized) or normalized.endswith("_right/image_raw")
        return False

    @classmethod
    def _is_valid_stereo_pair(cls, left_topic: str, right_topic: str) -> bool:
        left = str(left_topic or "").strip()
        right = str(right_topic or "").strip()
        if not left or not right or left == right:
            return False
        return bool(cls._looks_like_side_topic(left, "left") and cls._looks_like_side_topic(right, "right"))

    def _resolve_stereo_topics(self, warmup_timeout: float) -> Tuple[str, str, Dict[str, Any]]:
        expected_left = str(self.LEFT_IMAGE_TOPIC)
        expected_right = str(self.RIGHT_IMAGE_TOPIC)
        cached_left = str(self._resolved_left_topic or "").strip()
        cached_right = str(self._resolved_right_topic or "").strip()
        sensor_name = str(self.sensor_name)

        preferred_pairs: List[Tuple[str, str, str]] = []
        if self._is_valid_stereo_pair(cached_left, cached_right):
            preferred_pairs.append((cached_left, cached_right, "scene_resolved"))
        if expected_left and expected_right and (expected_left, expected_right) != (cached_left, cached_right):
            preferred_pairs.append((expected_left, expected_right, "expected"))
        preferred_pairs.extend(
            [
                (f"/{sensor_name}_left/image_raw", f"/{sensor_name}_right/image_raw", "name_underscore"),
                (f"/{sensor_name}/left/image_raw", f"/{sensor_name}/right/image_raw", "name_namespace"),
            ]
        )

        deadline = time.time() + float(warmup_timeout)
        last_topics: List[str] = []
        while time.time() < deadline:
            topics = self._list_image_topics()
            last_topics = topics
            for left_topic, right_topic, source in preferred_pairs:
                if self._is_valid_stereo_pair(left_topic, right_topic) and left_topic in topics and right_topic in topics:
                    return left_topic, right_topic, {
                        "expected_left": expected_left,
                        "expected_right": expected_right,
                        "selected_left": left_topic,
                        "selected_right": right_topic,
                        "selected_source": source,
                        "topics_found": topics,
                        "topic_mapping_changed": bool(left_topic != expected_left or right_topic != expected_right),
                    }
            time.sleep(0.2)

        if self._is_valid_stereo_pair(expected_left, expected_right) and expected_left in last_topics and expected_right in last_topics:
            return expected_left, expected_right, {
                "expected_left": expected_left,
                "expected_right": expected_right,
                "selected_left": expected_left,
                "selected_right": expected_right,
                "selected_source": "expected_after_warmup",
                "topics_found": last_topics,
                "topic_mapping_changed": False,
            }

        for left_topic, right_topic, source in preferred_pairs[1:]:
            if self._is_valid_stereo_pair(left_topic, right_topic) and left_topic in last_topics and right_topic in last_topics:
                return left_topic, right_topic, {
                    "expected_left": expected_left,
                    "expected_right": expected_right,
                    "selected_left": left_topic,
                    "selected_right": right_topic,
                    "selected_source": f"{source}_after_warmup",
                    "topics_found": last_topics,
                    "topic_mapping_changed": True,
                }
        safe_candidates = [
            {
                "left": left_topic,
                "right": right_topic,
                "source": source,
                "published": bool(left_topic in last_topics and right_topic in last_topics),
            }
            for left_topic, right_topic, source in preferred_pairs
            if self._is_valid_stereo_pair(left_topic, right_topic)
        ]
        published_left_candidates = [topic for topic in last_topics if self._looks_like_side_topic(topic, "left")]
        published_right_candidates = [topic for topic in last_topics if self._looks_like_side_topic(topic, "right")]
        sensor_namespace_left = [topic for topic in published_left_candidates if sensor_name and sensor_name in topic]
        sensor_namespace_right = [topic for topic in published_right_candidates if sensor_name and sensor_name in topic]

        return "", "", {
            "expected_left": expected_left,
            "expected_right": expected_right,
            "selected_left": "",
            "selected_right": "",
            "selected_source": "unresolved_no_safe_pair",
            "topics_found": last_topics,
            "topic_mapping_changed": False,
            "scene_resolved_left": cached_left,
            "scene_resolved_right": cached_right,
            "safe_candidates": safe_candidates,
            "published_left_candidates": published_left_candidates,
            "published_right_candidates": published_right_candidates,
            "sensor_namespace_left_candidates": sensor_namespace_left,
            "sensor_namespace_right_candidates": sensor_namespace_right,
            "resolve_reason": "no_safe_stereo_pair_after_warmup",
        }

    def _wait_pair_closest(
        self,
        timeout: float = 25.0,
        retries: int = 3,
        max_skew_s: float = 0.08,
        min_pair_stamp_s: Optional[float] = None,
    ) -> Tuple[Image, Image, float]:
        try:
            import message_filters
        except Exception as exc:  # noqa: BLE001
            self._set_test_diagnostics(stereo_pair_capture={"reason": "message_filters_import_error", "error": str(exc)})
            raise RuntimeError(f"message_filters import failed: {exc}")

        left_topic, right_topic, topic_diag = self._resolve_stereo_topics(self.TOPIC_WARMUP_TIMEOUT_S)
        pair_diag: Dict[str, Any] = dict(topic_diag)
        pair_diag.update(
            {
                "timeout_s": float(timeout),
                "retries": int(retries),
                "max_skew_s": float(max_skew_s),
                "min_pair_stamp_s": None if min_pair_stamp_s is None else float(min_pair_stamp_s),
                "queue_size": int(self.PAIR_QUEUE_SIZE),
                "slop_s": float(self.PAIR_SLOP_S),
                "attempts": [],
            }
        )

        if not left_topic or not right_topic:
            pair_diag["reason"] = "stereo_topics_unresolved"
            self._set_test_diagnostics(stereo_pair_capture=pair_diag)
            raise RuntimeError("Stereo topics could not be resolved")
        if left_topic == right_topic:
            pair_diag["reason"] = "stereo_topics_collapsed"
            self._set_test_diagnostics(stereo_pair_capture=pair_diag)
            raise RuntimeError(f"Stereo topics collapsed to one topic: {left_topic}")

        for attempt in range(1, int(retries) + 1):
            attempt_diag: Dict[str, Any] = {"attempt": int(attempt), "left_msgs": 0, "right_msgs": 0}
            lock = threading.Lock()
            pair_holder: Dict[str, Any] = {}

            def _left_count(_msg: Image) -> None:
                attempt_diag["left_msgs"] += 1

            def _right_count(_msg: Image) -> None:
                attempt_diag["right_msgs"] += 1

            def _pair_cb(left_msg: Image, right_msg: Image) -> None:
                with lock:
                    if pair_holder:
                        return
                    pair_stamp = self._pair_stamp(left_msg, right_msg)
                    if min_pair_stamp_s is not None and pair_stamp <= float(min_pair_stamp_s) + 1e-6:
                        attempt_diag["pairs_rejected_before_min_stamp"] = int(
                            attempt_diag.get("pairs_rejected_before_min_stamp", 0)
                        ) + 1
                        return
                    skew = abs(self._msg_stamp(left_msg) - self._msg_stamp(right_msg))
                    pair_holder["left"] = left_msg
                    pair_holder["right"] = right_msg
                    pair_holder["skew"] = float(skew)
                    pair_holder["pair_stamp"] = float(pair_stamp)

            left_counter_sub = rospy.Subscriber(left_topic, Image, _left_count, queue_size=200)
            right_counter_sub = rospy.Subscriber(right_topic, Image, _right_count, queue_size=200)

            left_mf = message_filters.Subscriber(left_topic, Image)
            right_mf = message_filters.Subscriber(right_topic, Image)
            sync = message_filters.ApproximateTimeSynchronizer(
                [left_mf, right_mf],
                queue_size=int(self.PAIR_QUEUE_SIZE),
                slop=float(self.PAIR_SLOP_S),
                allow_headerless=False,
            )
            sync.registerCallback(_pair_cb)

            started = time.time()
            try:
                while (time.time() - started) < float(timeout):
                    with lock:
                        if pair_holder:
                            break
                    time.sleep(0.02)
            finally:
                left_counter_sub.unregister()
                right_counter_sub.unregister()
                try:
                    left_mf.sub.unregister()
                    right_mf.sub.unregister()
                except Exception:
                    pass

            with lock:
                has_pair = bool(pair_holder)
                if has_pair:
                    skew = float(pair_holder["skew"])
                    pair_stamp = float(pair_holder["pair_stamp"])
                    attempt_diag["pair_received"] = True
                    attempt_diag["pair_skew_s"] = skew
                    attempt_diag["pair_stamp_s"] = pair_stamp
                    pair_diag["attempts"].append(attempt_diag)
                    self._set_test_diagnostics(stereo_pair_capture=pair_diag)
                    if skew <= float(max_skew_s):
                        return pair_holder["left"], pair_holder["right"], skew
                    attempt_diag["pair_rejected"] = True
                    attempt_diag["pair_reject_reason"] = "skew_above_threshold"
                else:
                    attempt_diag["pair_received"] = False

            pair_diag["attempts"].append(attempt_diag)

        pair_diag["reason"] = "pair_timeout"
        self._set_test_diagnostics(stereo_pair_capture=pair_diag)
        raise RuntimeError("Failed to capture left/right pair with timestamp proximity")

    @staticmethod
    def _fx_from_fov(width_px: int, horizontal_fov_rad: float) -> float:
        if width_px <= 0 or horizontal_fov_rad <= 0.0:
            raise RuntimeError(f"Invalid camera intrinsics for fx estimation: width={width_px}, fov={horizontal_fov_rad}")
        return float((width_px / 2.0) / tan(horizontal_fov_rad / 2.0))

    def _compute_disparity_and_depth(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        if left_bgr.shape[:2] != right_bgr.shape[:2]:
            raise RuntimeError(f"Stereo size mismatch: left={left_bgr.shape[:2]}, right={right_bgr.shape[:2]}")

        if self.baseline <= 0.0:
            raise RuntimeError(f"Invalid baseline: {self.baseline}")

        gray_left = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray_left.shape[:2]
        fx = self._fx_from_fov(w, float(self.horizontal_fov))

        num_disp = max(16, min(256, ((w // 4) // 16) * 16))
        if num_disp < 16:
            num_disp = 16
        block_size = 7

        matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=int(num_disp),
            blockSize=int(block_size),
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            preFilterCap=31,
            uniquenessRatio=8,
            speckleWindowSize=50,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = disparity > 0.0
        depth[valid] = float(fx * float(self.baseline)) / disparity[valid]
        return disparity, depth, float(fx)

    @staticmethod
    def _disparity_to_viz(disparity: np.ndarray) -> np.ndarray:
        valid = np.isfinite(disparity) & (disparity > 0.0)
        h, w = disparity.shape[:2]
        norm_u8 = np.zeros((h, w), dtype=np.uint8)

        if np.any(valid):
            vals = disparity[valid]
            lo = float(np.percentile(vals, 5))
            hi = float(np.percentile(vals, 95))
            if hi <= lo:
                hi = lo + 1e-3
            scaled = np.clip((disparity - lo) / (hi - lo), 0.0, 1.0)
            norm_u8[valid] = (scaled[valid] * 255.0).astype(np.uint8)

        return cv2.applyColorMap(norm_u8, cv2.COLORMAP_TURBO)

    @staticmethod
    def _expand_bbox(
        bbox: Tuple[int, int, int, int],
        width: int,
        height: int,
        pad: int = 6,
    ) -> Tuple[int, int, int, int]:
        x, y, bw, bh = bbox
        x0 = max(0, int(x) - int(pad))
        y0 = max(0, int(y) - int(pad))
        x1 = min(int(width), int(x + bw + pad))
        y1 = min(int(height), int(y + bh + pad))
        return int(x0), int(y0), int(max(1, x1 - x0)), int(max(1, y1 - y0))

    @staticmethod
    def _median_depth_in_bbox(
        depth_map: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[Optional[float], int]:
        x, y, w, h = bbox
        roi = depth_map[y:y + h, x:x + w]
        with np.errstate(invalid="ignore"):
            finite = np.isfinite(roi)
        valid = roi[finite]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None, 0
        return float(np.median(valid)), int(valid.size)

    @staticmethod
    def _valid_ratio(
        disparity: np.ndarray,
        mask: np.ndarray,
    ) -> float:
        area = int(np.count_nonzero(mask))
        if area <= 0:
            return 0.0
        valid = mask & np.isfinite(disparity) & (disparity > 0.0)
        return float(np.count_nonzero(valid) / area)

    @staticmethod
    def _crop_valid_ratio_bm(
        gray_left: np.ndarray,
        gray_right: np.ndarray,
        block_size: int,
        texture_threshold: int,
        uniqueness_ratio: int,
    ) -> float:
        crop_h, crop_w = gray_left.shape[:2]
        if crop_h <= 0 or crop_w <= 0:
            return 0.0

        num_disp = max(16, min(256, ((crop_w // 4) // 16) * 16))
        if num_disp < 16:
            num_disp = 16

        matcher = cv2.StereoBM_create(numDisparities=int(num_disp), blockSize=int(block_size))
        matcher.setTextureThreshold(int(texture_threshold))
        matcher.setUniquenessRatio(int(uniqueness_ratio))
        matcher.setSpeckleWindowSize(50)
        matcher.setSpeckleRange(2)
        matcher.setDisp12MaxDiff(1)

        disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
        valid = np.isfinite(disparity) & (disparity > 0.0)
        return float(np.count_nonzero(valid) / max(1, valid.size))

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        return closed

    def _red_mask(self, hsv: np.ndarray) -> np.ndarray:
        lower_1 = np.array([0, 90, 60], dtype=np.uint8)
        upper_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_2 = np.array([170, 90, 60], dtype=np.uint8)
        upper_2 = np.array([180, 255, 255], dtype=np.uint8)
        mask_1 = cv2.inRange(hsv, lower_1, upper_1)
        mask_2 = cv2.inRange(hsv, lower_2, upper_2)
        return self._clean_mask(cv2.bitwise_or(mask_1, mask_2))

    def _color_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        if color == "red":
            return self._red_mask(hsv)

        if color == "blue":
            # Для stereo C7 допускаем более широкий диапазон по S/V,
            # т.к. при headless-рендере синий объект часто темнее ожидаемого.
            blue_main = cv2.inRange(
                hsv,
                np.array([85, 40, 25], dtype=np.uint8),
                np.array([145, 255, 255], dtype=np.uint8),
            )
            blue_dark = cv2.inRange(
                hsv,
                np.array([80, 20, 10], dtype=np.uint8),
                np.array([150, 180, 170], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(blue_main, blue_dark)
            cleaned = self._clean_mask(mask)
            kernel = np.ones((3, 3), np.uint8)
            return cv2.dilate(cleaned, kernel, iterations=1)

        ranges: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
            "green": ((40, 80, 60), (85, 255, 255)),
            "yellow": ((20, 90, 90), (40, 255, 255)),
        }
        if color not in ranges:
            raise ValueError(f"Unsupported color: {color}")

        lower, upper = ranges[color]
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        return self._clean_mask(mask)

    @staticmethod
    def _bbox(mask: np.ndarray) -> Tuple[int, Tuple[int, int, int, int]]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, (0, 0, 0, 0)

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        return int(w * h), (int(x), int(y), int(w), int(h))

    @staticmethod
    def _count_pixels(mask: np.ndarray) -> int:
        return int(cv2.countNonZero(mask))

    @staticmethod
    def _move_and_settle(simulator, model_name: str, x: float, y: float, z: float, settle_s: float = 0.8) -> None:
        simulator.set_pose(model_name, x=x, y=y, z=z)
        time.sleep(settle_s)

    @staticmethod
    def _set_model_pose_and_readback(model_name: str, x: float, y: float, z: float, settle_s: float = 0.5) -> Dict[str, Any]:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"
        state.pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0.0, 0.0, 0.0, 1.0))

        set_resp = set_state(state)
        time.sleep(float(settle_s))

        pose_diag: Dict[str, Any] = {
            "request_pose": {"x": float(x), "y": float(y), "z": float(z)},
            "set_model_state": {
                "success": bool(set_resp.success),
                "status_message": str(set_resp.status_message),
            },
        }

        try:
            get_resp = get_state(model_name, "world")
            pose_diag["get_model_state"] = {
                "success": bool(get_resp.success),
                "status_message": str(get_resp.status_message),
                "pose": {
                    "x": float(get_resp.pose.position.x),
                    "y": float(get_resp.pose.position.y),
                    "z": float(get_resp.pose.position.z),
                },
            }
        except Exception as exc:  # noqa: BLE001
            pose_diag["get_model_state_error"] = str(exc)
        return pose_diag

    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        timeout: float = 1.0,
        convert2cv: bool = False,
    ) -> Optional[Dict[str, Any]]:
        if world_path:
            self._reset_resolved_stereo_topics()
            display_env = self._ensure_render_display_env()
            if display_env:
                self._set_test_diagnostics(stereo_render_env={"display_env": dict(display_env)})
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
            self._update_resolved_stereo_topics(simulator)

        left_msg, right_msg = self._wait_pair(timeout=timeout)
        result: Dict[str, Any] = {
            "raw_left": left_msg,
            "raw_right": right_msg,
        }
        if convert2cv:
            result["left_cv"] = self._msg_to_bgr(left_msg)
            result["right_cv"] = self._msg_to_bgr(right_msg)
        return result

    def set_params(self, **params) -> None:
        if "baseline" in params:
            baseline = float(params["baseline"])
            if baseline <= 0:
                raise ValueError("baseline must be > 0")
            self.baseline = baseline

    def get_params(self) -> Dict[str, Any]:
        return {
            "image_width": self.image_width,
            "image_height": self.image_height,
            "horizontal_fov": self.horizontal_fov,
            "clip_near": self.clip_near,
            "clip_far": self.clip_far,
            "baseline": self.baseline,
        }

    def stereo_topics_presence_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_topics_presence_test")

        data = self.capture_data(simulator, world_path=None, timeout=35.0, convert2cv=True)
        if data is None:
            raise RuntimeError("No stereo frames")

        left = data["left_cv"]
        right = data["right_cv"]

        if left.shape[:2] != (self.image_height, self.image_width):
            raise AssertionError(f"Left frame shape mismatch: {left.shape[:2]} != {(self.image_height, self.image_width)}")
        if right.shape[:2] != (self.image_height, self.image_width):
            raise AssertionError(f"Right frame shape mismatch: {right.shape[:2]} != {(self.image_height, self.image_width)}")

        metrics: Dict[str, Any] = {"left": {}, "right": {}, "threshold": int(self.C4_MIN_PIXELS)}
        pair_diag = self.get_last_test_diagnostics().get("stereo_pair_capture", {})
        if pair_diag:
            metrics["topic_diagnostics"] = pair_diag

        for side, frame in (("left", left), ("right", right)):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            counts = {color: self._count_pixels(self._color_mask(hsv, color)) for color in ("red", "green", "blue", "yellow")}
            metrics[side] = counts
            self._save_frame(f"stereo_topics_{side}.png", frame)

        missing: Dict[str, Dict[str, int]] = {}
        for side in ("left", "right"):
            low = {c: n for c, n in metrics[side].items() if int(n) <= int(self.C4_MIN_PIXELS)}
            if low:
                missing[side] = low

        if missing:
            raise AssertionError(f"Stereo C4 presence failed: {missing}")

        return {"id": "STEREO_TOPICS", "metrics": metrics}

    def stereo_disparity_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_disparity_test")

        if not simulator.wait_for_model_spawn(self.C1_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C1_CUBE_NAME}")

        warm_left_msg, warm_right_msg, _ = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        prev_pair_stamp = self._pair_stamp(warm_left_msg, warm_right_msg)
        self._move_and_settle(simulator, self.C1_CUBE_NAME, x=3.0, y=0.0, z=0.25)
        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
            min_pair_stamp_s=float(prev_pair_stamp),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)

        left_hsv = cv2.cvtColor(left, cv2.COLOR_BGR2HSV)
        right_hsv = cv2.cvtColor(right, cv2.COLOR_BGR2HSV)

        l_area, (lx, ly, lw, lh) = self._bbox(self._red_mask(left_hsv))
        r_area, (rx, ry, rw, rh) = self._bbox(self._red_mask(right_hsv))

        if l_area == 0 or r_area == 0:
            raise AssertionError("Failed to detect red cube in stereo frames")

        l_center_x = lx + lw / 2.0
        r_center_x = rx + rw / 2.0
        disparity_px_signed = float(l_center_x - r_center_x)
        disparity_px = float(abs(disparity_px_signed))

        left_dbg = left.copy()
        right_dbg = right.copy()
        cv2.rectangle(left_dbg, (lx, ly), (lx + lw, ly + lh), (255, 255, 255), 2)
        cv2.rectangle(right_dbg, (rx, ry), (rx + rw, ry + rh), (255, 255, 255), 2)
        self._save_frame("stereo_disparity_left.png", left_dbg)
        self._save_frame("stereo_disparity_right.png", right_dbg)

        metrics = {
            "left_center_x": float(l_center_x),
            "right_center_x": float(r_center_x),
            "disparity_px": float(disparity_px),
            "disparity_px_signed": float(disparity_px_signed),
            "min_disparity_px": float(self.MIN_DISPARITY_PX),
            "pair_skew_s": float(skew),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
        }
        self._set_test_diagnostics(stereo_disparity={"metrics": dict(metrics)})

        if disparity_px_signed < float(self.MIN_DISPARITY_PX):
            raise AssertionError(
                f"Signed disparity too small or inverted: {disparity_px_signed} px < {self.MIN_DISPARITY_PX}"
            )

        return {
            "id": "STEREO_DISPARITY",
            "metrics": metrics,
        }

    def stereo_occlusion_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "stereo_occlusion_test")

        if not simulator.wait_for_model_spawn(self.C7_FRONT_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_FRONT_CUBE_NAME}")
        if not simulator.wait_for_model_spawn(self.C7_BACK_CUBE_NAME, timeout=20):
            raise RuntimeError(f"Model not spawned: {self.C7_BACK_CUBE_NAME}")

        back_x = 3.6
        front_x = 3.0
        baseline_front_y = 0.40

        # Базово делаем синий объект (back cube) видимым в центре,
        # затем двигаем передний окклюдер по Y.
        move_back = self._set_model_pose_and_readback(self.C7_BACK_CUBE_NAME, x=back_x, y=0.0, z=0.25, settle_s=0.45)
        if not move_back.get("set_model_state", {}).get("success", False):
            raise RuntimeError(f"Failed to position back cube: {move_back}")

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
        move_before = self._set_model_pose_and_readback(
            self.C7_FRONT_CUBE_NAME,
            x=front_x,
            y=baseline_front_y,
            z=0.25,
            settle_s=0.45,
        )
        if not move_before.get("set_model_state", {}).get("success", False):
            raise RuntimeError(f"Failed to position front cube before occlusion cases: {move_before}")

        metrics: Dict[str, Any] = {
            "left": {"blue_pixels": {}},
            "right": {"blue_pixels": {}},
            "cases": dict(self.C7_CASES),
            "threshold": int(self.C7_MIN_PIXELS),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
            "occluder_model": str(self.C7_FRONT_CUBE_NAME),
            "occluded_model": str(self.C7_BACK_CUBE_NAME),
            "occluder_motion": {"before": move_before, "cases": {}, "back_cube": move_back},
            "pair_skew_s": {"before": None, "cases": {}},
        }

        # Базовый кадр до окклюзии
        base_left_msg, base_right_msg, base_skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        prev_pair_stamp = self._pair_stamp(base_left_msg, base_right_msg)
        metrics["pair_skew_s"]["before"] = float(base_skew)
        for side, frame in (("left", self._msg_to_bgr(base_left_msg)), ("right", self._msg_to_bgr(base_right_msg))):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blue = self._color_mask(hsv, "blue")
            blue_count = self._count_pixels(blue)
            metrics[side]["blue_pixels"]["before"] = int(blue_count)

            contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            overlay = frame.copy()
            if contours:
                contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(overlay, f"before: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            self._save_frame(f"stereo_c7_{side}_before_raw.png", frame)
            self._save_frame(f"stereo_c7_{side}_before_blue_mask.png", blue)
            self._save_frame(f"stereo_c7_{side}_before_overlay.png", overlay)

        for case_name, y in self.C7_CASES.items():
            move_diag = self._set_model_pose_and_readback(
                self.C7_FRONT_CUBE_NAME,
                x=front_x,
                y=float(y),
                z=0.25,
                settle_s=0.45,
            )
            metrics["occluder_motion"]["cases"][case_name] = move_diag
            if not move_diag.get("set_model_state", {}).get("success", False):
                raise RuntimeError(f"set_model_state failed for {case_name}: {move_diag}")

            left_msg, right_msg, skew = self._wait_pair_closest(
                timeout=float(self.PAIR_TIMEOUT_S),
                retries=int(self.PAIR_RETRIES),
                max_skew_s=float(self.PAIR_MAX_SKEW_S),
                min_pair_stamp_s=float(prev_pair_stamp),
            )
            prev_pair_stamp = self._pair_stamp(left_msg, right_msg)
            metrics["pair_skew_s"]["cases"][case_name] = float(skew)

            for side, frame in (("left", self._msg_to_bgr(left_msg)), ("right", self._msg_to_bgr(right_msg))):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                blue = self._color_mask(hsv, "blue")
                blue_count = self._count_pixels(blue)
                metrics[side]["blue_pixels"][case_name] = int(blue_count)

                contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                dbg = frame.copy()
                if contours:
                    contour = max(contours, key=cv2.contourArea)
                    x, yb, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(dbg, (x, yb), (x + w, yb + h), (255, 255, 255), 2)
                cv2.putText(dbg, f"{case_name}: blue={blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                self._save_frame(f"stereo_c7_{side}_{case_name}_raw.png", frame)
                self._save_frame(f"stereo_c7_{side}_{case_name}_blue_mask.png", blue)
                self._save_frame(f"stereo_c7_{side}_{case_name}_overlay.png", dbg)

        for side in ("left", "right"):
            blue_25 = metrics[side]["blue_pixels"].get("occ_25", 0)
            blue_50 = metrics[side]["blue_pixels"].get("occ_50", 0)

            relation_ok = blue_25 > blue_50
            threshold_ok = blue_25 > self.C7_MIN_PIXELS and blue_50 > self.C7_MIN_PIXELS
            metrics[side]["checks"] = {
                "occlusion_relation": bool(relation_ok),
                "threshold_ok": bool(threshold_ok),
            }

            if not (relation_ok and threshold_ok):
                self._set_test_diagnostics(stereo_occlusion={"metrics": copy.deepcopy(metrics)})
                raise AssertionError(
                    f"Stereo C7 failed on {side}: blue_25={blue_25}, blue_50={blue_50}, threshold={self.C7_MIN_PIXELS}"
                )

        self._set_test_diagnostics(stereo_occlusion={"metrics": copy.deepcopy(metrics)})
        return {"id": "STEREO_C7", "metrics": metrics}

    def s1_stereo_accuracy_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "s1_stereo_accuracy_test")
        for model_name in self.C8_OBJECTS:
            if not simulator.wait_for_model_spawn(model_name, timeout=20):
                raise RuntimeError(f"Model not spawned: {model_name}")

        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)
        disparity, depth_map, fx = self._compute_disparity_and_depth(left, right)

        h, w = left.shape[:2]
        left_hsv = cv2.cvtColor(left, cv2.COLOR_BGR2HSV)
        disparity_viz = self._disparity_to_viz(disparity)

        objects = {
            "obj_near_cube": {"color": "red", "depth_gt": 2.0},
            "obj_near_sphere": {"color": "green", "depth_gt": 2.0},
            "obj_far_cube": {"color": "blue", "depth_gt": 4.5},
            "obj_far_sphere": {"color": "yellow", "depth_gt": 4.5},
        }

        metrics: Dict[str, Any] = {
            "pair_skew_s": float(skew),
            "baseline_m": float(self.baseline),
            "horizontal_fov_rad": float(self.horizontal_fov),
            "fx_px": float(fx),
            "objects": {},
            "max_rel_error": float(self.S1_MAX_REL_ERROR),
            "min_pass_objects": int(self.S1_MIN_PASS_OBJECTS),
        }
        pair_diag = self.get_last_test_diagnostics().get("stereo_pair_capture", {})
        if pair_diag:
            metrics["topic_diagnostics"] = pair_diag

        left_dbg = left.copy()
        disp_dbg = disparity_viz.copy()
        passed = 0

        for obj_name, cfg in objects.items():
            mask = self._color_mask(left_hsv, cfg["color"])
            area, bbox = self._bbox(mask)
            if area <= 0:
                metrics["objects"][obj_name] = {"detected": False, "reason": "color contour not found"}
                continue

            roi = self._expand_bbox(bbox, width=w, height=h, pad=6)
            depth_est, valid_px = self._median_depth_in_bbox(depth_map, roi)
            if depth_est is None:
                metrics["objects"][obj_name] = {
                    "detected": True,
                    "depth_gt_m": float(cfg["depth_gt"]),
                    "depth_est_m": None,
                    "valid_px": int(valid_px),
                    "ok": False,
                    "reason": "no valid disparity in ROI",
                }
                continue

            rel_err = abs(float(depth_est) - float(cfg["depth_gt"])) / float(cfg["depth_gt"])
            ok = rel_err <= float(self.S1_MAX_REL_ERROR)
            if ok:
                passed += 1

            metrics["objects"][obj_name] = {
                "detected": True,
                "depth_gt_m": float(cfg["depth_gt"]),
                "depth_est_m": float(depth_est),
                "valid_px": int(valid_px),
                "relative_error": float(rel_err),
                "ok": bool(ok),
            }

            x, y, bw, bh = roi
            color = (0, 255, 0) if ok else (0, 0, 255)
            cv2.rectangle(left_dbg, (x, y), (x + bw, y + bh), color, 2)
            cv2.rectangle(disp_dbg, (x, y), (x + bw, y + bh), color, 2)
            label = f"{obj_name}: z={depth_est:.2f}m gt={cfg['depth_gt']:.2f} err={rel_err:.2f}"
            cv2.putText(left_dbg, label, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2, cv2.LINE_AA)

        metrics["passed_objects"] = int(passed)
        metrics["checks"] = {"pass_count_ok": bool(passed >= int(self.S1_MIN_PASS_OBJECTS))}

        artifacts = [
            self._save_frame("s1_c8_left.png", left),
            self._save_frame("s1_c8_right.png", right),
            self._save_frame("s1_c8_disparity.png", disparity_viz),
            self._save_frame("s1_c8_left_rois.png", left_dbg),
            self._save_frame("s1_c8_disparity_rois.png", disp_dbg),
        ]

        if passed < int(self.S1_MIN_PASS_OBJECTS):
            raise AssertionError(
                f"S1 failed: passed_objects={passed} < {self.S1_MIN_PASS_OBJECTS}, "
                f"objects={metrics['objects']}"
            )

        return {"id": "S1", "metrics": metrics, "artifacts": artifacts}

    def s2_texture_vs_smooth_stability_test(self, simulator) -> Dict[str, Any]:
        self._open_test_scene(simulator, "s2_texture_vs_smooth_stability_test")
        for model_name in self.C8_OBJECTS:
            if not simulator.wait_for_model_spawn(model_name, timeout=20):
                raise RuntimeError(f"Model not spawned: {model_name}")

        left_msg, right_msg, skew = self._wait_pair_closest(
            timeout=float(self.PAIR_TIMEOUT_S),
            retries=int(self.PAIR_RETRIES),
            max_skew_s=float(self.PAIR_MAX_SKEW_S),
        )
        left = self._msg_to_bgr(left_msg)
        right = self._msg_to_bgr(right_msg)
        disparity, depth_map, _ = self._compute_disparity_and_depth(left, right)
        disparity_viz = self._disparity_to_viz(disparity)
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        h, w = disparity.shape[:2]
        y0 = int(float(self.S2_ROI_Y0_RATIO) * h)
        y1 = int(float(self.S2_ROI_Y1_RATIO) * h)
        x0 = int(float(self.S2_ROI_X0_RATIO) * w)
        x1 = int(float(self.S2_ROI_X1_RATIO) * w)
        xm = (x0 + x1) // 2

        wall_band = np.zeros((h, w), dtype=bool)
        wall_band[y0:y1, x0:x1] = True

        with np.errstate(invalid="ignore"):
            finite_depth = np.isfinite(depth_map)
        wall_depth_mask = np.zeros((h, w), dtype=bool)
        with np.errstate(invalid="ignore"):
            wall_depth_mask[finite_depth] = (depth_map[finite_depth] >= 4.2) & (depth_map[finite_depth] <= 5.8)
        wall_mask = wall_band & wall_depth_mask

        left_half_mask = np.zeros((h, w), dtype=bool)
        right_half_mask = np.zeros((h, w), dtype=bool)
        left_half_mask[y0:y1, x0:xm] = True
        right_half_mask[y0:y1, xm:x1] = True

        left_wall = wall_mask & left_half_mask
        right_wall = wall_mask & right_half_mask

        # Fallback if depth-based wall isolation is too sparse.
        if np.count_nonzero(left_wall) < 200 or np.count_nonzero(right_wall) < 200:
            left_wall = left_half_mask
            right_wall = right_half_mask

        left_texture = float(np.std(gray[left_wall])) if np.count_nonzero(left_wall) > 0 else 0.0
        right_texture = float(np.std(gray[right_wall])) if np.count_nonzero(right_wall) > 0 else 0.0

        left_crop_left = gray[y0:y1, x0:xm]
        left_crop_right = gray_right[y0:y1, x0:xm]
        right_crop_left = gray[y0:y1, xm:x1]
        right_crop_right = gray_right[y0:y1, xm:x1]

        left_ratio = self._crop_valid_ratio_bm(
            left_crop_left,
            left_crop_right,
            block_size=int(self.S2_CROP_BLOCK_SIZE),
            texture_threshold=int(self.S2_CROP_TEXTURE_THRESHOLD),
            uniqueness_ratio=int(self.S2_CROP_UNIQUENESS_RATIO),
        )
        right_ratio = self._crop_valid_ratio_bm(
            right_crop_left,
            right_crop_right,
            block_size=int(self.S2_CROP_BLOCK_SIZE),
            texture_threshold=int(self.S2_CROP_TEXTURE_THRESHOLD),
            uniqueness_ratio=int(self.S2_CROP_UNIQUENESS_RATIO),
        )

        if right_texture >= left_texture:
            textured_side = "right"
            smooth_side = "left"
            valid_ratio_textured = float(right_ratio)
            valid_ratio_smooth = float(left_ratio)
        else:
            textured_side = "left"
            smooth_side = "right"
            valid_ratio_textured = float(left_ratio)
            valid_ratio_smooth = float(right_ratio)

        gain = float(valid_ratio_textured - valid_ratio_smooth)
        check_gain_005 = gain >= float(self.S2_MIN_VALID_GAIN)
        check_non_worse = valid_ratio_textured >= valid_ratio_smooth

        metrics = {
            "pair_skew_s": float(skew),
            "topic_diagnostics": self.get_last_test_diagnostics().get("stereo_pair_capture", {}),
            "assignment_by_texture_std": {
                "left_std": float(left_texture),
                "right_std": float(right_texture),
                "textured_side": textured_side,
                "smooth_side": smooth_side,
            },
            "crop_matcher": {
                "type": "StereoBM",
                "block_size": int(self.S2_CROP_BLOCK_SIZE),
                "texture_threshold": int(self.S2_CROP_TEXTURE_THRESHOLD),
                "uniqueness_ratio": int(self.S2_CROP_UNIQUENESS_RATIO),
                "crop_bounds": {"x0": int(x0), "x1": int(x1), "xm": int(xm), "y0": int(y0), "y1": int(y1)},
            },
            "valid_ratio": {
                "left": float(left_ratio),
                "right": float(right_ratio),
                "textured": float(valid_ratio_textured),
                "smooth": float(valid_ratio_smooth),
                "gain_textured_minus_smooth": float(gain),
            },
            "checks": {
                "textured_ge_smooth_plus_0_05": bool(check_gain_005),
                "textured_ge_smooth": bool(check_non_worse),
            },
        }

        dbg = disparity_viz.copy()
        left_color = (255, 255, 0) if textured_side == "left" else (200, 200, 200)
        right_color = (255, 255, 0) if textured_side == "right" else (200, 200, 200)
        cv2.rectangle(dbg, (x0, y0), (xm, y1), left_color, 2)
        cv2.rectangle(dbg, (xm, y0), (x1, y1), right_color, 2)
        cv2.putText(dbg, f"left ratio={left_ratio:.3f} std={left_texture:.1f}", (x0, max(20, y0 - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, left_color, 2, cv2.LINE_AA)
        cv2.putText(dbg, f"right ratio={right_ratio:.3f} std={right_texture:.1f}", (xm, max(20, y0 - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, right_color, 2, cv2.LINE_AA)
        cv2.putText(dbg, f"gain={gain:.3f}", (x0, max(20, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        artifacts = [
            self._save_frame("s2_c8_left.png", left),
            self._save_frame("s2_c8_right.png", right),
            self._save_frame("s2_c8_disparity.png", disparity_viz),
            self._save_frame("s2_c8_disparity_rois.png", dbg),
        ]

        self._set_test_diagnostics(stereo_s2={"metrics": copy.deepcopy(metrics)})

        if not (check_non_worse and check_gain_005):
            raise AssertionError(
                "S2 failed: "
                f"textured_ratio={valid_ratio_textured:.4f}, "
                f"smooth_ratio={valid_ratio_smooth:.4f}, "
                f"gain={gain:.4f}, "
                f"required_gain>={self.S2_MIN_VALID_GAIN:.4f}"
            )

        return {"id": "S2", "metrics": metrics, "artifacts": artifacts}


def _camera_method_passed(result: dict) -> bool:
    if not isinstance(result, dict):
        return True
    if "passed" in result:
        return bool(result["passed"])
    metrics = result.get("metrics")
    if isinstance(metrics, dict):
        status = str(metrics.get("status", "") or "").strip().upper()
        if status:
            return status == "PASS"
    return True


def _run_camera_context_test(context_cls, method_name: str, simulator, sensor, progress_cb=None) -> dict:
    ctx = context_cls(sensor)
    method = getattr(ctx, method_name)
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    result = method(simulator)
    if not isinstance(result, dict):
        result = {"result": result}
    else:
        result = dict(result)
    result.setdefault("passed", _camera_method_passed(result))
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return result


def _run_camera_function_test(build_ctx, test_fn, simulator, sensor, progress_cb=None) -> dict:
    ctx = build_ctx(sensor)
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    result = test_fn(ctx, simulator)
    if not isinstance(result, dict):
        result = {"result": result}
    else:
        result = dict(result)
    result.setdefault("passed", _camera_method_passed(result))
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return result


def c1_size_order_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c1_size_order_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "positions": list(ctx.C1_POSITIONS),
        "cube_pose": {"y": float(ctx.C1_TRACK_Y), "z": float(ctx.C1_TRACK_Z)},
        "bbox_area_px": {},
        "bbox_px": {},
        "red_pixels": {},
        "frame_stamp_s": {},
        "min_margin_ratio": float(ctx.C1_MIN_MARGIN_RATIO),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c1_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c1_size_order_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c1_size_order={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c1_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c1_size_order_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c1_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c1_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C1_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C1_CUBE_NAME}"
        _store_c1_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C1_CUBE_NAME}")
    
    prev_stamp_s: Optional[float] = None
    for x in ctx.C1_POSITIONS:
        label = f"x{int(x)}"
        _mono__move_and_settle(ctx, 
            simulator,
            ctx.C1_CUBE_NAME,
            x=float(x),
            y=float(ctx.C1_TRACK_Y),
            z=float(ctx.C1_TRACK_Z),
        )
    
        try:
            msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        except Exception as exc:
            metrics["error_reason"] = f"image_receive_failed:{exc}"
            _store_c1_diag()
            raise RuntimeError(f"Failed to receive fresh image for C1 from topic {resolved_topic}: {exc}") from exc
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        red = _mono__red_mask(ctx, hsv)
        area, (bx, by, bw, bh) = _mono__bbox_area(ctx, red)
        metrics["bbox_area_px"][label] = int(area)
        metrics["bbox_px"][label] = {"x": int(bx), "y": int(by), "w": int(bw), "h": int(bh)}
        metrics["red_pixels"][label] = int(_mono__count_pixels(ctx, red))
        metrics["frame_stamp_s"][label] = float(prev_stamp_s)
    
        artifacts.append(_mono__save_frame(ctx, f"c1_{label}_raw.png", frame))
    
        debug = frame.copy()
        if area > 0:
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
        cv2.putText(
            debug,
            f"{label}: area={area} ts={prev_stamp_s:.6f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        artifacts.append(_mono__save_frame(ctx, f"c1_{label}.png", debug))
    
    x1 = metrics["bbox_area_px"].get("x1", 0)
    x3 = metrics["bbox_area_px"].get("x3", 0)
    x5 = metrics["bbox_area_px"].get("x5", 0)
    order_ok = x1 > x3 > x5
    margin_ok = (x1 >= x3 * ctx.C1_MIN_MARGIN_RATIO) and (x3 >= x5 * ctx.C1_MIN_MARGIN_RATIO)
    metrics["checks"] = {"size_order": bool(order_ok), "size_margin": bool(margin_ok)}
    metrics["status"] = "PASS" if (order_ok and margin_ok) else "FAIL"
    if not (order_ok and margin_ok):
        metrics["error_reason"] = (
            f"size_order_failed: checks={metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}"
        )
    
    metrics_path = _store_c1_diag()
    if not (order_ok and margin_ok):
        raise AssertionError(f"C1 checks failed: {metrics['checks']}, bbox_area_px={metrics['bbox_area_px']}")
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C1", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c2_resolution_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c2_resolution_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "expected_resolution": {
            "width": int(ctx.image_width),
            "height": int(ctx.image_height),
        },
        "actual_resolution": None,
        "encoding": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "status": "ERROR",
        "error_reason": "",
        "display_env": {},
    }
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    
    world = ctx.test_to_world["c2_resolution_test"]
    if not simulator.open_scene(world, ctx.sensor_sdf_path):
        scene_diag = _mono__scene_diag(ctx, simulator)
        metrics["resolved_topic"] = str(ctx.IMAGE_TOPIC)
        metrics["topic_mapping_changed"] = False
        metrics["scene_open_success"] = False
        metrics["error_reason"] = f"scene_open_failed:{scene_diag.get('reason', 'unknown')}" if scene_diag else "scene_open_failed"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"Failed to open scene for c2_resolution_test: {world} "
            f"(reason={scene_diag.get('reason', 'unknown') if scene_diag else 'unknown'})"
        )
    
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
    metrics["scene_open_success"] = True
    
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    
    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(f"Failed to receive image for C2 from topic {resolved_topic}: {exc}") from exc
    
    actual_width = int(getattr(msg, "width", 0) or 0)
    actual_height = int(getattr(msg, "height", 0) or 0)
    metrics["actual_resolution"] = {
        "width": actual_width,
        "height": actual_height,
    }
    metrics["encoding"] = str(getattr(msg, "encoding", "") or "")
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    
    if actual_width <= 0 or actual_height <= 0:
        metrics["error_reason"] = "invalid_image_resolution"
        _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
        raise RuntimeError(
            f"C2 received invalid image dimensions from topic {resolved_topic}: "
            f"width={actual_width}, height={actual_height}"
        )
    
    try:
        frame = _mono__msg_to_bgr(ctx, msg)
    except Exception:
        frame = None
    
    if frame is not None:
        debug = _mono__annotate(ctx, 
            frame,
            [
                f"topic={resolved_topic}",
                f"expected={ctx.image_width}x{ctx.image_height}",
                f"actual={actual_width}x{actual_height}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, "c2_resolution_frame.png", debug))
    
    resolution_matches = (
        actual_width == int(ctx.image_width) and actual_height == int(ctx.image_height)
    )
    metrics["checks"] = {"resolution_matches": bool(resolution_matches)}
    metrics["status"] = "PASS" if resolution_matches else "FAIL"
    if not resolution_matches:
        metrics["error_reason"] = (
            f"resolution_mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}"
        )
    
    _mono__set_test_diagnostics(ctx, c2_resolution=dict(metrics))
    metrics_path = _mono__save_metrics_json(ctx, "c2_resolution_metrics.json", metrics)
    if not resolution_matches:
        raise AssertionError(
            f"C2 resolution mismatch: expected={ctx.image_width}x{ctx.image_height}, "
            f"actual={actual_width}x{actual_height}, topic={resolved_topic}"
        )
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C2", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c3_view_angle_stability_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_DepthProfileTestContext, "c3_view_angle_stability_test", simulator, sensor, progress_cb)


def c4_geometries_presence_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c4_geometries_presence_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "pixel_counts": {},
        "threshold": int(ctx.C4_MIN_PIXELS),
        "display_env": {},
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c4_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c4_geometries_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c4_geometries_presence={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c4_diag()
    
    try:
        _mono__open_c4_scene_with_retry(ctx, simulator)
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c4_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c4_diag()
    
    try:
        msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"image_receive_failed:{exc}"
        _store_c4_diag()
        raise RuntimeError(f"Failed to receive image for C4 from topic {resolved_topic}: {exc}") from exc
    
    frame = _mono__msg_to_bgr(ctx, msg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    for color in ("red", "green", "blue", "yellow"):
        metrics["pixel_counts"][color] = _mono__count_pixels(ctx, _mono__color_mask(ctx, hsv, color))
    
    missing = {c: n for c, n in metrics["pixel_counts"].items() if int(n) <= int(metrics["threshold"])}
    metrics["checks"] = {"all_present": len(missing) == 0, "missing_or_low": missing}
    metrics["status"] = "PASS" if len(missing) == 0 else "FAIL"
    if missing:
        metrics["error_reason"] = f"missing_or_low:{missing}"
    
    debug = _mono__annotate(ctx, 
        frame,
        [
            f"topic={resolved_topic}",
            f"threshold={metrics['threshold']}",
            f"red={metrics['pixel_counts'].get('red', 0)}",
            f"green={metrics['pixel_counts'].get('green', 0)}",
            f"blue={metrics['pixel_counts'].get('blue', 0)}",
            f"yellow={metrics['pixel_counts'].get('yellow', 0)}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c4_geometries.png", debug))
    metrics_path = _store_c4_diag()
    
    if missing:
        raise AssertionError(f"C4 checks failed: missing_or_low={missing}, threshold={metrics['threshold']}")
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C4", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c5_working_range_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_DepthProfileTestContext, "c5_working_range_test", simulator, sensor, progress_cb)


def c6_small_displacement_sensitivity_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_DepthProfileTestContext, "c6_small_displacement_sensitivity_test", simulator, sensor, progress_cb)


def c7_occlusion_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c7_occlusion_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "cases": dict(ctx.C7_CASES),
        "blue_pixels": {},
        "threshold": int(ctx.C7_MIN_PIXELS),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c7_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c7_occlusion_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c7_occlusion={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c7_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c7_occlusion_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c7_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c7_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C7_FRONT_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_FRONT_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_FRONT_CUBE_NAME}")
    if not simulator.wait_for_model_spawn(ctx.C7_BACK_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C7_BACK_CUBE_NAME}"
        _store_c7_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C7_BACK_CUBE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c7_diag()
        raise RuntimeError(f"Failed to receive warmup image for C7 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    
    def _move_and_capture(model_name: str, x: float, y: float, z: float, settle_s: float = 0.35) -> np.ndarray:
        nonlocal prev_stamp_s
    
        _mono__move_and_settle(ctx, simulator, model_name, x=float(x), y=float(y), z=float(z), settle_s=settle_s)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        return _mono__msg_to_bgr(ctx, msg)
    
    _move_and_capture(ctx.C7_BACK_CUBE_NAME, x=3.6, y=0.0, z=0.25)
    
    for case_name, y in ctx.C7_CASES.items():
        frame = _move_and_capture(ctx.C7_FRONT_CUBE_NAME, x=3.0, y=float(y), z=0.25)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        blue = _mono__color_mask(ctx, hsv, "blue")
        blue_count = _mono__count_pixels(ctx, blue)
        metrics["blue_pixels"][case_name] = int(blue_count)
    
        debug = _mono__annotate(ctx, 
            frame,
            [
                f"topic={resolved_topic}",
                f"{case_name}: blue={blue_count}",
                f"front_y={float(y):.2f}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, f"c7_{case_name}.png", debug))
    
    blue_25 = metrics["blue_pixels"].get("occ_25", 0)
    blue_50 = metrics["blue_pixels"].get("occ_50", 0)
    relation_ok = blue_25 > blue_50
    threshold_ok = blue_25 > ctx.C7_MIN_PIXELS and blue_50 > ctx.C7_MIN_PIXELS
    metrics["checks"] = {"occlusion_relation": bool(relation_ok), "threshold_ok": bool(threshold_ok)}
    metrics["status"] = "PASS" if (relation_ok and threshold_ok) else "FAIL"
    if not (relation_ok and threshold_ok):
        metrics["error_reason"] = (
            f"occlusion_mismatch: occ_25={blue_25}, occ_50={blue_50}, threshold={ctx.C7_MIN_PIXELS}"
        )
    
    metrics_path = _store_c7_diag()
    
    if not (relation_ok and threshold_ok):
        raise AssertionError(
            f"C7 checks failed: {metrics['checks']}, blue_pixels={metrics['blue_pixels']}, "
            f"threshold={ctx.C7_MIN_PIXELS}"
        )
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C7", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c9_fov_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    x_fixed = 2.0
    
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c9_fov_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "x_fixed_m": float(x_fixed),
        "step_y_m": float(ctx.C9_STEP),
        "target_fov_rad": float(ctx.horizontal_fov),
        "sphere_radius_m": float(ctx.C9_SPHERE_RADIUS_M),
        "samples": [],
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c9_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c9_fov_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c9_fov={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c9_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c9_fov_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c9_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c9_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C9_SPHERE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C9_SPHERE_NAME}"
        _store_c9_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C9_SPHERE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c9_diag()
        raise RuntimeError(f"Failed to receive warmup image for C9 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    last_visible: Optional[Tuple[float, np.ndarray, int]] = None
    first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None
    
    for y in _mono__iter_float_range(ctx, 0.0, float(ctx.C9_MAX_Y), float(ctx.C9_STEP)):
        _mono__move_and_settle(ctx, simulator, ctx.C9_SPHERE_NAME, x=x_fixed, y=float(y), z=0.2, settle_s=0.35)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        white = _mono__white_mask(ctx, frame)
        contours = _mono__large_contours(ctx, white, min_area=ctx.C9_MIN_CONTOUR_AREA, border_margin=4)
        contour_count = len(contours)
        visible = contour_count > 0
    
        metrics["samples"].append({"y_m": float(y), "visible": bool(visible), "contours": int(contour_count)})
    
        if visible:
            last_visible = (float(y), frame, contour_count)
        elif last_visible is not None:
            first_not_visible = (float(y), frame, contour_count)
            break
    
    if last_visible is None:
        metrics["error_reason"] = "object_never_detected"
        _store_c9_diag()
        raise AssertionError("C9 failed: object was never detected in frame")
    if first_not_visible is None:
        metrics["error_reason"] = "object_never_lost"
        _store_c9_diag()
        raise AssertionError("C9 failed: object did not disappear within tested Y range")
    
    y_visible = float(last_visible[0])
    y_lost = float(first_not_visible[0])
    y_transition = float((y_visible + y_lost) / 2.0)
    y_edge_estimate = float(y_transition + float(ctx.C9_SPHERE_RADIUS_M))
    measured_fov = float(2.0 * atan(y_edge_estimate / x_fixed))
    target_fov = float(ctx.horizontal_fov)
    rel_error = float(abs(measured_fov - target_fov) / target_fov) if target_fov > 0 else float("inf")
    
    dbg_visible = _mono__annotate(ctx, 
        last_visible[1],
        [
            f"topic={resolved_topic}",
            f"Yvisible={y_visible:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVmeasured={measured_fov:.5f} rad",
            "visible=True",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c9_ymax_visible.png", dbg_visible))
    
    dbg_lost = _mono__annotate(ctx, 
        first_not_visible[1],
        [
            f"topic={resolved_topic}",
            f"Ylost={y_lost:.2f} m",
            f"Yedge={y_edge_estimate:.2f} m",
            f"FOVtarget={target_fov:.5f} rad",
            "visible=False",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c9_after_ymax_not_visible.png", dbg_lost))
    
    metrics["y_max_visible_m"] = y_visible
    metrics["y_first_not_visible_m"] = y_lost
    metrics["y_transition_m"] = y_transition
    metrics["y_edge_estimate_m"] = y_edge_estimate
    metrics["fov_measured_rad"] = measured_fov
    metrics["fov_measured_deg"] = float(degrees(measured_fov))
    metrics["fov_target_deg"] = float(degrees(target_fov))
    metrics["relative_error"] = rel_error
    metrics["checks"] = {"rel_error_le_0_02": bool(rel_error <= 0.02)}
    metrics["status"] = "PASS" if rel_error <= 0.02 else "FAIL"
    if rel_error > 0.02:
        metrics["error_reason"] = (
            f"fov_mismatch: measured={measured_fov:.6f}, target={target_fov:.6f}, rel_error={rel_error:.4f}"
        )
    
    metrics_path = _store_c9_diag()
    if rel_error > 0.02:
        raise AssertionError(
            f"C9 failed: measured={measured_fov:.6f} rad, target={target_fov:.6f} rad, rel_error={rel_error:.4f}"
        )
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C9", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c10_clipping_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    clip_cfg = _mono__read_clip_from_sdf(ctx, ctx.sensor_sdf_path)
    near_target = float(clip_cfg["near_m"]) if clip_cfg.get("near_m") is not None else float(ctx.clip_near)
    far_target = float(clip_cfg["far_m"]) if clip_cfg.get("far_m") is not None else float(ctx.clip_far)
    near_tol = max(0.05, abs(near_target) * 0.05)
    far_tol = max(0.05, abs(far_target) * 0.05)
    near_start = min(float(ctx.C10_NEAR_START_X), max(0.01, near_target * 0.3))
    near_end = max(float(ctx.C10_NEAR_SEARCH_END_X), near_target * 8.0)
    
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c10_clipping_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "near_clip_target_m": float(near_target),
        "far_clip_target_m": float(far_target),
        "clip_source": clip_cfg,
        "near_tolerance_m": float(near_tol),
        "far_tolerance_m": float(far_tol),
        "near_search": {"start_x": float(near_start), "end_x": float(near_end)},
        "far_search": {"coarse_step": float(ctx.C10_FAR_COARSE_STEP), "fine_step": float(ctx.C10_FAR_FINE_STEP)},
        "min_red_pixels": int(ctx.C10_MIN_RED_PIXELS),
        "visibility_rule": f"red_pixels>={int(ctx.C10_MIN_RED_PIXELS)}",
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c10_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c10_clipping_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c10_clipping={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c10_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c10_clipping_test")
    except Exception as exc:
        metrics["error_reason"] = f"scene_open_failed:{exc}"
        _store_c10_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    metrics["scene_reason"] = str(scene_diag.get("reason", "")) if scene_diag else ""
    _store_c10_diag()
    
    if not simulator.wait_for_model_spawn(ctx.C10_CUBE_NAME, timeout=20):
        metrics["error_reason"] = f"model_not_spawned:{ctx.C10_CUBE_NAME}"
        _store_c10_diag()
        raise RuntimeError(f"Model not spawned: {ctx.C10_CUBE_NAME}")
    
    try:
        warmup_msg = _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_failed:{exc}"
        _store_c10_diag()
        raise RuntimeError(f"Failed to receive warmup image for C10 from topic {resolved_topic}: {exc}") from exc
    
    prev_stamp_s = _mono__msg_stamp_s(ctx, warmup_msg)
    
    def _move_and_capture(x: float, settle_s: float) -> Tuple[np.ndarray, Dict[str, float], int]:
        nonlocal prev_stamp_s
    
        _mono__move_and_settle(ctx, simulator, ctx.C10_CUBE_NAME, x=float(x), y=0.0, z=0.25, settle_s=settle_s)
        msg = _mono__wait_image_after(ctx, prev_stamp_s, timeout=35.0, topic=resolved_topic)
        prev_stamp_s = _mono__msg_stamp_s(ctx, msg)
        frame = _mono__msg_to_bgr(ctx, msg)
        red_stats = _mono__red_stats(ctx, frame)
        red_pixels = int(red_stats["red_pixels"])
        return frame, red_stats, red_pixels
    
    near_before: Optional[Tuple[float, np.ndarray, int]] = None
    near_after: Optional[Tuple[float, np.ndarray, int]] = None
    
    for x in _mono__iter_float_range(ctx, near_start, near_end, ctx.C10_NEAR_STEP):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.2)
        if bool(red_stats["visible_by_pixels"]):
            near_after = (float(x), frame, red_pixels)
            break
        near_before = (float(x), frame, red_pixels)
    
    if near_after is None:
        metrics["error_reason"] = "near_boundary_not_found"
        _store_c10_diag()
        raise AssertionError("C10 failed: clip_cube did not appear in near search range")
    
    near_x = float(near_after[0])
    metrics["x_near_m"] = near_x
    metrics["near_boundary"] = {
        "before_x_m": float(near_before[0]) if near_before is not None else None,
        "after_x_m": float(near_after[0]),
    }
    
    far_last_visible = near_after
    far_first_not_visible: Optional[Tuple[float, np.ndarray, int]] = None
    
    far_search_stop = float(far_target) * 1.2 + max(2.0, 0.2 * float(far_target))
    metrics["far_search_stop_m"] = float(far_search_stop)
    for x in _mono__iter_float_range(ctx, 
        near_x + float(ctx.C10_FAR_COARSE_STEP),
        far_search_stop,
        float(ctx.C10_FAR_COARSE_STEP),
    ):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.2)
        visible = bool(red_stats["visible_by_pixels"])
        if visible:
            far_last_visible = (float(x), frame, red_pixels)
        else:
            far_first_not_visible = (float(x), frame, red_pixels)
            break
    
    if far_first_not_visible is None:
        metrics["error_reason"] = "far_boundary_not_found"
        _store_c10_diag()
        raise AssertionError("C10 failed: clip_cube did not disappear in far search range")
    
    fine_start = max(float(near_x), float(far_last_visible[0]) - float(ctx.C10_FAR_COARSE_STEP))
    fine_end = float(far_first_not_visible[0])
    far_last_visible_fine = far_last_visible
    far_first_not_visible_fine = far_first_not_visible
    
    for x in _mono__iter_float_range(ctx, fine_start, fine_end, float(ctx.C10_FAR_FINE_STEP)):
        frame, red_stats, red_pixels = _move_and_capture(x=float(x), settle_s=0.15)
        visible = bool(red_stats["visible_by_pixels"])
        if visible:
            far_last_visible_fine = (float(x), frame, red_pixels)
        else:
            far_first_not_visible_fine = (float(x), frame, red_pixels)
            break
    
    far_x = float(far_last_visible_fine[0])
    metrics["x_far_m"] = far_x
    metrics["x_far_first_not_visible_m"] = float(far_first_not_visible_fine[0])
    metrics["far_boundary"] = {
        "last_visible_x_m": float(far_last_visible_fine[0]),
        "first_not_visible_x_m": float(far_first_not_visible_fine[0]),
    }
    
    near_ok = abs(near_x - float(near_target)) <= float(near_tol)
    far_ok = abs(far_x - float(far_target)) <= float(far_tol)
    metrics["checks"] = {
        "near_abs_error_m": float(abs(near_x - float(near_target))),
        "far_abs_error_m": float(abs(far_x - float(far_target))),
        "near_ok": bool(near_ok),
        "far_ok": bool(far_ok),
    }
    
    near_after_stats = _mono__red_stats(ctx, near_after[1])
    far_before_stats = _mono__red_stats(ctx, far_last_visible_fine[1])
    far_after_stats = _mono__red_stats(ctx, far_first_not_visible_fine[1])
    metrics["visibility_debug"] = {
        "near_before_red_pixels": int(near_before[2]) if near_before is not None else None,
        "near_before_pixel_ratio": float(_mono__red_stats(ctx, near_before[1])["pixel_ratio"]) if near_before is not None else None,
        "near_after_red_pixels": int(near_after[2]),
        "near_after_pixel_ratio": float(near_after_stats["pixel_ratio"]),
        "far_before_red_pixels": int(far_last_visible_fine[2]),
        "far_before_pixel_ratio": float(far_before_stats["pixel_ratio"]),
        "far_after_red_pixels": int(far_first_not_visible_fine[2]),
        "far_after_pixel_ratio": float(far_after_stats["pixel_ratio"]),
    }
    
    if near_before is not None:
        dbg = _mono__annotate(ctx, 
            near_before[1],
            [
                f"near_before x={near_before[0]:.2f}",
                f"red_px={near_before[2]}",
                f"ratio={_mono__red_stats(ctx, near_before[1])['pixel_ratio']:.4f}",
                "visible=False",
                f"topic={resolved_topic}",
            ],
        )
        artifacts.append(_mono__save_frame(ctx, "c10_near_before.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        near_after[1],
        [
            f"near_after x={near_after[0]:.2f}",
            f"red_px={near_after[2]}",
            f"ratio={near_after_stats['pixel_ratio']:.4f}",
            "visible=True",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_near_after.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        far_last_visible_fine[1],
        [
            f"far_before x={far_last_visible_fine[0]:.2f}",
            f"red_px={far_last_visible_fine[2]}",
            f"ratio={far_before_stats['pixel_ratio']:.4f}",
            "visible=True",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_far_before.png", dbg))
    
    dbg = _mono__annotate(ctx, 
        far_first_not_visible_fine[1],
        [
            f"far_after x={far_first_not_visible_fine[0]:.2f}",
            f"red_px={far_first_not_visible_fine[2]}",
            f"ratio={far_after_stats['pixel_ratio']:.4f}",
            "visible=False",
            f"topic={resolved_topic}",
        ],
    )
    artifacts.append(_mono__save_frame(ctx, "c10_far_after.png", dbg))
    
    metrics["status"] = "PASS" if (near_ok and far_ok) else "FAIL"
    if not (near_ok and far_ok):
        metrics["error_reason"] = (
            f"clipping_mismatch: near={near_x:.3f}/{near_target:.3f}, "
            f"far={far_x:.3f}/{far_target:.3f}"
        )
    
    metrics_path = _store_c10_diag()
    if not (near_ok and far_ok):
        raise AssertionError(
            f"C10 failed: near={near_x:.3f} (target {near_target:.3f}, tol={near_tol:.3f}), "
            f"far={far_x:.3f} (target {far_target:.3f}, tol={far_tol:.3f})"
        )
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C10", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}
    

def c11_fps_stability_test(simulator, sensor, progress_cb=None) -> dict:
    if progress_cb:
        try:
            progress_cb(5)
        except Exception:
            pass
    ctx = _mono_build_ctx(sensor)
    artifacts: List[str] = []
    metrics: Dict[str, Any] = {
        "world_file": str(ctx.test_to_world["c11_fps_stability_test"]),
        "expected_topic": str(ctx.IMAGE_TOPIC),
        "resolved_topic": "",
        "scene_open_success": False,
        "topic_mapping_changed": False,
        "display_env": {},
        "duration_target_s": float(ctx.C11_DURATION_S),
        "update_rate_target_hz": float(ctx.update_rate),
        "warmup_seconds": float(ctx.C11_WARMUP_SECONDS),
        "jitter_percentile": int(ctx.C11_JITTER_PERCENTILE),
        "jitter_limit_s": float(ctx.C11_MAX_JITTER_S),
        "status": "ERROR",
        "error_reason": "",
    }
    
    def _store_c11_diag() -> str:
        metrics_path = _mono__save_metrics_json(ctx, "c11_fps_stability_metrics.json", metrics)
        _mono__set_test_diagnostics(ctx, 
            c11_fps_stability={
                "metrics": dict(metrics),
                "artifacts": list(artifacts),
                "metrics_json": metrics_path,
            }
        )
        return metrics_path
    
    ctx._last_test_diagnostics = {}
    metrics["display_env"] = _mono__ensure_render_display_env(ctx, )
    _store_c11_diag()
    
    try:
        _mono__open_test_scene(ctx, simulator, "c11_fps_stability_test")
    except Exception:
        resolved_topic, scene_diag = _mono__resolved_image_topic(ctx, simulator)
        metrics["resolved_topic"] = str(resolved_topic)
        metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
        metrics["scene_open_success"] = False
        reason = "unknown"
        if isinstance(scene_diag, dict):
            reason = str(scene_diag.get("reason", "unknown"))
        metrics["error_reason"] = f"scene_open_failed:{reason}"
        _store_c11_diag()
        raise
    
    metrics["scene_open_success"] = True
    resolved_topic, _ = _mono__resolved_image_topic(ctx, simulator)
    metrics["resolved_topic"] = str(resolved_topic)
    metrics["topic_mapping_changed"] = bool(str(resolved_topic) != str(ctx.IMAGE_TOPIC))
    _store_c11_diag()
    
    try:
        _mono__wait_image(ctx, timeout=35.0, topic=resolved_topic)
    except Exception as exc:
        metrics["error_reason"] = f"warmup_image_receive_failed:{exc}"
        _store_c11_diag()
        raise RuntimeError(f"Failed to receive warmup image for C11 from topic {resolved_topic}: {exc}") from exc
    
    timestamps: List[float] = []
    first_msg: Dict[str, Optional[Image]] = {"msg": None}
    last_msg: Dict[str, Optional[Image]] = {"msg": None}
    header_stamp_missing_count = 0
    
    def _on_image(msg: Image) -> None:
        nonlocal header_stamp_missing_count
        stamp = float(msg.header.stamp.to_sec())
        if stamp <= 0.0:
            header_stamp_missing_count += 1
            stamp = float(rospy.Time.now().to_sec())
        timestamps.append(stamp)
        if first_msg["msg"] is None:
            first_msg["msg"] = msg
        last_msg["msg"] = msg
    
    sub = rospy.Subscriber(resolved_topic, Image, _on_image, queue_size=2000)
    started_wall = time.perf_counter()
    try:
        while (time.perf_counter() - started_wall) < float(ctx.C11_DURATION_S):
            time.sleep(0.1)
    finally:
        sub.unregister()
    
    metrics["duration_actual_s"] = round(time.perf_counter() - started_wall, 4)
    metrics["raw_frames_captured"] = int(len(timestamps))
    metrics["header_stamp_missing_count"] = int(header_stamp_missing_count)
    metrics["first_raw_stamp_s"] = float(timestamps[0]) if timestamps else None
    metrics["last_raw_stamp_s"] = float(timestamps[-1]) if timestamps else None
    if len(timestamps) < 2:
        metrics["error_reason"] = f"not_enough_frames_captured:{len(timestamps)}"
        _store_c11_diag()
        raise AssertionError(f"C11 failed: not enough frames captured ({len(timestamps)})")
    
    monotonic_stamps: List[float] = []
    for ts in timestamps:
        if not monotonic_stamps or ts > monotonic_stamps[-1]:
            monotonic_stamps.append(float(ts))
    
    metrics["monotonic_frames_captured"] = int(len(monotonic_stamps))
    metrics["non_monotonic_dropped"] = int(len(timestamps) - len(monotonic_stamps))
    if len(monotonic_stamps) < 2:
        metrics["error_reason"] = "no_monotonic_timestamp_sequence"
        _store_c11_diag()
        raise AssertionError("C11 failed: no monotonic timestamp sequence")
    
    warmup_frames = int(max(1, round(float(ctx.update_rate) * float(ctx.C11_WARMUP_SECONDS))))
    if len(monotonic_stamps) <= (warmup_frames + 1):
        metrics["error_reason"] = (
            f"not_enough_frames_after_warmup:{len(monotonic_stamps)} total,warmup={warmup_frames}"
        )
        _store_c11_diag()
        raise AssertionError(
            f"C11 failed: not enough frames after warmup ({len(monotonic_stamps)} total, warmup={warmup_frames})"
        )
    
    eval_stamps = monotonic_stamps[warmup_frames:]
    total_dt = float(eval_stamps[-1] - eval_stamps[0])
    if total_dt <= 0.0:
        metrics["error_reason"] = f"invalid_timestamps_interval:{total_dt}"
        _store_c11_diag()
        raise AssertionError(f"C11 failed: invalid timestamps interval ({total_dt})")
    
    n_frames = len(eval_stamps)
    fps_actual = float((n_frames - 1) / total_dt)
    ideal_dt = float(1.0 / float(ctx.update_rate))
    deltas = np.diff(np.array(eval_stamps, dtype=np.float64))
    
    abs_jitter = np.abs(deltas - ideal_dt) if deltas.size > 0 else np.array([], dtype=np.float64)
    # Было раньше: jitter = max(|dt - ideal_dt|), что слишком чувствительно к единичным пикам.
    # Теперь: устойчивый jitter = P95(|dt - ideal_dt|), max оставляем как диагностическую метрику.
    jitter = float(np.percentile(abs_jitter, ctx.C11_JITTER_PERCENTILE)) if abs_jitter.size > 0 else 0.0
    jitter_max_abs = float(np.max(abs_jitter)) if abs_jitter.size > 0 else 0.0
    max_dt = float(np.max(deltas)) if deltas.size > 0 else 0.0
    dropouts = int(np.sum(deltas > (2.0 * ideal_dt))) if deltas.size > 0 else 0
    
    fps_ok = fps_actual >= (0.95 * float(ctx.update_rate))
    jitter_ok = jitter <= float(ctx.C11_MAX_JITTER_S)
    dropouts_ok = dropouts == 0
    
    metrics.update(
        {
            "frames_captured": int(n_frames),
            "frames_skipped_warmup": int(warmup_frames),
            "first_eval_stamp_s": float(eval_stamps[0]),
            "last_eval_stamp_s": float(eval_stamps[-1]),
            "timestamps_interval_s": total_dt,
            "fps_actual_hz": fps_actual,
            "ideal_dt_s": ideal_dt,
            "jitter_s": jitter,
            "jitter_old_max_abs_s": jitter_max_abs,
            "max_dt_s": max_dt,
            "dropouts_count": dropouts,
            "checks": {
                "fps_ok": bool(fps_ok),
                "jitter_ok": bool(jitter_ok),
                "dropouts_ok": bool(dropouts_ok),
            },
        }
    )
    
    if first_msg["msg"] is not None:
        frame_first = _mono__msg_to_bgr(ctx, first_msg["msg"])
        debug_first = _mono__annotate(ctx, frame_first, ["C11 first frame", f"fps={fps_actual:.2f}", f"jitter={jitter:.4f}s"])
        artifacts.append(_mono__save_frame(ctx, "c11_first_frame.png", debug_first))
    if last_msg["msg"] is not None:
        frame_last = _mono__msg_to_bgr(ctx, last_msg["msg"])
        debug_last = _mono__annotate(ctx, frame_last, ["C11 last frame", f"dropouts={dropouts}", f"max_dt={max_dt:.4f}s"])
        artifacts.append(_mono__save_frame(ctx, "c11_last_frame.png", debug_last))
    
    metrics["status"] = "PASS" if (fps_ok and jitter_ok and dropouts_ok) else "FAIL"
    if not (fps_ok and jitter_ok and dropouts_ok):
        metrics["error_reason"] = (
            f"fps_jitter_or_dropout_failed: fps={fps_actual:.3f}, jitter={jitter:.4f}, dropouts={dropouts}"
        )
    
    metrics_path = _store_c11_diag()
    if not (fps_ok and jitter_ok and dropouts_ok):
        raise AssertionError(
            f"C11 failed: fps={fps_actual:.3f} (target>={0.95 * ctx.update_rate:.3f}), "
            f"jitter={jitter:.4f}s (limit<={ctx.C11_MAX_JITTER_S:.4f}s), dropouts={dropouts}"
        )
    
    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass
    return {"id": "C11", "metrics": metrics, "artifacts": artifacts, "metrics_json": metrics_path}

def depth_perception_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_DepthProfileTestContext, "depth_perception_test", simulator, sensor, progress_cb)


def stereo_topics_presence_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_StereoProfileTestContext, "stereo_topics_presence_test", simulator, sensor, progress_cb)


def stereo_disparity_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_StereoProfileTestContext, "stereo_disparity_test", simulator, sensor, progress_cb)


def stereo_occlusion_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_StereoProfileTestContext, "stereo_occlusion_test", simulator, sensor, progress_cb)


def s1_stereo_accuracy_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_StereoProfileTestContext, "s1_stereo_accuracy_test", simulator, sensor, progress_cb)


def s2_texture_vs_smooth_stability_test(simulator, sensor, progress_cb=None) -> dict:
    return _run_camera_context_test(_StereoProfileTestContext, "s2_texture_vs_smooth_stability_test", simulator, sensor, progress_cb)

# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████╗░█████╗░░█████╗░████████╗██╗██╗░░░░░███████╗░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░╚══██╔══╝██╔══██╗██╔══██╗╚══██╔══╝██║██║░░░░░██╔════╝░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██║░░░███████║██║░░╚═╝░░░██║░░░██║██║░░░░░█████╗░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██║░░░██╔══██║██║░░██╗░░░██║░░░██║██║░░░░░██╔══╝░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██║░░░██║░░██║╚█████╔╝░░░██║░░░██║███████╗███████╗░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
# ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░╚═╝░░░╚═╝░░╚═╝░╚════╝░░░░╚═╝░░░╚═╝╚══════╝╚══════╝░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

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
    import time
    import rospy
    from gazebo_msgs.msg import ContactsState
    
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
    import time
    import numpy as np
    import rospy
    from gazebo_msgs.msg import ContactsState
    
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


# ── Register tests ────────────────────────────────────────────────────────────


# ── World definitions ─────────────────────────────────────────────────────────

from enum import Enum

class Worlds(Enum):
    # ... existing worlds ...
    TACTILE_FORCE = "assets/sensors/worlds/tactile_force.world"
    TACTILE_UNIFORMITY = "assets/sensors/worlds/tactile_uniformity.world"



# ── Generic test (any sensor type) ───────────────────────────────────────────

# Ordered probe list: (importable pkg, class name, short hint, warmup_sec).
# warmup_sec: how long to wait before starting capture for this type.
#   Camera plugins need ~3s to register their topic after Gazebo starts.
#   Fast publishers (RFID, IMU, etc.) need no warmup.
# capture_data is called once per candidate type; first success wins.
_PROBE_TYPES = [
    ("sensor_msgs.msg",   "Image",                          "image",       3.0),
    ("sensor_msgs.msg",   "PointCloud2",                    "pointcloud2", 3.0),
    ("geometry_msgs.msg", "PoseStamped",                    "pose_stamped",0.0),
    ("geometry_msgs.msg", "PoseWithCovarianceStamped",      "pose_cov",    0.0),
    ("sensor_msgs.msg",   "LaserScan",                      "laser_scan",  0.0),
    ("sensor_msgs.msg",   "Range",                          "range",       0.0),
    ("sensor_msgs.msg",   "Imu",                            "imu",         0.0),
    ("std_msgs.msg",      "String",                         "string",      0.0),
    ("std_msgs.msg",      "Float32",                        "float32",     0.0),
    ("std_msgs.msg",      "Float64",                        "float64",     0.0),
]


def _import_msg(pkg: str, cls: str):
    import importlib
    return getattr(importlib.import_module(pkg), cls)


def _describe_msg(msg, hint: str) -> dict:
    """Extract a flat dict of human-readable fields from a ROS message."""
    import numpy as np
    info = {"msg_class": type(msg).__name__}

    if hint == "image":
        info["width"]       = msg.width
        info["height"]      = msg.height
        info["encoding"]    = msg.encoding
        arr = np.frombuffer(msg.data, dtype=np.float32 if "32FC" in msg.encoding else np.uint8)
        info["data_bytes"]  = len(msg.data)
        info["non_zero_px"] = int(np.count_nonzero(arr))

    elif hint == "pointcloud2":
        info["width"]      = msg.width
        info["height"]     = msg.height
        info["point_step"] = msg.point_step
        info["data_bytes"] = len(msg.data)
        info["fields"]     = [f.name for f in msg.fields]

    elif hint in ("pose_stamped", "pose_cov"):
        p = msg.pose.pose if hint == "pose_cov" else msg.pose
        info["frame_id"] = msg.header.frame_id
        info["position"] = f"({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f})"

    elif hint == "laser_scan":
        import math
        valid = [r for r in msg.ranges if math.isfinite(r)]
        info["num_beams"]   = len(msg.ranges)
        info["valid_beams"] = len(valid)
        info["range_min_m"] = msg.range_min
        info["range_max_m"] = msg.range_max
        if valid:
            info["closest_m"] = round(min(valid), 3)

    elif hint == "range":
        info["range_m"]           = round(msg.range, 4)
        info["range_min"]         = msg.min_range
        info["range_max"]         = msg.max_range
        info["field_of_view_rad"] = round(msg.field_of_view, 4)

    elif hint == "imu":
        a = msg.linear_acceleration
        g = msg.angular_velocity
        info["accel"] = f"({a.x:.3f}, {a.y:.3f}, {a.z:.3f})"
        info["gyro"]  = f"({g.x:.3f}, {g.y:.3f}, {g.z:.3f})"

    elif hint == "string":
        info["data"] = msg.data[:120]

    elif hint in ("float32", "float64"):
        info["data"] = msg.data

    return info


def _save_raw(msg, hint: str, save_dir: str) -> str:
    """Save whatever is useful from the message. Returns saved path."""
    import os, json
    os.makedirs(save_dir, exist_ok=True)

    if hint == "image":
        arr  = _img_to_numpy(msg)
        return _save_image(arr, save_dir, "capture")

    meta = {"type": type(msg).__name__, "hint": hint}
    if hasattr(msg, "header"):
        meta["frame_id"] = msg.header.frame_id
        meta["stamp"]    = str(msg.header.stamp)
    path = os.path.join(save_dir, "capture_meta.json")
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    return path


def sensor_capture_basic(simulator, sensor, progress_cb=None) -> dict:
    import os
    import time
    import rospy
    import rostopic
    
    print(f"\n{'='*60}")
    print(f"TESTING SENSOR: {getattr(sensor, 'sensor_name', 'unknown')} ({getattr(sensor, 'sensor_type', 'unknown')})")
    print(f"TOPIC: {sensor.topic}")
    print(f"{'='*60}")
    
    result = {
        "passed": False,
        "topic": sensor.topic,
        "sensor_type": getattr(sensor, "sensor_type", "unknown"),
        "sensor_name": getattr(sensor, "sensor_name", "unknown"),
        "duration": 0,
        "message_type": None,
        "data_received": False,
        "error": None
    }
    
    t0 = time.time()
    
    world_path = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/resources/worlds/rfid/overlap_tags.world"
    print(f"[DEBUG] World path: {world_path}")
    print(f"[DEBUG] World exists: {os.path.exists(world_path)}")
    print(f"[DEBUG] Sensor SDF path: {sensor.sdf_path}")
    print(f"[DEBUG] Sensor SDF exists: {os.path.exists(sensor.sdf_path)}")
    
    if not os.path.exists(world_path):
        print(f"[WARN] World file not found, using fallback")
        fallback = "/tmp/default_world.world"
        with open(fallback, 'w') as f:
            f.write('''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
  </world>
</sdf>''')
        world_path = fallback
        print(f"[DEBUG] Created fallback world: {world_path}")
    
    print(f"[DEBUG] Opening Gazebo scene...")
    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Failed to open Gazebo scene")
        return result
    
    print(f"[DEBUG] Gazebo scene opened successfully")
    
    if progress_cb:
        try:
            progress_cb(20)
            print(f"[DEBUG] Progress callback 20%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result
    
    print(f"[DEBUG] Waiting 3 seconds for plugins to initialize...")
    time.sleep(3)
    
    print(f"[DEBUG] Getting published topics...")
    topics = rospy.get_published_topics()
    print(f"[DEBUG] Found {len(topics)} topics total")
    
    # Print all available topics for debugging
    print(f"[DEBUG] Available topics:")
    for t, t_type in topics[:15]:  # Show first 15
        if not t.startswith('/rosout'):
            print(f"        {t} -> {t_type}")
    
    topic_exists = any(t == sensor.topic for t, _ in topics)
    print(f"[DEBUG] Topic '{sensor.topic}' exists: {topic_exists}")
    
    if not topic_exists:
        result["error"] = f"Topic {sensor.topic} not found"
        result["available_topics"] = [t for t, _ in topics[:20] if not t.startswith('/rosout')]
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Topic not found!")
        return result
    
    if progress_cb:
        try:
            progress_cb(40)
            print(f"[DEBUG] Progress callback 40%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result
    
    print(f"[DEBUG] Getting message class for topic: {sensor.topic}")
    msg_class, real_topic, _ = rostopic.get_topic_class(sensor.topic)
    print(f"[DEBUG] rostopic.get_topic_class returned: {msg_class}")
    print(f"[DEBUG] Real topic: {real_topic}")
    
    if msg_class is None:
        print(f"[DEBUG] Message class is None, trying fallback by sensor type")
        common_types = {
            "camera": "sensor_msgs/Image",
            "image": "sensor_msgs/Image",
            "laser": "sensor_msgs/LaserScan",
            "scan": "sensor_msgs/LaserScan",
            "imu": "sensor_msgs/Imu",
            "contact": "gazebo_msgs/ContactsState",
            "bumper": "gazebo_msgs/ContactsState",
            "pointcloud": "sensor_msgs/PointCloud2",
            "depth": "sensor_msgs/Image",
            "sonar": "sensor_msgs/Range",
            "gps": "sensor_msgs/NavSatFix",
            "joint": "sensor_msgs/JointState",
            "temperature": "sensor_msgs/Temperature",
            "fluid": "sensor_msgs/FluidPressure",
            "magnetic": "sensor_msgs/MagneticField"
        }
        
        sensor_type = getattr(sensor, "sensor_type", "").lower()
        print(f"[DEBUG] Sensor type: {sensor_type}")
        msg_type_name = common_types.get(sensor_type)
        print(f"[DEBUG] Fallback message type: {msg_type_name}")
        
        if msg_type_name:
            try:
                package, msg_name = msg_type_name.split('/')
                print(f"[DEBUG] Importing {package}.msg.{msg_name}")
                module = __import__(f"{package}.msg", fromlist=[msg_name])
                msg_class = getattr(module, msg_name)
                print(f"[DEBUG] Successfully imported: {msg_class}")
            except Exception as e:
                print(f"[DEBUG] Failed to import: {e}")
                pass
    
    if msg_class is None:
        result["error"] = "Could not determine message type"
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Could not determine message type")
        return result
    
    result["message_type"] = msg_class.__name__
    print(f"[DEBUG] Using message class: {msg_class.__name__}")
    
    if progress_cb:
        try:
            progress_cb(60)
            print(f"[DEBUG] Progress callback 60%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result
    
    print(f"[DEBUG] Waiting for message on {sensor.topic} (timeout=10s)...")
    try:
        msg = rospy.wait_for_message(sensor.topic, msg_class, timeout=10.0)
        print(f"[DEBUG] ✓ Message received! Type: {type(msg).__name__}")
        result["data_received"] = True
        result["passed"] = True
        
        if hasattr(msg, 'header'):
            result["timestamp"] = msg.header.stamp.to_sec() if msg.header.stamp else None
            result["frame_id"] = msg.header.frame_id
            print(f"[DEBUG] Header: frame={msg.header.frame_id}, stamp={msg.header.stamp}")
        
        if hasattr(msg, 'states'):
            result["contact_count"] = len(msg.states)
            print(f"[DEBUG] Contact count: {len(msg.states)}")
            if msg.states:
                result["first_contact"] = {
                    "collision1": msg.states[0].collision1_name,
                    "collision2": msg.states[0].collision2_name
                }
                print(f"[DEBUG] First contact: {msg.states[0].collision1_name} -> {msg.states[0].collision2_name}")
        elif hasattr(msg, 'data') and hasattr(msg, 'height'):
            result["width"] = msg.width
            result["height"] = msg.height
            result["encoding"] = msg.encoding
            print(f"[DEBUG] Image: {msg.width}x{msg.height}, encoding={msg.encoding}")
            print(f"[DEBUG] Data length: {len(msg.data)} bytes")
        elif hasattr(msg, 'ranges'):
            result["num_ranges"] = len(msg.ranges)
            result["angle_min"] = msg.angle_min
            result["angle_max"] = msg.angle_max
            print(f"[DEBUG] Laser scan: {len(msg.ranges)} ranges")
        elif hasattr(msg, 'angular_velocity'):
            result["angular_velocity"] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            result["linear_acceleration"] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            print(f"[DEBUG] IMU data received")
        
    except rospy.ROSException as e:
        result["error"] = f"Timeout: No message received within 10s"
        print(f"[ERROR] Timeout: No message received on {sensor.topic}")
    except Exception as e:
        result["error"] = str(e)
        print(f"[ERROR] Exception: {e}")
        import traceback
        traceback.print_exc()
    
    if progress_cb:
        try:
            progress_cb(100)
            print(f"[DEBUG] Progress callback 100%")
        except:
            pass
    
    result["duration"] = round(time.time() - t0, 2)
    print(f"[DEBUG] Test completed in {result['duration']}s")
    print(f"[DEBUG] Result: {'PASSED' if result['passed'] else 'FAILED'}")
    if result.get('error'):
        print(f"[DEBUG] Error: {result['error']}")
    print(f"{'='*60}\n")
    
    if simulator:
        try:
            simulator.notify_capture({
                "sensor_type": result["sensor_type"],
                "sensor_name": result["sensor_name"],
                "topic": sensor.topic,
                "success": result["passed"]
            }, None)
        except:
            pass
    
    return result
# ── TESTS registry ────────────────────────────────────────────────────────────

TESTS: dict[str, callable] = {
    # Generic (assign to any sensor type)
    "sensor_capture_basic":          sensor_capture_basic,
    # RFID
    "rfid_max_stable_read_distance": rfid_max_stable_read_distance,
    "rfid_min_stable_read_distance": rfid_min_stable_read_distance,
    "rfid_mass_read":                rfid_mass_read,
    "rfid_overlap_tags":             rfid_overlap_tags,
    "rfid_angle_dependence":         rfid_angle_dependence,
    "rfid_move_tags":                rfid_move_tags,
    "rfid_antenna_rotation":         rfid_antenna_rotation,
    # Camera (libgazebo_ros_camera.so)
    "camera_data_received":          camera_data_received,
    "camera_depth_accuracy":         camera_depth_accuracy,
    "camera_resolution":             camera_resolution,
    "c1_size_order_test":            c1_size_order_test,
    "c2_resolution_test":            c2_resolution_test,
    "c3_view_angle_stability_test":  c3_view_angle_stability_test,
    "c4_geometries_presence_test":   c4_geometries_presence_test,
    "c5_working_range_test":         c5_working_range_test,
    "c6_small_displacement_sensitivity_test": c6_small_displacement_sensitivity_test,
    "c7_occlusion_test":             c7_occlusion_test,
    "c9_fov_test":                   c9_fov_test,
    "c10_clipping_test":             c10_clipping_test,
    "c11_fps_stability_test":        c11_fps_stability_test,
    "depth_perception_test":         depth_perception_test,
    "stereo_topics_presence_test":   stereo_topics_presence_test,
    "stereo_disparity_test":         stereo_disparity_test,
    "stereo_occlusion_test":         stereo_occlusion_test,
    "s1_stereo_accuracy_test":       s1_stereo_accuracy_test,
    "s2_texture_vs_smooth_stability_test": s2_texture_vs_smooth_stability_test,
    # Tactile (libgazebo_ros_bumper.so)
    "tactile_min_force_threshold": tactile_min_force_threshold,
    "tactile_response_uniformity": tactile_response_uniformity,

}
