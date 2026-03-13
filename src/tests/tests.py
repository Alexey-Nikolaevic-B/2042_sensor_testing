import math
import time
import logging
from enum import Enum

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
    # RFID
    RFID_CHANGE_DISTANCE  = "assets/worlds/rfid_change_distance.world"
    RFID_MASS_READ        = "assets/worlds/rfid_mass_read.world"
    RFID_OVERLAP_TAGS     = "assets/worlds/rfid_overlap_tags.world"
    RFID_ANGLE_DEPENDENCE = "assets/worlds/rfid_angle_dependence.world"
    RFID_MOVE_TAGS        = "assets/worlds/rfid_move_tags.world"
    RFID_ANTENNA_ROTATION = "assets/worlds/rfid_antenna_rotation.world"
    # Camera
    CAMERA_SMOKE          = "assets/worlds/camera_smoke.world"
    CAMERA_DEPTH_ACCURACY = "assets/worlds/camera_depth_accuracy.world"
    CAMERA_RESOLUTION     = "assets/worlds/camera_resolution.world"


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
}