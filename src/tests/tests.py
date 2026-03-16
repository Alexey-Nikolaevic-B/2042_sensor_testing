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
    path  = '/home/alexey/Documents/projects/2042/github/2042_sensor_testing/assets/worlds'

    # RFID
    RFID_CHANGE_DISTANCE  = f"{path}/rfid_change_distance.world"
    RFID_MASS_READ        = f"{path}/rfid_mass_read.world"
    RFID_OVERLAP_TAGS     = f"{path}/rfid_overlap_tags.world"
    RFID_ANGLE_DEPENDENCE = f"{path}/rfid_angle_dependence.world"
    RFID_MOVE_TAGS        = f"{path}/rfid_move_tags.world"
    RFID_ANTENNA_ROTATION = f"{path}/rfid_antenna_rotation.world"
    # Camera
    CAMERA_SMOKE          = f"{path}/rfid_antenna_rotation.world"
    CAMERA_DEPTH_ACCURACY = f"{path}/camera_depth.world"
    CAMERA_RESOLUTION     = f"{path}/camera_resolution.world"
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
    import traceback

    world = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/resources/worlds/rfid/overlap_tags.world"
    
    try:
        if not simulator.open_scene(world, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
    except Exception as e:
        print(f"Error opening scene: {e}")
        traceback.print_exc()
        return {
            "passed": False,
            "error": f"Scene open failed: {str(e)}",
        }

    if progress_cb:
        progress_cb(20)

    t0 = time.time()
    
    try:
        msgs = sensor.capture_data(Image, window=15.0, timeout=2.0, warmup=3.0, simulator=simulator)
    except Exception as e:
        print(f"Error capturing data: {e}")
        traceback.print_exc()
        return {
            "passed": False,
            "duration": round(time.time() - t0, 2),
            "error": f"Capture failed: {str(e)}",
        }

    if not msgs:
        return {
            "passed": False,
            "duration": round(time.time() - t0, 2),
            "detail": "No image received within 15 s",
        }

    # Process the first image with error handling
    try:
        arr = _img_to_numpy(msgs[0])
        non_zero = int(np.count_nonzero(arr))
        
        try:
            save_path = _save_image(arr, _get_save_dir(sensor, "camera_data_received"), "frame_0")
        except Exception as e:
            print(f"Error saving image: {e}")
            save_path = None
            
    except Exception as e:
        print(f"Error processing image: {e}")
        traceback.print_exc()
        return {
            "passed": False,
            "duration": round(time.time() - t0, 2),
            "error": f"Image processing failed: {str(e)}",
            "frames_recv": len(msgs),
        }

    if progress_cb:
        progress_cb(100)

    return {
        "passed": non_zero > 0,
        "duration": round(time.time() - t0, 2),
        "resolution": f"{msgs[0].width}x{msgs[0].height}",
        "encoding": msgs[0].encoding,
        "frames_recv": len(msgs),
        "non_zero_px": non_zero,
        "saved_to": save_path if save_path else None,
    }


def camera_depth_accuracy(simulator, sensor, progress_cb=None) -> dict:
    """
    Method C1 — Depth Accuracy.
    Pass criterion: relative error δ ≤ 2 % at Z_true = 3.0 m.
    """
    from sensor_msgs.msg import Image
    import numpy as np
    import rospy

    Z_TRUE    = 3.0
    PASS_PCT  = 2.0

    # Debug: print sensor info
    print(f"\n{'='*60}")
    print(f"Depth Accuracy Test")
    print(f"Sensor name: {sensor.sensor_name}")
    print(f"Sensor type: {sensor.sensor_type}")
    print(f"SDF path: {sensor.sdf_path}")
    print(f"Topic: {sensor.topic}")
    print(f"{'='*60}")

    world = _world_from_db(sensor, "camera_depth_accuracy") or Worlds.CAMERA_DEPTH_ACCURACY.value
    print(f"Using world: {world}")
    
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    if progress_cb:
        progress_cb(20)

    # Wait longer for depth camera to initialize
    print("Waiting 5 seconds for depth camera to initialize...")
    time.sleep(5)

    # Check what topics are available
    print("Checking available topics...")
    topics = rospy.get_published_topics()
    image_topics = [t for t, t_type in topics if 'image' in t_type or 'Image' in t_type]
    print(f"Image topics found: {image_topics}")
    
    if sensor.topic not in [t for t, _ in topics]:
        print(f"WARNING: Topic {sensor.topic} not found!")
        print(f"Available topics: {[t for t, _ in topics[:10]]}")

    t0 = time.time()
    
    # Try with a longer window and no timeout
    print(f"Capturing depth data for 10 seconds...")
    msgs = sensor.capture_data(Image, window=10.0, timeout=1.0, warmup=2.0, simulator=simulator)

    if not msgs:
        print("No messages received!")
        # Try one more time with a different approach
        print("Retrying with direct ROS wait_for_message...")
        try:
            msg = rospy.wait_for_message(sensor.topic, Image, timeout=5.0)
            msgs = [msg]
            print(f"Success! Received one message")
        except Exception as e:
            print(f"Direct wait failed: {e}")
            raise RuntimeError("No depth frames received")

    print(f"Received {len(msgs)} depth frames")

    if progress_cb:
        progress_cb(60)

    # Process frames
    frames = []
    for i, m in enumerate(msgs):
        print(f"Frame {i}: encoding={m.encoding}, size={m.width}x{m.height}")
        try:
            frame = _img_to_numpy(m)
            print(f"  Converted shape: {frame.shape}")
            if len(frame.shape) >= 2:
                frames.append(frame[:, :, 0] if len(frame.shape) == 3 else frame)
        except Exception as e:
            print(f"  Error converting frame: {e}")

    if not frames:
        raise RuntimeError("Could not convert any frames to numpy")

    print(f"Stacking {len(frames)} frames...")
    depth_map = np.nanmedian(np.stack(frames, axis=0), axis=0)
    print(f"Depth map shape: {depth_map.shape}")
    
    cy, cx = depth_map.shape[0] // 2, depth_map.shape[1] // 2
    print(f"Center pixel: ({cy}, {cx})")
    
    patch = depth_map[cy - 5:cy + 5, cx - 5:cx + 5]
    print(f"Patch shape: {patch.shape}")
    print(f"Patch stats: min={np.nanmin(patch):.3f}, max={np.nanmax(patch):.3f}, median={np.nanmedian(patch):.3f}")
    
    z_measured = float(np.nanmedian(patch))
    abs_err = abs(z_measured - Z_TRUE)
    rel_err = abs_err / Z_TRUE * 100.0

    save_path = _save_image(
        depth_map[:, :, None] if depth_map.ndim == 2 else depth_map,
        _get_save_dir(sensor, "camera_depth_accuracy"), "depth_frame"
    )

    if progress_cb:
        progress_cb(100)

    print(f"Depth measured: {z_measured:.3f}m, error: {rel_err:.2f}%")
    print(f"{'='*60}\n")

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


def _basic_(simulator, sensor, progress_cb=None) -> dict:
    import os
    import time
    import rospy
    import rostopic
 
    def _progress(value):
        if progress_cb:
            try:
                progress_cb(value)
            except Exception:
                return False
        return True
 
    t0 = time.time()
 
    # ── Sensor identity ───────────────────────────────────────────────────────
    result = {
        "passed":         False,
 
        # Sensor identity
        "sensor_name":    getattr(sensor, "sensor_name",  "unknown"),
        "sensor_type":    getattr(sensor, "sensor_type",  "unknown"),
        "sdf_path":       getattr(sensor, "sdf_path",     ""),
        "sdf_exists":     os.path.exists(getattr(sensor, "sdf_path", "") or ""),
        "topics":         list(getattr(sensor, "topics",  [])),
        "topic":          getattr(sensor, "topic",        ""),
 
        # Sensor params (all fields)
        "params":         dict(getattr(sensor, "params",  {})),
 
        # Runtime
        "world_path":     "",
        "world_exists":   False,
        "available_topics": [],
        "topic_found":    False,
        "message_type":   None,
        "data_received":  False,
        "duration":       0.0,
        "error":          None,
    }
 
    # ── World path ────────────────────────────────────────────────────────────
    world_path = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/resources/worlds/rfid/overlap_tags.world"
    world_exists = os.path.exists(world_path)
 
    if not world_exists:
        fallback = "/tmp/default_world.world"
        with open(fallback, "w") as f:
            f.write(
                '<?xml version="1.0"?>\n'
                '<sdf version="1.6">\n'
                '  <world name="default">\n'
                '    <include><uri>model://sun</uri></include>\n'
                '    <include><uri>model://ground_plane</uri></include>\n'
                '  </world>\n'
                '</sdf>\n'
            )
        world_path = fallback
        world_exists = True
 
    result["world_path"]  = world_path
    result["world_exists"] = world_exists
 
    # ── Open Gazebo scene ─────────────────────────────────────────────────────
    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"]    = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result
 
    if not _progress(20):
        result["cancelled"] = True
        result["duration"]  = round(time.time() - t0, 2)
        return result
 
    # ── Wait for plugins ──────────────────────────────────────────────────────
    time.sleep(3)
 
    # ── Discover published topics ─────────────────────────────────────────────
    all_topics = rospy.get_published_topics()
    visible = [t for t, _ in all_topics if not t.startswith("/rosout")]
    result["available_topics"] = visible
 
    topic_found = any(t == sensor.topic for t, _ in all_topics)
    result["topic_found"] = topic_found
 
    if not topic_found:
        result["error"]    = f"Topic '{sensor.topic}' not found in published topics"
        result["duration"] = round(time.time() - t0, 2)
        return result
 
    if not _progress(40):
        result["cancelled"] = True
        result["duration"]  = round(time.time() - t0, 2)
        return result
 
    # ── Resolve message class ─────────────────────────────────────────────────
    msg_class, _, _ = rostopic.get_topic_class(sensor.topic)
 
    if msg_class is None:
        _FALLBACK_TYPES = {
            "camera":      "sensor_msgs/Image",
            "image":       "sensor_msgs/Image",
            "depth":       "sensor_msgs/Image",
            "laser":       "sensor_msgs/LaserScan",
            "scan":        "sensor_msgs/LaserScan",
            "imu":         "sensor_msgs/Imu",
            "contact":     "gazebo_msgs/ContactsState",
            "bumper":      "gazebo_msgs/ContactsState",
            "pointcloud":  "sensor_msgs/PointCloud2",
            "sonar":       "sensor_msgs/Range",
            "gps":         "sensor_msgs/NavSatFix",
            "joint":       "sensor_msgs/JointState",
            "temperature": "sensor_msgs/Temperature",
            "fluid":       "sensor_msgs/FluidPressure",
            "magnetic":    "sensor_msgs/MagneticField",
        }
        sensor_type   = getattr(sensor, "sensor_type", "").lower()
        msg_type_name = _FALLBACK_TYPES.get(sensor_type)
        if msg_type_name:
            try:
                pkg, name  = msg_type_name.split("/")
                module     = __import__(f"{pkg}.msg", fromlist=[name])
                msg_class  = getattr(module, name)
            except Exception:
                pass
 
    if msg_class is None:
        result["error"]    = "Could not determine message type for topic"
        result["duration"] = round(time.time() - t0, 2)
        return result
 
    result["message_type"] = msg_class.__name__
 
    if not _progress(60):
        result["cancelled"] = True
        result["duration"]  = round(time.time() - t0, 2)
        return result
 
    # ── Capture one message ───────────────────────────────────────────────────
    try:
        msg = rospy.wait_for_message(sensor.topic, msg_class, timeout=10.0)
        result["data_received"] = True
        result["passed"]        = True
 
        # Common header fields
        if hasattr(msg, "header"):
            result["frame_id"]  = msg.header.frame_id
            result["timestamp"] = (
                round(msg.header.stamp.to_sec(), 3)
                if msg.header.stamp else None
            )
 
        # Image
        if hasattr(msg, "encoding"):
            result["width"]    = msg.width
            result["height"]   = msg.height
            result["encoding"] = msg.encoding
            result["data_size_bytes"] = len(msg.data)
 
        # Laser scan
        elif hasattr(msg, "ranges"):
            result["num_ranges"] = len(msg.ranges)
            result["angle_min"]  = round(msg.angle_min, 4)
            result["angle_max"]  = round(msg.angle_max, 4)
            result["range_min"]  = round(msg.range_min, 4)
            result["range_max"]  = round(msg.range_max, 4)
 
        # IMU
        elif hasattr(msg, "angular_velocity") and hasattr(msg, "linear_acceleration"):
            av = msg.angular_velocity
            la = msg.linear_acceleration
            result["angular_velocity"]    = [round(av.x, 4), round(av.y, 4), round(av.z, 4)]
            result["linear_acceleration"] = [round(la.x, 4), round(la.y, 4), round(la.z, 4)]
 
        # Contact / bumper
        elif hasattr(msg, "states"):
            result["contact_count"] = len(msg.states)
            if msg.states:
                result["first_contact"] = {
                    "collision1": msg.states[0].collision1_name,
                    "collision2": msg.states[0].collision2_name,
                }
 
        # PointCloud2
        elif hasattr(msg, "point_step"):
            result["width"]        = msg.width
            result["height"]       = msg.height
            result["point_step"]   = msg.point_step
            result["fields"]       = [f.name for f in msg.fields]
 
        # NavSatFix (GPS)
        elif hasattr(msg, "latitude"):
            result["latitude"]  = round(msg.latitude,  6)
            result["longitude"] = round(msg.longitude, 6)
            result["altitude"]  = round(msg.altitude,  3)
 
        # Range (sonar)
        elif hasattr(msg, "range"):
            result["range"]     = round(msg.range, 4)
            result["min_range"] = round(msg.min_range, 4)
            result["max_range"] = round(msg.max_range, 4)
 
        # Temperature
        elif hasattr(msg, "temperature"):
            result["temperature"] = round(msg.temperature, 3)
 
        # MagneticField
        elif hasattr(msg, "magnetic_field"):
            mf = msg.magnetic_field
            result["magnetic_field"] = [round(mf.x, 6), round(mf.y, 6), round(mf.z, 6)]
 
    except rospy.ROSException:
        result["error"] = f"Timeout: no message received on '{sensor.topic}' within 10 s"
    except Exception as exc:
        result["error"] = str(exc)
 
    _progress(100)
 
    result["duration"] = round(time.time() - t0, 2)
 
    # ── Notify UI ─────────────────────────────────────────────────────────────
    if simulator:
        try:
            simulator.notify_capture(
                {
                    "sensor_type": result["sensor_type"],
                    "sensor_name": result["sensor_name"],
                    "topic":       result["topic"],
                    "success":     result["passed"],
                },
                None,
            )
        except Exception:
            pass
 
    return result
# ── TESTS registry ────────────────────────────────────────────────────────────

TESTS: dict[str, callable] = {
    # Generic (assign to any sensor type)
    "_basic_":                       _basic_,
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
    # Tactile (libgazebo_ros_bumper.so)
    "tactile_min_force_threshold": tactile_min_force_threshold,
    "tactile_response_uniformity": tactile_response_uniformity,
}