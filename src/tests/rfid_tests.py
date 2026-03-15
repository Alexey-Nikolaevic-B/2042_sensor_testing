"""RFID sensor tests."""
import math
import time

from ._common import _PoseStamped, Worlds


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
