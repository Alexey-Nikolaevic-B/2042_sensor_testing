import math
import time
import logging

logger = logging.getLogger(__name__)

def _PoseStamped():
    from geometry_msgs.msg import PoseStamped
    return PoseStamped


_RFID_WORLDS: dict[str, str] = {
    "rfid_max_stable_read_distance": "rfid/change_distance.world",
    "rfid_min_stable_read_distance": "rfid/change_distance.world",
    "rfid_mass_read":                "rfid/mass_read.world",
    "rfid_overlap_tags":             "rfid/overlap_tags.world",
    "rfid_angle_dependence":         "rfid/angle_dependence.world",
    "rfid_move_tags":                "rfid/move_tags.world",
    "rfid_antenna_rotation":         "rfid/antenna_rotation.world",
}


def _world_path(sensor, func_name: str) -> str:
    """Return the world file path for a test function."""
    from config import CONFIG
    base = CONFIG["WORLDS_PATH"]
    rel  = _RFID_WORLDS.get(func_name, "")
    return f"{base}{rel}" if rel else ""


def _rfid_read(sensor, simulator, window: float = 3.0) -> dict:
    """Capture PoseStamped messages on the sensor's primary topic."""
    return sensor.capture_data(_PoseStamped(), window=window, simulator=simulator)


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
# 3. Restart the app. Your test will appear in the list inside
#    Add Sensor Type → Tests panel. Assign a world .sdf file there.
#
#
# ── simulator API ─────────────────────────────────────────────────────────────
#
#   simulator.open_scene(world_path, sdf_path) -> bool
#       Start Gazebo with given world + sensor SDF. Returns False on failure.
#
#   simulator.wait_for_model_spawn(model_name, timeout_sec) -> bool
#       Block until the model appears in the simulation.
#
#   simulator.set_pose(model_name, x, y, z,
#                      quaternion=None, linear_velocity=None)
#       Teleport or set velocity of a Gazebo model.
#
#   simulator.kill()
#       Kill the Gazebo process.
#
#
# ── sensor fields ─────────────────────────────────────────────────────────────
#
#   sensor.sensor_type  str
#   sensor.sensor_name  str
#   sensor.sdf_path     str
#   sensor.topic        str    ROS topic this sensor publishes to
#   sensor.params       dict   SDF params extracted on add, e.g. {"rzero": "3"}
#
#   sensor.read_params_from_sdf(["rzero", "width"]) -> dict
#       Re-read current XML tag values from the SDF file.
#
#   sensor.write_params_to_sdf({"rzero": "5"})
#       Write values back into the SDF file in-place.
#
#
# ── progress_cb ───────────────────────────────────────────────────────────────
#
#   Call progress_cb(0-100) to update the UI progress bar.
#   Catch StopIteration to handle user cancel:
#
#       if progress_cb:
#           try:
#               progress_cb(50)
#           except StopIteration:
#               return {"passed": False, "cancelled": True}
#
#
# ── return value ──────────────────────────────────────────────────────────────
#
#   Must return a dict with at least {"passed": bool}.
#   All extra keys are shown in the results panel:
#       return {"passed": True, "duration": 12.4, "measured": 3.0}
#
#
# ── reading from a ROS topic ──────────────────────────────────────────────────
#
#   results = sensor.capture_data(_PoseStamped(), window=3.0, simulator=simulator)
#   # returns {frame_id: pose} for every message received in the window
#
#
# ── example test ──────────────────────────────────────────────────────────────
#
# def example_test(simulator, sensor, progress_cb=None) -> dict:
#     import rospy
#     from geometry_msgs.msg import PoseStamped
#     import src.database.sensor_storage as db
#
#     world_path = next(
#         (t["world_path"] for t in db.get_type_tests(sensor.sensor_type)
#          if t["func_name"] == "example_test"), ""
#     )
#
#     if not simulator.open_scene(world_path, sensor.sdf_path):
#         raise RuntimeError("failed to open Gazebo scene")
#
#     tag = "my_model"
#     if not simulator.wait_for_model_spawn(tag, timeout_sec=30):
#         raise RuntimeError(f"{tag} did not spawn")
#
#     if progress_cb:
#         progress_cb(30)
#
#     simulator.set_pose(tag, x=1.0, y=0.0, z=0.0)
#     time.sleep(0.1)
#
#     results = {}
#     deadline = time.time() + 3.0
#     while time.time() < deadline:
#         try:
#             msg = rospy.wait_for_message(sensor.topic, PoseStamped, timeout=0.25)
#             results[msg.header.frame_id] = msg.pose
#         except rospy.ROSException:
#             continue
#
#     if progress_cb:
#         progress_cb(100)
#
#     return {
#         "passed":          tag in results,
#         "detected_count":  len(results),
#     }



# ── RFID tests ────────────────────────────────────────────────────────────────

def rfid_max_stable_read_distance(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    read_distance = float(sensor.params.get("rzero", 3))

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        f.write("fix1 1 0.5 0 0\n")

    if not simulator.open_scene(_world_path(sensor, "rfid_max_stable_read_distance"), sensor.sdf_path):
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
            if tag in sensor.capture_data(_PoseStamped(), window=3, simulator=simulator):
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

    if not simulator.open_scene(_world_path(sensor, "rfid_min_stable_read_distance"), sensor.sdf_path):
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
            if tag in sensor.capture_data(_PoseStamped(), window=2, simulator=simulator):
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

    if not simulator.open_scene(_world_path(sensor, "rfid_mass_read"), sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    for i in range(tags_count):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")
        if progress_cb:
            progress_cb(int((i + 1) / tags_count * 50))

    t0   = time.time()
    data = sensor.capture_data(_PoseStamped(), window=20, simulator=simulator)
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
    world         = _world_path(sensor, "rfid_overlap_tags")

    for step, distance in enumerate(distances):
        with open(CONFIG["RFID_MAP_PATH"], "w") as f:
            for i in range(tags_count):
                x = radius * math.cos(distance / radius * i)
                y = radius * math.sin(distance / radius * i)
                f.write(f"fix{i+1} {i+1} {x} {y} 0\n")

        if not simulator.open_scene(world, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
                raise RuntimeError(f"tag {i+1} not spawned")

        data = sensor.capture_data(_PoseStamped(), window=5, simulator=simulator)
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

    if not simulator.open_scene(_world_path(sensor, "rfid_angle_dependence"), sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    for i in range(len(angles)):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")
        if progress_cb:
            progress_cb(int((i + 1) / len(angles) * 50))

    t0   = time.time()
    data = sensor.capture_data(_PoseStamped(), window=20, simulator=simulator)
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
    world         = _world_path(sensor, "rfid_move_tags")

    for step, velocity in enumerate(velocities):
        with open(CONFIG["RFID_MAP_PATH"], "w") as f:
            f.write(f"fix1 1 {start_dist} 0 0\n")
        if not simulator.open_scene(world, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        if not simulator.wait_for_model_spawn("rfid_tag1", 30):
            raise RuntimeError("tag not spawned")

        simulator.set_pose("rfid_tag1", x=start_dist, y=0, z=0, linear_velocity=velocity)
        data = _rfid_read(sensor, simulator, window=factor * read_distance / velocity.x * 2)
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

    if not simulator.open_scene(_world_path(sensor, "rfid_antenna_rotation"), sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    for i in range(tags_count):
        if not simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            raise RuntimeError(f"tag {i+1} not spawned")

    t0 = time.time()
    for step, angle in enumerate(angles):
        q = Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
        simulator.set_pose("rfid_antenna", x=0, y=0, z=0, quaternion=q)
        time.sleep(0.01)
        data = sensor.capture_data(_PoseStamped(), window=7, simulator=simulator)
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


# ── Add your tests here ───────────────────────────────────────────────────────

TESTS: dict[str, callable] = {
    "rfid_max_stable_read_distance": rfid_max_stable_read_distance,
    "rfid_min_stable_read_distance": rfid_min_stable_read_distance,
    "rfid_mass_read":                rfid_mass_read,
    "rfid_overlap_tags":             rfid_overlap_tags,
    "rfid_angle_dependence":         rfid_angle_dependence,
    "rfid_move_tags":                rfid_move_tags,
    "rfid_antenna_rotation":         rfid_antenna_rotation,
}