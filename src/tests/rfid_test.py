import time
import rospy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def _wait(topic, msg_type, timeout=1.0):
    """Одна попытка получить сообщение с таймаутом. None если не пришло."""
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        return None


def _wait_until(fn, timeout_s=30.0, period_s=0.2) -> bool:
    """Ждём, пока fn() вернёт True."""
    t0 = time.time()
    while time.time() - t0 < timeout_s and not rospy.is_shutdown():
        if fn():
            return True
        time.sleep(period_s)
    return False


def _set_pose(set_state, model, x, y, z):
    st = ModelState()
    st.model_name = model
    st.reference_frame = "world"
    st.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    resp = set_state(st)
    if not resp.success:
        raise RuntimeError(resp.status_message)


def _saw_tag_once(tag_name: str, timeout_s: float):
    """Ждём одно сообщение /detected_tags именно про tag_name."""
    t_end = time.time() + timeout_s
    while time.time() < t_end and not rospy.is_shutdown():
        msg = _wait("/detected_tags", PoseStamped, timeout=min(0.3, t_end - time.time()))
        if msg and msg.header.frame_id == tag_name:
            return msg
    return None


def change_distance_test(simulator, CONFIG, sensor_type, sensor):
    WORLDS_PATH = CONFIG["WORLDS_PATH"]
    SENSORS_PATH = CONFIG["SENSORS_PATH"]
    sensor_name = sensor["name"]

    world_path = f"{WORLDS_PATH}rfid/test_0.world"
    sensor_model_path = f"{SENSORS_PATH}{sensor_type}/{sensor_name}.sdf"

    if not simulator.open_scene(world_path, sensor_model_path):
        return False

    # 1) ждём /clock (если use_sim_time)
    if not _wait_until(lambda: _wait("/clock", Clock, 1.0) is not None, timeout_s=60):
        return {"error": "no /clock (Gazebo not publishing simulated time)"}

    # 2) ждём спавна модели
    tag = "rfid_tag1"
    if not _wait_until(lambda: (lambda ms: ms and tag in ms.name)(_wait("/gazebo/model_states", ModelStates, 1.0)),
                      timeout_s=60):
        return {"error": f"tag model '{tag}' not spawned"}

    # 3) готовим сервис перемещения
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    y, z = 0.0, 0.25
    reset_x = 50.0

    results = {"tag": tag, "max_detected_distance": None, "steps": []}
    max_dist = None

    dist = 0.5
    while dist <= 10.0 + 1e-9 and not rospy.is_shutdown():
        # reset: чтобы не зависеть от event_only
        _set_pose(set_state, tag, reset_x, y, z)
        time.sleep(0.1)

        _set_pose(set_state, tag, dist, y, z)

        msg = _saw_tag_once(tag, timeout_s=1.0)
        detected = msg is not None

        results["steps"].append({
            "distance": dist,
            "detected": detected,
            "pose": {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z} if msg else
                    {"x": None, "y": None, "z": None}
        })

        if detected:
            max_dist = dist

        dist = round(dist + 0.5, 3)

    results["max_detected_distance"] = max_dist
    return results
