import time
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def _set_pose(set_state, model, x, y, z):
    state = ModelState()
    state.model_name = model
    state.reference_frame = "world"
    state.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    response = set_state(state)
    if not response.success:
        raise RuntimeError(response.status_message)


def change_distance_test(simulator, CONFIG, sensor_type, sensor):
    WORLDS_PATH = CONFIG["WORLDS_PATH"]
    SENSORS_PATH = CONFIG["SENSORS_PATH"]
    sensor_name = sensor["name"]

    world_path = f"{WORLDS_PATH}rfid/change_distance.world"
    sensor_model_path = f"{SENSORS_PATH}{sensor_type}/{sensor_name}.sdf"

    # плагин читает файл map.txt и по нему создает метки в gazebo (такая уж особенность работы)
    with open(CONFIG['RFID_MAP_PATH'], 'w') as f:
        f.write('fix1 1 0.5 0.0 0')
        f.flush()

    if not simulator.open_scene(world_path, sensor_model_path):
        return False

    tag_name = "rfid_tag1"

    # ждем спавна rfid_tag1
    t0 = time.time()
    while (time.time() - t0 < 30):
        try:
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
            if tag_name in msg.name:
                break
        except rospy.ROSException:
            msg = None

    # создаем сервис для перемещения rfid_tag1
    rospy.wait_for_service("/gazebo/set_model_state", timeout=5)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    results = {"tag": tag_name, "max_detected_distance": None, "steps": []}
    max_dist = None
    reset_distance = 100
    dist = 0.5

    while dist <= 10.0 + 1e-9:

        detected_count = 0

        # делаем 10 попыток считывания метки
        tag_msg = None
        for _ in range(10):
            try:
                # перемещаем метку далеко, чтобы сбросить попытку
                _set_pose(set_state, tag_name, reset_distance, 0, 0)
                time.sleep(0.025)
                # перемещаем метку на тестовую дистанцию
                _set_pose(set_state, tag_name, dist, 0, 0)
                msg = rospy.wait_for_message('/detected_tags', PoseStamped, timeout=0.1)
                tag_msg = msg
                if msg.header.frame_id == tag_name:
                    detected_count += 1
            except:
                continue

        is_tag_detected = bool(detected_count >= 8)

        results["steps"].append({
            "distance": dist,
            "detected": is_tag_detected,
            "pose": {"x": tag_msg.pose.position.x, "y": tag_msg.pose.position.y, "z": tag_msg.pose.position.z} if tag_msg else
                    {"x": None, "y": None, "z": None}
        })

        if is_tag_detected:
            max_dist = dist

        dist = round(dist + 0.5, 1)

    results["max_detected_distance"] = max_dist
    return results
