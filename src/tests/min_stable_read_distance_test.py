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

def min_stable_read_distance_test(simulator, CONFIG, sensor_type, sensor):
    WORLDS_PATH = CONFIG["WORLDS_PATH"]
    SENSORS_PATH = CONFIG["SENSORS_PATH"]
    sensor_name = sensor["name"]

    world_path = f"{WORLDS_PATH}rfid/change_distance.world"
    sensor_model_path = f"{SENSORS_PATH}{sensor_type}/{sensor_name}.sdf"

    dist = 1

    with open(CONFIG['RFID_MAP_PATH'], 'w') as f:
        f.write(f'fix1 {dist} 0 0 0')
        f.flush()

    if not simulator.open_scene(world_path, sensor_model_path):
        return False

    tag_name = "rfid_tag1"

    t0 = time.time()
    while (time.time() - t0 < 30):
        try:
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
            if tag_name in msg.name:
                break
        except rospy.ROSException:
            msg = None


    rospy.wait_for_service("/gazebo/set_model_state", timeout=5)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    results = {"tag": tag_name, "min_detected_distance": None}
    reset_distance = 100
    is_tag_detected = True

    while is_tag_detected and dist >= 0:

        detected_count = 0

        # делаем 5 попыток считывания метки
        for _ in range(5):
            try:
                # перемещаем метку далеко, чтобы сбросить попытку
                _set_pose(set_state, tag_name, reset_distance, 0, 0)
                time.sleep(0.001)
                # перемещаем метку на тестовую дистанцию
                _set_pose(set_state, tag_name, dist, 0, 0)
                msg = rospy.wait_for_message('/detected_tags', PoseStamped, timeout=0.1)
                if msg.header.frame_id == tag_name:
                    detected_count += 1
            except:
                continue

        is_tag_detected = bool(detected_count >= 4)

        if not is_tag_detected:
            results['min_detected_distance'] = round(dist + 0.01, 1)
            is_tag_detected = False

        # шаг на 1 см назад
        dist = round(dist - 0.01, 3)

    print("test ended", flush=True)
    return results