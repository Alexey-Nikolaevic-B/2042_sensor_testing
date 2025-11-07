import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError

class TestManager:
    def __init__(self, config, ros, node, gazebo):
        self.BASE_WORLD_PATH = 'catkin_ws/src/scenario_test_pkg/worlds/base_world.world'
        self.SENSOR_PKG = 'scenario_test_pkg'
        self.LAUNCH_FILE = 'scenario.launch'
        self.CATKIN_SETUP_DIR = 'catkin_ws/devel/setup.bash'
        self.TEST_SCRIPT_PATH = 'catkin_ws/src/scenario_test_pkg/scripts/get_sensor_data.py'
        self.WORLDS_PATH = 'resources/worlds/'
        self.SENSORS_PATH = 'resources/sensors/'
        self.MESSAGE_TIMEOUT = 10
        self.SAVE_SENSOR_DATA = True
        self.SAVE_DIR = 'captured_data'

        self.ros = ros
        self.node = node
        self.gazebo = gazebo

    def test_sensor(self, sensor_type: str, sensor: list):
        result = []

        self.sensor_name = sensor['name']
        self.scene = sensor['test_world']
        self.world_path = f'{self.WORLDS_PATH}{self.scene}.world'
        self.camera_model_path = f'{self.SENSORS_PATH}{sensor_type}/{self.sensor_name}.sdf'

        self.gazebo.generate_world(self.world_path, self.camera_model_path, self.BASE_WORLD_PATH)
        self.gazebo.launch(self.CATKIN_SETUP_DIR, self.SENSOR_PKG, self.LAUNCH_FILE)

        tests_to_run = list(sensor['tests'].keys())

        print('\n========================  Tests  =========================')
        for test_name in tests_to_run:
            if test_name in test_functions:
                test_result = test_functions[test_name](self.node)
                result.append(test_result)
            else:
                print(f"\n⚠️  Warning: Test '{test_name}' not found")

        # self.gazebo.clear_world() # сделать загрузку новых датчиков без закрытия мира.

        self.gazebo.kill()
        time.sleep(2)
        print('\n==========================================================')
        return result

def depth_perception_test(node):  #TODO: это только примеры, нужны тесты
    print(f"\nТест на глубину")
    print("----------------------------------------------------------")
    print(f"{'Z_true(m)':>10} {'Z_meas(m)':>10} {'Δ_abs(m)':>10} {'Δ_rel(%)':>10}")
    print("----------------------------------------------------------")

    topic = '/sensor/depth/image_raw'
    msg = node.get_sensor_data(topic)
    encoding = msg.encoding
    
    distances = [1.0, 3.0, 5.0]
    
    for distance in distances:
        try:
            if encoding == '32FC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif encoding == '16UC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_image = depth_image.astype(np.float32) * 0.001
            else:
                return None
        except CvBridgeError as e:
            return None
        if depth_image is None:
            return None
        h, w = depth_image.shape
        z = depth_image[h // 2, w // 2]

        if z is None:
            print(f"{distances:>10.2f} {'N/A':>10} {'N/A':>10} {'N/A':>10}")
        else:
            abs_err = abs(z - distance)
            rel_err = abs_err / distance * 100.0
            print(f"{distance:>10.2f} {z:>10.2f} {abs_err:>10.2f} {rel_err:>10.2f}")
    return 0 #TODO: Что-то должен возращать, но пока лень

def view_angle_test(node):
    print(f"\nТест на угол обзора")
    print("----------------------------------------------------------")
    print(f"Нужно создать сам тест")
    print("----------------------------------------------------------")

    topic = '/mono_camera/image_raw'
    msg = node.get_sensor_data(topic)

    return 0 #TODO: Что-то должен возращать, но пока лень

test_functions = {
    "Тест на угол обзора": view_angle_test,
    "Тест на точность глубины": depth_perception_test,
}