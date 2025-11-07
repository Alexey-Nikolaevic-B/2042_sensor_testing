import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError
import config

class TestManager:
    BASE_WORLD_PATH = config.BASE_WORLD_PATH
    SENSOR_PKG = config.SENSOR_PKG
    LAUNCH_FILE = config.LAUNCH_FILE
    CATKIN_SETUP_DIR = config.CATKIN_SETUP_DIR
    WORLDS_PATH = config.WORLDS_PATH
    SENSORS_PATH = config.SENSORS_PATH

    def __init__(self, ros, node, gazebo):
        self.ros = ros
        self.node = node
        self.gazebo = gazebo

    def test_sensor(self, sensor_type: str, sensor: list):
        result = []
        tests_to_run = list(sensor['tests'].keys())
        print('\n========================  Tests  =========================')
        for test_name in tests_to_run:
            if test_name in self.test_functions:
                test_result = self.test_functions[test_name](self, sensor_type, sensor)
                result.append(test_result)
            else:
                print(f"\n⚠️  Warning: Test '{test_name}' not found")

        self.gazebo.kill()
        time.sleep(1)
        print('\n==========================================================')
        return result

    def depth_perception_test(self, sensor_type, sensor):
        """
        Тест точности измерения глубины камеры на дистанциях 1м, 3м, 5м.
        """
        
        # 1. ПОДГОТОВКА ДАННЫХ
        sensor_name = sensor['name']
        test_name = 'depth_perception_test'
        
        worlds = ['cube_1', 'cube_3', 'cube_5']
        topic = '/sensor/depth/image_raw'  # ROS-топик с данными глубины
        sensor_data = []  # Здесь будут храниться данные с датчика

        # 2. СБОР ДАННЫХ ИЗ GAZEBO
        print('Collecting data from gazebo: ')
        for world in worlds:
            world_path = f'{self.WORLDS_PATH}{test_name}/{world}.world'
            camera_model_path = f'{self.SENSORS_PATH}{sensor_type}/{sensor_name}.sdf'
            self.gazebo.generate_world(world_path, camera_model_path, self.BASE_WORLD_PATH) # Создаем мир, который запускает газибо. Объединяя мир и камеру.
            self.gazebo.launch(self.CATKIN_SETUP_DIR, self.SENSOR_PKG, self.LAUNCH_FILE) # Запускаем газибо со сценой
            
            msg = self.node.get_sensor_data(topic) # Тут получаем данные из сенсора по топику
            sensor_data.append(msg)
            time.sleep(5)
            self.gazebo.kill()
        print('----------------------------------------------------------')

        # 3. ОБРАБОТКА И АНАЛИЗ ДАННЫХ
        distances = [1.0, 3.0, 5.0]
        
        print(f"\n\n{'Distance':>10} {'Measured':>10} {'Abs Error':>11} {'Rel Error %':>10}")
        print('----------------------------------------------------------')
        for distance, msg in zip(distances, sensor_data):
            encoding = msg.encoding
        
            try:
                if encoding == '32FC1':
                    depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='32FC1')
                elif encoding == '16UC1':
                    depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    depth_image = depth_image.astype(np.float32) * 0.001
                else:
                    print(f"{distance:>10.2f} {'Invalid encoding':>30}")
                    continue
            except CvBridgeError as e:
                print(f"{distance:>10.2f} {'CV Bridge Error':>30}")
                continue
            
            if depth_image is None:
                print(f"{distance:>10.2f} {'No image data':>30}")
                continue

            h, w = depth_image.shape
            z = depth_image[h // 2, w // 2]

            if np.isnan(z) or z == 0:
                print(f"{distance:>10.2f} {'N/A':>10} {'N/A':>10} {'N/A':>10}")
            else:
                abs_err = abs(z - distance)
                rel_err = abs_err / distance * 100.0
                print(f"{distance:>10.2f} {z:>10.2f} {abs_err:>10.2f} {rel_err:>10.2f}")

        return (abs_err, rel_err)
    
    test_functions = {
        "Тест на точность глубины": depth_perception_test,
    }