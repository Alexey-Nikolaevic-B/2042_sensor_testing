import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def depth_perception_test(simulator, CONFIG, sensor_type, sensor):
    """
    Тест точности измерения глубины камеры на дистанциях 1м, 3м, 5м.
    """
    WORLDS_PATH = CONFIG['WORLDS_PATH']
    SENSORS_PATH = CONFIG['SENSORS_PATH']
    
    # 1. ПОДГОТОВКА ДАННЫХ
    sensor_name = sensor['name']
    worlds = ['cube_1', 'cube_3', 'cube_5']
    topic = '/sensor/depth/image_raw'

    sensor_data = []

    # 2. СБОР ДАННЫХ ИЗ GAZEBO
    for world in worlds:
        world_path = f'{WORLDS_PATH}depth_perception_test/{world}.world'
        camera_model_path = f'{SENSORS_PATH}{sensor_type}/{sensor_name}.sdf'

        simulator.open_scene(world_path, camera_model_path)
        msg = simulator.receive_sensor_data(topic)
        sensor_data.append(msg)

    # 3. ОБРАБОТКА И АНАЛИЗ ДАННЫХ
    distances = [1.0, 3.0, 5.0]
    
    results = []
    for distance, msg in zip(distances, sensor_data):
        encoding = msg.encoding
    
        try:
            if encoding == '32FC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif encoding == '16UC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_image = depth_image.astype(np.float32) * 0.001
            else:
                results.append({'distance': distance, 'error': 'Invalid encoding'})
                continue
        except CvBridgeError as e:
            results.append({'distance': distance, 'error': 'CV Bridge Error'})
            continue
        
        if depth_image is None:
            results.append({'distance': distance, 'error': 'No image data'})
            continue

        h, w = depth_image.shape
        z = depth_image[h // 2, w // 2]

        if np.isnan(z) or z == 0:
            results.append({'distance': distance, 'error': 'Invalid measurement'})
        else:
            abs_err = abs(z - distance)
            rel_err = abs_err / distance * 100.0
            results.append({
                'distance': distance,
                'measured': z,
                'abs_error': abs_err,
                'rel_error': rel_err
            })

    return results