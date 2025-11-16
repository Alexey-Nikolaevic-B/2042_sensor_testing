import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def depth_perception_test(simulator, config, sensor_type, sensor):
    """
    Тест точности измерения глубины камеры на дистанциях 1м, 3м, 5м.
    """
    WORLDS_PATH = config.get('WORLDS_PATH')
    SENSORS_PATH = config.get('SENSORS_PATH')
    BASE_WORLD_PATH = config.get('BASE_WORLD_PATH')
    
    # 1. ПОДГОТОВКА ДАННЫХ
    sensor_name = sensor['name']
    worlds = ['cube_1', 'cube_3', 'cube_5']
    topic = '/sensor/depth/image_raw'
    
    sensor_data = []

    # 2. СБОР ДАННЫХ ИЗ GAZEBO
    print('Collecting data from gazebo: ')
    for world in worlds:
        world_path = f'{WORLDS_PATH}depth_perception_test/{world}.world'
        camera_model_path = f'{SENSORS_PATH}{sensor_type}/{sensor_name}.sdf'

        simulator.generate_world(world_path, camera_model_path, BASE_WORLD_PATH)
        simulator.open_scene()
        msg = simulator.receive_sensor_data(topic)
        sensor_data.append(msg)
    
    print('----------------------------------------------------------')

    # 3. ОБРАБОТКА И АНАЛИЗ ДАННЫХ
    distances = [1.0, 3.0, 5.0]
    
    print(f"\n\n{'Distance':>10} {'Measured':>10} {'Abs Error':>11} {'Rel Error %':>10}")
    print('----------------------------------------------------------')
    
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
                print(f"{distance:>10.2f} {'Invalid encoding':>30}")
                results.append({'distance': distance, 'error': 'Invalid encoding'})
                continue
        except CvBridgeError as e:
            print(f"{distance:>10.2f} {'CV Bridge Error':>30}")
            results.append({'distance': distance, 'error': 'CV Bridge Error'})
            continue
        
        if depth_image is None:
            print(f"{distance:>10.2f} {'No image data':>30}")
            results.append({'distance': distance, 'error': 'No image data'})
            continue

        h, w = depth_image.shape
        z = depth_image[h // 2, w // 2]

        if np.isnan(z) or z == 0:
            print(f"{distance:>10.2f} {'N/A':>10} {'N/A':>10} {'N/A':>10}")
            results.append({'distance': distance, 'error': 'Invalid measurement'})
        else:
            abs_err = abs(z - distance)
            rel_err = abs_err / distance * 100.0
            print(f"{distance:>10.2f} {z:>10.2f} {abs_err:>10.2f} {rel_err:>10.2f}")
            results.append({
                'distance': distance,
                'measured': z,
                'abs_error': abs_err,
                'rel_error': rel_err
            })

    return results