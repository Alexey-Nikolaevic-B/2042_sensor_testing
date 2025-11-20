import cv2
import numpy as np

from cv_bridge import CvBridge
from datetime import datetime

import matplotlib.pyplot as plt
import os

def apply_depth(color_cv, depth_cv, focus_distance):
    depth_cv[np.isnan(depth_cv)] = 0
    focus_distance_norm =  focus_distance / depth_cv.max()

    depth_norm = (depth_cv - depth_cv.min()) / (depth_cv.max() - depth_cv.min())

    blur_strength = depth_norm - focus_distance_norm

    negative_mask = blur_strength < - (1 / depth_cv.max())
    positive_mask = blur_strength >= 0
    blur_strength[negative_mask] = np.abs(blur_strength[negative_mask]) * 20
    blur_strength[positive_mask] = blur_strength[positive_mask]
    blur_strength = np.clip(blur_strength, 0, 1)
    blur_strength = np.clip(blur_strength * 5, 0, 1)

    blurred_img = cv2.GaussianBlur(color_cv, (depth_cv.max(), depth_cv.max()), 0)
    out = (1 - blur_strength[:, :, np.newaxis]) * color_cv + blur_strength[:, :, np.newaxis] * blurred_img

    return out.astype(np.uint8)

def capture(CONFIG, simulator, camera_model_path, world_path):
    focus_distance = 2
    mono_camera_topic = '/mono_camera/image_raw'
    depth_camera_topic = '/sensor/depth/image_raw'
    
    simulator.open_scene(world_path, camera_model_path)
    depth_msg = simulator.receive_sensor_data(depth_camera_topic)
    color_msg = simulator.receive_sensor_data(mono_camera_topic)
    
    bridge = CvBridge()
    try:
        color_cv = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_cv = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    except Exception as e:
        return None

    focused_image = apply_depth(color_cv, depth_cv, focus_distance)

    if CONFIG['SAVE_SENSOR_DATA']:
        save_data(CONFIG, focused_image)
    if CONFIG['SAVE_DEBUG_DATA']:
        save_for_debug(CONFIG, color_cv, depth_cv)
    simulator.kill_gazebo()
    return focused_image

def save_data(CONFIG, data):    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        os.makedirs(CONFIG['SAVE_DIR'], exist_ok=True)
        
        focused_filename = f"{CONFIG['SAVE_DIR']}/focused_2m_{timestamp}.png"
        cv2.imwrite(focused_filename, data)
    except Exception as e:
        return None

def save_for_debug(CONFIG, color_cv, depth_cv):
    SAVE_DEBUG_DIR = CONFIG['SAVE_DEBUG_DIR']
    
    np.save(f'{SAVE_DEBUG_DIR}last_color.npy', color_cv)
    np.save(f'{SAVE_DEBUG_DIR}last_depth.npy', depth_cv)