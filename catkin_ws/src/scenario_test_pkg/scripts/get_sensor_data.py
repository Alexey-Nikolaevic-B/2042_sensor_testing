#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from subprocess import run

def get_depth_at_center(topic, timeout=10.0):
    try:
        msg = rospy.wait_for_message(topic, Image, timeout=timeout)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout waiting for message on {topic}: {e}")
        return None
    encoding = msg.encoding
    try:
        if encoding == '32FC1':
            depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='32FC1')
        elif encoding == '16UC1':
            depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image = depth_image.astype(np.float32) * 0.001
        else:
            rospy.logerr(f"Received non-depth image encoding '{encoding}' on {topic}")
            return None
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge error: {e}")
        return None
    if depth_image is None:
        rospy.logerr("Получено пустое изображение глубины")
        return None
    h, w = depth_image.shape
    return float(depth_image[h // 2, w // 2])

if __name__ == '__main__':
    rospy.init_node('test_camera_depth_camera', anonymous=True)
    distances = [1.0, 3.0, 5.0]
    depth_topic = rospy.get_param('~depth_topic', '/depth_camera/depth/image_raw')
    timeout = rospy.get_param('~timeout', 10.0)

    print(f"\nТест точности измерения глубины по топику '{depth_topic}'")
    print("----------------------------------------------------------")
    print(f"{'Z_true(m)':>10} {'Z_meas(m)':>10} {'Δ_abs(m)':>10} {'Δ_rel(%)':>10}")
    print("----------------------------------------------------------")

    for d in distances:
        rospy.loginfo(f"Запрос глубины на расстоянии {d} м...")
        z = get_depth_at_center(depth_topic, timeout)
        if z is None:
            print(f"{d:>10.2f} {'N/A':>10} {'N/A':>10} {'N/A':>10}")
        else:
            abs_err = abs(z - d)
            rel_err = abs_err / d * 100.0
            print(f"{d:>10.2f} {z:>10.2f} {abs_err:>10.2f} {rel_err:>10.2f}")
        rospy.sleep(1.0)

    print("----------------------------------------------------------")
    rospy.loginfo("Тест завершён.")
