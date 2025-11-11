import os
import cv2
import rospy

from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Node:
    def __init__(self, config):
        self.TIMEOUT = config.get('TIMEOUT', 10)
        self.SAVE_DATA = config.get('SAVE_DATA', True)
        self.SAVE_PATH = config.get('SAVE_DIR', './captured_data')   

        if not os.path.exists(self.SAVE_PATH):
            os.makedirs(self.SAVE_PATH)

    def launch(self):
        print(f"✅ Node \'sendor_data_receiver\' is working")
        self.ros_node = rospy.init_node('sendor_data_receiver', anonymous=True)

    def save_sensor_data(self, msg): # Эту функцию нужно переделать
        try:
            if msg.encoding == '32FC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, "32FC1")
                depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.SAVE_PATH, filename)
                
                cv2.imwrite(full_path, depth_visual)
                print(f"📂 Sensor captured data saved: {full_path}")
                
            elif msg.encoding == '16UC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, "16UC1")
                depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.SAVE_PATH, filename)
                
                cv2.imwrite(full_path, depth_visual)
                print(f"📂 Sensor captured data saved: {full_path}")
                
            elif msg.encoding == 'rgb8':
                image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.SAVE_PATH, filename)

                cv2.imwrite(full_path, image)
                print(f"📂 Sensor captured data saved: {full_path}")

        except Exception as e:
            print("❌ Error:", e)
            return None

    def get_sensor_data(self, topic): 
        msg = rospy.wait_for_message(topic, Image, timeout=self.TIMEOUT)
        if self.SAVE_DATA: self.save_sensor_data(msg)
        return msg
        
    def kill(self):
        rospy.signal_shutdown("Done")
        print("💀 Node was killed.")
