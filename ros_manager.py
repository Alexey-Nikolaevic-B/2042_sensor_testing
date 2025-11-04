import os
import cv2
import rospy
import time
import subprocess
import xml.etree.ElementTree as ET

from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Ros:
    def __init__(self):
        self.ros_process = subprocess.Popen(["bash", "-c", "roscore"])

    def kill(self):
        subprocess.run(["bash", "-c", "pkill -f ros"])
        self.ros_process.kill()

class Gazebo: 
    def generate_world(self, world_path, camera_model_path, base_world_path):
        tree = ET.parse(world_path)
        root = tree.getroot()
        world = root.find('world')
        camera_tree = ET.parse(camera_model_path)
        camera_root = camera_tree.getroot()
        camera_model = camera_root.find('model')
        if camera_model is None:
            print("Error: No model found in camera SDF file")
            return
        world.append(camera_model)
        tree.write(base_world_path, encoding='utf-8', xml_declaration=True)

    def run(self, catkin_setup_dir, sensor_pkg, launch_file):
        roslaunch_cmd = f"source {catkin_setup_dir} && roslaunch {sensor_pkg} {launch_file}"
        try:
            subprocess.Popen(["bash", "-c", roslaunch_cmd])
            time.sleep(5)
        except Exception as e:
            return f"Ошибка при запуске gazebo: {e}"
        
    # def clear_world(self): #TODO: Пока не работает.
    #     """Очистить мир от всех моделей (кроме ground_plane)"""
    #     try:
    #         # Получаем список всех моделей в мире
    #         world_props = self.get_world_properties()
            
    #         # Удаляем каждую модель, кроме основных элементов
    #         excluded_models = ['ground_plane', 'sun']  # Модели, которые не удаляем
            
    #         for model_name in world_props.model_names:
    #             if model_name not in excluded_models:
    #                 self.delete_model(model_name)
    #                 print(f"🗑️ Удалена модель: {model_name}")
    #                 time.sleep(0.1)  # Небольшая задержка между удалениями
            
    #         # Очищаем силы и сбрасываем симуляцию
    #         self.clear_body_forces()
    #         print("✅ Мир очищен от моделей")
    #         return True
        
    #     except Exception as e:
    #         print(f"❌ Ошибка при очистке мира: {e}")
    #         return False
        
    def load_new_world():
        pass

    def kill(self):
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            print("🔁 Gazebo был завершён принудительно.")
            time.sleep(5)
        except Exception as e:
            print("⚠️ Ошибка при завершении Gazebo:", e)

class Node:
    def __init__(self, timeout, save_data, save_path):
        self.timeout = timeout
        self.save_data = save_data
        self.save_path = save_path
        ros_node = rospy.init_node('sendor_data_receiver', anonymous=True)

    def save_sensor_data(self, msg):
        try:
            if msg.encoding == '32FC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, "32FC1")
                depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.save_path, filename)
                
                cv2.imwrite(full_path, depth_visual)
                print(f"✅ Sensor data saved: {full_path}")
                
            elif msg.encoding == '16UC1':
                depth_image = CvBridge().imgmsg_to_cv2(msg, "16UC1")
                depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.save_path, filename)
                
                cv2.imwrite(full_path, depth_visual)
                print(f"✅ Sensor data saved: {full_path}")
                
            elif msg.encoding == 'rgb8':
                image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{timestamp}.png"
                full_path = os.path.join(self.save_path, filename)

                cv2.imwrite(full_path, image)

        except Exception as e:
            print("❌ Error:", e)
            return None

    def get_sensor_data(self, topic): 
        msg = rospy.wait_for_message(topic, Image, timeout=self.timeout)
        if self.save_data: self.save_sensor_data(msg)
        return msg
        
    def kill(self):
        rospy.signal_shutdown("Done")