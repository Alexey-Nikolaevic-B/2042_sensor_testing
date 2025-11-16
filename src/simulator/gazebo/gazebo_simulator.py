import time
import subprocess
import xml.etree.ElementTree as ET

from src.simulator.simulator import Simulator

from .ros import Ros
from .node import Node

class GazeboSimulator(Simulator):
    def __init__(self, CONFIG: dict = None):
        self.is_launched = False

        self.CATKIN_SETUP_DIR = CONFIG['CATKIN_SETUP_DIR']
        self.SENSOR_PKG = CONFIG['SENSOR_PKG']
        self.LAUNCH_FILE = CONFIG['LAUNCH_FILE']

        self.ros  = Ros() 
        self.node = Node(CONFIG)

        self.launch()

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

    def launch(self) -> str:
        self.ros.launch()
        self.node.launch()

    def open_scene(self) -> bool:        
        roslaunch_cmd = f"source {self.CATKIN_SETUP_DIR} && roslaunch {self.SENSOR_PKG} {self.LAUNCH_FILE}"
        try:
            self.gazebo_process = subprocess.Popen(["bash", "-c", roslaunch_cmd], 
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True)
            print(f"✅ Gazebo started with PID: {self.gazebo_process.pid}") # это пид попена, а не газибо.
            time.sleep(5)
            
            self.is_launched = True
            return f"Gazebo started successfully with PID: {self.gazebo_process.pid}"
            
        except Exception as e:
            return f"Error launching Gazebo: {e}"

    def receive_sensor_data(self, topic: str) -> any:
        msg = self.node.get_sensor_data(topic)
    
        print("💀 Gazebo was killed.")
        subprocess.run(["pkill", "-f", "gzserver"], check=False) # костыль, который перезапускать газибо
        subprocess.run(["pkill", "-f", "gzclient"], check=False)
        return msg 

    def kill(self) -> bool:
        print("💀 Gazebo was killed.")
        subprocess.run(["pkill", "-f", "gzserver"], check=False)
        subprocess.run(["pkill", "-f", "gzclient"], check=False)
        
        self.node.kill()    
        self.ros.kill()
        
        self.is_launched = False
        return True

    def is_running(self) -> bool:
        if self.gazebo_process is None:
            return False
        return self.gazebo_process.poll() is None