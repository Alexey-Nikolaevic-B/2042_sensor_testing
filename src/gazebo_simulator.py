import os
import time
import subprocess
import rospy
import threading

import xml.etree.ElementTree as ET

from sensor_msgs.msg import Image

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='[%(levelname)s]- %(name)s - %(message)s - %(asctime)s', filemode='w')

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
formatter = logging.Formatter('[%(levelname)s]- %(name)s - %(message)s - %(asctime)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


class Simulator():
    def __init__(self, CONFIG: dict = None):
        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False

        if not CONFIG:
            logger.error("CONFIG dictionary is required")
            raise ValueError("CONFIG dictionary is required")
        
        required_params = ['CATKIN_SETUP_DIR', 'SENSOR_PKG', 'LAUNCH_FILE', 'MESSAGE_TIMEOUT', 'BASE_WORLD_PATH']
        for param in required_params:
            if param not in CONFIG:
                logger.error(f"Missing required configuration parameter: {param}")
                raise ValueError(f"Missing required configuration parameter: {param}")
        
        self.CATKIN_SETUP_DIR = CONFIG['CATKIN_SETUP_DIR']
        self.SENSOR_PKG = CONFIG['SENSOR_PKG']
        self.LAUNCH_FILE = CONFIG['LAUNCH_FILE']
        self.TIMEOUT = CONFIG['MESSAGE_TIMEOUT']
        self.BASE_WORLD_PATH = CONFIG['BASE_WORLD_PATH']

    def launch_ros(self):
        self._kill_ros()
        try:
            env = os.environ.copy()
            env['ROS_LOG_DIR'] = "logs"
            
            self.ros_process = subprocess.Popen(
                ["bash", "-c", "roscore"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            self.ros_is_running = True
            logger.info('ROS core started successfully')
        except Exception as e:
            logger.error(f'Failed to start ROS core: {str(e)}')

    def launch_node(self):
        if threading.current_thread() is not threading.main_thread():
            logger.error("ROS node must be initialized in main thread")
            return

        try:
            rospy.init_node('sensor_data_receiver', anonymous=True)
            self.node_is_running = True
            logger.info('ROS node initialized successfully')
        except Exception as e:
            logger.error(f'Failed to initialize ROS node: {str(e)}')

    def launch(self) -> str:
        self.launch_ros()
        self.launch_node()

    def receive_sensor_data(self, topic):
        try: 
            msg = rospy.wait_for_message(topic, Image, timeout=self.TIMEOUT)
            logger.info(f'Successfully received sensor data from topic: {topic}')
            return msg
        except Exception as e:
            logger.error(f'Failed to receive sensor data from topic {topic}: {str(e)}')

    def is_gazebo_ready(self):
        try:
            ps_output = subprocess.check_output(['ps', 'aux'])
            ps_output_str = ps_output.decode('utf-8')
            if 'gzclient' in ps_output_str:
                return True
            else:
                return False
        except Exception as e:
            logger.error(f'Error checking Gazebo status: {str(e)}')
            return False

    def open_scene(self, world_path, camera_model_path) -> bool:
        self._generate_world(world_path, camera_model_path)     
        roslaunch_cmd = f"source {self.CATKIN_SETUP_DIR} && roslaunch {self.SENSOR_PKG} {self.LAUNCH_FILE}"
        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
                )
            # while not self.is_gazebo_ready():
            #     print('TEST')
            #     time.sleep(1)
            
            self.gazebo_is_running = True
            logger.info('Gazebo started successfully')
        except Exception as e:
            logger.error(f'Failed to start Gazebo: {str(e)}')

    def _generate_world(self, world_path, camera_model_path):
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')
            
            camera_tree = ET.parse(camera_model_path)
            camera_root = camera_tree.getroot()
            
            camera_models = camera_root.findall('model')
            
            for i, camera_model in enumerate(camera_models):
                model_name = camera_model.get('name', f'unknown_{i}')
                world.append(camera_model)
            
            tree.write(self.BASE_WORLD_PATH, encoding='utf-8', xml_declaration=True)
            logger.info('World file generated successfully')
        except Exception as e:
            logger.error(f'Failed to generate world file: {str(e)}')

    def _kill_ros(self):
        if not self.ros_is_running:
            return
        try:
            result = subprocess.run(
                ["bash", "-c", "pkill -f ros"], 
                capture_output=True, 
                timeout=10
            )
            logger.info('ROS processes killed successfully')
        except Exception as e:
            logger.error(f'Failed to kill ROS processes: {str(e)}')

    def _kill_node(self):
        if not self.node_is_running:
            return
        try:
            rospy.signal_shutdown("Simulator shutdown")
            self.node_is_running = False
            logger.info('ROS node shut down successfully')
        except Exception as e:
            logger.error(f'Failed to shut down ROS node: {str(e)}')

    def kill_gazebo(self) -> None:
        if not self.gazebo_is_running:
            return
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            logger.info('Gazebo processes killed successfully')
        except Exception as e:
            logger.error(f'Failed to kill Gazebo processes: {str(e)}')

    def kill(self) -> bool:
        self.kill_gazebo()
        self._kill_node()    
        self._kill_ros()

        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False
        logger.info('All simulator components stopped successfully')