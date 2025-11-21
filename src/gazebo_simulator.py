import os
import json
import subprocess
import rospy
import threading

import xml.etree.ElementTree as ET

from sensor_msgs.msg import Image

import logging
with open('log_config.json') as f_in:
    log_config = json.load(f_in)
logging.config.dictConfig(log_config)

logger = logging.getLogger(__name__)

class Simulator():
    def __init__(self, CONFIG: dict = None):
        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False
        
        self.CATKIN_SETUP_DIR = CONFIG['CATKIN_SETUP_DIR']
        self.SENSOR_PKG = CONFIG['SENSOR_PKG']
        self.LAUNCH_FILE = CONFIG['LAUNCH_FILE']
        self.TIMEOUT = CONFIG['MESSAGE_TIMEOUT']
        self.BASE_WORLD_PATH = CONFIG['BASE_WORLD_PATH']
        self.ROS_LOG_PATH = CONFIG['ROS_LOG_PATH']

    def launch_ros(self):
        self._kill_ros()
        try:
            env = os.environ.copy()
            env['ROS_LOG_DIR'] = self.ROS_LOG_PATH

            os.makedirs(self.ROS_LOG_PATH, exist_ok=True)
            
            self.ros_process = subprocess.Popen(
                ["bash", "-c", "roscore"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            self.ros_is_running = True
            logger.info('ROS score started')
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
            logger.info(f'Received sensor data from topic: {topic}')
            return msg
        except Exception as e:
            logger.error(f'Failed to receive sensor data from topic: {str(e)}')

    def is_gazebo_running(self):
        try:
            import rospy
            from gazebo_msgs.srv import GetWorldProperties
            
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            response = get_world_properties()
            return True
        except:
            return False

    def open_scene(self, world_path, camera_model_path) -> bool:
        if not self.ros_is_running:
            logger.error('Failed to start Gazebo: Ros is not running')
            return False
        if not self.node_is_running:
            logger.error('Failed to start Gazebo: Node is not running')
            return False
            
        self._generate_world(world_path, camera_model_path)     
        roslaunch_cmd = f"source {self.CATKIN_SETUP_DIR} && roslaunch {self.SENSOR_PKG} {self.LAUNCH_FILE}"
        
        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            stdout_thread = threading.Thread(
                target=self._log_stdout_output,
                args=(self.gazebo_process.stdout,)
            )
            stdout_thread.daemon = True
            stdout_thread.start()
            
            stderr_thread = threading.Thread(
                target=self._log_stderr_output,
                args=(self.gazebo_process.stderr,)
            )
            stderr_thread.daemon = True
            stderr_thread.start()
            
            self.gazebo_is_running = True

            if self.is_gazebo_running():
                logger.info('Gazebo started')
                return True
            else:
                logger.error('Failed to start Gazebo')
                return False
                
        except Exception as e:
            logger.error(f'Failed to start Gazebo: {str(e)}')
            return False

    def _log_stdout_output(self, stdout_stream):
        try:
            for line in iter(stdout_stream.readline, ''):
                if line.strip():
                    line_clean = line.strip()
                    self._process_output_line(line_clean, "stdout")
        except ValueError:
            pass

    def _log_stderr_output(self, stderr_stream):
        try:
            for line in iter(stderr_stream.readline, ''):
                if line.strip():
                    line_clean = line.strip()
                    self._process_output_line(line_clean, "stderr")
        except ValueError:
            pass

    def _process_output_line(self, line, stream_type):
        line_lower = line.lower()
        if line.startswith('bash:') or 'command not found' in line_lower:
            logger.warning(f"[Gazebo/bash] {line}")
        elif any(word in line_lower for word in ['error', 'exception', 'fail', 'cannot', 'invalid']):
            logger.error(f"[Gazebo] {line}")
        elif 'warning' in line_lower:
            logger.warning(f"[Gazebo] {line}")
        else:
            if any(keyword in line_lower for keyword in ['start', 'complete', 'ready', 'initializ']):
                # logger.info(f"[Gazebo] {line}")
                pass


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
            logger.info('Base .world file generated')
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
            logger.info('ROS processes killed')
        except Exception as e:
            logger.error(f'Failed to kill ROS processes: {str(e)}')

    def _kill_node(self):
        if not self.node_is_running:
            return
        try:
            rospy.signal_shutdown("Simulator shutdown")
            self.node_is_running = False
            logger.info('ROS node shut down')
        except Exception as e:
            logger.error(f'Failed to shut down ROS node: {str(e)}')

    def kill_gazebo(self) -> None:
        if not self.gazebo_is_running:
            return
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            logger.info('Gazebo processes killed')
        except Exception as e:
            logger.error(f'Failed to kill Gazebo processes: {str(e)}')

    def kill(self) -> bool:
        self.kill_gazebo()
        self._kill_node()    
        self._kill_ros()

        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False