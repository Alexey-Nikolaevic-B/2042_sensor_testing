import copy
import json
import logging
import logging.config
import os
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetWorldProperties, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

with open('log_config.json') as f_in:
    log_config = json.load(f_in)
logging.config.dictConfig(log_config)

logger = logging.getLogger(__name__)


class Simulator():
    def __init__(self, CONFIG: dict = None):
        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False

        self.CONFIG = CONFIG
        self.REPO_ROOT = self._discover_repo_root(CONFIG)

        self.CATKIN_SETUP_DIR = CONFIG['CATKIN_SETUP_DIR']
        self.SENSOR_PKG = CONFIG['SENSOR_PKG']
        self.LAUNCH_FILE = CONFIG['LAUNCH_FILE']
        self.TIMEOUT = CONFIG['MESSAGE_TIMEOUT']

        base_world = CONFIG['BASE_WORLD_PATH']
        if not os.path.isabs(base_world):
            base_world = os.path.join(self.REPO_ROOT, base_world)
        self.BASE_WORLD_PATH = os.path.abspath(base_world)

        ros_log_path = CONFIG['ROS_LOG_PATH']
        if not os.path.isabs(ros_log_path):
            ros_log_path = os.path.join(self.REPO_ROOT, ros_log_path)
        self.ROS_LOG_PATH = os.path.abspath(ros_log_path)

        self._diag_lock = threading.Lock()
        self._launch_stdout_lines = deque(maxlen=500)
        self._launch_stderr_lines = deque(maxlen=500)
        self._last_scene_diagnostics = {}

    @staticmethod
    def _discover_repo_root(CONFIG: dict) -> str:
        root = CONFIG.get("ROOT_PATH", "") if CONFIG else ""
        if root:
            root_abs = os.path.abspath(root)
            if os.path.isfile(os.path.join(root_abs, "test_runner.py")):
                return root_abs

        try:
            result = subprocess.run(
                ["git", "rev-parse", "--show-toplevel"],
                capture_output=True,
                text=True,
                timeout=2,
                check=False,
            )
            if result.returncode == 0:
                git_root = result.stdout.strip()
                if git_root:
                    return os.path.abspath(git_root)
        except Exception:
            pass

        return os.path.abspath(os.getcwd())

    def _clear_launch_buffers(self) -> None:
        with self._diag_lock:
            self._launch_stdout_lines.clear()
            self._launch_stderr_lines.clear()

    def _append_launch_line(self, stream_type: str, line: str) -> None:
        formatted = f"[{stream_type}] {line}"
        with self._diag_lock:
            if stream_type == "stderr":
                self._launch_stderr_lines.append(formatted)
            else:
                self._launch_stdout_lines.append(formatted)

    def _launch_output_snapshot(self) -> dict:
        with self._diag_lock:
            return {
                "launch_stdout_tail": list(self._launch_stdout_lines),
                "launch_stderr_tail": list(self._launch_stderr_lines),
            }

    @staticmethod
    def _extract_missing_model_uris(lines) -> list:
        joined = "\n".join(lines)
        # Gazebo обычно пишет: Unable to find uri[model://<name>]
        uris = re.findall(r"model://([A-Za-z0-9_./-]+)", joined)
        unique = []
        for uri in uris:
            if uri not in unique:
                unique.append(uri)
        return unique

    def _set_scene_diag(self, **kwargs) -> None:
        with self._diag_lock:
            self._last_scene_diagnostics.update(kwargs)

    def _replace_scene_diag(self, payload: dict) -> None:
        with self._diag_lock:
            self._last_scene_diagnostics = dict(payload)

    def get_last_scene_diagnostics(self) -> dict:
        with self._diag_lock:
            return copy.deepcopy(self._last_scene_diagnostics)

    def _resolve_input_path(self, path_like: str) -> str:
        if not path_like:
            return ""

        if os.path.isabs(path_like):
            return os.path.abspath(path_like)

        from_cwd = os.path.abspath(path_like)
        if os.path.exists(from_cwd):
            return from_cwd

        from_repo = os.path.abspath(os.path.join(self.REPO_ROOT, path_like))
        if os.path.exists(from_repo):
            return from_repo

        # Возвращаем наиболее ожидаемую форму для диагностики.
        return from_repo

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
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            get_world_properties()
            return True
        except Exception:
            return False

    def open_scene(self, world_path, camera_model_path) -> bool:
        self._clear_launch_buffers()

        world_abs = self._resolve_input_path(world_path)
        sensor_abs = self._resolve_input_path(camera_model_path)
        catkin_setup_abs = self._resolve_input_path(self.CATKIN_SETUP_DIR)

        diag = {
            "attempt_ts": time.time(),
            "cwd": os.path.abspath(os.getcwd()),
            "repo_root": self.REPO_ROOT,
            "world_path_input": str(world_path),
            "world_path_abs": world_abs,
            "world_exists": bool(os.path.exists(world_abs)),
            "world_size_bytes": int(os.path.getsize(world_abs)) if os.path.exists(world_abs) else None,
            "camera_model_path_input": str(camera_model_path),
            "camera_model_path_abs": sensor_abs,
            "camera_model_exists": bool(os.path.exists(sensor_abs)),
            "camera_model_size_bytes": int(os.path.getsize(sensor_abs)) if os.path.exists(sensor_abs) else None,
            "base_world_path_abs": self.BASE_WORLD_PATH,
            "catkin_setup_input": str(self.CATKIN_SETUP_DIR),
            "catkin_setup_abs": catkin_setup_abs,
            "catkin_setup_exists": bool(os.path.exists(catkin_setup_abs)),
            "env": {
                "GAZEBO_MODEL_PATH": os.environ.get("GAZEBO_MODEL_PATH", ""),
                "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", ""),
                "GAZEBO_RESOURCE_PATH": os.environ.get("GAZEBO_RESOURCE_PATH", ""),
                "GAZEBO_PLUGIN_PATH": os.environ.get("GAZEBO_PLUGIN_PATH", ""),
            },
            "reason": "",
        }
        self._replace_scene_diag(diag)

        if self.is_gazebo_running():
            self.kill_gazebo()
            time.sleep(1.0)

        if not self.ros_is_running:
            logger.error('Failed to start Gazebo: Ros is not running')
            self._set_scene_diag(reason="ros_not_running")
            return False

        if not self.node_is_running:
            logger.error('Failed to start Gazebo: Node is not running')
            self._set_scene_diag(reason="ros_node_not_running")
            return False

        if not os.path.exists(world_abs):
            self._set_scene_diag(reason="world_path_not_found")
            logger.error(f"World path does not exist: {world_abs}")
            return False

        if not os.path.exists(sensor_abs):
            self._set_scene_diag(reason="camera_model_path_not_found")
            logger.error(f"Camera model path does not exist: {sensor_abs}")
            return False

        if not os.path.exists(catkin_setup_abs):
            self._set_scene_diag(reason="catkin_setup_not_found")
            logger.error(f"catkin setup not found: {catkin_setup_abs}")
            return False

        generated_ok, generate_error = self._generate_world(world_abs, sensor_abs)
        if not generated_ok:
            snapshot = self._launch_output_snapshot()
            self._set_scene_diag(
                reason="base_world_generation_failed",
                generation_error=generate_error,
                **snapshot,
            )
            return False

        roslaunch_cmd = f"source {catkin_setup_abs} && roslaunch {self.SENSOR_PKG} {self.LAUNCH_FILE}"
        self._set_scene_diag(roslaunch_cmd=roslaunch_cmd)

        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )

            stdout_thread = threading.Thread(
                target=self._log_stdout_output,
                args=(self.gazebo_process.stdout,),
            )
            stdout_thread.daemon = True
            stdout_thread.start()

            stderr_thread = threading.Thread(
                target=self._log_stderr_output,
                args=(self.gazebo_process.stderr,),
            )
            stderr_thread.daemon = True
            stderr_thread.start()

            if not self.wait_gazebo_quiet(30.0):
                snapshot = self._launch_output_snapshot()
                lines = snapshot["launch_stdout_tail"] + snapshot["launch_stderr_tail"]
                self._set_scene_diag(
                    reason="gazebo_services_not_available",
                    roslaunch_returncode=self.gazebo_process.poll(),
                    missing_model_uris=self._extract_missing_model_uris(lines),
                    **snapshot,
                )
                logger.error("Gazebo services did not appear")
                return False

            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

            self.gazebo_is_running = True

            if self.is_gazebo_running():
                snapshot = self._launch_output_snapshot()
                lines = snapshot["launch_stdout_tail"] + snapshot["launch_stderr_tail"]
                self._set_scene_diag(
                    reason="ok",
                    roslaunch_returncode=self.gazebo_process.poll(),
                    missing_model_uris=self._extract_missing_model_uris(lines),
                    **snapshot,
                )
                logger.info('Gazebo started')
                return True

            snapshot = self._launch_output_snapshot()
            lines = snapshot["launch_stdout_tail"] + snapshot["launch_stderr_tail"]
            self._set_scene_diag(
                reason="gazebo_not_running_after_start",
                roslaunch_returncode=self.gazebo_process.poll(),
                missing_model_uris=self._extract_missing_model_uris(lines),
                **snapshot,
            )
            logger.error('Failed to start Gazebo')
            return False

        except Exception as e:
            snapshot = self._launch_output_snapshot()
            lines = snapshot["launch_stdout_tail"] + snapshot["launch_stderr_tail"]
            self._set_scene_diag(
                reason="open_scene_exception",
                exception=str(e),
                roslaunch_returncode=getattr(self, "gazebo_process", None).poll() if hasattr(self, "gazebo_process") else None,
                missing_model_uris=self._extract_missing_model_uris(lines),
                **snapshot,
            )
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
        self._append_launch_line(stream_type, line)

        line_lower = line.lower()
        if line.startswith('bash:') or 'command not found' in line_lower:
            logger.warning(f"[Gazebo/bash] {line}")
        elif any(word in line_lower for word in ['error', 'exception', 'fail', 'cannot', 'invalid']):
            logger.error(f"[Gazebo] {line}")
        elif 'warning' in line_lower:
            logger.warning(f"[Gazebo] {line}")
        else:
            if any(keyword in line_lower for keyword in ['start', 'complete', 'ready', 'initializ']):
                pass

    def _generate_world(self, world_path, camera_model_path):
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')
            if world is None:
                return False, "No <world> node in world file"

            camera_tree = ET.parse(camera_model_path)
            camera_root = camera_tree.getroot()

            camera_models = camera_root.findall('model')
            if not camera_models:
                return False, "No <model> in camera SDF"

            for i, camera_model in enumerate(camera_models):
                _ = camera_model.get('name', f'unknown_{i}')
                world.append(camera_model)

            os.makedirs(os.path.dirname(self.BASE_WORLD_PATH), exist_ok=True)
            tree.write(self.BASE_WORLD_PATH, encoding='utf-8', xml_declaration=True)
            logger.info('Base .world file generated')
            return True, ""
        except Exception as e:
            msg = f'Failed to generate world file: {str(e)}'
            logger.error(msg)
            return False, str(e)

    def _kill_ros(self):
        if not self.ros_is_running:
            return
        try:
            subprocess.run(
                ["bash", "-c", "pkill -f ros"],
                capture_output=True,
                timeout=10,
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

    def wait_for_model_spawn(self, model_name: str, timeout=10) -> bool:
        """Метод чтобы дождаться появления модели в симуляции"""
        start_time = time.time()
        while (time.time() - start_time < timeout):
            try:
                msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
                if model_name in msg.name:
                    return True
            except rospy.ROSException:
                continue
        return False

    def set_pose(self, model, x=0, y=0, z=0):
        """Метод для перемещения моделей в симуляции"""
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = model
        state.reference_frame = "world"
        state.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
        response = set_state(state)
        if not response.success:
            raise RuntimeError(response.status_message)

    def kill_gazebo(self) -> None:
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            self.gazebo_is_running = False
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

    def wait_gazebo_quiet(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                proxy = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                proxy()
                return True
            except Exception:
                time.sleep(0.2)
        return False
