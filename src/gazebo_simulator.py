import os
import json
import socket
import subprocess
import threading
import time

from PyQt5.QtCore import QThread, pyqtSignal

import xml.etree.ElementTree as ET

import logging
with open('log_config.json') as f_in:
    log_config = json.load(f_in)
logging.config.dictConfig(log_config)

logger = logging.getLogger(__name__)

ROSCORE_PORT         = 11311
ROSCORE_POLL_INTERVAL = 0.5
ROSCORE_TIMEOUT      = 30.0


class _RoscoreWatcher(QThread):
    ready  = pyqtSignal()
    log    = pyqtSignal(str, str)   # level, message
    error  = pyqtSignal(str)        # message

    def __init__(self, simulator):
        super().__init__()
        self._sim = simulator

    def run(self):
        self.log.emit("info", "Starting roscore...")
        self._sim.launch_ros()
        self.log.emit("info", "Waiting for roscore...")
        deadline = time.time() + ROSCORE_TIMEOUT
        while time.time() < deadline:
            try:
                with socket.create_connection(("localhost", ROSCORE_PORT), timeout=1.0):
                    pass
                self.log.emit("info", "roscore ready.")
                self.ready.emit()
                return
            except OSError:
                time.sleep(ROSCORE_POLL_INTERVAL)
        self.error.emit(f"roscore did not become ready within {ROSCORE_TIMEOUT:.0f}s.")


class Simulator():
    def __init__(self, CONFIG: dict = None):
        self.ros_process       = None
        self.gazebo_process    = None
        self.on_log                = None
        self.on_capture            = None
        self.on_waiting_for_step   = None
        self._step_mode            = False
        self._step_gate            = None
        self._observer_topic       = "/observer/image_raw"
        self._node_initialized     = False

        self.CATKIN_SETUP_DIR = CONFIG['CATKIN_SETUP_DIR']
        self.SENSOR_PKG = CONFIG['SENSOR_PKG']
        self.LAUNCH_FILE = CONFIG['LAUNCH_FILE']
        self.TIMEOUT = CONFIG['MESSAGE_TIMEOUT']
        self.BASE_WORLD_PATH = CONFIG['BASE_WORLD_PATH']
        self.ROS_LOG_PATH = CONFIG['ROS_LOG_PATH']

    # ── derived state properties ──────────────────────────────────────────────

    @property
    def ros_is_running(self) -> bool:
        """True while roscore is accepting connections on its port.
        Checking the process is unreliable because bash forks roscore
        as a child, so the parent bash process exits immediately."""
        import socket as _socket
        try:
            with _socket.create_connection(("localhost", 11311), timeout=0.5):
                return True
        except OSError:
            return False

    @property
    def node_is_running(self) -> bool:
        """True after rospy.init_node has succeeded."""
        return self._node_initialized

    @property
    def gazebo_is_running(self) -> bool:
        """True while a gzserver process is running on this machine.
        roslaunch forks gzserver as a child, so the roslaunch process
        exiting does not mean Gazebo stopped."""
        import subprocess as _sp
        try:
            r = _sp.run(["pgrep", "-x", "gzserver"],
                        capture_output=True, timeout=1)
            return r.returncode == 0
        except Exception:
            return False

    # ── setters kept for kill() / compat — they are no-ops now ───────────────

    @ros_is_running.setter
    def ros_is_running(self, _): pass

    @node_is_running.setter
    def node_is_running(self, _): pass

    @gazebo_is_running.setter
    def gazebo_is_running(self, _): pass

    def launch_ros(self):
        self._kill_ros()
        try:
            env = os.environ.copy()
            # Write ROS logs to a temp dir that gets cleared each run.
            # Avoids accumulation in ~/.ros/log while keeping ROS happy.
            import tempfile
            _ros_log_tmp = os.path.join(tempfile.gettempdir(), "ros_logs")
            os.makedirs(_ros_log_tmp, exist_ok=True)
            env['ROS_LOG_DIR'] = _ros_log_tmp
            env['ROSCONSOLE_STDOUT_LINE_BUFFERED'] = '1'

            roscore_cmd = f"source {self.CATKIN_SETUP_DIR} && roscore"

            self.ros_process = subprocess.Popen(
                ["bash", "-c", roscore_cmd],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            logger.info('ROS core started')
        except Exception as e:
            logger.error(f'Failed to start ROS core: {str(e)}')

    def launch_node(self):
        if threading.current_thread() is not threading.main_thread():
            logger.warning("launch_node called from non-main thread — signal handlers may fail")
        try:
            import rospy
            import rospy as _rospy
            from gazebo_msgs.srv import SetModelState, GetWorldProperties
            from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist
            from gazebo_msgs.msg import ModelState, ModelStates

            import sys
            sys.modules[__name__]  # ensure module exists

            # Make imports available to all methods via module-level injection
            globals()["rospy"]              = rospy
            globals()["SetModelState"]      = SetModelState
            globals()["GetWorldProperties"] = GetWorldProperties
            globals()["Pose"]               = Pose
            globals()["Point"]              = Point
            globals()["Quaternion"]         = Quaternion
            globals()["Vector3"]            = Vector3
            globals()["Twist"]              = Twist
            globals()["ModelState"]         = ModelState
            globals()["ModelStates"]        = ModelStates

            rospy.init_node('sensor_data_receiver', anonymous=True)
            self._node_initialized = True
            logger.info('ROS node initialized successfully')
        except Exception as e:
            logger.error(f'Failed to initialize ROS node: {str(e)}')

    def start_async(self, on_ready, on_log, on_error):
        self._watcher = _RoscoreWatcher(self)
        self._watcher.ready.connect(on_ready)
        self._watcher.log.connect(on_log)
        self._watcher.error.connect(on_error)
        self._watcher.start()


    def is_gazebo_running(self):
        try:
            from gazebo_msgs.srv import GetWorldProperties
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
            rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
            return True
        except Exception as exc:
            logger.debug(f'is_gazebo_running: {exc}')
            return False

    def open_scene(self, world_path, camera_model_path) -> bool:
        logger.info(f'open_scene: world={world_path}')

        gazebo_running = self.is_gazebo_running()
        logger.info(f'open_scene: is_gazebo_running={gazebo_running}')
        if gazebo_running:
            self.kill_gazebo()
            time.sleep(1.0)

        if not self.ros_is_running:
            logger.error('open_scene: ros_is_running=False')
            return False

        if not self.node_is_running:
            logger.error('open_scene: node_is_running=False')
            return False

        logger.info('open_scene: generating world file')
        self._generate_world(world_path, camera_model_path)
        roslaunch_cmd = f"source {self.CATKIN_SETUP_DIR} && roslaunch {self.SENSOR_PKG} {self.LAUNCH_FILE}"
        logger.info(f'open_scene: roslaunch_cmd={roslaunch_cmd}')

        try:
            launch_env = os.environ.copy()
            import tempfile
            _ros_log_tmp = os.path.join(tempfile.gettempdir(), "ros_logs")
            os.makedirs(_ros_log_tmp, exist_ok=True)
            launch_env['ROS_LOG_DIR'] = _ros_log_tmp
            launch_env['ROSCONSOLE_STDOUT_LINE_BUFFERED'] = '1'
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=launch_env,
            )
            logger.info(f'open_scene: gazebo process pid={self.gazebo_process.pid}')

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

            logger.info('open_scene: waiting for gazebo services (30s timeout)')
            if not self.wait_gazebo_quiet(30.0):
                logger.error('open_scene: wait_gazebo_quiet timed out')
                return False

            if self.is_gazebo_running():
                logger.info('open_scene: Gazebo started successfully')
                # Debug: list all active ROS topics
                return True
            else:
                logger.error('open_scene: is_gazebo_running() returned False after startup')
                return False

        except Exception as exc:
            import traceback
            logger.error(f'open_scene exception: {exc}{traceback.format_exc()}')
            return False
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
        # Strip ANSI escape codes (colour codes from ROS/Gazebo output)
        import re as _re
        line = _re.sub(r'\x1b\[[0-9;]*[mKHJ]|\[0m', '', line).strip()
        if not line:
            return

        # Strip the ROS timestamp prefix: "[INFO] [1234567890.123]: message"
        ros_msg = _re.sub(r'^\[(?:INFO|WARN|ERROR|DEBUG)\]\s*\[\d+\.\d+\]:\s*', '', line)

        line_lower = ros_msg.lower()

        # Classify and forward only meaningful lines — skip pure ROS chatter
        if line.startswith('bash:') or 'command not found' in line_lower:
            if self.on_log:
                self.on_log('warning', ros_msg)
        elif any(w in line_lower for w in ['error', 'exception', 'fail', 'cannot', 'invalid']):
            if self.on_log:
                self.on_log('error', ros_msg)
        elif 'warning' in line_lower:
            if self.on_log:
                self.on_log('warning', ros_msg)
        # All other Gazebo stdout lines are discarded — they are captured
        # by logger.* calls inside the simulator methods instead.


    def _generate_world(self, world_path, camera_model_path):
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')

            # Inject sensor model(s)
            camera_tree = ET.parse(camera_model_path)
            camera_root = camera_tree.getroot()
            for camera_model in camera_root.findall('model'):
                world.append(camera_model)

            # Inject observer camera from assets/observer_camera.sdf
            observer_sdf = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "assets", "observer_camera.sdf"
            )
            if os.path.exists(observer_sdf):
                obs_tree = ET.parse(observer_sdf)
                obs_root = obs_tree.getroot()
                # Accept both bare <model> root and <sdf><model> wrapper
                models = obs_root.findall('model') or ([obs_root] if obs_root.tag == 'model' else [])
                for m in models:
                    world.append(m)
                logger.info('Observer camera loaded from %s', observer_sdf)
            else:
                logger.warning('Observer SDF not found at %s — observer disabled', observer_sdf)

            tree.write(self.BASE_WORLD_PATH, encoding='utf-8', xml_declaration=True)
            logger.info('World file written to %s', self.BASE_WORLD_PATH)
        except Exception as e:
            import traceback
            logger.error('Failed to generate world file: %s\n%s', e, traceback.format_exc())

    def _kill_ros(self):
        if not self.ros_is_running:
            return
        try:
            if self.ros_process and self.ros_process.poll() is None:
                self.ros_process.terminate()
                try:
                    self.ros_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.ros_process.kill()
                    self.ros_process.wait()
            self.ros_process = None
            logger.info('ROS core stopped')
        except Exception as e:
            logger.error(f'Failed to stop ROS core: {str(e)}')

    def _kill_node(self):
        if not self.node_is_running:
            return
        try:
            rospy.signal_shutdown("Simulator shutdown")
            self._node_initialized = False
            logger.info('ROS node shut down')
        except Exception as e:
            logger.error(f'Failed to shut down ROS node: {str(e)}')


    def wait_for_model_spawn(self, model_name: str, timeout = 10) -> bool:
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


    def set_pose(
        self, model : str,
        x : int = 0, y : int = 0, z : int = 0,
        quaternion : Quaternion = None,
        linear_velocity : Vector3 = None,
        angular_velocity : Vector3 = None,
    ):
        """Метод для перемещения моделей в симуляции"""
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = model
        state.reference_frame = "world"
        quaternion = quaternion if quaternion else Quaternion(0, 0, 0, 1)
        state.pose = Pose(Point(x, y, z), quaternion)
        if linear_velocity or angular_velocity:
            state.twist = Twist(
                linear=linear_velocity if linear_velocity else Vector3(0, 0, 0),
                angular=angular_velocity if angular_velocity else Vector3(0, 0, 0),
            )
        response = set_state(state)
        if not response.success:
            raise RuntimeError(response.status_message)


    def set_step_mode(self, enabled: bool) -> None:
        import threading
        self._step_mode = enabled
        if enabled and self._step_gate is None:
            self._step_gate = threading.Event()
            self._step_gate.set()

    def wait_for_step(self) -> None:
        """Block the worker thread until the user clicks Step.
        Fires on_waiting_for_step so the UI can enable the Step button."""
        if not self._step_mode or self._step_gate is None:
            return
        # Notify UI — must be queued since we're on the worker thread
        if self.on_waiting_for_step:
            try:
                from PyQt5.QtCore import QMetaObject, Qt
                QMetaObject.invokeMethod(
                    self.on_waiting_for_step.__self__,
                    self.on_waiting_for_step.__func__.__name__,
                    Qt.QueuedConnection,
                )
            except Exception:
                try:
                    self.on_waiting_for_step()
                except Exception:
                    pass
        self._step_gate.clear()
        self._step_gate.wait()

    def advance_step(self) -> None:
        if self._step_gate is not None:
            self._step_gate.set()

    def continue_all(self) -> None:
        self._step_mode = False
        if self._step_gate is not None:
            self._step_gate.set()

    def capture_observer_frame(self) -> bytes | None:
        """Grab one JPEG frame from the observer camera ROS topic."""
        if not self.gazebo_is_running:
            return None
        try:
            from sensor_msgs.msg import Image
            msg = rospy.wait_for_message(self._observer_topic, Image, timeout=2.0)
            import numpy as np
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            import cv2
            arr_bgr = arr[:, :, ::-1].copy()
            ok, buf = cv2.imencode(".jpg", arr_bgr)
            return bytes(buf) if ok else None
        except Exception as e:
            logger.debug("capture_observer_frame failed: %s", e)
            return None
    def notify_capture(self, sensor_data: dict, observer_img: bytes | None = None) -> None:
        """Fire on_capture callback after each sensor capture.
        Always grabs a fresh observer frame so the UI stays current."""
        if observer_img is None and self.gazebo_is_running:
            observer_img = self.capture_observer_frame()

        if self.on_capture:
            try:
                self.on_capture(sensor_data, observer_img)
            except Exception as e:
                import traceback
                logger.error("notify_capture callback error: %s\n%s", e, traceback.format_exc())

    def kill_gazebo(self) -> None:
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            if self.gazebo_process:
                self.gazebo_process = None
            logger.info('Gazebo processes killed')
        except Exception as e:
            logger.error(f'Failed to kill Gazebo processes: {str(e)}')


    def kill(self) -> bool:
        self.kill_gazebo()
        self._kill_node()
        self._kill_ros()

        self._node_initialized = False


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