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
        self.ros_is_running    = False
        self.node_is_running   = False
        self.gazebo_is_running = False
        self.ros_process       = None
        self.gazebo_process    = None
        self.on_log            = None   # set by __main__: col_4.append_log
        self.on_capture        = None   # callback(sensor_data, obs_img) fired after each capture
        self._step_mode        = False
        self._step_gate        = None   # threading.Event
        self._observer_topic   = "/observer/image_raw"

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

            roscore_cmd = f"source {self.CATKIN_SETUP_DIR} && roscore"

            self.ros_process = subprocess.Popen(
                ["bash", "-c", roscore_cmd],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            self.ros_is_running = True
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
            self.node_is_running = True
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
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
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

            self.gazebo_is_running = True

            if self.is_gazebo_running():
                logger.info('open_scene: Gazebo started successfully')
                # Debug: list all active ROS topics
                try:
                    import subprocess as _sp
                    result = _sp.run(["rostopic", "list"], capture_output=True, text=True, timeout=5)
                    topics = result.stdout.strip()
                    print(f"[DEBUG sim] open_scene: active topics:\n{topics}")
                    obs_present = self._observer_topic in topics
                    print(f"[DEBUG sim] open_scene: observer topic {self._observer_topic!r} present={obs_present}")
                except Exception as te:
                    print(f"[DEBUG sim] open_scene: rostopic list failed: {te}")
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
        line_lower = line.lower()
        # Always print lines related to observer or camera plugin
        if 'observer' in line_lower or 'libgazebo_ros_camera' in line_lower:
            print(f"[DEBUG sim] GAZEBO[{stream_type}] {line}")
        if line.startswith('bash:') or 'command not found' in line_lower:
            level = 'warning'
            logger.warning(f"[Gazebo/bash] {line}")
        elif any(word in line_lower for word in ['error', 'exception', 'fail', 'cannot', 'invalid']):
            level = 'error'
            logger.error(f"[Gazebo] {line}")
            # Also print plugin errors to stdout for visibility
            if any(w in line_lower for w in ['plugin', 'libgazebo', 'camera']):
                print(f"[DEBUG sim] GAZEBO PLUGIN ERROR[{stream_type}]: {line}")
        elif 'warning' in line_lower:
            level = 'warning'
            logger.warning(f"[Gazebo] {line}")
        else:
            level = 'info'
        if self.on_log:
            self.on_log(level, f"[Gazebo] {line}")


    def _generate_world(self, world_path, camera_model_path):
        print(f"[DEBUG sim] _generate_world: world={world_path} sensor_sdf={camera_model_path}")
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')

            camera_tree = ET.parse(camera_model_path)
            camera_root = camera_tree.getroot()
            camera_models = camera_root.findall('model')
            print(f"[DEBUG sim] _generate_world: sensor models found={len(camera_models)}")
            for i, camera_model in enumerate(camera_models):
                world.append(camera_model)

            # Inject observer camera directly as XML — no external file needed
            observer_xml = f"""<model name="observer_camera">
  <pose>0 -5 3 0 0.5 1.5708</pose>
  <static>true</static>
  <link name="observer_link">
    <visual name="observer_visual">
      <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
    </visual>
    <sensor name="observer_cam" type="camera">
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>100</far></clip>
      </camera>
      <plugin name="observer_camera_plugin" filename="libgazebo_ros_camera.so">
        <robotNamespace></robotNamespace>
        <cameraName>observer</cameraName>
        <imageTopicName>{self._observer_topic}</imageTopicName>
        <frameName>observer_frame</frameName>
      </plugin>
    </sensor>
  </link>
</model>"""
            observer_el = ET.fromstring(observer_xml)
            world.append(observer_el)
            print(f"[DEBUG sim] _generate_world: observer camera injected, topic={self._observer_topic}")

            tree.write(self.BASE_WORLD_PATH, encoding='utf-8', xml_declaration=True)
            # Verify
            written = open(self.BASE_WORLD_PATH).read()
            print(f"[DEBUG sim] _generate_world: wrote {self.BASE_WORLD_PATH}")
            print(f"[DEBUG sim] _generate_world: 'observer_camera' in file={'observer_camera' in written}")
            print(f"[DEBUG sim] _generate_world: 'imageTopicName' in file={'imageTopicName' in written}")
            logger.info('Base .world file generated')
        except Exception as e:
            import traceback
            print(f"[DEBUG sim] _generate_world FAILED: {e}")
            print(traceback.format_exc())
            logger.error(f'Failed to generate world file: {str(e)}')

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
            self.ros_is_running = False
            logger.info('ROS core stopped')
        except Exception as e:
            logger.error(f'Failed to stop ROS core: {str(e)}')

    def _kill_node(self):
        if not self.node_is_running:
            return
        try:
            rospy.signal_shutdown("Simulator shutdown")
            self.node_is_running = False
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
        print(f"[DEBUG sim] set_step_mode: {enabled}")
        self._step_mode = enabled
        if enabled and self._step_gate is None:
            self._step_gate = threading.Event()
            self._step_gate.set()

    def wait_for_step(self) -> None:
        logger.info(f"[DEBUG sim] wait_for_step: _step_mode={self._step_mode} _step_gate={self._step_gate}")
        if not self._step_mode or self._step_gate is None:
            logger.info("[DEBUG sim] wait_for_step: not in step mode, continuing")
            return
        logger.info("[DEBUG sim] wait_for_step: BLOCKING")
        self._step_gate.clear()
        self._step_gate.wait()
        logger.info("[DEBUG sim] wait_for_step: UNBLOCKED")

    def advance_step(self) -> None:
        logger.info(f"[DEBUG sim] advance_step: _step_gate={self._step_gate}")
        if self._step_gate is not None:
            self._step_gate.set()

    def continue_all(self) -> None:
        self._step_mode = False
        if self._step_gate is not None:
            self._step_gate.set()

    def capture_observer_frame(self) -> bytes | None:
        """Grab one JPEG frame from the observer camera ROS topic.
        Requires: ros-noetic-gazebo-ros-pkgs
        """
        print(f"[DEBUG sim] capture_observer_frame: gazebo_is_running={self.gazebo_is_running} topic={self._observer_topic}")
        if not self.gazebo_is_running:
            return None
        try:
            from sensor_msgs.msg import Image
            msg = rospy.wait_for_message(self._observer_topic, Image, timeout=2.0)
            print(f"[DEBUG sim] capture_observer_frame: got {msg.width}x{msg.height} enc={msg.encoding}")
            import numpy as np
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            import cv2
            # Gazebo camera publishes RGB, cv2 encodes BGR
            arr_bgr = arr[:, :, ::-1].copy()
            ok, buf = cv2.imencode(".jpg", arr_bgr)
            print(f"[DEBUG sim] capture_observer_frame: imencode ok={ok} size={len(buf) if ok else 0}")
            return bytes(buf) if ok else None
        except Exception as e:
            import traceback
            print(f"[DEBUG sim] capture_observer_frame FAILED: {e}")
            print(traceback.format_exc())
            return None
    def notify_capture(self, sensor_data: dict, observer_img: bytes | None = None) -> None:
        """Fire on_capture callback after each sensor capture."""
        import threading
        print(f"[DEBUG sim] notify_capture: thread={threading.current_thread().name} on_capture={self.on_capture} sensor_data_keys={list(sensor_data.keys()) if sensor_data else None} obs_len={len(observer_img) if observer_img else 0}")
        if self.on_capture:
            try:
                self.on_capture(sensor_data, observer_img)
                print("[DEBUG sim] notify_capture: callback returned OK")
            except Exception as e:
                import traceback
                print(f"[DEBUG sim] notify_capture callback ERROR: {e}")
                print(traceback.format_exc())
        else:
            print("[DEBUG sim] notify_capture: on_capture is None — UI not wired!")

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