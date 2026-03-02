import copy
import json
import logging
import logging.config
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from typing import Dict, List, Tuple

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
        self._launch_stdout_lines = deque(maxlen=1200)
        self._launch_stderr_lines = deque(maxlen=1200)
        self._last_scene_diagnostics = {}
        self._last_ros_launch_error = ""

        self.ROS_MASTER_READY_TIMEOUT_S = 30.0
        self.GAZEBO_SERVICES_TIMEOUT_S = 45.0
        self.GAZEBO_REQUIRED_SERVICES = (
            "/gazebo/get_world_properties",
            "/gazebo/spawn_sdf_model",
            "/gazebo/set_model_state",
        )

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

    @staticmethod
    def _extract_gazebo_log_paths(lines) -> list:
        joined = "\n".join(lines)
        paths = []

        patterns = [
            r"__log:=([^\s\]\)]+\.log)",
            r"(\/[^\s\]\)]*(?:gazebo|gzserver)[^\s\]\)]*\.log)",
        ]

        for pattern in patterns:
            for match in re.findall(pattern, joined):
                path = match.strip().strip('"').strip("'")
                if path and path not in paths:
                    paths.append(path)
        return paths

    @staticmethod
    def _read_file_tail(path: str, max_lines: int = 250) -> List[str]:
        try:
            if not path or not os.path.exists(path):
                return [f"<log-not-found> {path}"]
            with open(path, "r", encoding="utf-8", errors="replace") as f_in:
                return list(deque((line.rstrip("\n") for line in f_in), maxlen=max_lines))
        except Exception as exc:
            return [f"<log-read-error> {path}: {exc}"]

    @staticmethod
    def _discover_recent_gazebo_logs(search_dirs: List[str], max_files: int = 4) -> List[str]:
        candidates: List[Tuple[float, str]] = []
        for root in search_dirs:
            if not root or not os.path.isdir(root):
                continue
            try:
                for dirpath, _, filenames in os.walk(root):
                    for name in filenames:
                        low = name.lower()
                        if not low.endswith(".log"):
                            continue
                        if "gazebo" not in low and "gzserver" not in low:
                            continue
                        path = os.path.join(dirpath, name)
                        try:
                            mtime = float(os.path.getmtime(path))
                        except Exception:
                            mtime = 0.0
                        candidates.append((mtime, path))
            except Exception:
                continue

        candidates.sort(key=lambda item: item[0], reverse=True)
        paths: List[str] = []
        for _, path in candidates:
            if path not in paths:
                paths.append(path)
            if len(paths) >= int(max_files):
                break
        return paths

    @staticmethod
    def _run_shell_capture(cmd: str, timeout_s: float = 5.0, max_lines: int = 200) -> Dict[str, object]:
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=float(timeout_s),
                check=False,
            )
            out_lines = (result.stdout or "").splitlines()[-int(max_lines):]
            err_lines = (result.stderr or "").splitlines()[-int(max_lines):]
            return {
                "cmd": cmd,
                "returncode": int(result.returncode),
                "stdout_tail": out_lines,
                "stderr_tail": err_lines,
            }
        except Exception as exc:
            return {
                "cmd": cmd,
                "returncode": None,
                "stdout_tail": [],
                "stderr_tail": [str(exc)],
            }

    def _collect_ros_runtime_diagnostics(self, sensor_token: str = "") -> Dict[str, object]:
        token = (sensor_token or "").strip()
        topic_cmd = "rostopic list"
        if token:
            topic_cmd = f"rostopic list | grep {shlex.quote(token)} || true"
        return {
            "rosservice_gazebo": self._run_shell_capture("rosservice list | grep gazebo || true"),
            "rosnode_list": self._run_shell_capture("rosnode list || true"),
            "rostopic_sensor": self._run_shell_capture(topic_cmd),
        }

    def _collect_launch_failure_details(self, snapshot: Dict[str, List[str]]) -> Dict[str, object]:
        lines = snapshot.get("launch_stdout_tail", []) + snapshot.get("launch_stderr_tail", [])
        missing_model_uris = self._extract_missing_model_uris(lines)
        launch_log_paths = self._extract_gazebo_log_paths(lines)

        fallback_roots = [
            self.ROS_LOG_PATH,
            os.path.expanduser("~/.ros/log"),
        ]
        discovered = self._discover_recent_gazebo_logs(fallback_roots, max_files=4)

        gazebo_logs: List[str] = []
        for path in launch_log_paths + discovered:
            if path not in gazebo_logs:
                gazebo_logs.append(path)

        gazebo_log_tails = {path: self._read_file_tail(path) for path in gazebo_logs}
        return {
            "missing_model_uris": missing_model_uris,
            "gazebo_log_paths": gazebo_logs,
            "gazebo_log_tails": gazebo_log_tails,
        }

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

    def _wait_ros_master_ready(self, timeout_s: float = None) -> Tuple[bool, str]:
        timeout_s = float(timeout_s or self.ROS_MASTER_READY_TIMEOUT_S)
        deadline = time.time() + timeout_s
        last_error = ""

        while time.time() < deadline:
            if hasattr(self, "ros_process") and self.ros_process and self.ros_process.poll() is not None:
                return False, f"roscore exited early with code {self.ros_process.poll()}"
            try:
                import rosgraph
                master = rosgraph.Master("/sensor_data_receiver")
                _ = master.getPid()
                return True, ""
            except Exception as exc:
                last_error = str(exc)
                time.sleep(0.2)

        if not last_error:
            last_error = f"ROS master did not become ready within {timeout_s:.1f}s"
        return False, last_error

    def _scene_world_output_path(self, source_world_abs: str) -> str:
        scene_dir = os.path.join(self.ROS_LOG_PATH, "generated_worlds")
        os.makedirs(scene_dir, exist_ok=True)
        stamp = int(time.time() * 1000)
        base_name = os.path.basename(source_world_abs) or "scene.world"
        root, ext = os.path.splitext(base_name)
        if not ext:
            ext = ".world"
        return os.path.abspath(os.path.join(scene_dir, f"{root}_{stamp}{ext}"))

    @staticmethod
    def _terminate_process(proc: subprocess.Popen, name: str, timeout_s: float = 12.0) -> bool:
        if proc is None:
            return True
        try:
            if proc.poll() is not None:
                return True
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                proc.terminate()

            deadline = time.time() + float(timeout_s)
            while time.time() < deadline:
                if proc.poll() is not None:
                    return True
                time.sleep(0.2)

            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                proc.kill()
            time.sleep(0.3)
            return proc.poll() is not None
        except Exception as exc:
            logger.warning(f"Failed to terminate process {name}: {exc}")
            return False

    @staticmethod
    def _wait_for_services(
        services: Tuple[str, ...],
        timeout_s: float,
    ) -> Tuple[bool, Dict[str, str], List[Dict[str, object]]]:
        deadline = time.time() + timeout_s
        pending = list(services)
        last_errors: Dict[str, str] = {}
        attempts: List[Dict[str, object]] = []
        attempt_num = 0

        while time.time() < deadline:
            attempt_num += 1
            still_pending = []
            for srv in pending:
                try:
                    rospy.wait_for_service(srv, timeout=0.5)
                except Exception as exc:
                    last_errors[srv] = str(exc)
                    still_pending.append(srv)
            attempts.append(
                {
                    "attempt": int(attempt_num),
                    "pending_services": list(still_pending),
                    "elapsed_s": float(timeout_s - max(0.0, deadline - time.time())),
                }
            )
            if still_pending:
                logger.info(f"Waiting Gazebo services (attempt {attempt_num}): pending={still_pending}")
            pending = still_pending
            if not pending:
                return True, {}, attempts
            time.sleep(0.1)

        return False, {srv: last_errors.get(srv, "timeout") for srv in pending}, attempts

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
            ready, reason = self._wait_ros_master_ready(timeout_s=self.ROS_MASTER_READY_TIMEOUT_S)
            if not ready:
                self.ros_is_running = False
                self._last_ros_launch_error = reason
                logger.error(f"ROS master is not ready: {reason}")
                self._kill_ros()
                return

            self._last_ros_launch_error = ""
            self.ros_is_running = True
            logger.info('ROS core started and master is ready')
        except Exception as e:
            self.ros_is_running = False
            self._last_ros_launch_error = str(e)
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
        if not self.ros_is_running:
            logger.error("Skipping ROS node init because ROS master is not ready")
            return
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
        sensor_token = os.path.splitext(os.path.basename(sensor_abs))[0] if sensor_abs else ""
        launch_world_abs = self._scene_world_output_path(world_abs)

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
            "launch_world_path_abs": launch_world_abs,
            "env": {
                "GAZEBO_MODEL_PATH": os.environ.get("GAZEBO_MODEL_PATH", ""),
                "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", ""),
                "GAZEBO_RESOURCE_PATH": os.environ.get("GAZEBO_RESOURCE_PATH", ""),
                "GAZEBO_PLUGIN_PATH": os.environ.get("GAZEBO_PLUGIN_PATH", ""),
            },
            "reason": "",
        }
        self._replace_scene_diag(diag)

        # Перед запуском новой сцены закрываем процессы прошлого запуска
        # и дожидаемся их завершения.
        if hasattr(self, "gazebo_process") and self.gazebo_process:
            self._terminate_process(self.gazebo_process, "roslaunch_gazebo", timeout_s=10.0)
            self.gazebo_process = None
        self.kill_gazebo()
        time.sleep(1.0)

        if not self.ros_is_running:
            logger.error('Failed to start Gazebo: Ros is not running')
            self._set_scene_diag(
                reason="ros_not_running",
                ros_master_error=self._last_ros_launch_error,
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
            )
            return False

        if not self.node_is_running:
            logger.error('Failed to start Gazebo: Node is not running')
            self._set_scene_diag(
                reason="ros_node_not_running",
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
            )
            return False

        if not os.path.exists(world_abs):
            self._set_scene_diag(
                reason="world_path_not_found",
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
            )
            logger.error(f"World path does not exist: {world_abs}")
            return False

        if not os.path.exists(sensor_abs):
            self._set_scene_diag(
                reason="camera_model_path_not_found",
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
            )
            logger.error(f"Camera model path does not exist: {sensor_abs}")
            return False

        if not os.path.exists(catkin_setup_abs):
            self._set_scene_diag(
                reason="catkin_setup_not_found",
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
            )
            logger.error(f"catkin setup not found: {catkin_setup_abs}")
            return False

        generated_ok, generate_error = self._generate_world(world_abs, sensor_abs, launch_world_abs)
        if not generated_ok:
            snapshot = self._launch_output_snapshot()
            failure_details = self._collect_launch_failure_details(snapshot)
            self._set_scene_diag(
                reason="base_world_generation_failed",
                generation_error=generate_error,
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
                **failure_details,
                **snapshot,
            )
            return False

        roslaunch_cmd = (
            f"source {shlex.quote(catkin_setup_abs)} && "
            f"roslaunch {shlex.quote(self.SENSOR_PKG)} {shlex.quote(self.LAUNCH_FILE)} "
            f"world_path:={shlex.quote(launch_world_abs)} "
            "paused:=false gui:=false headless:=true"
        )
        launch_env = os.environ.copy()
        launch_env["ROS_LOG_DIR"] = self.ROS_LOG_PATH
        self._set_scene_diag(
            roslaunch_cmd=roslaunch_cmd,
            launch_env={
                "GAZEBO_MODEL_PATH": launch_env.get("GAZEBO_MODEL_PATH", ""),
                "ROS_PACKAGE_PATH": launch_env.get("ROS_PACKAGE_PATH", ""),
                "GAZEBO_RESOURCE_PATH": launch_env.get("GAZEBO_RESOURCE_PATH", ""),
                "GAZEBO_PLUGIN_PATH": launch_env.get("GAZEBO_PLUGIN_PATH", ""),
                "ROS_LOG_DIR": launch_env.get("ROS_LOG_DIR", ""),
            },
        )

        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-c", roslaunch_cmd],
                env=launch_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
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

            services_ok, pending_errors, service_attempts = self._wait_for_services(
                self.GAZEBO_REQUIRED_SERVICES,
                timeout_s=self.GAZEBO_SERVICES_TIMEOUT_S,
            )
            if not services_ok:
                snapshot = self._launch_output_snapshot()
                failure_details = self._collect_launch_failure_details(snapshot)
                self._set_scene_diag(
                    reason="gazebo_services_not_available",
                    roslaunch_returncode=self.gazebo_process.poll(),
                    services_expected=list(self.GAZEBO_REQUIRED_SERVICES),
                    services_pending_errors=pending_errors,
                    services_wait_attempts=service_attempts,
                    ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
                    **failure_details,
                    **snapshot,
                )
                logger.error("Gazebo services did not appear")
                return False

            self.gazebo_is_running = True

            if self.is_gazebo_running():
                snapshot = self._launch_output_snapshot()
                failure_details = self._collect_launch_failure_details(snapshot)
                self._set_scene_diag(
                    reason="ok",
                    roslaunch_returncode=self.gazebo_process.poll(),
                    services_expected=list(self.GAZEBO_REQUIRED_SERVICES),
                    services_wait_attempts=service_attempts,
                    **failure_details,
                    **snapshot,
                )
                logger.info('Gazebo started')
                return True

            snapshot = self._launch_output_snapshot()
            failure_details = self._collect_launch_failure_details(snapshot)
            self._set_scene_diag(
                reason="gazebo_not_running_after_start",
                roslaunch_returncode=self.gazebo_process.poll(),
                services_expected=list(self.GAZEBO_REQUIRED_SERVICES),
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
                **failure_details,
                **snapshot,
            )
            logger.error('Failed to start Gazebo')
            return False

        except Exception as e:
            snapshot = self._launch_output_snapshot()
            failure_details = self._collect_launch_failure_details(snapshot)
            self._set_scene_diag(
                reason="open_scene_exception",
                exception=str(e),
                roslaunch_returncode=getattr(self, "gazebo_process", None).poll() if hasattr(self, "gazebo_process") else None,
                services_expected=list(self.GAZEBO_REQUIRED_SERVICES),
                ros_runtime=self._collect_ros_runtime_diagnostics(sensor_token=sensor_token),
                **failure_details,
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

    def _generate_world(self, world_path, camera_model_path, output_world_path):
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

            os.makedirs(os.path.dirname(output_world_path), exist_ok=True)
            tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
            self._set_scene_diag(generated_world_path_abs=os.path.abspath(output_world_path))
            logger.info(f'Generated launch world: {output_world_path}')
            return True, ""
        except Exception as e:
            msg = f'Failed to generate world file: {str(e)}'
            logger.error(msg)
            return False, str(e)

    def _kill_ros(self):
        ros_process_alive = bool(
            hasattr(self, "ros_process")
            and self.ros_process
            and self.ros_process.poll() is None
        )
        if not self.ros_is_running and not ros_process_alive:
            return
        try:
            if ros_process_alive:
                self._terminate_process(self.ros_process, "roscore", timeout_s=8.0)

            subprocess.run(
                ["bash", "-c", "pkill -f ros"],
                capture_output=True,
                timeout=10,
            )
            self.ros_is_running = False
            self.ros_process = None
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
            if hasattr(self, "gazebo_process") and self.gazebo_process:
                self._terminate_process(self.gazebo_process, "roslaunch_gazebo", timeout_s=10.0)
                self.gazebo_process = None

            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            subprocess.run(["pkill", "-f", "roslaunch"], check=False)
            deadline = time.time() + 8.0
            while time.time() < deadline:
                if not self.is_gazebo_running():
                    break
                time.sleep(0.2)
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
