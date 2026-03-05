from __future__ import annotations

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
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetWorldProperties, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

_MODULE_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _MODULE_DIR.parent
_LOG_CONFIG_PATH = _REPO_ROOT / "log_config.json"

if _LOG_CONFIG_PATH.exists():
    with open(_LOG_CONFIG_PATH, "r", encoding="utf-8") as f_in:
        log_config = json.load(f_in)
    logging.config.dictConfig(log_config)
else:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s - %(message)s")

logger = logging.getLogger(__name__)


class SimulationManager:
    """Единый lifecycle-менеджер ROS/Gazebo для test runner.

    Особенности:
    - поднимает только свои процессы (roscore/roslaunch) через process groups;
    - завершает только процессы, которые сам запускал;
    - собирает диагностику запуска сцены и readiness топиков.
    """

    def __init__(self, CONFIG: dict | None = None):
        self.CONFIG = CONFIG or {}
        self.REPO_ROOT = self._discover_repo_root(self.CONFIG)

        self.CATKIN_SETUP_DIR = self._resolve_path(self.CONFIG.get("CATKIN_SETUP_DIR", "catkin_ws/devel/setup.bash"))
        self.SENSOR_PKG = str(self.CONFIG.get("SENSOR_PKG", "scenario_test_pkg"))
        self.LAUNCH_FILE = str(self.CONFIG.get("LAUNCH_FILE", "scenario.launch"))
        self.TIMEOUT = float(self.CONFIG.get("MESSAGE_TIMEOUT", 10))

        self.BASE_WORLD_PATH = self._resolve_path(
            self.CONFIG.get("BASE_WORLD_PATH", "catkin_ws/src/scenario_test_pkg/worlds/base_world.world")
        )
        self.ROS_LOG_PATH = self._resolve_path(self.CONFIG.get("ROS_LOG_PATH", "ros_log"))

        self.ROS_MASTER_READY_TIMEOUT_S = 30.0
        self.GAZEBO_SERVICES_TIMEOUT_S = 90.0
        self.TOPIC_READY_TIMEOUT_S = 60.0
        self.TOPIC_MSG_WINDOW_S = 2.0
        self.MIN_TOPIC_MESSAGES = 1
        self.REQUIRED_GAZEBO_SERVICES = (
            "/gazebo/get_world_properties",
            "/gazebo/set_model_state",
        )

        self.ros_is_running = False
        self.node_is_running = False
        self.gazebo_is_running = False

        self.ros_process: Optional[subprocess.Popen] = None
        self.gazebo_process: Optional[subprocess.Popen] = None

        self._scene_stdout_thread: Optional[threading.Thread] = None
        self._scene_stderr_thread: Optional[threading.Thread] = None

        self._diag_lock = threading.Lock()
        self._launch_stdout_lines = deque(maxlen=2000)
        self._launch_stderr_lines = deque(maxlen=2000)
        self._last_scene_diagnostics: Dict[str, object] = {}
        self._last_topic_diag: Dict[str, object] = {}
        self._scene_log_files: Dict[str, str] = {}

    @staticmethod
    def _discover_repo_root(config: dict) -> str:
        root = str(config.get("ROOT_PATH", "") or "").strip()
        if root:
            return os.path.abspath(root)
        return os.path.abspath(_REPO_ROOT)

    def _resolve_path(self, path_like: str | os.PathLike | None) -> str:
        if path_like is None:
            return ""
        raw = str(path_like).strip()
        if not raw:
            return ""
        p = Path(raw)
        if p.is_absolute():
            return str(p.resolve())
        return str((Path(self.REPO_ROOT) / p).resolve())

    def _prepare_ros_log_dir(self) -> None:
        os.makedirs(self.ROS_LOG_PATH, exist_ok=True)
        probe = os.path.join(self.ROS_LOG_PATH, ".write_probe")
        with open(probe, "w", encoding="utf-8") as f_out:
            f_out.write("ok\n")
        try:
            os.remove(probe)
        except Exception:
            pass

    def _scene_log_paths(self, attempt: int) -> Dict[str, str]:
        suffix = f"attempt{int(attempt)}"
        return {
            "roslaunch_stdout": os.path.join(self.ROS_LOG_PATH, f"roslaunch.{suffix}.out"),
            "roslaunch_stderr": os.path.join(self.ROS_LOG_PATH, f"roslaunch.{suffix}.err"),
        }

    def _clear_launch_buffers(self) -> None:
        with self._diag_lock:
            self._launch_stdout_lines.clear()
            self._launch_stderr_lines.clear()

    def _append_launch_line(self, stream_type: str, line: str) -> None:
        formatted = f"[{stream_type}] {line.rstrip()}"
        with self._diag_lock:
            if stream_type == "stderr":
                self._launch_stderr_lines.append(formatted)
            else:
                self._launch_stdout_lines.append(formatted)

    def _launch_output_snapshot(self, max_lines: int = 80) -> Dict[str, List[str]]:
        max_lines = max(1, int(max_lines))
        with self._diag_lock:
            return {
                "launch_stdout_tail": list(self._launch_stdout_lines)[-max_lines:],
                "launch_stderr_tail": list(self._launch_stderr_lines)[-max_lines:],
            }

    @staticmethod
    def _extract_missing_model_uris(lines: List[str]) -> List[str]:
        joined = "\n".join(lines)
        matches = re.findall(r"model://([A-Za-z0-9_./-]+)", joined)
        unique: List[str] = []
        for item in matches:
            if item not in unique:
                unique.append(item)
        return unique

    @staticmethod
    def _is_process_alive(proc: Optional[subprocess.Popen]) -> bool:
        return bool(proc is not None and proc.poll() is None)

    @staticmethod
    def _terminate_process_group(proc: Optional[subprocess.Popen], name: str, timeout_s: float = 12.0) -> bool:
        if proc is None:
            return True
        if proc.poll() is not None:
            return True

        try:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                proc.terminate()

            proc.wait(timeout=float(timeout_s))
            return True
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                proc.kill()
            try:
                proc.wait(timeout=3.0)
            except Exception:
                pass
            return proc.poll() is not None
        except Exception as exc:
            logger.warning(f"Failed to terminate process {name}: {exc}")
            return False

    def _drain_stream(self, stream, stream_type: str, out_path: str) -> None:
        f_out = None
        try:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            f_out = open(out_path, "a", encoding="utf-8")
        except Exception:
            f_out = None

        try:
            for line in iter(stream.readline, ""):
                if not line:
                    break
                clean = line.rstrip("\n")
                self._append_launch_line(stream_type, clean)
                if f_out is not None:
                    try:
                        f_out.write(clean + "\n")
                        f_out.flush()
                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            try:
                stream.close()
            except Exception:
                pass
            if f_out is not None:
                try:
                    f_out.close()
                except Exception:
                    pass

    def _start_launch_readers(self, log_files: Dict[str, str]) -> None:
        if self.gazebo_process is None:
            return
        self._scene_stdout_thread = threading.Thread(
            target=self._drain_stream,
            args=(self.gazebo_process.stdout, "stdout", log_files["roslaunch_stdout"]),
            daemon=True,
        )
        self._scene_stderr_thread = threading.Thread(
            target=self._drain_stream,
            args=(self.gazebo_process.stderr, "stderr", log_files["roslaunch_stderr"]),
            daemon=True,
        )
        self._scene_stdout_thread.start()
        self._scene_stderr_thread.start()

    def _set_scene_diag(self, **kwargs) -> None:
        with self._diag_lock:
            self._last_scene_diagnostics.update(kwargs)

    def _replace_scene_diag(self, payload: Dict[str, object]) -> None:
        with self._diag_lock:
            self._last_scene_diagnostics = dict(payload)

    def get_last_scene_diagnostics(self) -> Dict[str, object]:
        with self._diag_lock:
            return copy.deepcopy(self._last_scene_diagnostics)

    @staticmethod
    def _run_shell_capture(cmd: str, timeout_s: float = 4.0, max_lines: int = 200) -> Dict[str, object]:
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=float(timeout_s),
                check=False,
            )
            return {
                "cmd": cmd,
                "returncode": int(result.returncode),
                "stdout_tail": (result.stdout or "").splitlines()[-int(max_lines):],
                "stderr_tail": (result.stderr or "").splitlines()[-int(max_lines):],
            }
        except Exception as exc:
            return {
                "cmd": cmd,
                "returncode": None,
                "stdout_tail": [],
                "stderr_tail": [str(exc)],
            }

    def _all_topics(self) -> List[str]:
        try:
            published = rospy.get_published_topics()
            return sorted([name for name, _ in published])
        except Exception:
            snap = self._run_shell_capture("rostopic list || true", timeout_s=3.0, max_lines=300)
            return [str(line).strip() for line in snap.get("stdout_tail", []) if str(line).strip()]

    def _image_topics(self) -> List[str]:
        try:
            published = rospy.get_published_topics()
            return sorted([name for name, msg_type in published if msg_type == "sensor_msgs/Image"])
        except Exception:
            return sorted([name for name in self._all_topics() if "image" in name])

    @staticmethod
    def _match_expected_topic(expected: str, published_topics: List[str]) -> str:
        if not expected:
            return ""
        if expected in published_topics:
            return expected

        parts = [p for p in expected.strip("/").split("/") if p]
        token = parts[0] if parts else ""
        tail2 = "/" + "/".join(parts[-2:]) if len(parts) >= 2 else expected
        tail1 = "/" + parts[-1] if parts else expected

        candidates = [t for t in published_topics if t.endswith(tail2)]
        if token and len(candidates) > 1:
            token_candidates = [t for t in candidates if token in t]
            if token_candidates:
                candidates = token_candidates
        if candidates:
            return sorted(candidates, key=lambda s: (len(s), s))[0]

        candidates = [t for t in published_topics if t.endswith(tail1)]
        if token and len(candidates) > 1:
            token_candidates = [t for t in candidates if token in t]
            if token_candidates:
                candidates = token_candidates
        if candidates:
            return sorted(candidates, key=lambda s: (len(s), s))[0]

        return ""

    def _count_topic_messages(self, topics: List[str], window_s: float = 2.0) -> Tuple[Dict[str, int], Dict[str, float]]:
        counters: Dict[str, int] = {topic: 0 for topic in topics}
        lock = threading.Lock()
        subscribers = []

        def _cb_factory(topic_name: str):
            def _cb(_msg: Image) -> None:
                with lock:
                    counters[topic_name] = counters.get(topic_name, 0) + 1
            return _cb

        for topic in topics:
            subscribers.append(rospy.Subscriber(topic, Image, _cb_factory(topic), queue_size=100))

        started = time.time()
        try:
            while time.time() - started < float(window_s):
                time.sleep(0.05)
        finally:
            for sub in subscribers:
                try:
                    sub.unregister()
                except Exception:
                    pass

        elapsed = max(1e-6, time.time() - started)
        hz = {topic: float(counters.get(topic, 0) / elapsed) for topic in topics}
        return counters, hz

    def wait_for_topics(
        self,
        expected_topics: List[str],
        timeout_s: float | None = None,
        msg_window_s: float | None = None,
        min_messages: int | None = None,
    ) -> Tuple[bool, Dict[str, object]]:
        timeout_s = float(timeout_s or self.TOPIC_READY_TIMEOUT_S)
        msg_window_s = float(msg_window_s or self.TOPIC_MSG_WINDOW_S)
        min_messages = int(min_messages or self.MIN_TOPIC_MESSAGES)

        expected = [str(topic) for topic in expected_topics if str(topic).strip()]
        diag: Dict[str, object] = {
            "expected_topics": list(expected),
            "resolved_topics": {},
            "msg_counters": {},
            "msg_hz": {},
            "published_topics_last": [],
            "timeout_s": float(timeout_s),
            "msg_window_s": float(msg_window_s),
            "min_messages": int(min_messages),
            "reason": "",
        }

        if not expected:
            diag["reason"] = "no_expected_topics"
            self._last_topic_diag = dict(diag)
            return True, diag

        resolved: Dict[str, str] = {}
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            if self._is_process_alive(self.gazebo_process) is False and self.gazebo_process is not None:
                diag["reason"] = "roslaunch_exited_while_waiting_topics"
                break

            published = self._image_topics()
            diag["published_topics_last"] = list(published)

            for expected_topic in expected:
                if expected_topic in resolved:
                    continue
                selected = self._match_expected_topic(expected_topic, published)
                if selected:
                    resolved[expected_topic] = selected

            if len(resolved) == len(expected):
                unique_topics = sorted(set(resolved.values()))
                msg_counters, msg_hz = self._count_topic_messages(unique_topics, window_s=msg_window_s)
                diag["resolved_topics"] = dict(resolved)
                diag["msg_counters"] = dict(msg_counters)
                diag["msg_hz"] = dict(msg_hz)
                missing_messages = [topic for topic, count in msg_counters.items() if int(count) < int(min_messages)]
                if missing_messages:
                    diag["reason"] = "topics_no_messages"
                    diag["topics_without_messages"] = missing_messages
                    self._last_topic_diag = dict(diag)
                    return False, diag

                diag["reason"] = "ok"
                self._last_topic_diag = dict(diag)
                return True, diag

            time.sleep(0.2)

        if not diag.get("reason"):
            diag["reason"] = "topics_not_found"
        diag["resolved_topics"] = dict(resolved)
        diag["missing_expected_topics"] = [topic for topic in expected if topic not in resolved]
        self._last_topic_diag = dict(diag)
        return False, diag

    def _wait_ros_master_ready(self, timeout_s: float | None = None) -> Tuple[bool, str]:
        timeout_s = float(timeout_s or self.ROS_MASTER_READY_TIMEOUT_S)
        deadline = time.time() + timeout_s
        last_error = ""

        while time.time() < deadline:
            if self.ros_process is not None and self.ros_process.poll() is not None:
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

    def wait_for_master(self, timeout_s: float | None = None) -> Tuple[bool, str]:
        return self._wait_ros_master_ready(timeout_s=timeout_s)

    def _wait_for_services(self, services: Tuple[str, ...], timeout_s: float) -> Tuple[bool, Dict[str, str], str]:
        deadline = time.time() + float(timeout_s)
        pending = list(dict.fromkeys(services))
        last_errors: Dict[str, str] = {}

        while pending and time.time() < deadline:
            if self.gazebo_process is not None and self.gazebo_process.poll() is not None:
                return False, dict(last_errors), "roslaunch_exited"

            still_pending = []
            for service_name in pending:
                try:
                    rospy.wait_for_service(service_name, timeout=0.25)
                except Exception as exc:
                    last_errors[service_name] = str(exc)
                    still_pending.append(service_name)
            pending = still_pending
            if pending:
                time.sleep(0.15)

        if pending:
            return False, {name: last_errors.get(name, "timeout") for name in pending}, "gazebo_services_timeout"
        return True, {}, "ok"

    def _collect_runtime_diag(self, expected_topics: Optional[List[str]] = None) -> Dict[str, object]:
        snapshot = self._launch_output_snapshot(max_lines=80)
        lines = list(snapshot["launch_stdout_tail"]) + list(snapshot["launch_stderr_tail"])
        scene = self.get_last_scene_diagnostics()

        diag: Dict[str, object] = {
            "cwd": os.path.abspath(os.getcwd()),
            "world_path_abs": scene.get("world_path_abs"),
            "generated_world_path_abs": scene.get("generated_world_path_abs"),
            "launch_stdout_tail": snapshot["launch_stdout_tail"],
            "launch_stderr_tail": snapshot["launch_stderr_tail"],
            "missing_model_uris": self._extract_missing_model_uris(lines),
            "expected_topics": list(expected_topics or []),
            "resolved_topics": self._last_topic_diag.get("resolved_topics", {}),
            "msg_counters": self._last_topic_diag.get("msg_counters", {}),
            "msg_hz": self._last_topic_diag.get("msg_hz", {}),
            "published_topics_last": self._last_topic_diag.get("published_topics_last", []),
            "rostopic_list": self._all_topics(),
            "scene_log_files": dict(self._scene_log_files),
        }
        return diag

    def collect_failure_diagnostics(self, expected_topics: Optional[List[str]] = None) -> Dict[str, object]:
        return self._collect_runtime_diag(expected_topics=expected_topics)

    def launch_ros(self) -> None:
        self._kill_ros()
        self._prepare_ros_log_dir()
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = self.ROS_LOG_PATH

        roscore_stdout = os.path.join(self.ROS_LOG_PATH, "roscore.out")
        roscore_stderr = os.path.join(self.ROS_LOG_PATH, "roscore.err")
        cmd = "roscore"

        with open(roscore_stdout, "a", encoding="utf-8") as out, open(roscore_stderr, "a", encoding="utf-8") as err:
            self.ros_process = subprocess.Popen(
                ["bash", "-lc", cmd],
                env=env,
                stdout=out,
                stderr=err,
                text=True,
                start_new_session=True,
            )

        ready, reason = self._wait_ros_master_ready()
        if not ready:
            self.ros_is_running = False
            self._set_scene_diag(reason="ros_master_not_ready", ros_master_error=reason)
            self._kill_ros()
            return

        self.ros_is_running = True
        self._set_scene_diag(ros_master_ready=True)
        logger.info("ROS core started and master is ready")

    def launch_node(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            logger.error("ROS node must be initialized in main thread")
            return

        try:
            if not rospy.core.is_initialized():
                rospy.init_node("sensor_data_receiver", anonymous=True)
            self.node_is_running = True
            logger.info("ROS node initialized successfully")
        except Exception as exc:
            self.node_is_running = False
            logger.error(f"Failed to initialize ROS node: {exc}")

    def launch(self) -> None:
        self.launch_ros()
        if not self.ros_is_running:
            return
        self.launch_node()

    def receive_sensor_data(self, topic: str):
        try:
            return rospy.wait_for_message(topic, Image, timeout=self.TIMEOUT)
        except Exception as exc:
            logger.error(f"Failed to receive sensor data from topic {topic}: {exc}")
            return None

    def is_gazebo_running(self) -> bool:
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=2.0)
            proxy = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
            proxy()
            return True
        except Exception:
            return False

    def _scene_world_output_path(self, source_world_abs: str) -> str:
        scene_dir = os.path.join(self.ROS_LOG_PATH, "generated_worlds")
        os.makedirs(scene_dir, exist_ok=True)
        stamp = int(time.time() * 1000)
        base = os.path.basename(source_world_abs) or "scene.world"
        root, ext = os.path.splitext(base)
        if not ext:
            ext = ".world"
        return os.path.abspath(os.path.join(scene_dir, f"{root}_{stamp}{ext}"))

    def _generate_world(self, world_path: str, camera_model_path: str, output_world_path: str) -> Tuple[bool, str]:
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find("world")
            if world is None:
                return False, "No <world> node in world file"

            camera_tree = ET.parse(camera_model_path)
            camera_root = camera_tree.getroot()
            camera_models = camera_root.findall("model")
            if not camera_models:
                return False, "No <model> in camera SDF"

            for model in camera_models:
                world.append(model)

            os.makedirs(os.path.dirname(output_world_path), exist_ok=True)
            tree.write(output_world_path, encoding="utf-8", xml_declaration=True)
            ET.parse(output_world_path)  # validation
            return True, ""
        except Exception as exc:
            return False, str(exc)

    def open_scene(
        self,
        world_path: str,
        camera_model_path: str,
        expected_topics: Optional[List[str]] = None,
    ) -> bool:
        self._clear_launch_buffers()
        self._last_topic_diag = {}

        world_abs = self._resolve_path(world_path)
        sensor_abs = self._resolve_path(camera_model_path)
        expected_topics = [str(topic) for topic in (expected_topics or []) if str(topic).strip()]

        scene_diag = {
            "attempt_ts": time.time(),
            "cwd": os.path.abspath(os.getcwd()),
            "repo_root": self.REPO_ROOT,
            "world_path_input": str(world_path),
            "world_path_abs": world_abs,
            "world_exists": os.path.exists(world_abs),
            "camera_model_path_input": str(camera_model_path),
            "camera_model_path_abs": sensor_abs,
            "camera_model_exists": os.path.exists(sensor_abs),
            "catkin_setup_abs": self.CATKIN_SETUP_DIR,
            "catkin_setup_exists": os.path.exists(self.CATKIN_SETUP_DIR),
            "expected_topics": list(expected_topics),
            "reason": "",
        }
        self._replace_scene_diag(scene_diag)

        if not os.path.exists(world_abs):
            self._set_scene_diag(reason="world_path_not_found")
            return False
        if not os.path.exists(sensor_abs):
            self._set_scene_diag(reason="camera_model_path_not_found")
            return False
        if not os.path.exists(self.CATKIN_SETUP_DIR):
            self._set_scene_diag(reason="catkin_setup_not_found")
            return False

        # Для каждого теста гарантируем чистый lifecycle только своих процессов.
        self.kill_gazebo()

        if not self.ros_is_running or not self._is_process_alive(self.ros_process):
            self.launch_ros()
            if not self.ros_is_running:
                runtime = self._collect_runtime_diag(expected_topics=expected_topics)
                self._set_scene_diag(reason="ros_not_running", **runtime)
                return False

        if not self.node_is_running:
            self.launch_node()
            if not self.node_is_running:
                runtime = self._collect_runtime_diag(expected_topics=expected_topics)
                self._set_scene_diag(reason="ros_node_not_running", **runtime)
                return False

        self._prepare_ros_log_dir()
        generated_world = self._scene_world_output_path(world_abs)
        generated_ok, generated_error = self._generate_world(world_abs, sensor_abs, generated_world)
        if not generated_ok:
            runtime = self._collect_runtime_diag(expected_topics=expected_topics)
            self._set_scene_diag(
                reason="base_world_generation_failed",
                generation_error=generated_error,
                generated_world_path_abs=generated_world,
                **runtime,
            )
            return False

        log_files = self._scene_log_paths(attempt=1)
        self._scene_log_files = dict(log_files)

        roslaunch_cmd = (
            f"source {shlex.quote(self.CATKIN_SETUP_DIR)} && "
            f"roslaunch {shlex.quote(self.SENSOR_PKG)} {shlex.quote(self.LAUNCH_FILE)} "
            f"world_path:={shlex.quote(generated_world)} "
            "paused:=false gui:=false headless:=true"
        )

        launch_env = os.environ.copy()
        launch_env["ROS_LOG_DIR"] = self.ROS_LOG_PATH

        self._set_scene_diag(
            generated_world_path_abs=generated_world,
            roslaunch_cmd=roslaunch_cmd,
            launch_log_files=dict(log_files),
        )

        try:
            self.gazebo_process = subprocess.Popen(
                ["bash", "-lc", roslaunch_cmd],
                env=launch_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self._start_launch_readers(log_files)
        except Exception as exc:
            runtime = self._collect_runtime_diag(expected_topics=expected_topics)
            self._set_scene_diag(reason="roslaunch_spawn_exception", exception=str(exc), **runtime)
            return False

        services_ok, service_errors, service_reason = self._wait_for_services(
            self.REQUIRED_GAZEBO_SERVICES,
            timeout_s=self.GAZEBO_SERVICES_TIMEOUT_S,
        )
        if not services_ok:
            runtime = self._collect_runtime_diag(expected_topics=expected_topics)
            self._set_scene_diag(
                reason=service_reason,
                services_expected=list(self.REQUIRED_GAZEBO_SERVICES),
                services_errors=service_errors,
                roslaunch_returncode=self.gazebo_process.poll() if self.gazebo_process else None,
                **runtime,
            )
            self.kill_gazebo()
            return False

        if expected_topics:
            topics_ok, topics_diag = self.wait_for_topics(expected_topics=expected_topics, timeout_s=self.TOPIC_READY_TIMEOUT_S)
            self._set_scene_diag(
                expected_topics=list(expected_topics),
                resolved_topics=topics_diag.get("resolved_topics", {}),
                msg_counters=topics_diag.get("msg_counters", {}),
                msg_hz=topics_diag.get("msg_hz", {}),
            )
            if not topics_ok:
                runtime = self._collect_runtime_diag(expected_topics=expected_topics)
                self._set_scene_diag(reason="topics_not_ready", topics_wait=topics_diag, **runtime)
                self.kill_gazebo()
                return False

        self.gazebo_is_running = True
        runtime = self._collect_runtime_diag(expected_topics=expected_topics)
        self._set_scene_diag(reason="ok", **runtime)
        logger.info("Gazebo scene started")
        return True

    def wait_for_model_spawn(self, model_name: str, timeout: float = 10.0) -> bool:
        started = time.time()
        while (time.time() - started) < float(timeout):
            try:
                msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=1.0)
                if model_name in msg.name:
                    return True
            except rospy.ROSException:
                continue
        return False

    def set_pose(self, model: str, x: float = 0, y: float = 0, z: float = 0) -> None:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = model
        state.reference_frame = "world"
        state.pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0, 0, 0, 1))
        response = set_state(state)
        if not response.success:
            raise RuntimeError(response.status_message)

    @staticmethod
    def _image_msg_to_bgr(msg: Image):
        import cv2
        import numpy as np

        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "").lower()

        if enc in ("rgb8", "r8g8b8"):
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if enc == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if enc in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"Unsupported image encoding for debug capture: {msg.encoding}")

    def capture_debug_frame(self, topic: str, output_path: str, timeout_s: float = 3.0) -> Dict[str, object]:
        resolved = topic
        published = self._image_topics()
        if topic not in published:
            resolved_alt = self._match_expected_topic(topic, published)
            if resolved_alt:
                resolved = resolved_alt

        payload: Dict[str, object] = {
            "requested_topic": topic,
            "resolved_topic": resolved,
            "output_path": output_path,
            "ok": False,
            "error": "",
        }
        try:
            msg = rospy.wait_for_message(resolved, Image, timeout=float(timeout_s))
            frame = self._image_msg_to_bgr(msg)
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            import cv2

            cv2.imwrite(output_path, frame)
            payload["ok"] = True
        except Exception as exc:
            payload["error"] = str(exc)
        return payload

    def kill_gazebo(self) -> None:
        if self._is_process_alive(self.gazebo_process):
            self._terminate_process_group(self.gazebo_process, "roslaunch_gazebo", timeout_s=10.0)
        self.gazebo_process = None
        self.gazebo_is_running = False

    def _kill_node(self) -> None:
        if not self.node_is_running:
            return
        try:
            rospy.signal_shutdown("SimulationManager shutdown")
        except Exception:
            pass
        self.node_is_running = False

    def _kill_ros(self) -> None:
        if self._is_process_alive(self.ros_process):
            self._terminate_process_group(self.ros_process, "roscore", timeout_s=8.0)
        self.ros_process = None
        self.ros_is_running = False

    def kill(self) -> None:
        self.kill_gazebo()
        self._kill_node()
        self._kill_ros()
        self.gazebo_is_running = False
        self.node_is_running = False
        self.ros_is_running = False

    def wait_gazebo_quiet(self, timeout: float = 30.0) -> bool:
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            try:
                proxy = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
                proxy()
                return True
            except Exception:
                time.sleep(0.2)
        return False


class Simulator(SimulationManager):
    """Обратная совместимость со старым именем класса."""
