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
from typing import Dict, List, Optional, Tuple, Union

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, GetModelProperties, GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from rosgraph_msgs.msg import Clock
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

    def __init__(self, CONFIG: Optional[dict] = None):
        self.CONFIG = CONFIG or {}
        self.REPO_ROOT = self._discover_repo_root(self.CONFIG)

        self.CATKIN_SETUP_DIR = self._resolve_path(self.CONFIG.get("CATKIN_SETUP_DIR", "catkin_ws/devel/setup.bash"))
        self.SENSOR_PKG = str(self.CONFIG.get("SENSOR_PKG", "scenario_test_pkg"))
        self.LAUNCH_FILE = str(self.CONFIG.get("LAUNCH_FILE", "scenario.launch"))
        self.TIMEOUT = float(self.CONFIG.get("MESSAGE_TIMEOUT", 10))

        self.BASE_WORLD_PATH = self._resolve_path(
            self.CONFIG.get("BASE_WORLD_PATH", "catkin_ws/src/scenario_test_pkg/worlds/base_world.world")
        )
        self.ROS_LOG_PATH = os.path.abspath(self._resolve_path(self.CONFIG.get("ROS_LOG_PATH", "ros_log")))

        self.ROS_MASTER_READY_TIMEOUT_S = float(self.CONFIG.get("ROS_MASTER_READY_TIMEOUT_S", 30.0))
        self.GAZEBO_SERVICES_TIMEOUT_S = float(self.CONFIG.get("GAZEBO_LAUNCH_READY_TIMEOUT_S", 30.0))
        self.CLOCK_READY_TIMEOUT_S = float(self.CONFIG.get("GAZEBO_CLOCK_READY_TIMEOUT_S", 30.0))
        self.TOPIC_READY_TIMEOUT_S = float(self.CONFIG.get("GAZEBO_TOPIC_READY_TIMEOUT_S", 30.0))
        self.TOPIC_MSG_WINDOW_S = 2.0
        self.MIN_TOPIC_MESSAGES = 1
        self.REQUIRED_GAZEBO_SERVICES = (
            "/gazebo/get_world_properties",
            "/gazebo/get_model_state",
            "/gazebo/set_model_state",
            "/gazebo/spawn_sdf_model",
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

    def _resolve_path(self, path_like: Optional[Union[str, os.PathLike]]) -> str:
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
    def _split_env_paths(raw_value: str) -> List[str]:
        return [item for item in str(raw_value or "").split(os.pathsep) if item]

    @classmethod
    def _merge_env_paths(cls, extras: List[str], existing_raw: str) -> str:
        merged: List[str] = []
        for entry in list(extras) + cls._split_env_paths(existing_raw):
            val = str(entry or "").strip()
            if not val:
                continue
            if val not in merged:
                merged.append(val)
        return os.pathsep.join(merged)

    @staticmethod
    def _existing_dirs(paths: List[Path]) -> List[str]:
        existing: List[str] = []
        for path in paths:
            try:
                if path.exists() and path.is_dir():
                    resolved = str(path.resolve())
                    if resolved not in existing:
                        existing.append(resolved)
            except Exception:
                continue
        return existing

    @staticmethod
    def _env_diag_subset(env: Dict[str, str]) -> Dict[str, str]:
        keys = ("ROS_LOG_DIR", "GAZEBO_MODEL_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_RESOURCE_PATH")
        return {key: str(env.get(key, "")) for key in keys}

    def _build_gazebo_launch_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = os.path.abspath(self.ROS_LOG_PATH)

        resources_root = Path(self.REPO_ROOT) / "resources"
        models_root = resources_root / "models"
        sensors_root = resources_root / "sensors"
        camera_sensors_root = sensors_root / "camera"
        catkin_plugin_root = Path(self.REPO_ROOT) / "catkin_ws" / "devel" / "lib"
        ros_plugin_root = Path("/opt/ros/noetic/lib")
        ros_plugin_package_roots = [
            Path("/opt/ros/noetic/lib/gazebo_plugins"),
            Path("/opt/ros/noetic/lib/gazebo_ros"),
        ]
        system_model_roots = [
            Path("/usr/share/gazebo-11/models"),
            Path("/usr/share/gazebo/models"),
            Path.home() / ".gazebo" / "models",
        ]
        system_plugin_roots = [
            Path("/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
            Path("/usr/lib/x86_64-linux-gnu/gazebo/plugins"),
        ]
        system_resource_roots = [
            Path("/usr/share/gazebo-11"),
            Path("/usr/share/gazebo"),
            Path.home() / ".gazebo",
        ]

        model_extras = self._existing_dirs([resources_root, models_root, sensors_root, camera_sensors_root] + system_model_roots)
        plugin_extras = self._existing_dirs([catkin_plugin_root, ros_plugin_root] + ros_plugin_package_roots + system_plugin_roots)
        resource_extras = self._existing_dirs([resources_root] + system_resource_roots)

        merged_model_path = self._merge_env_paths(model_extras, env.get("GAZEBO_MODEL_PATH", ""))
        merged_plugin_path = self._merge_env_paths(plugin_extras, env.get("GAZEBO_PLUGIN_PATH", ""))
        merged_resource_path = self._merge_env_paths(resource_extras, env.get("GAZEBO_RESOURCE_PATH", ""))

        if merged_model_path:
            env["GAZEBO_MODEL_PATH"] = merged_model_path
        if merged_plugin_path:
            env["GAZEBO_PLUGIN_PATH"] = merged_plugin_path
        if merged_resource_path:
            env["GAZEBO_RESOURCE_PATH"] = merged_resource_path
        return env

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
        timeout_s: Optional[float] = None,
        msg_window_s: Optional[float] = None,
        min_messages: Optional[int] = None,
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
        last_topics_without_messages: List[str] = []

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
                    last_topics_without_messages = list(missing_messages)
                    diag["topics_without_messages"] = missing_messages
                    time.sleep(0.2)
                    continue

                diag["reason"] = "ok"
                self._last_topic_diag = dict(diag)
                return True, diag

            time.sleep(0.2)

        if not diag.get("reason"):
            if last_topics_without_messages and len(resolved) == len(expected):
                diag["reason"] = "topics_no_messages_timeout"
                diag["topics_without_messages"] = list(last_topics_without_messages)
            else:
                diag["reason"] = "topics_not_found"
        diag["resolved_topics"] = dict(resolved)
        diag["missing_expected_topics"] = [topic for topic in expected if topic not in resolved]
        self._last_topic_diag = dict(diag)
        return False, diag

    def _wait_ros_master_ready(self, timeout_s: Optional[float] = None) -> Tuple[bool, str]:
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

    def wait_for_master(self, timeout_s: Optional[float] = None) -> Tuple[bool, str]:
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

    def _wait_for_clock(self, timeout_s: Optional[float] = None) -> Tuple[bool, Dict[str, object]]:
        timeout_s = float(timeout_s or self.CLOCK_READY_TIMEOUT_S)
        diag: Dict[str, object] = {
            "timeout_s": float(timeout_s),
            "use_sim_time": True,
            "reason": "",
            "error": "",
        }
        try:
            use_sim_time = bool(rospy.get_param("/use_sim_time", True))
        except Exception:
            use_sim_time = True

        diag["use_sim_time"] = bool(use_sim_time)
        if not use_sim_time:
            diag["reason"] = "use_sim_time_disabled"
            return True, diag

        try:
            rospy.wait_for_message("/clock", Clock, timeout=float(timeout_s))
            diag["reason"] = "ok"
            return True, diag
        except Exception as exc:
            diag["reason"] = "clock_timeout"
            diag["error"] = str(exc)
            return False, diag

    @staticmethod
    def _read_file_tail(path: Path, max_lines: int = 200) -> List[str]:
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f_in:
                return f_in.read().splitlines()[-int(max_lines):]
        except Exception as exc:
            return [f"<failed_to_read_log> {exc}"]

    @staticmethod
    def _read_file_head(path: Path, max_lines: int = 120) -> List[str]:
        lines: List[str] = []
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f_in:
                for line in f_in:
                    lines.append(line.rstrip("\n"))
                    if len(lines) >= int(max_lines):
                        break
            return lines
        except Exception as exc:
            return [f"<failed_to_read_log_head> {exc}"]

    @staticmethod
    def _strip_ansi(text: str) -> str:
        return re.sub(r"\x1B\[[0-?]*[ -/]*[@-~]", "", str(text or ""))

    def _launch_output_lines_for_log_detection(self) -> List[str]:
        lines: List[str] = []
        snapshot = self._launch_output_snapshot(max_lines=300)
        lines.extend(snapshot["launch_stdout_tail"])
        lines.extend(snapshot["launch_stderr_tail"])

        for path_str in self._scene_log_files.values():
            path = Path(str(path_str or ""))
            if not path.exists() or not path.is_file():
                continue
            lines.extend(self._read_file_head(path, max_lines=120))
            lines.extend(self._read_file_tail(path, max_lines=40))

        deduped: List[str] = []
        for line in lines:
            clean = self._strip_ansi(str(line).strip())
            if clean and clean not in deduped:
                deduped.append(clean)
        return deduped

    def _detect_roslaunch_log_dir(self) -> Dict[str, str]:
        roslaunch_log_path = ""
        for line in self._launch_output_lines_for_log_detection():
            match = re.search(r"logging to\s+(.+?\.log)\s*$", line)
            if match:
                roslaunch_log_path = match.group(1).strip().strip("\"'")

        if not roslaunch_log_path:
            return {
                "roslaunch_log_path_detected": "",
                "gazebo_log_dir_detected": "",
                "path_to_gazebo1_log": "",
            }

        try:
            roslaunch_log = Path(roslaunch_log_path).expanduser().resolve()
        except Exception:
            roslaunch_log = Path(roslaunch_log_path).expanduser()
        log_dir = roslaunch_log.parent
        return {
            "roslaunch_log_path_detected": str(roslaunch_log),
            "gazebo_log_dir_detected": str(log_dir),
            "path_to_gazebo1_log": str(log_dir / "gazebo-1.log"),
        }

    def _collect_latest_gazebo_log_tail(self, max_lines: int = 200) -> Dict[str, object]:
        detected = self._detect_roslaunch_log_dir()
        detected_log_dir = Path(str(detected.get("gazebo_log_dir_detected", "") or "")).expanduser()
        search_roots: List[Path] = []
        if str(detected.get("gazebo_log_dir_detected", "")).strip():
            search_roots.append(detected_log_dir)
        search_roots.extend(
            [
                Path(self.ROS_LOG_PATH),
                Path.home() / ".ros" / "log" / "latest",
                Path.home() / ".ros" / "log",
            ]
        )
        patterns = (
            "gazebo-1.log",
            "gazebo.log",
            "gazebo-*.log",
            "gzserver-*.log",
            "server-*.log",
            "*.log",
        )
        searched_paths: List[str] = []
        candidates: List[Path] = []

        for root in search_roots:
            root_str = str(root)
            if root_str and root_str not in searched_paths:
                searched_paths.append(root_str)
            if not root.exists() or not root.is_dir():
                continue
            for pattern in patterns:
                try:
                    for match in root.rglob(pattern):
                        if match.is_file() and match not in candidates:
                            candidates.append(match)
                except Exception:
                    continue

        if not candidates:
            return {
                "path": "",
                "tail": ["<gazebo-log-not-found>"],
                "searched_paths": searched_paths,
                "gazebo_log_dir_detected": str(detected.get("gazebo_log_dir_detected", "")),
                "path_to_gazebo1_log": str(detected.get("path_to_gazebo1_log", "")),
                "roslaunch_log_path_detected": str(detected.get("roslaunch_log_path_detected", "")),
            }

        def _mtime(path: Path) -> float:
            try:
                return float(path.stat().st_mtime)
            except Exception:
                return 0.0

        def _basename_rank(path: Path) -> int:
            name = path.name.lower()
            if name == "gazebo-1.log":
                return 0
            if re.fullmatch(r"gazebo-\d+\.log", name):
                return 1
            if name == "gazebo.log":
                return 2
            if name.startswith("gzserver-"):
                return 3
            if name.startswith("server-"):
                return 4
            if "gazebo" in name:
                return 5
            if name.startswith("roslaunch-"):
                return 9
            return 6

        chosen = sorted(
            candidates,
            key=lambda path: (
                0 if str(detected.get("gazebo_log_dir_detected", "")) and path.parent == detected_log_dir else 1,
                _basename_rank(path),
                -_mtime(path),
                str(path),
            ),
        )[0]
        return {
            "path": str(chosen),
            "tail": self._read_file_tail(chosen, max_lines=max_lines),
            "searched_paths": searched_paths,
            "gazebo_log_dir_detected": str(detected.get("gazebo_log_dir_detected", "")),
            "path_to_gazebo1_log": str(detected.get("path_to_gazebo1_log", "")),
            "roslaunch_log_path_detected": str(detected.get("roslaunch_log_path_detected", "")),
        }

    def _collect_runtime_diag(self, expected_topics: Optional[List[str]] = None) -> Dict[str, object]:
        snapshot = self._launch_output_snapshot(max_lines=80)
        lines = list(snapshot["launch_stdout_tail"]) + list(snapshot["launch_stderr_tail"])
        scene = self.get_last_scene_diagnostics()
        rostopic_dump = self._run_shell_capture("rostopic list || true", timeout_s=6.0, max_lines=400)
        rosservice_dump = self._run_shell_capture("rosservice list || true", timeout_s=6.0, max_lines=400)
        rosnode_dump = self._run_shell_capture("rosnode list || true", timeout_s=6.0, max_lines=400)

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
            "rostopic_list": [str(line).strip() for line in rostopic_dump.get("stdout_tail", []) if str(line).strip()],
            "rosservice_list": [str(line).strip() for line in rosservice_dump.get("stdout_tail", []) if str(line).strip()],
            "rosnode_list": [str(line).strip() for line in rosnode_dump.get("stdout_tail", []) if str(line).strip()],
            "rostopic_list_cmd": rostopic_dump,
            "rosservice_list_cmd": rosservice_dump,
            "rosnode_list_cmd": rosnode_dump,
            "scene_log_files": dict(self._scene_log_files),
            "launch_env": scene.get("launch_env", {}),
        }
        gazebo_log_info = self._collect_latest_gazebo_log_tail(max_lines=200)
        diag["gazebo_log_tail"] = gazebo_log_info
        diag["gazebo_log_dir_detected"] = gazebo_log_info.get("gazebo_log_dir_detected", "")
        diag["path_to_gazebo1_log"] = gazebo_log_info.get("path_to_gazebo1_log", "")
        diag["roslaunch_log_path_detected"] = gazebo_log_info.get("roslaunch_log_path_detected", "")
        return diag

    def collect_failure_diagnostics(self, expected_topics: Optional[List[str]] = None) -> Dict[str, object]:
        return self._collect_runtime_diag(expected_topics=expected_topics)

    def launch_ros(self) -> None:
        self._kill_ros()
        self._prepare_ros_log_dir()
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = os.path.abspath(self.ROS_LOG_PATH)

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

    @staticmethod
    def _extract_model_name_from_sdf(camera_model_path: str) -> str:
        try:
            tree = ET.parse(camera_model_path)
            root = tree.getroot()
            model = root.find(".//model")
            if model is not None:
                model_name = str(model.get("name", "") or "").strip()
                if model_name:
                    return model_name
        except Exception as exc:
            logger.warning(f"Failed to parse model name from SDF {camera_model_path}: {exc}")
        stem = Path(camera_model_path).stem
        return f"{stem}_model"

    @staticmethod
    def _sensor_name_from_topic(topic_name: str) -> str:
        parts = [part for part in str(topic_name or "").strip("/").split("/") if part]
        if not parts:
            return ""
        return str(parts[0]).strip()

    def _resolve_sensor_name(
        self,
        sensor_name: Optional[str],
        expected_topics: List[str],
        camera_model_path: str,
    ) -> Tuple[str, str]:
        preferred = str(sensor_name or "").strip()
        if preferred:
            return preferred, "open_scene.sensor_name"

        for topic in expected_topics:
            candidate = self._sensor_name_from_topic(topic)
            if candidate:
                return candidate, "expected_topics"

        fallback = self._extract_model_name_from_sdf(camera_model_path)
        if fallback:
            return fallback, "camera_sdf_fallback"
        return "sensor_model", "hardcoded_fallback"

    def _delete_model_best_effort(self, model_name: str) -> Dict[str, object]:
        diag: Dict[str, object] = {
            "model_name": str(model_name),
            "attempted": False,
            "success": False,
            "status_message": "",
            "exception": "",
        }
        if not str(model_name or "").strip():
            diag["status_message"] = "model_name is empty"
            return diag

        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
            delete_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            diag["attempted"] = True
            response = delete_proxy(str(model_name))
            diag["success"] = bool(response.success)
            diag["status_message"] = str(response.status_message or "")
        except Exception as exc:
            diag["exception"] = str(exc)
        return diag

    @staticmethod
    def _spawn_status_contains_exists(status_message: str) -> bool:
        msg = str(status_message or "").strip().lower()
        if not msg:
            return False
        return ("already exists" in msg) or (" exists" in msg) or msg.endswith("exists")

    def _spawn_sensor_model(self, camera_model_path: str, model_name: str) -> Tuple[bool, Dict[str, object]]:
        robot_namespace = ""
        reference_frame = "world"
        diag: Dict[str, object] = {
            "model_name": str(model_name),
            "camera_model_path_abs": str(camera_model_path),
            "robot_namespace": str(robot_namespace),
            "reference_frame": str(reference_frame),
            "delete_before_spawn": {},
            "delete_before_spawn_retry": {},
            "spawn_attempts": [],
            "retry_on_exists_used": False,
            "success": False,
            "status_message": "",
            "exception": "",
        }
        try:
            with open(camera_model_path, "r", encoding="utf-8") as f_in:
                model_xml = f_in.read()
            if not model_xml.strip():
                diag["status_message"] = "camera model SDF is empty"
                return False, diag
        except Exception as exc:
            diag["exception"] = str(exc)
            return False, diag

        delete_diag = self._delete_model_best_effort(model_name)
        diag["delete_before_spawn"] = delete_diag

        def _do_spawn_once() -> Tuple[bool, Dict[str, object]]:
            payload: Dict[str, object] = {
                "success": False,
                "status_message": "",
                "exception": "",
            }
            try:
                rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=8.0)
                spawn_proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
                initial_pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
                response = spawn_proxy(
                    str(model_name),
                    model_xml,
                    robot_namespace,
                    initial_pose,
                    reference_frame,
                )
                payload["success"] = bool(response.success)
                payload["status_message"] = str(response.status_message or "")
                return bool(response.success), payload
            except Exception as exc:
                payload["exception"] = str(exc)
                return False, payload

        try:
            first_ok, first_payload = _do_spawn_once()
            diag["spawn_attempts"].append({"attempt": 1, **first_payload})
            if first_ok:
                diag["success"] = True
                diag["status_message"] = str(first_payload.get("status_message", ""))
                return True, diag

            first_status = str(first_payload.get("status_message", ""))
            if self._spawn_status_contains_exists(first_status):
                diag["retry_on_exists_used"] = True
                retry_delete_diag = self._delete_model_best_effort(model_name)
                diag["delete_before_spawn_retry"] = retry_delete_diag
                time.sleep(0.3)

                second_ok, second_payload = _do_spawn_once()
                diag["spawn_attempts"].append({"attempt": 2, **second_payload})
                diag["success"] = bool(second_ok)
                diag["status_message"] = str(second_payload.get("status_message", ""))
                if second_ok:
                    return True, diag
                diag["exception"] = str(second_payload.get("exception", "") or "")
                return False, diag

            diag["success"] = False
            diag["status_message"] = first_status
            diag["exception"] = str(first_payload.get("exception", "") or "")
            return False, diag
        except Exception as exc:
            diag["exception"] = str(exc)
            return False, diag

    def _get_model_properties_diag(self, model_name: str) -> Dict[str, object]:
        diag: Dict[str, object] = {
            "model_name": str(model_name or ""),
            "success": False,
            "status_message": "",
            "parent_model_name": "",
            "canonical_body_name": "",
            "body_names": [],
            "geom_names": [],
            "joint_names": [],
            "child_model_names": [],
            "is_static": None,
            "exception": "",
        }
        if not str(model_name or "").strip():
            diag["status_message"] = "model_name is empty"
            return diag

        try:
            rospy.wait_for_service("/gazebo/get_model_properties", timeout=5.0)
            get_model_properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
            response = get_model_properties(str(model_name))
            diag["success"] = bool(response.success)
            diag["status_message"] = str(response.status_message or "")
            diag["parent_model_name"] = str(getattr(response, "parent_model_name", "") or "")
            diag["canonical_body_name"] = str(getattr(response, "canonical_body_name", "") or "")
            diag["body_names"] = [str(item) for item in getattr(response, "body_names", [])]
            diag["geom_names"] = [str(item) for item in getattr(response, "geom_names", [])]
            diag["joint_names"] = [str(item) for item in getattr(response, "joint_names", [])]
            diag["child_model_names"] = [str(item) for item in getattr(response, "child_model_names", [])]
            diag["is_static"] = bool(getattr(response, "is_static", False))
        except Exception as exc:
            diag["exception"] = str(exc)
        return diag

    def open_scene(
        self,
        world_path: str,
        camera_model_path: str,
        expected_topics: Optional[List[str]] = None,
        sensor_name: Optional[str] = None,
    ) -> bool:
        self._clear_launch_buffers()
        self._last_topic_diag = {}

        world_abs = self._resolve_path(world_path)
        sensor_abs = self._resolve_path(camera_model_path)
        expected_topics = [str(topic) for topic in (expected_topics or []) if str(topic).strip()]
        resolved_sensor_name, resolved_sensor_name_source = self._resolve_sensor_name(
            sensor_name=sensor_name,
            expected_topics=expected_topics,
            camera_model_path=sensor_abs,
        )

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
            "sensor_name_input": str(sensor_name or ""),
            "sensor_name": str(resolved_sensor_name),
            "sensor_name_source": str(resolved_sensor_name_source),
            "catkin_setup_abs": self.CATKIN_SETUP_DIR,
            "catkin_setup_exists": os.path.exists(self.CATKIN_SETUP_DIR),
            "expected_topics": list(expected_topics),
            "launch_ready_timeout_s": float(self.GAZEBO_SERVICES_TIMEOUT_S),
            "clock_ready_timeout_s": float(self.CLOCK_READY_TIMEOUT_S),
            "topic_ready_timeout_s": float(self.TOPIC_READY_TIMEOUT_S),
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
        launch_world_abs = world_abs
        log_files = self._scene_log_paths(attempt=1)
        self._scene_log_files = dict(log_files)

        roslaunch_cmd = (
            f"source {shlex.quote(self.CATKIN_SETUP_DIR)} && "
            f"roslaunch {shlex.quote(self.SENSOR_PKG)} {shlex.quote(self.LAUNCH_FILE)} "
            f"world_path:={shlex.quote(launch_world_abs)} "
            "paused:=false gui:=false headless:=true"
        )

        launch_env = self._build_gazebo_launch_env()
        launch_env_diag = self._env_diag_subset(launch_env)

        self._set_scene_diag(
            generated_world_path_abs="",
            launch_world_path_abs=launch_world_abs,
            roslaunch_cmd=roslaunch_cmd,
            launch_log_files=dict(log_files),
            launch_env=launch_env_diag,
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
                launch_ready_timeout_s=float(self.GAZEBO_SERVICES_TIMEOUT_S),
                roslaunch_returncode=self.gazebo_process.poll() if self.gazebo_process else None,
                **runtime,
            )
            self.kill_gazebo()
            return False

        clock_ok, clock_diag = self._wait_for_clock(timeout_s=self.CLOCK_READY_TIMEOUT_S)
        self._set_scene_diag(clock_wait=clock_diag)

        sensor_model_name = str(resolved_sensor_name)
        spawn_ok, spawn_diag = self._spawn_sensor_model(sensor_abs, sensor_model_name)
        self._set_scene_diag(
            sensor_model_name=sensor_model_name,
            model_name_used_for_spawn=sensor_model_name,
            spawn_model=spawn_diag,
            clock_wait=clock_diag,
            clock_ready=bool(clock_ok),
        )
        if not spawn_ok:
            runtime = self._collect_runtime_diag(expected_topics=expected_topics)
            self._set_scene_diag(
                reason="camera_model_spawn_failed",
                sensor_model_name=sensor_model_name,
                model_name_used_for_spawn=sensor_model_name,
                spawn_model=spawn_diag,
                clock_wait=clock_diag,
                clock_ready=bool(clock_ok),
                **runtime,
            )
            self.kill_gazebo()
            return False

        model_properties_after_spawn = self._get_model_properties_diag(sensor_model_name)
        self._set_scene_diag(model_properties_after_spawn=model_properties_after_spawn)

        topics_after_spawn_cmd = self._run_shell_capture("rostopic list || true", timeout_s=6.0, max_lines=500)
        topics_after_spawn = [
            str(line).strip()
            for line in topics_after_spawn_cmd.get("stdout_tail", [])
            if str(line).strip()
        ]
        image_topic_candidates = [
            topic
            for topic in topics_after_spawn
            if topic.endswith("/image_raw")
            or topic == "/image_raw"
            or topic.endswith("/image_raw/compressed")
            or topic == "/image_raw/compressed"
        ]
        self._set_scene_diag(
            topics_after_spawn=topics_after_spawn,
            image_topic_candidates=image_topic_candidates,
            topics_after_spawn_cmd=topics_after_spawn_cmd,
        )

        if expected_topics:
            topics_ok, topics_diag = self.wait_for_topics(
                expected_topics=expected_topics,
                timeout_s=self.TOPIC_READY_TIMEOUT_S,
            )
            self._set_scene_diag(
                expected_topics=list(expected_topics),
                resolved_topics=topics_diag.get("resolved_topics", {}),
                msg_counters=topics_diag.get("msg_counters", {}),
                msg_hz=topics_diag.get("msg_hz", {}),
                topic_ready_timeout_s=float(self.TOPIC_READY_TIMEOUT_S),
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
