from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, BodyRequest, GetModelProperties
from geometry_msgs.msg import Point, Wrench

from config import CONFIG
from src.gazebo_simulator import Simulator
from src.sensors.tactile_specs import get_tactile_spec, tactile_world_root, validate_tactile_sensor_name


@dataclass
class WrenchSample:
    timestamp: float
    force: Dict[str, float]
    torque: Dict[str, float]

    @property
    def normal_force(self) -> float:
        return abs(float(self.force.get("z", 0.0)))


class GazeboWrenchStream:
    def __init__(self, topic_name: str) -> None:
        self.topic_name = str(topic_name)
        self.proc: Optional[subprocess.Popen] = None
        self._buffer = ""

    def start(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            return
        self.proc = subprocess.Popen(
            ["gz", "topic", "-e", self.topic_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
        )

    def stop(self) -> None:
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.proc = None
        self._buffer = ""

    def _read_chunk(self, timeout_s: float) -> str:
        if self.proc is None or self.proc.stdout is None:
            return ""
        deadline = time.time() + float(timeout_s)
        chunks: List[str] = []
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if line:
                chunks.append(line)
                if "torque" in "".join(chunks):
                    break
            else:
                time.sleep(0.01)
        return "".join(chunks)

    @staticmethod
    def _extract_xyz(block: str, section: str) -> Dict[str, float]:
        match = re.search(rf"{section}\s*\{{(.*?)\}}", block, flags=re.S)
        if not match:
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        payload = match.group(1)
        values: Dict[str, float] = {}
        for axis in ("x", "y", "z"):
            axis_match = re.search(rf"{axis}\s*:\s*([-+eE0-9.]+)", payload)
            values[axis] = float(axis_match.group(1)) if axis_match else 0.0
        return values

    def read_sample(self, timeout_s: float = 1.0) -> Optional[WrenchSample]:
        chunk = self._read_chunk(timeout_s=timeout_s)
        if not chunk:
            return None
        force = self._extract_xyz(chunk, "force")
        torque = self._extract_xyz(chunk, "torque")
        return WrenchSample(timestamp=time.time(), force=force, torque=torque)


class TactileHarness:
    # TODO: replace this with a verified ROS/Gazebo topic contract after Ubuntu validation.
    SENSOR_PLACEMENT_WARNING = (
        "Assumes Gazebo publishes wrench data for the tactile force_torque sensor. "
        "Current tactile SDF files define the sensor on a link, while Gazebo Classic "
        "documentation expects force_torque sensors on joints."
    )

    def __init__(
        self,
        sensor_name: str,
        simulator: Optional[Simulator] = None,
        result_root: Optional[Path] = None,
    ) -> None:
        self.sensor_name = str(sensor_name)
        self._owns_simulator = simulator is None
        self.simulator = simulator or Simulator(CONFIG)
        self.root_path = Path(CONFIG["ROOT_PATH"])
        self.sensor_model_path = validate_tactile_sensor_name(CONFIG["ROOT_PATH"], self.sensor_name)
        self.world_root = tactile_world_root(CONFIG["ROOT_PATH"])
        self.spec = get_tactile_spec(self.sensor_name)
        self.result_root = result_root or (self.root_path / "results" / self.sensor_name)
        self.metrics_dir = self.result_root / "metrics"
        self.metrics_dir.mkdir(parents=True, exist_ok=True)

    def prepare(self) -> None:
        if self._owns_simulator:
            self.simulator.launch()

    def shutdown(self) -> None:
        if not self._owns_simulator:
            return
        try:
            self.simulator.kill_gazebo()
        finally:
            for method_name in ("_kill_node", "_kill_ros"):
                method = getattr(self.simulator, method_name, None)
                if callable(method):
                    try:
                        method()
                    except Exception:
                        pass

    def open_world(self, world_filename: str) -> None:
        world_path = self.world_root / world_filename
        ok = self.simulator.open_scene(
            str(world_path),
            str(self.sensor_model_path),
            expected_topics=[],
            sensor_name=self.sensor_name,
        )
        if not ok:
            diag = self.simulator.get_last_scene_diagnostics()
            raise RuntimeError(f"Failed to open tactile scene {world_path}: {json.dumps(diag, ensure_ascii=False)}")
        if not self.simulator.wait_for_model_spawn(self.sensor_name, timeout=10.0):
            raise RuntimeError(f"Tactile sensor model did not appear in Gazebo: {self.sensor_name}")

    def sensor_body_name(self) -> str:
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=10.0)
        get_model_properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        response = get_model_properties(self.sensor_name)
        if not bool(response.success):
            raise RuntimeError(f"get_model_properties failed for {self.sensor_name}: {response.status_message}")
        body_names = [str(item) for item in getattr(response, "body_names", [])]
        for preferred in ("body", "sensor_body", "top", "plate"):
            if preferred in body_names:
                return preferred
        if not body_names:
            return f"{self.sensor_name}::body"
        return body_names[0]

    def discover_wrench_topic(self, timeout_s: float = 10.0) -> str:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                check=False,
            )
            candidates = [line.strip() for line in (result.stdout or "").splitlines() if line.strip()]
            matching = [
                topic for topic in candidates
                if self.sensor_name in topic and topic.endswith("/wrench")
            ]
            if matching:
                # Prefer the explicit joint-mounted force_torque sensor topic over
                # generic body wrench topics, which may exist but stay silent.
                def _rank(topic: str) -> tuple[int, int, str]:
                    topic_lower = topic.lower()
                    score = 0
                    if "/ft_sensor/" in topic_lower:
                        score += 4
                    if "/ft_joint/" in topic_lower:
                        score += 2
                    if "/body/" in topic_lower:
                        score -= 3
                    return (-score, len(topic), topic)

                return sorted(matching, key=_rank)[0]
            time.sleep(0.3)
        raise RuntimeError(
            "Failed to discover tactile wrench topic. "
            f"Sensor={self.sensor_name}. {self.SENSOR_PLACEMENT_WARNING}"
        )

    def make_stream(self, timeout_s: float = 10.0) -> GazeboWrenchStream:
        topic = self.discover_wrench_topic(timeout_s=timeout_s)
        stream = GazeboWrenchStream(topic)
        stream.start()
        return stream

    def apply_body_wrench(
        self,
        force_xyz: Iterable[float],
        duration_s: float,
        reference_point_xyz: Iterable[float] = (0.0, 0.0, 0.0),
    ) -> Dict[str, object]:
        body_name = self.sensor_body_name()
        rospy.wait_for_service("/gazebo/apply_body_wrench", timeout=10.0)
        proxy = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        wrench = Wrench()
        force_xyz = list(force_xyz)
        wrench.force.x = float(force_xyz[0])
        wrench.force.y = float(force_xyz[1])
        wrench.force.z = float(force_xyz[2])
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = 0.0
        reference_point_xyz = list(reference_point_xyz)
        proxy(
            body_name,
            "world",
            Point(
                float(reference_point_xyz[0]),
                float(reference_point_xyz[1]),
                float(reference_point_xyz[2]),
            ),
            wrench,
            rospy.Time(0),
            rospy.Duration.from_sec(float(duration_s)),
        )
        return {
            "body_name": body_name,
            "force_xyz": [float(v) for v in force_xyz],
            "reference_point_xyz": [float(v) for v in reference_point_xyz],
            "duration_s": float(duration_s),
        }

    def clear_body_wrenches(self) -> None:
        try:
            rospy.wait_for_service("/gazebo/clear_body_wrenches", timeout=2.0)
            proxy = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
            proxy(self.sensor_body_name())
        except Exception:
            pass

    def collect_window(
        self,
        stream: GazeboWrenchStream,
        duration_s: float,
        poll_timeout_s: float = 0.25,
    ) -> List[WrenchSample]:
        deadline = time.time() + float(duration_s)
        samples: List[WrenchSample] = []
        while time.time() < deadline:
            sample = stream.read_sample(timeout_s=poll_timeout_s)
            if sample is not None:
                samples.append(sample)
        return samples

    @staticmethod
    def sample_summary(samples: List[WrenchSample]) -> Dict[str, float]:
        if not samples:
            return {"mean_normal_force": 0.0, "std_normal_force": 0.0, "peak_normal_force": 0.0}
        values = [sample.normal_force for sample in samples]
        mean_value = sum(values) / len(values)
        variance = sum((value - mean_value) ** 2 for value in values) / len(values)
        return {
            "mean_normal_force": float(mean_value),
            "std_normal_force": float(math.sqrt(variance)),
            "peak_normal_force": float(max(values)),
        }

    def write_metric(self, test_name: str, payload: Dict[str, object]) -> str:
        path = self.metrics_dir / f"{test_name}.json"
        with path.open("w", encoding="utf-8") as f_out:
            json.dump(payload, f_out, ensure_ascii=False, indent=2)
        return str(path)

    @staticmethod
    def serialize_samples(samples: List[WrenchSample]) -> List[Dict[str, object]]:
        return [
            {
                "timestamp": float(sample.timestamp),
                "force": dict(sample.force),
                "torque": dict(sample.torque),
                "normal_force": float(sample.normal_force),
            }
            for sample in samples
        ]
