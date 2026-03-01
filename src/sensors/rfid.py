import copy
import os
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple

import rospy
from gazebo_msgs.srv import DeleteModel, GetModelState, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from .sensor import Sensor, register_sensor


@register_sensor("rfid", "rfid_antenna")
class Rfid(Sensor):
    DETECTED_TOPIC = "/detected_tags"
    TAG_NAME = "rfid_tag1"
    SERVICE_TIMEOUT_S = 30.0
    TAG_SPAWN_CONFIRM_TIMEOUT_S = 12.0

    """"Rfid антенна"""
    def __init__(self, CONFIG):
        super().__init__()
        self.CONFIG = CONFIG

        root_path = Path(CONFIG.get("ROOT_PATH", "") or os.getcwd())
        worlds_root = Path(CONFIG["WORLDS_PATH"])
        if not worlds_root.is_absolute():
            worlds_root = root_path / worlds_root

        sensors_root = Path(CONFIG["SENSORS_PATH"])
        if not sensors_root.is_absolute():
            sensors_root = root_path / sensors_root

        self.sensor_sdf_path = str((sensors_root / self.sensor_type / f"{self.sensor_name}.sdf").resolve())

        self.test_to_world = {
            'max_stable_read_distance_test': str((worlds_root / "rfid" / "change_distance.world").resolve()),
            'min_stable_read_distance_test': str((worlds_root / "rfid" / "change_distance.world").resolve()),
        }

        rfid_map_path = Path(CONFIG['RFID_MAP_PATH'])
        if not rfid_map_path.is_absolute():
            rfid_map_path = root_path / rfid_map_path
        self.rfid_map_path = str(rfid_map_path.resolve())

        self._repo_root = str(root_path.resolve())
        self._rfid_models_dir = str((root_path / "catkin_ws" / "src" / "RFID_Sensor_Plugin_gazebo" / "Models").resolve())
        self._last_test_diagnostics: Dict[str, Any] = {}

        # значение по умолчанию для read_distance = 10
        self.set_read_distance(10)

    def _set_test_diagnostics(self, **kwargs) -> None:
        self._last_test_diagnostics.update(kwargs)

    def get_last_test_diagnostics(self) -> Dict[str, Any]:
        return copy.deepcopy(self._last_test_diagnostics)

    @staticmethod
    def _split_env_paths(value: str) -> List[str]:
        if not value:
            return []
        return [p for p in value.split(os.pathsep) if p]

    @staticmethod
    def _wait_service(name: str, timeout_s: float) -> Tuple[bool, str]:
        try:
            rospy.wait_for_service(name, timeout=timeout_s)
            return True, ""
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    @staticmethod
    def _resolve_package_uri(raw_path: str, repo_root: str) -> str:
        if not raw_path.startswith("package://"):
            return raw_path
        rest = raw_path[len("package://"):]
        pkg_name, _, rel = rest.partition("/")
        if not pkg_name:
            return raw_path
        pkg_root = os.path.join(repo_root, "catkin_ws", "src", pkg_name)
        if rel:
            return os.path.join(pkg_root, rel)
        return pkg_root

    @staticmethod
    def _resolve_with_bases(raw_path: str, bases: List[str], repo_root: str) -> Tuple[str, List[str]]:
        expanded = os.path.expanduser(str(raw_path).strip())
        expanded = Rfid._resolve_package_uri(expanded, repo_root=repo_root)
        candidates: List[str] = []

        if os.path.isabs(expanded):
            candidates.append(os.path.abspath(expanded))
        else:
            for base in bases:
                candidates.append(os.path.abspath(os.path.join(base, expanded)))
            candidates.append(os.path.abspath(expanded))

        unique: List[str] = []
        for path in candidates:
            if path not in unique:
                unique.append(path)

        for path in unique:
            if os.path.exists(path):
                return path, unique
        return "", unique

    def _extract_rfid_paths_from_sensor_sdf(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        try:
            tree = ET.parse(self.sensor_sdf_path)
            root = tree.getroot()
            plugin = root.find(".//plugin[@name='rfid_tags']")
            if plugin is None:
                return {"error": "rfid_tags plugin node not found in sensor sdf"}

            out["tag_sdf_path_raw"] = (plugin.findtext("tag_sdf_path") or "").strip()
            out["map_path_raw"] = (plugin.findtext("map_path") or "").strip()
            out["fix_sdf_path_raw"] = (plugin.findtext("fix_sdf_path") or "").strip()
            return out
        except Exception as exc:  # noqa: BLE001
            return {"error": f"failed to parse sensor sdf: {exc}"}

    def _load_tag_sdf_for_spawn(self, simulator) -> Tuple[str, str, Dict[str, Any]]:
        base_candidates = [
            os.path.dirname(self.sensor_sdf_path),
            self._repo_root,
            os.getcwd(),
        ]
        if hasattr(simulator, "BASE_WORLD_PATH"):
            base_candidates.append(os.path.dirname(str(simulator.BASE_WORLD_PATH)))

        path_info = self._extract_rfid_paths_from_sensor_sdf()
        raw_tag_sdf_path = str(path_info.get("tag_sdf_path_raw", "") or "")

        resolved_path = ""
        attempted_paths: List[str] = []
        if raw_tag_sdf_path:
            resolved_path, attempted_paths = self._resolve_with_bases(
                raw_tag_sdf_path,
                bases=base_candidates,
                repo_root=self._repo_root,
            )

        fallback = os.path.join(self._rfid_models_dir, "RFID_1", "model.sdf")
        if fallback not in attempted_paths:
            attempted_paths.append(fallback)
        if not resolved_path and os.path.exists(fallback):
            resolved_path = fallback

        diag = {
            "sensor_sdf_path": self.sensor_sdf_path,
            "rfid_paths_from_sensor_sdf": path_info,
            "tag_sdf_path_raw": raw_tag_sdf_path,
            "tag_sdf_resolved_path": resolved_path,
            "tag_sdf_attempted_paths": attempted_paths,
        }

        if not resolved_path:
            raise RuntimeError(f"tag_sdf_path not resolved; attempted={attempted_paths}")

        try:
            with open(resolved_path, "r", encoding="utf-8") as f_in:
                sdf_text = f_in.read()
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"failed to read tag sdf at {resolved_path}: {exc}") from exc

        diag["tag_sdf_size_bytes"] = len(sdf_text.encode("utf-8"))
        diag["tag_sdf_preview"] = sdf_text[:500]
        return sdf_text, resolved_path, diag

    def _model_exists_via_services(self, model_name: str, timeout_s: float = 4.0) -> Tuple[bool, Dict[str, Any]]:
        diag: Dict[str, Any] = {}
        deadline = time.time() + float(timeout_s)

        get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        while time.time() < deadline:
            try:
                state_resp = get_state(model_name, "world")
                diag["last_get_model_state_status"] = str(state_resp.status_message)
                if bool(state_resp.success):
                    diag["confirmed_via"] = "get_model_state"
                    return True, diag
            except Exception as exc:  # noqa: BLE001
                diag["get_model_state_error"] = str(exc)

            try:
                world_resp = get_world()
                names = list(world_resp.model_names)
                diag["world_models_count"] = int(len(names))
                if model_name in names:
                    diag["confirmed_via"] = "get_world_properties"
                    return True, diag
            except Exception as exc:  # noqa: BLE001
                diag["get_world_properties_error"] = str(exc)

            time.sleep(0.2)

        return False, diag

    def _ensure_tag_spawned(self, simulator, tag_name: str, x: float, y: float, z: float) -> None:
        model_path_entries = self._split_env_paths(os.environ.get("GAZEBO_MODEL_PATH", ""))
        model_dir_abs = os.path.abspath(self._rfid_models_dir)
        model_path_has_rfid_models = any(os.path.abspath(p) == model_dir_abs for p in model_path_entries)

        spawn_diag: Dict[str, Any] = {
            "model_name": str(tag_name),
            "target_pose": {"x": float(x), "y": float(y), "z": float(z)},
            "rfid_map_path": self.rfid_map_path,
            "rfid_models_dir": model_dir_abs,
            "gazebo_model_path": os.environ.get("GAZEBO_MODEL_PATH", ""),
            "gazebo_model_path_has_rfid_models": bool(model_path_has_rfid_models),
            "service_wait": {},
        }

        required_services = (
            "/gazebo/get_world_properties",
            "/gazebo/get_model_state",
            "/gazebo/spawn_sdf_model",
        )
        for srv in required_services:
            ok, err = self._wait_service(srv, timeout_s=float(self.SERVICE_TIMEOUT_S))
            spawn_diag["service_wait"][srv] = {"ok": bool(ok), "error": str(err)}
            if not ok:
                self._set_test_diagnostics(rfid_spawn=spawn_diag)
                raise RuntimeError(f"tag not spawned: required service unavailable: {srv} ({err})")

        exists, exists_diag = self._model_exists_via_services(tag_name, timeout_s=2.0)
        spawn_diag["preexisting_model"] = bool(exists)
        spawn_diag["preexisting_model_diagnostics"] = exists_diag
        if exists:
            self._set_test_diagnostics(rfid_spawn=spawn_diag)
            return

        # Оставляем время на spawn из плагина rfid_tag_plugin.
        plugin_spawned = simulator.wait_for_model_spawn(tag_name, timeout=10)
        spawn_diag["plugin_spawn_wait_ok"] = bool(plugin_spawned)
        if plugin_spawned:
            exists, exists_diag = self._model_exists_via_services(tag_name, timeout_s=2.0)
            spawn_diag["post_plugin_spawn_diagnostics"] = exists_diag
            if exists:
                self._set_test_diagnostics(rfid_spawn=spawn_diag)
                return

        # Fallback: вручную spwan через gazebo service с полной диагностикой.
        ok_delete, err_delete = self._wait_service("/gazebo/delete_model", timeout_s=2.0)
        spawn_diag["service_wait"]["/gazebo/delete_model"] = {"ok": bool(ok_delete), "error": str(err_delete)}
        if ok_delete:
            try:
                delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
                delete_resp = delete_srv(tag_name)
                spawn_diag["delete_existing_response"] = {
                    "success": bool(delete_resp.success),
                    "status_message": str(delete_resp.status_message),
                }
            except Exception as exc:  # noqa: BLE001
                spawn_diag["delete_existing_exception"] = str(exc)

        try:
            tag_sdf_text, tag_sdf_path, sdf_diag = self._load_tag_sdf_for_spawn(simulator=simulator)
            spawn_diag.update(sdf_diag)
            spawn_diag["spawn_request"] = {
                "model_name": str(tag_name),
                "sdf_path": str(tag_sdf_path),
                "sdf_size_bytes": int(len(tag_sdf_text.encode("utf-8"))),
            }

            pose = Pose(Point(float(x), float(y), float(z)), Quaternion(0, 0, 0, 1))
            spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_resp = spawn_srv(
                model_name=str(tag_name),
                model_xml=tag_sdf_text,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world",
            )
            spawn_diag["spawn_response"] = {
                "success": bool(spawn_resp.success),
                "status_message": str(spawn_resp.status_message),
            }
            if not bool(spawn_resp.success):
                self._set_test_diagnostics(rfid_spawn=spawn_diag)
                raise RuntimeError(f"spawn_sdf_model returned success=False: {spawn_resp.status_message}")

            confirmed, confirmed_diag = self._model_exists_via_services(
                tag_name,
                timeout_s=float(self.TAG_SPAWN_CONFIRM_TIMEOUT_S),
            )
            spawn_diag["post_spawn_confirmed"] = bool(confirmed)
            spawn_diag["post_spawn_diagnostics"] = confirmed_diag
            self._set_test_diagnostics(rfid_spawn=spawn_diag)
            if not confirmed:
                raise RuntimeError(f"spawn_sdf_model success but model not visible in world: {tag_name}")

        except Exception as exc:  # noqa: BLE001
            spawn_diag["spawn_exception"] = str(exc)
            self._set_test_diagnostics(rfid_spawn=spawn_diag)
            raise RuntimeError("tag not spawned") from exc

    def _write_test_map(self) -> None:
        map_dir = os.path.dirname(self.rfid_map_path)
        if map_dir:
            os.makedirs(map_dir, exist_ok=True)
        with open(self.rfid_map_path, "w", encoding="utf-8") as f:
            f.write("fix1 1 0.5 0 0\n")
            f.flush()


    def set_read_distance(self, read_distance: int) -> None:
        """Меняет в sdf дальность считывания антенны"""
        self.read_distance = read_distance

        with open(self.sensor_sdf_path, "r", encoding="utf-8") as f:
            sdf_before_replace = f.read()

        sdf_after_replace, n = re.subn(r"<rzero>.*?</rzero>", f"<rzero>{self.read_distance}</rzero>", sdf_before_replace, count=1, flags=re.S)
        if n == 0:
            raise RuntimeError("rzero not found")

        with open(self.sensor_sdf_path, "w", encoding="utf-8") as f:
            f.write(sdf_after_replace)


    def get_params(self) -> Dict[str, Any]:
        return {"read_distance": self.read_distance}


    def set_params(self, **params) -> None:
        if "read_distance" in params:
            rd = int(params["read_distance"])
            if rd <= 0:
                raise ValueError("read_distance must be > 0")
            self.set_read_distance(rd)
    
    
    def capture_data(
        self,
        simulator,
        world_path: Optional[str] = None,
        window: float = 0.5,
        timeout_per_msg: float = 0.25
    ) -> Dict[str, Any]:
        """Считать все метки"""
        if world_path:
            if not simulator.open_scene(world_path, self.sensor_sdf_path):
                return None
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        tags: Dict[str, Any] = {}
        deadline = time.time() + max(0.0, float(window))

        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(self.DETECTED_TOPIC, PoseStamped, timeout=timeout_per_msg)
            except rospy.ROSException:
                continue
            
            tags[msg.header.frame_id] = msg.pose

        return tags


    def max_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения дальности считывания"""
        self._last_test_diagnostics = {}
        # плагин читает файл map.txt и по нему создает метки в gazebo
        self._write_test_map()

        if not simulator.open_scene(self.test_to_world['max_stable_read_distance_test'], self.sensor_sdf_path):
            scene_diag = simulator.get_last_scene_diagnostics() if hasattr(simulator, "get_last_scene_diagnostics") else {}
            reason = scene_diag.get("reason", "unknown") if isinstance(scene_diag, dict) else "unknown"
            raise RuntimeError(f"failed to open RFID scene (reason={reason})")
    
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        tag_name = self.TAG_NAME
        self._ensure_tag_spawned(simulator, tag_name=tag_name, x=0.5, y=0.0, z=0.0)

        current_dist = 0.5
        reset_distance = 25
        max_dist = 0

        while current_dist <= self.read_distance + 1e-9:

            detected_count = 0

            # делаем 10 попыток считывания метки
            for _ in range(10):
                try:
                    # перемещаем метку далеко, чтобы сбросить попытку
                    simulator.set_pose(tag_name, reset_distance, 0, 0)
                    time.sleep(0.005)

                    simulator.set_pose(tag_name, current_dist, 0, 0)

                    data = self.capture_data(simulator=simulator, world_path=None, window=1)

                    if data is not None and tag_name in data:
                        detected_count += 1

                except:
                    continue

            is_tag_detected = bool(detected_count >= 7)

            if is_tag_detected:
                max_dist = current_dist

            current_dist = round(current_dist + 0.5, 1)

        return {'max_read_distance': max_dist}


    def min_stable_read_distance_test(self, simulator) -> Optional[Dict[str, Any]]:
        """Тест для определения минимальной дальности считывания"""
        self._last_test_diagnostics = {}

        self._write_test_map()

        if not simulator.open_scene(self.test_to_world['min_stable_read_distance_test'], self.sensor_sdf_path):
            scene_diag = simulator.get_last_scene_diagnostics() if hasattr(simulator, "get_last_scene_diagnostics") else {}
            reason = scene_diag.get("reason", "unknown") if isinstance(scene_diag, dict) else "unknown"
            raise RuntimeError(f"failed to open RFID scene (reason={reason})")
        
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        tag_name = self.TAG_NAME
        self._ensure_tag_spawned(simulator, tag_name=tag_name, x=0.5, y=0.0, z=0.0)

        min_dist = current_dist = 0.25
        reset_distance = 25

        while current_dist >= 0:

            detected_count = 0

            for _ in range(4):
                try:
                    simulator.set_pose(tag_name, reset_distance, 0, 0)
                    time.sleep(0.005)

                    simulator.set_pose(tag_name, current_dist, 0, 0)
                    data = self.capture_data(simulator=simulator, world_path=None, window=1)

                    if tag_name in data:
                        detected_count += 1
                except:
                    continue

            is_tag_detected = bool(detected_count >= 3)

            if is_tag_detected:
                min_dist = current_dist

            current_dist = round(current_dist - 0.01, 5)

        return {'min_read_distance': min_dist}
