"""Shared utilities for all sensor test modules."""

import copy
import json
import logging
import math
import os
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from enum import Enum
from math import atan, atan2, cos, degrees, pi, sin, tan
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from types import SimpleNamespace

import cv2
import numpy as np
import rospy
import rostopic
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image

from config import CONFIG

logger = logging.getLogger(__name__)


class Worlds(str, Enum):
    path = f"{CONFIG['ROOT_PATH']}/assets/worlds"

    # BASIC
    BASIC = f"{path}/_basic_.world"

    # RFID
    RFID_CHANGE_DISTANCE = f"{path}/rfid_change_distance.world"
    RFID_MASS_READ = f"{path}/rfid_mass_read.world"
    RFID_OVERLAP_TAGS = f"{path}/rfid_overlap_tags.world"
    RFID_ANGLE_DEPENDENCE = f"{path}/rfid_angle_dependence.world"
    RFID_MOVE_TAGS = f"{path}/rfid_move_tags.world"
    RFID_ANTENNA_ROTATION = f"{path}/rfid_antenna_rotation.world"

    # Camera
    CAMERA_SMOKE = f"{path}/camera_c1_single_cube.world"
    CAMERA_DEPTH_ACCURACY = f"{path}/camera_depth_perception.world"
    CAMERA_RESOLUTION = f"{path}/camera_c2_resolution.world"
    c1 = f"{path}/camera_c1_single_cube.world"

    # TACTILE
    TACTILE_FORCE = f"{path}/tactile_force.world"
    TACTILE_UNIFORMITY = f"{path}/tactile_uniformity.world"
    TACTILE_STABILITY = f"{path}/tactile_force.world"
    TACTILE_PEAK = f"{path}/tactile_force.world"


def _PoseStamped():
    from geometry_msgs.msg import PoseStamped

    return PoseStamped


def _world_from_db(sensor, func_name: str) -> str:
    """
    Primary world lookup: reads world_path from the DB entry for this test.
    Falls back to the Worlds enum value whose name matches func_name (upper-cased).
    """
    import src.sensor_storage as db

    tests = {t["func_name"]: t for t in db.get_type_tests(sensor.sensor_type)}
    path = tests.get(func_name, {}).get("world_path", "")
    if path:
        return path
    # Fallback: try to match by name convention
    key = func_name.upper()
    if key in Worlds.__members__:
        return Worlds[key].value
    return ""


def get_test(func_name: str):
    fn = TESTS.get(func_name)
    if fn is None:
        raise KeyError(f"No test registered under name {func_name!r}")
    return fn


def get_tests_for_type(sensor_type: str) -> dict[str, callable]:
    try:
        import src.sensor_storage as db

        result = {}
        for test in db.get_type_tests(sensor_type):
            fn = TESTS.get(test["func_name"])
            if fn is not None:
                result[test["func_name"]] = fn
            else:
                logger.warning(
                    "get_tests_for_type: %r assigned to type %r but not in TESTS",
                    test["func_name"],
                    sensor_type,
                )
        return result
    except Exception as exc:
        logger.error("get_tests_for_type failed: %s", exc)
        return {}


def _img_to_numpy(msg):
    """Convert a sensor_msgs/Image to a numpy array."""
    import numpy as np

    dtype = np.float32 if "32FC" in msg.encoding else np.uint8
    ch = 1 if msg.encoding in ("32FC1", "mono8", "8UC1") else 3
    return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, ch)


def _camera_worlds_root() -> Path:
    root = Path(str(CONFIG["ROOT_PATH"]))
    assets_worlds = root / "assets" / "worlds"
    print(
        f"[DEBUG _camera_worlds_root] ROOT_PATH={root}, assets_worlds={assets_worlds}, exists={assets_worlds.exists()}"
    )
    if assets_worlds.exists():
        return assets_worlds

    configured = Path(str(CONFIG.get("WORLDS_PATH", "") or ""))
    if configured.is_absolute():
        print(f"[DEBUG _camera_worlds_root] using configured absolute: {configured}")
        return configured
    result = root / configured
    print(f"[DEBUG _camera_worlds_root] using relative: {result}")
    return result


def _text(node: Optional[ET.Element], path: str, default: str = "") -> str:
    if node is None:
        return default
    child = node.find(path)
    if child is None or child.text is None:
        return default
    return str(child.text).strip()


def _to_float(value: str, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def _to_int(value: str, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(round(float(str(value).strip())))
    except Exception:
        return default


def _normalize_topic(value: str) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    parts = [part for part in token.split("/") if part]
    return "/" + "/".join(parts)


def _join_topic(namespace: str, topic: str) -> str:
    topic_name = str(topic or "").strip()
    if not topic_name:
        return ""
    if topic_name.startswith("/"):
        return _normalize_topic(topic_name)
    ns = _normalize_topic(namespace)
    if not ns:
        return _normalize_topic(topic_name)
    return _normalize_topic(f"{ns}/{topic_name}")


def _parse_pose(value: str) -> Tuple[float, float, float, float, float, float]:
    items = [item for item in str(value or "").split() if item]
    if len(items) != 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    floats = [_to_float(item, 0.0) for item in items]
    return tuple(float(item or 0.0) for item in floats)  # type: ignore[return-value]


def _plugin_namespace(plugin: ET.Element) -> str:
    return (
        _text(plugin, "ros/namespace")
        or _text(plugin, "robotNamespace")
        or _text(plugin, "robot_namespace")
    )


def _plugin_remaps(plugin: ET.Element) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for arg in plugin.findall("ros/argument"):
        text = str(arg.text or "").strip()
        if ":=" not in text:
            continue
        lhs, rhs = text.split(":=", 1)
        lhs = lhs.strip()
        rhs = rhs.strip()
        if lhs:
            mapping[lhs] = rhs or lhs
    return mapping


def _plugin_update_rate(plugin: ET.Element, sensor_node: ET.Element) -> Optional[int]:
    return (
        _to_int(_text(plugin, "update_rate"), None)
        or _to_int(_text(plugin, "updateRate"), None)
        or _to_int(_text(sensor_node, "update_rate"), None)
    )


def _plugin_topic(
    plugin: Optional[ET.Element],
    sensor_node: ET.Element,
    remap_keys: List[str],
    plugin_tags: List[str],
    default_topic: str = "",
) -> str:
    if plugin is None:
        return ""

    namespace = _plugin_namespace(plugin)
    remaps = _plugin_remaps(plugin)
    for key in remap_keys:
        value = str(remaps.get(key, "")).strip()
        if value:
            return _join_topic(namespace, value)

    for tag in plugin_tags:
        value = _text(plugin, tag)
        if value:
            return _join_topic(namespace, value)

    default_value = str(default_topic or "").strip()
    if default_value:
        return _join_topic(namespace, default_value)

    sensor_name = _text(sensor_node, "camera/name") or sensor_node.get("name", "")
    if sensor_name:
        return _join_topic(namespace, f"{sensor_name}/image_raw")
    return ""


def _sensor_payload(sensor_node: ET.Element) -> Dict[str, Any]:
    plugin = sensor_node.find("plugin")
    pose = _parse_pose(_text(sensor_node, "pose"))
    return {
        "sensor_type": str(sensor_node.get("type", "")).strip(),
        "name": str(sensor_node.get("name", "")).strip(),
        "pose": pose,
        "image_width": _to_int(_text(sensor_node, "camera/image/width"), None),
        "image_height": _to_int(_text(sensor_node, "camera/image/height"), None),
        "image_format": _text(sensor_node, "camera/image/format"),
        "horizontal_fov": _to_float(_text(sensor_node, "camera/horizontal_fov"), None),
        "clip_near": _to_float(_text(sensor_node, "camera/clip/near"), None),
        "clip_far": _to_float(_text(sensor_node, "camera/clip/far"), None),
        "noise_type": _text(sensor_node, "camera/noise/type"),
        "noise_mean": _to_float(_text(sensor_node, "camera/noise/mean"), None),
        "noise_stddev": _to_float(_text(sensor_node, "camera/noise/stddev"), None),
        "update_rate": (
            _plugin_update_rate(plugin, sensor_node)
            if plugin is not None
            else _to_int(_text(sensor_node, "update_rate"), None)
        ),
        "plugin_filename": (
            str(plugin.get("filename", "")).strip() if plugin is not None else ""
        ),
        "namespace": _plugin_namespace(plugin) if plugin is not None else "",
        "image_topic": _plugin_topic(
            plugin,
            sensor_node,
            remap_keys=["image_raw"],
            plugin_tags=["imageTopicName"],
            default_topic="image_raw",
        ),
        "depth_topic": _plugin_topic(
            plugin,
            sensor_node,
            remap_keys=["depth/image_raw"],
            plugin_tags=["depthImageTopicName"],
            default_topic="",
        ),
    }


def _camera_load_sensor_profile(sensor_sdf_path: str) -> Dict[str, Any]:
    print(f"[DEBUG _camera_load_sensor_profile] path={sensor_sdf_path}")
    path = Path(sensor_sdf_path)
    if not path.exists():
        print(f"[DEBUG _camera_load_sensor_profile] FILE NOT FOUND: {sensor_sdf_path}")
    tree = ET.parse(path)
    root = tree.getroot()
    model = root.find("model")
    model_name = str(model.get("name", "")).strip() if model is not None else ""
    print(f"[DEBUG _camera_load_sensor_profile] model_name={model_name}")

    sensors = [_sensor_payload(node) for node in root.findall(".//sensor")]
    camera_sensors = [item for item in sensors if item["sensor_type"] == "camera"]
    depth_sensors = [item for item in sensors if item["sensor_type"] == "depth"]
    print(
        f"[DEBUG _camera_load_sensor_profile] found {len(sensors)} sensors total: {len(camera_sensors)} camera, {len(depth_sensors)} depth"
    )
    for i, s in enumerate(sensors):
        print(
            f"[DEBUG _camera_load_sensor_profile]   sensor[{i}]: type={s.get('sensor_type')}, name={s.get('name')}, image_topic={s.get('image_topic', 'N/A')}, depth_topic={s.get('depth_topic', 'N/A')}"
        )

    family = "mono"
    if depth_sensors:
        family = "depth"
    elif len(camera_sensors) >= 2:
        names = " ".join(
            f"{item.get('name', '')} {item.get('namespace', '')} {item.get('image_topic', '')}"
            for item in camera_sensors
        ).lower()
        if "left" in names and "right" in names:
            family = "stereo"

    print(f"[DEBUG _camera_load_sensor_profile] family={family}")
    profile: Dict[str, Any] = {
        "sensor_name": path.stem,
        "model_name": model_name,
        "family": family,
        "image_topic": "",
        "depth_topic": "",
        "left_topic": "",
        "right_topic": "",
        "image_width": None,
        "image_height": None,
        "image_format": "",
        "horizontal_fov": None,
        "clip_near": None,
        "clip_far": None,
        "noise_type": "",
        "noise_mean": None,
        "noise_stddev": None,
        "update_rate": None,
        "baseline": None,
    }

    if family == "depth":
        primary = (
            depth_sensors[0]
            if depth_sensors
            else (camera_sensors[0] if camera_sensors else {})
        )
        color = camera_sensors[0] if camera_sensors else primary
        profile.update(
            {
                "image_topic": str(
                    color.get("image_topic", "") or primary.get("image_topic", "") or ""
                ).strip(),
                "depth_topic": str(primary.get("depth_topic", "") or "").strip(),
                "image_width": primary.get("image_width"),
                "image_height": primary.get("image_height"),
                "image_format": str(primary.get("image_format", "")),
                "horizontal_fov": primary.get("horizontal_fov"),
                "clip_near": primary.get("clip_near"),
                "clip_far": primary.get("clip_far"),
                "noise_type": str(primary.get("noise_type", "")),
                "noise_mean": primary.get("noise_mean"),
                "noise_stddev": primary.get("noise_stddev"),
                "update_rate": primary.get("update_rate"),
            }
        )
        print(
            f"[DEBUG _camera_load_sensor_profile] depth profile: image_topic={profile['image_topic']}, depth_topic={profile['depth_topic']}, {profile.get('image_width')}x{profile.get('image_height')}"
        )
        return profile

    if family == "stereo":
        left = next(
            (
                item
                for item in camera_sensors
                if "left" in str(item.get("name", "")).lower()
                or "left" in str(item.get("namespace", "")).lower()
                or "left" in str(item.get("image_topic", "")).lower()
            ),
            camera_sensors[0] if camera_sensors else {},
        )
        right = next(
            (
                item
                for item in camera_sensors
                if item is not left
                and (
                    "right" in str(item.get("name", "")).lower()
                    or "right" in str(item.get("namespace", "")).lower()
                    or "right" in str(item.get("image_topic", "")).lower()
                )
            ),
            camera_sensors[1] if len(camera_sensors) > 1 else {},
        )
        left_pose = left.get("pose", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        right_pose = right.get("pose", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        baseline = math.dist(left_pose[:3], right_pose[:3]) if left and right else None
        profile.update(
            {
                "left_topic": str(left.get("image_topic", "") or "").strip(),
                "right_topic": str(right.get("image_topic", "") or "").strip(),
                "image_width": left.get("image_width"),
                "image_height": left.get("image_height"),
                "image_format": str(left.get("image_format", "")),
                "horizontal_fov": left.get("horizontal_fov"),
                "clip_near": left.get("clip_near"),
                "clip_far": left.get("clip_far"),
                "noise_type": str(left.get("noise_type", "")),
                "noise_mean": left.get("noise_mean"),
                "noise_stddev": left.get("noise_stddev"),
                "update_rate": left.get("update_rate"),
                "baseline": baseline,
            }
        )
        print(
            f"[DEBUG _camera_load_sensor_profile] stereo profile: left={profile['left_topic']}, right={profile['right_topic']}, baseline={baseline}, {profile.get('image_width')}x{profile.get('image_height')}"
        )
        return profile

    primary = (
        camera_sensors[0]
        if camera_sensors
        else (depth_sensors[0] if depth_sensors else {})
    )
    profile.update(
        {
            "image_topic": str(primary.get("image_topic", "") or "").strip(),
            "image_width": primary.get("image_width"),
            "image_height": primary.get("image_height"),
            "image_format": str(primary.get("image_format", "")),
            "horizontal_fov": primary.get("horizontal_fov"),
            "clip_near": primary.get("clip_near"),
            "clip_far": primary.get("clip_far"),
            "noise_type": str(primary.get("noise_type", "")),
            "noise_mean": primary.get("noise_mean"),
            "noise_stddev": primary.get("noise_stddev"),
            "update_rate": primary.get("update_rate"),
        }
    )
    print(
        f"[DEBUG _camera_load_sensor_profile] mono profile: image_topic={profile['image_topic']}, {profile.get('image_width')}x{profile.get('image_height')}"
    )
    return profile


def _camera_classify_sensor_profile(sensor_sdf_path: str) -> str:
    return str(_camera_load_sensor_profile(sensor_sdf_path).get("family", "mono"))


from .dcam_tests import *

from .generic_tests import *

from .mcam_tests import *

from .rfid_tests import *

from .scam_tests import *

from .tactile_tests import *

TESTS: dict[str, callable] = {
    # Basic sensor test
    "__basic__": sensor_capture_basic,
    # RFID tests
    "rfid_max_stable_read_distance": rfid_max_stable_read_distance,
    "rfid_min_stable_read_distance": rfid_min_stable_read_distance,
    "rfid_mass_read": rfid_mass_read,
    "rfid_overlap_tags": rfid_overlap_tags,
    "rfid_angle_dependence": rfid_angle_dependence,
    "rfid_move_tags": rfid_move_tags,
    "rfid_antenna_rotation": rfid_antenna_rotation,
    # Mono camera tests (mcam_ prefix, no numbers)
    "mcam_check": camera_check,
    "mcam_size_order_test": c1_size_order_test,
    "mcam_resolution_test": c2_resolution_test,
    "mcam_geometries_presence_test": c4_geometries_presence_test,
    "mcam_occlusion_test": c7_occlusion_test,
    "mcam_fov_test": c9_fov_test,
    "mcam_clipping_test": c10_clipping_test,
    "mcam_fps_stability_test": c11_fps_stability_test,
    # Depth camera tests (dcam_ prefix)
    "dcam_depth_perception_test": depth_perception_test,
    "dcam_working_range_test": c5_working_range_test,
    "dcam_small_displacement_sensitivity_test": c6_small_displacement_sensitivity_test,
    "dcam_view_angle_stability_test": c3_view_angle_stability_test,
    # Stereo camera tests (scam_ prefix)
    "scam_stereo_topics_presence_test": stereo_topics_presence_test,
    "scam_stereo_disparity_test": stereo_disparity_test,
    "scam_stereo_occlusion_test": stereo_occlusion_test,
    "scam_1_stereo_accuracy_test": s1_stereo_accuracy_test,
    "scam_2_texture_vs_smooth_stability_test": s2_texture_vs_smooth_stability_test,
    # Tactile tests
    "tactile_min_force_threshold": tactile_min_force_threshold,
    "tactile_response_uniformity": tactile_response_uniformity,
    "tactile_temporal_stability": tactile_temporal_stability,
    "tactile_peak_load_response": tactile_peak_load_response,
}
