from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


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
        "update_rate": _plugin_update_rate(plugin, sensor_node) if plugin is not None else _to_int(_text(sensor_node, "update_rate"), None),
        "plugin_filename": str(plugin.get("filename", "")).strip() if plugin is not None else "",
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


def load_sensor_profile(sensor_sdf_path: str) -> Dict[str, Any]:
    path = Path(sensor_sdf_path)
    tree = ET.parse(path)
    root = tree.getroot()
    model = root.find("model")
    model_name = str(model.get("name", "")).strip() if model is not None else ""

    sensors = [_sensor_payload(node) for node in root.findall(".//sensor")]
    camera_sensors = [item for item in sensors if item["sensor_type"] == "camera"]
    depth_sensors = [item for item in sensors if item["sensor_type"] == "depth"]

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
        primary = depth_sensors[0] if depth_sensors else (camera_sensors[0] if camera_sensors else {})
        color = camera_sensors[0] if camera_sensors else primary
        profile.update(
            {
                "image_topic": str(color.get("image_topic", "") or primary.get("image_topic", "") or "").strip(),
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
        return profile

    if family == "stereo":
        left = next(
            (
                item for item in camera_sensors
                if "left" in str(item.get("name", "")).lower()
                or "left" in str(item.get("namespace", "")).lower()
                or "left" in str(item.get("image_topic", "")).lower()
            ),
            camera_sensors[0] if camera_sensors else {},
        )
        right = next(
            (
                item for item in camera_sensors
                if item is not left and (
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
        return profile

    primary = camera_sensors[0] if camera_sensors else (depth_sensors[0] if depth_sensors else {})
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
    return profile


def classify_sensor_profile(sensor_sdf_path: str) -> str:
    return str(load_sensor_profile(sensor_sdf_path).get("family", "mono"))
