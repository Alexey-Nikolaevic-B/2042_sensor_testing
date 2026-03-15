from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

from .depth_profile_base import DepthProfileBase
from .mono_profile_base import MonoProfileBase
from .sdf_profile import classify_sensor_profile
from .sensor import REGISTRY, register_sensor
from .stereo_profile_base import StereoProfileBase


_SPECIAL_CAMERA_MODULES = {
    "mono_camera",
    "depth_camera",
    "uvc_profile_640x480_60deg",
}

_DEPTH_TEST_OVERRIDES: Dict[str, Dict[str, object]] = {
    "azure_kinect_wfov": {
        "TEST_DISTANCES": (0.8, 1.5, 2.5),
        "MAX_ABS_ERROR_M": 0.5,
    },
    "d455_like": {
        "MAX_ABS_ERROR_M": 0.9,
    },
}


def _camelize(value: str) -> str:
    tokens = [token for token in str(value).replace("-", "_").split("_") if token]
    return "".join(token[:1].upper() + token[1:] for token in tokens) or "CameraProfile"


def _build_camera_class(sensor_name: str, family: str):
    base_by_family = {
        "mono": MonoProfileBase,
        "depth": DepthProfileBase,
        "stereo": StereoProfileBase,
    }
    base = base_by_family[family]
    attrs: Dict[str, object] = {"__module__": __name__}
    if family == "depth":
        attrs.update(_DEPTH_TEST_OVERRIDES.get(sensor_name, {}))
    cls_name = f"{_camelize(sensor_name)}Auto"
    SensorType = type(cls_name, (base,), attrs)
    return register_sensor("camera", sensor_name)(SensorType)


def register_camera_profiles_from_sdf() -> Tuple[int, int]:
    repo_root = Path(__file__).resolve().parents[2]
    camera_dir = repo_root / "resources" / "sensors" / "camera"
    seen = 0
    created = 0

    for sdf_path in sorted(camera_dir.glob("*.sdf")):
        sensor_name = sdf_path.stem
        seen += 1
        if sensor_name in _SPECIAL_CAMERA_MODULES:
            continue
        if ("camera", sensor_name) in REGISTRY:
            continue
        family = classify_sensor_profile(str(sdf_path))
        _build_camera_class(sensor_name, family)
        created += 1

    return seen, created


register_camera_profiles_from_sdf()
