from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

from .sensor import REGISTRY, register_sensor
from .tactile_profile_base import TactileProfileBase


def _camelize(value: str) -> str:
    tokens = [token for token in str(value).replace("-", "_").split("_") if token]
    return "".join(token[:1].upper() + token[1:] for token in tokens) or "TactileProfile"


def _build_tactile_class(sensor_name: str):
    attrs: Dict[str, object] = {"__module__": __name__}
    cls_name = f"{_camelize(sensor_name)}Tactile"
    sensor_type = type(cls_name, (TactileProfileBase,), attrs)
    return register_sensor("tactile", sensor_name)(sensor_type)


def register_tactile_profiles_from_sdf() -> Tuple[int, int]:
    repo_root = Path(__file__).resolve().parents[2]
    tactile_root = repo_root / "resources" / "sensors" / "tactile"
    seen = 0
    created = 0

    for sensor_dir in sorted(tactile_root.iterdir()):
        if not sensor_dir.is_dir():
            continue
        sdf_path = sensor_dir / "model.sdf"
        if not sdf_path.exists():
            continue
        sensor_name = sensor_dir.name
        seen += 1
        if ("tactile", sensor_name) in REGISTRY:
            continue
        _build_tactile_class(sensor_name)
        created += 1

    return seen, created


register_tactile_profiles_from_sdf()
