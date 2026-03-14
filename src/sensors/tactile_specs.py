from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List


@dataclass(frozen=True)
class TactileSensorSpec:
    name: str
    rated_force_n: float
    rated_torque_nm: float
    active_zone_m: float = 0.05


TACTILE_SENSOR_SPECS: Dict[str, TactileSensorSpec] = {
    "ati_nano17": TactileSensorSpec(name="ati_nano17", rated_force_n=70.0, rated_torque_nm=0.5),
    "ati_nano25": TactileSensorSpec(name="ati_nano25", rated_force_n=1000.0, rated_torque_nm=6.0),
    "ati_mini45": TactileSensorSpec(name="ati_mini45", rated_force_n=1160.0, rated_torque_nm=20.0),
    "wacoh_dynpick_wlf_6a500_20_rad_b": TactileSensorSpec(
        name="wacoh_dynpick_wlf_6a500_20_rad_b",
        rated_force_n=500.0,
        rated_torque_nm=20.0,
    ),
    "leptrino_cfs018ca101u": TactileSensorSpec(
        name="leptrino_cfs018ca101u",
        rated_force_n=100.0,
        rated_torque_nm=0.5,
    ),
    "amti_he6x6_force_plate": TactileSensorSpec(
        name="amti_he6x6_force_plate",
        rated_force_n=4000.0,
        rated_torque_nm=200.0,
        active_zone_m=0.05,
    ),
}


def tactile_sensor_root(root_path: str) -> Path:
    return Path(root_path) / "resources" / "sensors" / "tactile"


def tactile_world_root(root_path: str) -> Path:
    return Path(root_path) / "resources" / "worlds" / "tactile"


def tactile_sensor_names(root_path: str) -> List[str]:
    root = tactile_sensor_root(root_path)
    names = [path.name for path in sorted(root.iterdir()) if path.is_dir() and (path / "model.sdf").exists()]
    return names


def get_tactile_spec(sensor_name: str) -> TactileSensorSpec:
    if sensor_name not in TACTILE_SENSOR_SPECS:
        raise KeyError(f"Unknown tactile sensor spec: {sensor_name}")
    return TACTILE_SENSOR_SPECS[sensor_name]


def validate_tactile_sensor_name(root_path: str, sensor_name: str) -> Path:
    sensor_dir = tactile_sensor_root(root_path) / sensor_name
    model_path = sensor_dir / "model.sdf"
    if not model_path.exists():
        raise FileNotFoundError(f"Tactile sensor model not found: {model_path}")
    return model_path


def list_specs(sensor_names: Iterable[str]) -> List[TactileSensorSpec]:
    return [get_tactile_spec(name) for name in sensor_names]
