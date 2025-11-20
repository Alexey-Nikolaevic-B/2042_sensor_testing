import os
import yaml
from typing import Any, Dict
from pathlib import Path


class Config:
    EXPECTED_TYPES = {
        'BASE_WORLD_PATH': str,
        'CATKIN_SETUP_DIR': str,
        'SENSOR_PKG': str,
        'LAUNCH_FILE': str,
        'WORLDS_PATH': str,
        'SENSORS_PATH': str,
        'SAVE_DIR': str,
        'MESSAGE_TIMEOUT': int,
        'SAVE_SENSOR_DATA': bool,

        'SAVE_DEBUG_DATA': bool,
        'SAVE_DEBUG_DIR': str,
        'ROS_LOG_PATH': str,
        'ROOT_PATH': str
    }

    config_file = "config.yaml"
    
    def __init__(self):
        self._data = self._load_defaults()
        self._data['ROOT_PATH'] = os.path.dirname(os.path.abspath(__file__))
    
    def _load_defaults(self) -> Dict[str, Any]:
        if self.config_file and Path(self.config_file).exists():
            return self._load_from_yaml()
        else:
            return self._get_hardcoded_defaults()
    
    def _load_from_yaml(self) -> Dict[str, Any]:
        try:
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Error loading YAML config {self.config_file}: {e}")
            return self._get_hardcoded_defaults()
    
    def _get_hardcoded_defaults(self) -> Dict[str, Any]:
        return {
            'BASE_WORLD_PATH': 'catkin_ws/scenario_test_pkg/worlds/base_world.world',
            'CATKIN_SETUP_DIR': 'catkin_ws/devel/setup.bash',
            'SENSOR_PKG': 'scenario_test_pkg',
            'LAUNCH_FILE': 'scenario.launch',
            'WORLDS_PATH': 'resources/worlds/',
            'SENSORS_PATH': 'resources/sensors/',
            'SAVE_DIR': 'captured_data',
            'MESSAGE_TIMEOUT': 10,
            'SAVE_SENSOR_DATA': True,
            'ROS_LOG_PATH': 'ros_log',

            'SAVE_DEBUG_DATA': True,
            'SAVE_DEBUG_DIR': 'sensor_debug/',
            'ROOT_PATH': ''
        }
    
    def _save_to_yaml(self) -> None:
        try:
            with open(self.config_file, 'w') as f:
                yaml.dump(self._data, f, default_flow_style=False)
        except Exception as e:
            print(f"Error saving config to {self.config_file}: {e}")
    
    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)
    
    def set(self, key: str, value: Any) -> None:
        if key in self.EXPECTED_TYPES:
            expected_type = self.EXPECTED_TYPES[key]
            if not isinstance(value, expected_type):
                raise TypeError(f"Config key '{key}' expects type {expected_type.__name__}, got {type(value).__name__}")
        self._data[key] = value
        self._save_to_yaml()
    
    def update(self, **kwargs) -> None:
        for key, value in kwargs.items():
            self.set(key, value)
    
    def reset(self, key: str) -> None:
        defaults = self._load_defaults()
        if key in defaults:
            self._data[key] = defaults[key]
            self._save_to_yaml()
    
    def reset_all(self) -> None:
        self._data = self._load_defaults().copy()
        self._save_to_yaml()
    
    def to_dict(self) -> Dict[str, Any]:
        return self._data.copy()
    
    def __getitem__(self, key: str) -> Any:
        return self._data[key]
    
    def __setitem__(self, key: str, value: Any) -> None:
        self.set(key, value)
    
    def __contains__(self, key: str) -> bool:
        return key in self._data
    
    def __str__(self) -> str:
        return f"Config({self._data})"

CONFIG = Config()