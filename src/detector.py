import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Type

logger = logging.getLogger(__name__)


# ── Legacy class-based detector registry (used by rfid_detector.py) ───────────

class SensorDetector(ABC):
    sensor_type: str
    priority: int = 0

    @abstractmethod
    def detect(self, sdf_content: str) -> bool:
        raise NotImplementedError


DETECTOR_REGISTRY: Dict[str, "SensorDetector"] = {}


def register_detector(cls: Type[SensorDetector]) -> Type[SensorDetector]:
    instance = cls()
    DETECTOR_REGISTRY[cls.sensor_type] = instance
    return cls


# ── Custom function-based detectors (new system, shown in Add Sensor Type UI) ─
#
# Write a plain function:
#
#     def my_detector(sdf_content: str) -> bool:
#         return "my_plugin.so" in sdf_content
#
# Add it to CUSTOM_DETECTORS and it appears in the dialog picker.
# The function name is stored in DB; called at detection time.

CUSTOM_DETECTORS: dict = {
    # "my_detector": my_detector_fn,
}


def get_custom_detector_names() -> list:
    return sorted(CUSTOM_DETECTORS.keys())


# ── Main detection entry point ────────────────────────────────────────────────

def _load_detector_modules() -> None:
    # Auto-import all *_detector.py files so they self-register into DETECTOR_REGISTRY
    import os, importlib
    pkg_dir = os.path.dirname(__file__)
    for fname in os.listdir(pkg_dir):
        if fname.endswith('_detector.py'):
            mod_name = f'src.{fname[:-3]}'
            try:
                importlib.import_module(mod_name)
            except Exception as e:
                logger.warning('Could not load detector module %s: %s', mod_name, e)


_detectors_loaded = False


def _load_sensor_modules() -> None:
    # Auto-import sensor class modules so they register into REGISTRY via @register_sensor
    import os, importlib
    pkg_dir = os.path.dirname(__file__)
    skip = {'__init__.py', 'sensor.py', 'detector.py'}
    for fname in os.listdir(pkg_dir):
        if fname.endswith('.py') and fname not in skip and not fname.endswith('_detector.py'):
            mod_name = f'src.{fname[:-3]}'
            try:
                importlib.import_module(mod_name)
            except Exception as e:
                logger.warning('Could not load sensor module %s: %s', mod_name, e)


def detect_sensor_type(sdf_path: str) -> Optional[str]:
    global _detectors_loaded
    if not _detectors_loaded:
        _load_detector_modules()
        _detectors_loaded = True

    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error("detect_sensor_type: cannot read %r: %s", sdf_path, e)
        return None

    # 1. Class-based registry (rfid_detector.py etc.)
    detectors = sorted(DETECTOR_REGISTRY.values(), key=lambda d: d.priority, reverse=True)
    matches = [d for d in detectors if d.detect(content)]
    if matches:
        return matches[0].sensor_type

    # 2. Function-based custom detectors
    for name, fn in CUSTOM_DETECTORS.items():
        try:
            if fn(content):
                # look up which sensor_type uses this detector fn
                import src.sensor_storage as db
                for t in db.get_all_sensor_types():
                    det = t.get("detection", {})
                    if det.get("mode") == "custom" and det.get("detector_fn") == name:
                        return t["sensor_type"]
        except Exception as e:
            logger.error("custom detector %r raised: %s", name, e)

    # 3. Simple plugin-string match from DB
    return _detect_from_db(content)


def _detect_from_db(content: str) -> Optional[str]:
    try:
        import src.sensor_storage as db
        for t in db.get_all_sensor_types():
            det = t.get("detection", {})
            if det.get("mode") == "simple":
                # Support both old single-plugin and new multi-plugin formats
                plugins = det.get("plugins") or (
                    [det["plugin"]] if det.get("plugin") else []
                )
                if plugins and all(p in content for p in plugins):
                    return t["sensor_type"]
    except Exception as e:
        logger.error("_detect_from_db failed: %s", e)
    return None