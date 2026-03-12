import logging
from typing import Optional

logger = logging.getLogger(__name__)


# ── Custom detector functions ─────────────────────────────────────────────────
#
# A detector is a plain function:
#
#     def my_detector(sdf_content: str) -> bool:
#         return "my_plugin.so" in sdf_content
#
# Add it to the CUSTOM_DETECTORS dict below and it will appear in the
# "Add Sensor Type" dialog under Custom detector.
#
# When selected, the detector name is stored in the DB and called automatically
# every time an SDF file is loaded to identify the sensor type.
#
# Example:
#
# def rfid_detector(sdf_content: str) -> bool:
#     return "libRFID_antenna_plugin.so" in sdf_content
#
# CUSTOM_DETECTORS = {
#     "rfid_detector": rfid_detector,
# }

CUSTOM_DETECTORS: dict[str, callable] = {
    # "my_detector": my_detector_fn,
}


# ── Runtime detectors (sensor_type → fn) ─────────────────────────────────────
#
# Populated automatically from DB at detection time. You don't need to touch
# this unless you want to hard-code a detector outside the DB.

DETECTORS: dict[str, callable] = {}


def detect_sensor_type(sdf_path: str) -> Optional[str]:
    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error("detect_sensor_type: cannot read %r: %s", sdf_path, e)
        return None

    for sensor_type, detect_fn in DETECTORS.items():
        try:
            if detect_fn(content):
                return sensor_type
        except Exception as e:
            logger.error("detector %r raised: %s", sensor_type, e)

    return _detect_from_db(content)


def _detect_from_db(content: str) -> Optional[str]:
    try:
        import src.database.sensor_storage as db
        for type_def in db.get_all_sensor_types():
            detection = type_def.get("detection", {})
            mode = detection.get("mode", "simple")
            if mode == "simple":
                plugin = detection.get("plugin", "")
                if plugin and plugin in content:
                    return type_def["sensor_type"]
            elif mode == "custom":
                fn_name = detection.get("detector_fn", "")
                fn = CUSTOM_DETECTORS.get(fn_name)
                if fn:
                    try:
                        if fn(content):
                            return type_def["sensor_type"]
                    except Exception as e:
                        logger.error("custom detector %r raised: %s", fn_name, e)
    except Exception as e:
        logger.error("_detect_from_db failed: %s", e)
    return None


def get_custom_detector_names() -> list[str]:
    """Return sorted list of names available for selection in the UI."""
    return sorted(CUSTOM_DETECTORS.keys())