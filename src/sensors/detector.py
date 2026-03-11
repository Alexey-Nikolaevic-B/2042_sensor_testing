from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Type
import logging

logger = logging.getLogger(__name__)


class SensorDetector(ABC):
    sensor_type: str
    priority:    int = 0

    @abstractmethod
    def detect(self, sdf_content: str) -> bool:
        raise NotImplementedError


DETECTOR_REGISTRY: Dict[str, SensorDetector] = {}


def register_detector(cls: Type[SensorDetector]) -> Type[SensorDetector]:
    instance = cls()
    DETECTOR_REGISTRY[cls.sensor_type] = instance
    return cls


def detect_sensor_type(sdf_path: str) -> Optional[str]:
    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error(f"detect_sensor_type: cannot read {sdf_path!r}: {e}")
        return None

    detectors = sorted(
        DETECTOR_REGISTRY.values(),
        key=lambda d: d.priority,
        reverse=True,
    )

    matches = [d for d in detectors if d.detect(content)]

    if not matches:
        return None

    if len(matches) > 1:
        names = [d.sensor_type for d in matches]
        logger.warning(
            f"SDF matched multiple sensor types: {names}. "
            f"Using {matches[0].sensor_type!r} (highest priority). "
            "Detection may be incomplete."
        )

    return matches[0].sensor_type