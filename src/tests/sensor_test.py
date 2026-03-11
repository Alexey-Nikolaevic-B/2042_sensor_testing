from abc import ABC, abstractmethod
from typing import Dict, List, Type, Optional
import logging

logger = logging.getLogger(__name__)


class SensorTest(ABC):
    # Subclasses must set these
    name:             str
    display_name:     str
    description:      str = ""
    compatible_types: List[str] = []   # ["*"] means all sensor types

    @abstractmethod
    def run(self, simulator, sensor, progress_cb=None) -> dict:
        raise NotImplementedError


# Registry: test name  ->  SensorTest instance
TEST_REGISTRY: Dict[str, "SensorTest"] = {}


def register_test(cls: Type[SensorTest]) -> Type[SensorTest]:
    required = ("name", "display_name", "compatible_types")
    for attr in required:
        if not hasattr(cls, attr):
            raise AttributeError(
                f"@register_test: {cls.__name__} is missing required attribute '{attr}'"
            )
    instance = cls()
    if instance.name in TEST_REGISTRY:
        logger.warning(
            "register_test: overwriting existing test %r with %s",
            instance.name, cls.__name__,
        )
    TEST_REGISTRY[instance.name] = instance
    return cls


def get_tests_for_sensor(sensor) -> Dict[str, callable]:
    sensor_type = getattr(sensor, "sensor_type", None)
    result: Dict[str, callable] = {}

    for name, test_instance in TEST_REGISTRY.items():
        ct = test_instance.compatible_types
        if "*" in ct or sensor_type in ct:
            # Bind sensor into the callable so the runner only passes simulator
            def _make_bound(ti, s):
                def _bound(simulator, progress_cb=None):
                    return ti.run(simulator, s, progress_cb=progress_cb)
                _bound.__name__ = ti.name
                return _bound
            result[name] = _make_bound(test_instance, sensor)

    return result
