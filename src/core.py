import logging
from typing import List, Dict, Any, Optional

from .gazebo_simulator import Simulator
from .sensors import REGISTRY, Sensor
from .sensors.detector import detect_sensor_type as _detect

from config import CONFIG

logger = logging.getLogger(__name__)


class Core:
    def __init__(self) -> None:
        self.simulator = Simulator(CONFIG)
        self._load_builtins()
        self._sync_tests()

    def _load_builtins(self) -> None:
        from .sensors import rfid_detecto
        import src.tests                   

    def _sync_tests(self) -> None:
        try:
            import src.database.sensor_storage as db
            from src.tests import TEST_REGISTRY

            db.init_db()

            
            registry_keys: set[tuple[str, str]] = set()
            for func_name, test_instance in TEST_REGISTRY.items():
                for sensor_type in test_instance.compatible_types:
                    if sensor_type == "*":
                    
                        for st in db.get_sensor_types():
                            registry_keys.add((st, func_name))
                    else:
                        registry_keys.add((sensor_type, func_name))

            db_keys: set[tuple[str, str]] = set()
            for sensor_type in {k[0] for k in registry_keys} | set(db.get_sensor_types()):
                for row in db.get_type_tests(sensor_type):
                    db_keys.add((sensor_type, row["func_name"]))

            new_keys = registry_keys - db_keys
            for sensor_type, func_name in new_keys:
                test_instance = TEST_REGISTRY.get(func_name)
                display_name  = getattr(test_instance, "display_name", func_name) if test_instance else func_name
                description   = getattr(test_instance, "description",  "")        if test_instance else ""
                db.upsert_type_test(sensor_type, func_name, display_name, description, "")
                logger.info("_sync_tests: added %r for sensor_type=%r", func_name, sensor_type)

            stale_keys = db_keys - registry_keys
            for sensor_type, func_name in stale_keys:
                db.delete_type_test(sensor_type, func_name)
                logger.info("_sync_tests: removed stale %r for sensor_type=%r", func_name, sensor_type)

        
            existing_keys = registry_keys & db_keys
            for sensor_type, func_name in existing_keys:
                test_instance = TEST_REGISTRY.get(func_name)
                if test_instance is None:
                    continue
                rows = db.get_type_tests(sensor_type)
                row  = next((r for r in rows if r["func_name"] == func_name), None)
                if row is None:
                    continue
            
                new_display = getattr(test_instance, "display_name", func_name)
                new_desc    = getattr(test_instance, "description",  "")
                if row["display_name"] in ("", func_name) and row["description"] == "":
                    db.upsert_type_test(sensor_type, func_name, new_display, new_desc, row["image_path"])

            total_new   = len(new_keys)
            total_stale = len(stale_keys)
            if total_new or total_stale:
                logger.info("_sync_tests complete: +%d new, -%d removed", total_new, total_stale)
            else:
                logger.debug("_sync_tests: registry matches DB, nothing to do")

        except Exception as exc:
            logger.error("_sync_tests failed: %s", exc, exc_info=True)

    def get_sensor_types(self) -> List[str]:
        return sorted(REGISTRY.keys())

    def get_tests(self, sensor: Sensor) -> Dict[str, Any]:
        from src.tests import get_tests_for_sensor
        return get_tests_for_sensor(sensor)

    def detect_sensor_type(self, sdf_path: str) -> Optional[str]:
        return _detect(sdf_path)

    def save_sensor_params(self, sensor_id: str, params: dict, repo) -> None:
        sensor_data = repo.get_sensor(sensor_id)
        if sensor_data is None:
            raise KeyError(f"No sensor with id {sensor_id!r}")

        sensor_type = sensor_data.get("type")
        sdf_path    = sensor_data.get("sdf_path")

        SensorClass = REGISTRY.get(sensor_type)
        if SensorClass is None:
            raise ValueError(f"No sensor class registered for type {sensor_type!r}")

        instance = SensorClass(sdf_path)
        instance.save_params_to_sdf(sdf_path, params)
        repo.update_sensor(sensor_id, {"params": params})

    def kill(self) -> None:
        self.simulator.kill()