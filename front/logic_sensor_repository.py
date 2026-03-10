import copy
from datetime import datetime

from PyQt5.QtCore import QObject, pyqtSignal

import src.database.sensor_storage as db


class Sensor:

    REQUIRED_FIELDS = ("id", "name", "type")

    def __init__(self, data: dict):
        if not isinstance(data, dict):
            raise TypeError("Sensor data must be a dict")
        for field in self.REQUIRED_FIELDS:
            if field not in data:
                raise ValueError(f"Sensor data missing required field: '{field}'")
        self._data = copy.deepcopy(data)
        self._data.setdefault("tests", [])

    def __getitem__(self, key):        return self._data[key]
    def __setitem__(self, key, value): self._data[key] = value
    def __contains__(self, key):       return key in self._data
    def get(self, key, default=None):  return self._data.get(key, default)

    @property
    def id(self)          -> str:  return self._data["id"]
    @property
    def name(self)        -> str:  return self._data["name"]
    @property
    def sensor_type(self) -> str:  return self._data["type"]
    @property
    def description(self) -> str:  return self._data.get("description", "")
    @property
    def image_path(self)  -> str:  return self._data.get("image_path", "")
    @property
    def params(self)      -> dict: return self._data.get("params", {})
    @property
    def tests(self)       -> list: return self._data["tests"]
    @property
    def last_update(self):
        return self._data.get("last_update", datetime.min)

    def to_dict(self) -> dict:
        return copy.deepcopy(self._data)

    def update_fields(self, fields: dict):
        fields = copy.deepcopy(fields)
        fields.pop("id", None)
        self._data.update(fields)

    def get_test(self, test_name: str) -> dict | None:
        for t in self._data["tests"]:
            if t.get("name") == test_name:
                return t
        return None

    def update_test(self, test_name: str, fields: dict) -> bool:
        for t in self._data["tests"]:
            if t.get("name") == test_name:
                t.update(fields)
                return True
        new_test = {"name": test_name}
        new_test.update(fields)
        self._data["tests"].append(new_test)
        return True

    def __repr__(self):
        return f"<Sensor id={self.id!r} name={self.name!r}>"


class SensorRepository(QObject):

    sensor_added   = pyqtSignal(dict)
    sensor_updated = pyqtSignal(dict)
    sensor_deleted = pyqtSignal(str)
    sensors_loaded = pyqtSignal()
    test_updated   = pyqtSignal(str, dict)

    _instance: "SensorRepository | None" = None

    @classmethod
    def instance(cls) -> "SensorRepository":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sensors: dict[str, Sensor] = {}
        SensorRepository._instance = self
        db.init_db()
        db.init_test_meta_table()
        self.load()

    def load(self):
        self._sensors.clear()
        for raw in db.get_all_sensors():
            try:
                s = Sensor(raw)
                self._sensors[s.id] = s
            except (TypeError, ValueError) as exc:
                print(f"[SensorRepository] Skipping invalid sensor: {exc}")
        print(f"[SensorRepository] Loaded {len(self._sensors)} sensors from DB")
        self.sensors_loaded.emit()

    def all_sensors(self) -> list[dict]:
        return [s.to_dict() for s in self._sensors.values()]

    def get_sensor(self, sensor_id: str) -> dict | None:
        s = self._sensors.get(sensor_id)
        return s.to_dict() if s else None

    def get_sensor_by_name(self, name: str) -> dict | None:
        for s in self._sensors.values():
            if s.name == name:
                return s.to_dict()
        return None

    def get_types(self) -> list[str]:
        return db.get_sensor_types()

    def count(self) -> int:
        return len(self._sensors)

    def add_sensor(self, data: dict) -> dict:
        db.add_sensor(
            sensor_name = data["name"],
            sensor_type = data["type"],
            sdf_path    = data.get("sdf_path", ""),
            description = data.get("description", ""),
            image_path  = data.get("image_path", ""),
            params      = data.get("params", {}),
        )
        fresh = db.get_sensor_by_name(data["name"])
        s = Sensor(fresh)
        self._sensors[s.id] = s
        self._extract_and_save_params(s)
        self._seed_test_meta(s)
        self._reload_sensor_from_db(s.id)
        self.sensor_added.emit(self._sensors[s.id].to_dict())
        return self._sensors[s.id].to_dict()

    def _seed_test_meta(self, sensor: "Sensor") -> None:
        try:
            from src.sensors import REGISTRY
            from config import CONFIG
            from src.test_utils import load_test_functions

            SensorClass = next(
                (cls for (stype, _), cls in REGISTRY.items() if stype == sensor.sensor_type),
                None
            )
            if SensorClass is None:
                return

            instance = SensorClass(CONFIG)
            funcs = load_test_functions(instance)
            existing = {r["func_name"] for r in db.get_test_meta(sensor.id)}
            for func_name in funcs:
                if func_name not in existing:
                    db.save_test_meta(sensor.id, func_name, func_name, "", "")
        except Exception:
            pass

    def _extract_and_save_params(self, sensor: "Sensor") -> None:
        # TODO: replace mock param with real get_params() once backend is stable
        try:
            from src.sensors import REGISTRY
            from config import CONFIG

            SensorClass = next(
                (cls for (stype, _), cls in REGISTRY.items() if stype == sensor.sensor_type),
                None
            )
            if SensorClass is None:
                params = {"TODO": "get parameters from sensor"}
            else:
                try:
                    instance = SensorClass(CONFIG)
                    params = instance.get_params()
                except Exception:
                    params = {"TODO": "get parameters from sensor"}
        except Exception:
            params = {"TODO": "get parameters from sensor"}

        db.update_sensor(sensor.name, params=params)
        sensor.update_fields({"params": params})

    def _reload_sensor_from_db(self, sensor_id: str) -> None:
        s = self._sensors.get(sensor_id)
        if s is None:
            return
        fresh = db.get_sensor_by_name(s.name)
        if fresh:
            self._sensors[sensor_id] = Sensor(fresh)

    def update_sensor(self, sensor_id: str, fields: dict) -> dict:
        s = self._sensors.get(sensor_id)
        if s is None:
            raise KeyError(f"No sensor with id {sensor_id!r}")
        db.update_sensor(
            sensor_name = s.name,
            description = fields.get("description"),
            image_path  = fields.get("image_path"),
            params      = fields.get("params"),
            sdf_path    = fields.get("sdf_path"),
        )
        s.update_fields(fields)
        self.sensor_updated.emit(s.to_dict())
        return s.to_dict()

    def delete_sensor(self, sensor_id: str):
        s = self._sensors.get(sensor_id)
        if s is None:
            raise KeyError(f"No sensor with id {sensor_id!r}")
        db.delete_sensor(s.name)
        del self._sensors[sensor_id]
        self.sensor_deleted.emit(sensor_id)

    def save_test_result(
        self,
        sensor_id: str,
        test_name: str,
        status: str,
        result,
        description: str = "",
        duration: float = 0.0,
    ) -> dict:
        s = self._sensors.get(sensor_id)
        if s is None:
            raise KeyError(f"No sensor with id {sensor_id!r}")

        db.save_test_result(
            sensor_name = s.name,
            test_name   = test_name,
            status      = status,
            result      = result,
            description = description,
            duration    = duration,
        )

        from datetime import datetime as _dt
        today = _dt.now().strftime("%Y-%m-%d")

        result_str = str(result) if not isinstance(result, str) else result
        s.update_test(test_name, {
            "status":   status,
            "result":   result_str,
            "duration": duration,
            "date":     today,
        })

        updated_test = s.get_test(test_name)
        self.test_updated.emit(sensor_id, copy.deepcopy(updated_test))
        return copy.deepcopy(updated_test)

    def get_test_meta(self, sensor_id: str) -> list[dict]:
        rows = db.get_test_meta(sensor_id)
        return {r["func_name"]: r for r in rows}

    def save_test_meta(self, sensor_id: str, func_name: str,
                       display_name: str, description: str,
                       image_path: str) -> None:
        db.save_test_meta(sensor_id, func_name, display_name, description, image_path)

        s = self._sensors.get(sensor_id)
        if s is None:
            return
        sensor_type = s.sensor_type
        for sensor in self._sensors.values():
            if sensor.sensor_type == sensor_type:
                sensor.update_test(func_name, {
                    "display_name": display_name,
                    "description":  description,
                    "image_path":   image_path,
                })