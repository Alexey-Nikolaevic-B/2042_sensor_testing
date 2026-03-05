import json
import os
import copy
from datetime import datetime

from PyQt5.QtCore import QObject, pyqtSignal


class Sensor:

    REQUIRED_FIELDS = ("id", "name", "type")

    def __init__(self, data: dict):
        if not isinstance(data, dict):
            raise TypeError("Sensor data must be a dict")
        for field in self.REQUIRED_FIELDS:
            if field not in data:
                raise ValueError(f"Sensor data missing required field: '{field}'")
        self._data = copy.deepcopy(data)
        # Normalise tests list
        if "tests" not in self._data:
            self._data["tests"] = []

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value

    def get(self, key, default=None):
        return self._data.get(key, default)

    def __contains__(self, key):
        return key in self._data

    @property
    def id(self) -> str:
        return self._data["id"]

    @property
    def name(self) -> str:
        return self._data["name"]

    @property
    def sensor_type(self) -> str:
        return self._data["type"]

    @property
    def description(self) -> str:
        return self._data.get("description", "")

    @property
    def image_path(self) -> str:
        return self._data.get("image_path", "")

    @property
    def params(self) -> dict:
        return self._data.get("params", {})

    @property
    def tests(self) -> list:
        return self._data["tests"]

    @property
    def last_update(self):
        return self._data.get("last_update", datetime.min)


    def to_dict(self) -> dict:
        """Return a deep copy of the underlying dict (safe to pass to UI)."""
        return copy.deepcopy(self._data)

    def update_fields(self, fields: dict):
        """Merge *fields* into this sensor.  'id' cannot be changed."""
        fields = copy.deepcopy(fields)
        fields.pop("id", None)          # id is immutable
        self._data.update(fields)


    def get_test(self, test_name: str) -> dict | None:
        for t in self._data["tests"]:
            if t.get("name") == test_name:
                return t
        return None

    def update_test(self, test_name: str, fields: dict):
        for t in self._data["tests"]:
            if t.get("name") == test_name:
                t.update(fields)
                return True
        return False

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
        """Return the application-wide singleton, creating it if necessary."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self, data_path: str = "_mock.json", parent=None):
        super().__init__(parent)
        self._data_path = data_path
        self._sensors: dict[str, Sensor] = {}   # id → Sensor
        self._types: list[str] = []
        SensorRepository._instance = self

        self.load()


    def load(self, path: str | None = None):
        if path:
            self._data_path = path

        raw_sensors, raw_types = self._read_json(self._data_path)

        self._sensors.clear()
        for raw in raw_sensors:
            try:
                s = Sensor(raw)
                self._sensors[s.id] = s
            except (TypeError, ValueError) as exc:
                print(f"[SensorRepository] Skipping invalid sensor: {exc}")

        self._types = raw_types or self._derive_types()
        self.sensors_loaded.emit()

    def save(self, path: str | None = None):
        path = path or self._data_path
        payload = {
            "sensors": [s.to_dict() for s in self._sensors.values()],
            "types": self._types,
        }
        try:
            with open(path, "w") as f:
                json.dump(payload, f, indent=2, default=str)
        except OSError as exc:
            print(f"[SensorRepository] Could not save to {path!r}: {exc}")

    def all_sensors(self) -> list[dict]:
        return [s.to_dict() for s in self._sensors.values()]

    def get_sensor(self, sensor_id: str) -> dict | None:
        s = self._sensors.get(sensor_id)
        return s.to_dict() if s else None

    def get_types(self) -> list[str]:
        return list(self._types)

    def count(self) -> int:
        return len(self._sensors)


    def add_sensor(self, data: dict) -> dict:
        sensor_id = data.get("id")
        if not sensor_id:
            raise ValueError("Sensor data must include an 'id' field.")
        if sensor_id in self._sensors:
            raise ValueError(f"A sensor with id {sensor_id!r} already exists.")

        s = Sensor(data)
        self._sensors[sensor_id] = s
        self.sensor_added.emit(s.to_dict())
        return s.to_dict()

    def update_sensor(self, sensor_id: str, fields: dict) -> dict:
        s = self._sensors.get(sensor_id)
        if s is None:
            raise KeyError(f"No sensor found with id {sensor_id!r}.")

        s.update_fields(fields)
        self.sensor_updated.emit(s.to_dict())
        return s.to_dict()

    def delete_sensor(self, sensor_id: str):
        if sensor_id not in self._sensors:
            raise KeyError(f"No sensor found with id {sensor_id!r}.")
        del self._sensors[sensor_id]
        self.sensor_deleted.emit(sensor_id)

    def update_test(self, sensor_id: str, test_name: str, fields: dict) -> dict:
        s = self._sensors.get(sensor_id)
        if s is None:
            raise KeyError(f"No sensor found with id {sensor_id!r}.")

        updated = s.update_test(test_name, fields)
        if not updated:
            raise ValueError(
                f"No test named {test_name!r} found in sensor {sensor_id!r}."
            )

        test = s.get_test(test_name)
        self.test_updated.emit(sensor_id, copy.deepcopy(test))
        return copy.deepcopy(test)

    @staticmethod
    def _read_json(path: str) -> tuple[list, list]:
        if not os.path.exists(path):
            print(f"[SensorRepository] File not found: {path!r}. Starting empty.")
            return [], []
        try:
            with open(path, "r") as f:
                raw = json.load(f)
            return raw.get("sensors", []), raw.get("types", [])
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[SensorRepository] Failed to read {path!r}: {exc}")
            return [], []

    def _derive_types(self) -> list[str]:
        seen = sorted({s.sensor_type for s in self._sensors.values() if s.sensor_type})
        return seen