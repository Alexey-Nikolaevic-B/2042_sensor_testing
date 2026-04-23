import copy
import os
from datetime import datetime

from PyQt5.QtCore import QObject, pyqtSignal

import src.sensor_storage as db

_PROJECT_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))


def _resolve_path(path: str) -> str:
    """Convert a relative DB path to absolute using project root."""
    if not path or os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(_PROJECT_ROOT, path))


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
        # Resolve relative paths from DB to absolute for file operations
        for key in ("sdf_path", "image_path"):
            if key in self._data and self._data[key]:
                self._data[key] = _resolve_path(self._data[key])

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value

    def __contains__(self, key):
        return key in self._data

    def get(self, key, default=None):
        return self._data.get(key, default)

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
    def topics(self) -> list:
        return self._data.get("topics", [])

    @property
    def topic(self) -> str:
        return self.topics[0] if self.topics else ""

    @property
    def tests(self) -> list:
        return self._data["tests"]

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

    sensor_added = pyqtSignal(dict)
    sensor_updated = pyqtSignal(dict)
    sensor_deleted = pyqtSignal(str)
    sensors_loaded = pyqtSignal()
    test_updated = pyqtSignal(str, dict)

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
        import inspect as _inspect

        _add_sig = _inspect.signature(db.add_sensor).parameters
        _kwargs = dict(
            sensor_name=data["name"],
            sensor_type=data["type"],
            sdf_path=data.get("sdf_path", ""),
            description=data.get("description", ""),
            image_path=data.get("image_path", ""),
            params=data.get("params", {}),
        )
        if "topics" in _add_sig:
            _kwargs["topics"] = data.get("topics", [])
        db.add_sensor(**_kwargs)
        fresh = db.get_sensor_by_name(data["name"])
        s = Sensor(fresh)
        self._sensors[s.id] = s
        self._seed_test_meta(s)
        self._reload_sensor_from_db(s.id)
        self.sensor_added.emit(self._sensors[s.id].to_dict())
        return self._sensors[s.id].to_dict()

    def _seed_test_meta(self, sensor: "Sensor") -> None:
        try:
            from src.sensors import REGISTRY
            from src.test_utils import load_test_functions

            SensorClass = REGISTRY.get(sensor.sensor_type)
            if SensorClass is None:
                return

            instance = SensorClass(sensor.sdf_path)
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

            SensorClass = REGISTRY.get(sensor.sensor_type)
            if SensorClass is None:
                params = {}
            else:
                try:
                    instance = SensorClass(sensor.sdf_path)
                    params = instance.get_params()
                except Exception:
                    params = {}
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
        # Build kwargs — only pass topics if db.update_sensor supports it.
        # Only include a field in the DB call if it's EXPLICITLY present
        # in `fields` (checked with `in`, not `.get()` which would return
        # None for both "absent" and "present but None").  This keeps
        # callers like `core.update_sensor_params({"params": ...})` from
        # implicitly asserting "description=None, image_path=None, …"
        # and — combined with the description-preservation logic in the
        # add-sensor dialog — prevents accidental metadata loss when
        # editing params.
        import inspect as _inspect

        _upd_sig = _inspect.signature(db.update_sensor).parameters
        _kwargs = {"sensor_name": s.name}
        for _k in ("description", "image_path", "params", "sdf_path"):
            if _k in fields:
                _kwargs[_k] = fields[_k]
        if "topics" in _upd_sig and "topics" in fields:
            _kwargs["topics"] = fields["topics"]
        db.update_sensor(**_kwargs)
        s.update_fields(fields)
        # Refresh the in-memory Sensor from DB.  Without this, the
        # `tests` list (including each test's description/display_name
        # from SensorTypeTests, fetched via LEFT JOIN in
        # get_latest_test_results) stays frozen at whatever was in the
        # DB at APP STARTUP.  If anything — the user, a CLI script,
        # another window — wrote to SensorTypeTests after launch, those
        # changes would never reach col_3 until the next restart.  The
        # visible symptom is exactly "I edited sensor params and now
        # the test descriptions are gone / stale" because load_sensor
        # reads tests straight out of in-memory _data["tests"].
        self._reload_sensor_from_db(sensor_id)
        s = self._sensors.get(sensor_id, s)
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
            sensor_name=s.name,
            test_name=test_name,
            status=status,
            result=result,
            description=description,
            duration=duration,
        )

        from datetime import datetime as _dt

        today = _dt.now().strftime("%Y-%m-%d")

        result_str = str(result) if not isinstance(result, str) else result
        s.update_test(
            test_name,
            {
                "status": status,
                "result": result_str,
                "duration": duration,
                "date": today,
            },
        )

        updated_test = s.get_test(test_name)
        self.test_updated.emit(sensor_id, copy.deepcopy(updated_test))
        return copy.deepcopy(updated_test)

    def get_test_meta(self, sensor_id: str) -> list[dict]:
        rows = db.get_test_meta(sensor_id)
        return {r["func_name"]: r for r in rows}

    def save_test_meta(
        self,
        sensor_id: str,
        func_name: str,
        display_name: str,
        description: str,
        image_path: str,
    ) -> None:
        db.save_test_meta(sensor_id, func_name, display_name, description, image_path)

        s = self._sensors.get(sensor_id)
        if s is None:
            return

        db.upsert_type_test(
            s.sensor_type, func_name, display_name, description, image_path
        )
        db.propagate_type_test_to_sensors(
            s.sensor_type, func_name, display_name, description, image_path
        )

        s.update_test(
            func_name,
            {
                "display_name": display_name,
                "description": description,
                "image_path": image_path,
            },
        )

        for sibling in self._sensors.values():
            if sibling.id != sensor_id and sibling.sensor_type == s.sensor_type:
                sibling.update_test(
                    func_name,
                    {
                        "display_name": display_name,
                        "description": description,
                        "image_path": image_path,
                    },
                )
