"""
sensor_storage.py

SQLite persistence layer for sensors and their test results.

Tables
------
Sensors     — one row per registered sensor
TestResults — one row per test run, FK → Sensors.id
"""

import sqlite3
import json
from datetime import datetime
from typing import Optional

DATABASE = "sensor_storage.db"


# ── Schema ────────────────────────────────────────────────────────────────────

_SCHEMA = """
CREATE TABLE IF NOT EXISTS Sensors (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_name TEXT    NOT NULL UNIQUE,
    sensor_type TEXT    NOT NULL,
    sdf_path    TEXT    NOT NULL,
    description TEXT    DEFAULT '',
    image_path  TEXT    DEFAULT '',
    params      TEXT    DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS TestResults (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_id   INTEGER NOT NULL REFERENCES Sensors(id) ON DELETE CASCADE,
    test_name   TEXT    NOT NULL,
    status      TEXT    NOT NULL DEFAULT 'Pending',
    result      TEXT    DEFAULT '',
    description TEXT    DEFAULT '',
    date        TEXT    NOT NULL,
    duration    REAL    DEFAULT 0.0
);
"""


# ── Internal helpers (defined first so all functions below can use them) ──────

def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DATABASE)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def _row_to_sensor_dict(row: sqlite3.Row) -> dict:
    params = {}
    try:
        params = json.loads(row["params"] or "{}")
    except (json.JSONDecodeError, TypeError):
        pass

    return {
        "id":          str(row["id"]),
        "name":        row["sensor_name"],
        "type":        row["sensor_type"],
        "sdf_path":    row["sdf_path"],
        "description": row["description"] or "",
        "image_path":  row["image_path"]  or "",
        "params":      params,
        "tests":       [],
    }


def _row_to_test_dict(row: sqlite3.Row) -> dict:
    result = row["result"] or ""
    try:
        result = json.loads(result)
    except (json.JSONDecodeError, TypeError):
        pass

    keys = row.keys()
    return {
        "name":         row["test_name"],
        "display_name": (row["display_name"] if "display_name" in keys else None) or row["test_name"],
        "status":       row["status"],
        "result":       result,
        "description":  (row["meta_description"] if "meta_description" in keys else None) or "",
        "image_path":   (row["image_path"] if "image_path" in keys else "") or "",
        "date":         row["date"],
        "duration":     row["duration"],
    }


def _meta_stub_dict(func_name: str, display_name: str, description: str, image_path: str) -> dict:
    return {
        "name":         func_name,
        "display_name": display_name or func_name,
        "status":       "Pending",
        "result":       "",
        "description":  description or "",
        "image_path":   image_path or "",
        "date":         "",
        "duration":     0.0,
    }


# ── Init ──────────────────────────────────────────────────────────────────────

def init_db() -> None:
    """Create tables if they don't exist. Safe to call multiple times."""
    with _connect() as conn:
        conn.executescript(_SCHEMA)


# ── Sensors ───────────────────────────────────────────────────────────────────

def add_sensor(
    sensor_name: str,
    sensor_type: str,
    sdf_path: str,
    description: str = "",
    image_path: str = "",
    params: dict = None,
) -> int:
    """Insert a new sensor. Returns the new row id.
    Raises ValueError if a sensor with that name already exists."""
    init_db()
    params_json = json.dumps(params or {})
    with _connect() as conn:
        existing = conn.execute(
            "SELECT id FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if existing:
            raise ValueError(f"Sensor '{sensor_name}' already exists.")
        cursor = conn.execute(
            """
            INSERT INTO Sensors (sensor_name, sensor_type, sdf_path, description, image_path, params)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (sensor_name, sensor_type, sdf_path, description, image_path, params_json),
        )
        return cursor.lastrowid


def update_sensor(
    sensor_name: str,
    description: str = None,
    image_path: str = None,
    params: dict = None,
    sdf_path: str = None,
) -> None:
    """Update mutable fields of an existing sensor."""
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            raise KeyError(f"Sensor '{sensor_name}' not found.")
        conn.execute(
            """
            UPDATE Sensors
            SET description = ?, image_path = ?, sdf_path = ?, params = ?
            WHERE sensor_name = ?
            """,
            (
                description if description is not None else row["description"],
                image_path  if image_path  is not None else row["image_path"],
                sdf_path    if sdf_path    is not None else row["sdf_path"],
                json.dumps(params) if params is not None else row["params"],
                sensor_name,
            ),
        )


def delete_sensor(sensor_name: str) -> None:
    """Delete a sensor and all its test results (CASCADE)."""
    init_db()
    with _connect() as conn:
        conn.execute("DELETE FROM Sensors WHERE sensor_name = ?", (sensor_name,))


def get_all_sensors() -> list[dict]:
    """Return all sensors as dicts with their latest test results."""
    init_db()
    with _connect() as conn:
        rows = conn.execute("SELECT * FROM Sensors ORDER BY sensor_name").fetchall()
    sensors = []
    for row in rows:
        sensor = _row_to_sensor_dict(row)
        sensor["tests"] = get_latest_test_results(sensor["id"])
        sensors.append(sensor)
    return sensors


def get_sensor_by_name(sensor_name: str) -> Optional[dict]:
    """Return a single sensor dict (with tests), or None if not found."""
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            return None
        sensor = _row_to_sensor_dict(row)
        sensor["tests"] = get_latest_test_results(sensor["id"])
    return sensor


def get_sensor_types() -> list[str]:
    """Return sorted list of unique sensor types present in the DB."""
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT DISTINCT sensor_type FROM Sensors ORDER BY sensor_type"
        ).fetchall()
    return [r["sensor_type"] for r in rows]


# ── Test results ──────────────────────────────────────────────────────────────

def save_test_result(
    sensor_name: str,
    test_name: str,
    status: str,
    result,
    description: str = "",
    duration: float = 0.0,
) -> int:
    """Insert a test result row. result can be dict (JSON) or plain string."""
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT id FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            raise KeyError(f"Sensor '{sensor_name}' not found.")
        result_str = json.dumps(result) if isinstance(result, dict) else str(result)
        cursor = conn.execute(
            """
            INSERT INTO TestResults (sensor_id, test_name, status, result, description, date, duration)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (
                row["id"],
                test_name,
                status,
                result_str,
                description,
                datetime.now().strftime("%Y-%m-%d"),
                duration,
            ),
        )
        return cursor.lastrowid


def get_latest_test_results(sensor_id) -> list[dict]:
    init_db()
    with _connect() as conn:
        # Latest result row per test, joined with SensorTests meta
        result_rows = conn.execute(
            """
            SELECT t1.*,
                   sm.display_name  AS display_name,
                   sm.description   AS meta_description,
                   sm.image_path    AS image_path
            FROM TestResults t1
            INNER JOIN (
                SELECT test_name, MAX(id) AS max_id
                FROM TestResults
                WHERE sensor_id = ?
                GROUP BY test_name
            ) t2 ON t1.test_name = t2.test_name AND t1.id = t2.max_id
            LEFT JOIN SensorTests sm
                   ON sm.sensor_id = t1.sensor_id
                  AND sm.func_name = t1.test_name
            ORDER BY t1.test_name
            """,
            (sensor_id,),
        ).fetchall()

        ran_names = {r["test_name"] for r in result_rows}

        # Tests that have meta but have never been run — show as Pending
        meta_only = conn.execute(
            """
            SELECT func_name, display_name, description, image_path
            FROM SensorTests
            WHERE sensor_id = ?
            ORDER BY func_name
            """,
            (int(sensor_id),),
        ).fetchall()

    tests = [_row_to_test_dict(r) for r in result_rows]
    for m in meta_only:
        if m["func_name"] not in ran_names:
            tests.append(_meta_stub_dict(
                m["func_name"], m["display_name"],
                m["description"], m["image_path"],
            ))
    return tests


def get_test_history(sensor_name: str, test_name: str) -> list[dict]:
    """Return all historical results for a specific test on a sensor."""
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT id FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            return []
        rows = conn.execute(
            """
            SELECT * FROM TestResults
            WHERE sensor_id = ? AND test_name = ?
            ORDER BY id DESC
            """,
            (row["id"], test_name),
        ).fetchall()
    return [_row_to_test_dict(r) for r in rows]


# ── Backwards-compatible alias ────────────────────────────────────────────────

def get_sensors() -> list[tuple]:
    """Original API: returns (id, sensor_name, sensor_type, sdf_path) tuples."""
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, sensor_name, sensor_type, sdf_path FROM Sensors ORDER BY sensor_name"
        ).fetchall()
    return [(r["id"], r["sensor_name"], r["sensor_type"], r["sdf_path"]) for r in rows]


# ── Test metadata ─────────────────────────────────────────────────────────────
# Stores display name, description and image for each test function.
# func_name  = the Python function name from core.get_tests()
# display_name = what's shown in the UI (defaults to func_name)

_SCHEMA_SENSOR_TYPE_TESTS = """
CREATE TABLE IF NOT EXISTS SensorTests (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_type  TEXT    NOT NULL,
    func_name    TEXT    NOT NULL,
    display_name TEXT    NOT NULL DEFAULT '',
    description  TEXT    NOT NULL DEFAULT '',
    image_path   TEXT    NOT NULL DEFAULT '',
    UNIQUE(sensor_type, func_name)
);
"""

_SCHEMA_TEST_META = """
CREATE TABLE IF NOT EXISTS SensorTests (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_id    INTEGER NOT NULL REFERENCES Sensors(id) ON DELETE CASCADE,
    func_name    TEXT    NOT NULL,
    display_name TEXT    NOT NULL DEFAULT '',
    description  TEXT    NOT NULL DEFAULT '',
    image_path   TEXT    NOT NULL DEFAULT '',
    UNIQUE(sensor_id, func_name)
);
"""


def init_test_meta_table() -> None:
    """Extend schema with SensorTests table. Safe to call multiple times."""
    with _connect() as conn:
        conn.executescript(_SCHEMA_TEST_META)


def get_test_meta(sensor_id: str) -> list[dict]:
    """Return all test metadata rows for a sensor."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM SensorTests WHERE sensor_id = ?", (int(sensor_id),)
        ).fetchall()
    return [
        {
            "func_name":    r["func_name"],
            "display_name": r["display_name"] or r["func_name"],
            "description":  r["description"] or "",
            "image_path":   r["image_path"]  or "",
        }
        for r in rows
    ]


def save_test_meta(sensor_id: str, func_name: str, display_name: str,
                   description: str, image_path: str) -> None:
    """Upsert a single test's metadata."""
    with _connect() as conn:
        conn.execute(
            """
            INSERT INTO SensorTests (sensor_id, func_name, display_name, description, image_path)
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(sensor_id, func_name) DO UPDATE SET
                display_name = excluded.display_name,
                description  = excluded.description,
                image_path   = excluded.image_path
            """,
            (int(sensor_id), func_name, display_name, description, image_path),
        )

def init_sensor_type_tests_table() -> None:
    """Create SensorTests table. Safe to call multiple times."""
    with _connect() as conn:
        conn.executescript(_SCHEMA_SENSOR_TYPE_TESTS)


def get_type_tests(sensor_type: str) -> list[dict]:
    """Return canonical test definitions for a sensor type."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM SensorTests WHERE sensor_type = ? ORDER BY func_name",
            (sensor_type,),
        ).fetchall()
    return [
        {
            "func_name":    r["func_name"],
            "display_name": r["display_name"] or r["func_name"],
            "description":  r["description"] or "",
            "image_path":   r["image_path"] or "",
        }
        for r in rows
    ]


def upsert_type_test(sensor_type: str, func_name: str, display_name: str,
                     description: str, image_path: str) -> None:
    """Upsert a canonical test definition for a sensor type."""
    with _connect() as conn:
        conn.execute(
            """
            INSERT INTO SensorTests (sensor_type, func_name, display_name, description, image_path)
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(sensor_type, func_name) DO UPDATE SET
                display_name = excluded.display_name,
                description  = excluded.description,
                image_path   = excluded.image_path
            """,
            (sensor_type, func_name, display_name, description, image_path),
        )


def sync_type_tests_to_sensor(sensor_id: str, sensor_type: str) -> None:
    """Copy SensorTests entries for sensor_type into SensorTests for sensor_id.
    Only inserts rows that don't already exist — never overwrites existing meta."""
    type_tests = get_type_tests(sensor_type)
    with _connect() as conn:
        for t in type_tests:
            conn.execute(
                """
                INSERT OR IGNORE INTO SensorTests (sensor_id, func_name, display_name, description, image_path)
                VALUES (?, ?, ?, ?, ?)
                """,
                (int(sensor_id), t["func_name"], t["display_name"], t["description"], t["image_path"]),
            )


def get_sensors_by_type(sensor_type: str) -> list[dict]:
    """Return all sensor rows for a given sensor_type."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_type = ?", (sensor_type,)
        ).fetchall()
    return [_row_to_sensor_dict(r) for r in rows]


def propagate_type_test_to_sensors(sensor_type: str, func_name: str,
                                   display_name: str, description: str,
                                   image_path: str) -> None:
    """After editing a type-level test, push display_name and description to all
    sensors of that type. image_path is NOT propagated — each sensor keeps its own."""
    sensors = get_sensors_by_type(sensor_type)
    with _connect() as conn:
        for s in sensors:
            conn.execute(
                """
                UPDATE SensorTests
                SET display_name = ?, description = ?
                WHERE sensor_id = ? AND func_name = ?
                """,
                (display_name, description, int(s["id"]), func_name),
            )