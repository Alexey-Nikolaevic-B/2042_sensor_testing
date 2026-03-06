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

    return {
        "name":        row["test_name"],
        "status":      row["status"],
        "result":      result,
        "description": row["description"] or "",
        "date":        row["date"],
        "duration":    row["duration"],
    }



def init_db() -> None:
    """Create tables if they don't exist. Safe to call multiple times."""
    with _connect() as conn:
        conn.executescript(_SCHEMA)



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
    """Return the most recent result for each test name for a given sensor."""
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            """
            SELECT t1.*
            FROM TestResults t1
            INNER JOIN (
                SELECT test_name, MAX(id) AS max_id
                FROM TestResults
                WHERE sensor_id = ?
                GROUP BY test_name
            ) t2 ON t1.test_name = t2.test_name AND t1.id = t2.max_id
            ORDER BY t1.test_name
            """,
            (sensor_id,),
        ).fetchall()
    return [_row_to_test_dict(r) for r in rows]


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


def get_sensors() -> list[tuple]:
    """Original API: returns (id, sensor_name, sensor_type, sdf_path) tuples."""
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, sensor_name, sensor_type, sdf_path FROM Sensors ORDER BY sensor_name"
        ).fetchall()
    return [(r["id"], r["sensor_name"], r["sensor_type"], r["sdf_path"]) for r in rows]