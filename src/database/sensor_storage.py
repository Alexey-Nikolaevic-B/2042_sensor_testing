import sqlite3
import json
from datetime import datetime
from typing import Optional

DATABASE = "sensor_storage.db"

_SCHEMA = """
CREATE TABLE IF NOT EXISTS SensorTypes (
    sensor_type  TEXT PRIMARY KEY,
    description  TEXT NOT NULL DEFAULT '',
    params       TEXT NOT NULL DEFAULT '[]',
    detection    TEXT NOT NULL DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS SensorTypeTests (
    sensor_type  TEXT NOT NULL REFERENCES SensorTypes(sensor_type) ON DELETE CASCADE,
    func_name    TEXT NOT NULL,
    display_name TEXT NOT NULL DEFAULT '',
    description  TEXT NOT NULL DEFAULT '',
    world_path   TEXT NOT NULL DEFAULT '',
    PRIMARY KEY (sensor_type, func_name)
);

CREATE TABLE IF NOT EXISTS Sensors (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_name  TEXT NOT NULL UNIQUE,
    sensor_type  TEXT NOT NULL REFERENCES SensorTypes(sensor_type),
    sdf_path     TEXT NOT NULL DEFAULT '',
    topic        TEXT NOT NULL DEFAULT '',
    description  TEXT NOT NULL DEFAULT '',
    image_path   TEXT NOT NULL DEFAULT '',
    params       TEXT NOT NULL DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS TestResults (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    sensor_id    INTEGER NOT NULL REFERENCES Sensors(id) ON DELETE CASCADE,
    func_name    TEXT NOT NULL,
    status       TEXT NOT NULL DEFAULT 'Pending',
    result       TEXT NOT NULL DEFAULT '',
    description  TEXT NOT NULL DEFAULT '',
    date         TEXT NOT NULL,
    duration     REAL NOT NULL DEFAULT 0.0
);
"""


def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DATABASE)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def init_db() -> None:
    with _connect() as conn:
        conn.executescript(_SCHEMA)
        _migrate(conn)


def _migrate(conn: sqlite3.Connection) -> None:
    tables = {r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()}

    if "SensorTypeTests" in tables:
        cols = {r[1] for r in conn.execute("PRAGMA table_info(SensorTypeTests)").fetchall()}
        if "world_path" not in cols:
            conn.execute("ALTER TABLE SensorTypeTests ADD COLUMN world_path TEXT NOT NULL DEFAULT ''")

    if "Sensors" in tables:
        cols = {r[1] for r in conn.execute("PRAGMA table_info(Sensors)").fetchall()}
        if "topic" not in cols:
            conn.execute("ALTER TABLE Sensors ADD COLUMN topic TEXT NOT NULL DEFAULT ''")

    if "TestResults" in tables:
        cols = {r[1] for r in conn.execute("PRAGMA table_info(TestResults)").fetchall()}
        if "test_name" in cols and "func_name" not in cols:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS TestResults_new (
                    id          INTEGER PRIMARY KEY AUTOINCREMENT,
                    sensor_id   INTEGER NOT NULL REFERENCES Sensors(id) ON DELETE CASCADE,
                    func_name   TEXT NOT NULL,
                    status      TEXT NOT NULL DEFAULT 'Pending',
                    result      TEXT NOT NULL DEFAULT '',
                    description TEXT NOT NULL DEFAULT '',
                    date        TEXT NOT NULL,
                    duration    REAL NOT NULL DEFAULT 0.0
                );
                INSERT INTO TestResults_new (id, sensor_id, func_name, status, result, description, date, duration)
                    SELECT id, sensor_id, test_name, status, result, description, date, duration
                    FROM TestResults;
                DROP TABLE TestResults;
                ALTER TABLE TestResults_new RENAME TO TestResults;
            """)

    if "SensorTypeDefs" in tables:
        cols = {r[1] for r in conn.execute("PRAGMA table_info(SensorTypeDefs)").fetchall()}
        if cols:
            existing = conn.execute("SELECT * FROM SensorTypeDefs").fetchall()
            for row in existing:
                conn.execute(
                    """
                    INSERT OR IGNORE INTO SensorTypes (sensor_type, description, params, detection)
                    VALUES (?, ?, ?, ?)
                    """,
                    (row["sensor_type"], row["description"], row["params"], row["detection"]),
                )



def _j(value: str, fallback):
    try:
        return json.loads(value or json.dumps(fallback))
    except (json.JSONDecodeError, TypeError):
        return fallback


# ── SensorTypes ───────────────────────────────────────────────────────────────

def upsert_sensor_type(sensor_type: str, description: str = "",
                       params: list = None, detection: dict = None) -> None:
    init_db()
    with _connect() as conn:
        conn.execute(
            """
            INSERT INTO SensorTypes (sensor_type, description, params, detection)
            VALUES (?, ?, ?, ?)
            ON CONFLICT(sensor_type) DO UPDATE SET
                description = excluded.description,
                params      = excluded.params,
                detection   = excluded.detection
            """,
            (sensor_type, description,
             json.dumps(params or []),
             json.dumps(detection or {})),
        )


def get_sensor_type(sensor_type: str) -> Optional[dict]:
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM SensorTypes WHERE sensor_type = ?", (sensor_type,)
        ).fetchone()
    if not row:
        return None
    return {
        "sensor_type": row["sensor_type"],
        "description": row["description"] or "",
        "params":      _j(row["params"], []),
        "detection":   _j(row["detection"], {}),
    }


def get_all_sensor_types() -> list[dict]:
    init_db()
    with _connect() as conn:
        rows = conn.execute("SELECT * FROM SensorTypes ORDER BY sensor_type").fetchall()
    return [
        {
            "sensor_type": r["sensor_type"],
            "description": r["description"] or "",
            "params":      _j(r["params"], []),
            "detection":   _j(r["detection"], {}),
        }
        for r in rows
    ]


def get_sensor_type_names() -> list[str]:
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT sensor_type FROM SensorTypes ORDER BY sensor_type"
        ).fetchall()
    return [r["sensor_type"] for r in rows]


def delete_sensor_type(sensor_type: str) -> None:
    init_db()
    with _connect() as conn:
        conn.execute("DELETE FROM SensorTypes WHERE sensor_type = ?", (sensor_type,))


# ── SensorTypeTests ───────────────────────────────────────────────────────────

def upsert_type_test(sensor_type: str, func_name: str, display_name: str = "",
                     description: str = "", world_path: str = "") -> None:
    init_db()
    with _connect() as conn:
        conn.execute(
            """
            INSERT INTO SensorTypeTests (sensor_type, func_name, display_name, description, world_path)
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(sensor_type, func_name) DO UPDATE SET
                display_name = excluded.display_name,
                description  = excluded.description,
                world_path   = excluded.world_path
            """,
            (sensor_type, func_name, display_name, description, world_path),
        )


def get_type_tests(sensor_type: str) -> list[dict]:
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM SensorTypeTests WHERE sensor_type = ? ORDER BY func_name",
            (sensor_type,),
        ).fetchall()
    return [
        {
            "func_name":    r["func_name"],
            "display_name": r["display_name"] or r["func_name"],
            "description":  r["description"] or "",
            "world_path":   r["world_path"] or "",
        }
        for r in rows
    ]


def delete_type_test(sensor_type: str, func_name: str) -> None:
    init_db()
    with _connect() as conn:
        conn.execute(
            "DELETE FROM SensorTypeTests WHERE sensor_type = ? AND func_name = ?",
            (sensor_type, func_name),
        )


# ── Sensors ───────────────────────────────────────────────────────────────────

def _sensor_row(row: sqlite3.Row) -> dict:
    return {
        "id":          str(row["id"]),
        "name":        row["sensor_name"],
        "type":        row["sensor_type"],
        "sdf_path":    row["sdf_path"] or "",
        "topic":       row["topic"] or "",
        "description": row["description"] or "",
        "image_path":  row["image_path"] or "",
        "params":      _j(row["params"], {}),
        "tests":       [],
    }


def _attach_tests(conn: sqlite3.Connection, sensor: dict) -> dict:
    sid = int(sensor["id"])
    stype = sensor["type"]

    ran_rows = conn.execute(
        """
        SELECT t1.* FROM TestResults t1
        INNER JOIN (
            SELECT func_name, MAX(id) AS max_id
            FROM TestResults WHERE sensor_id = ? GROUP BY func_name
        ) t2 ON t1.func_name = t2.func_name AND t1.id = t2.max_id
        ORDER BY t1.func_name
        """,
        (sid,),
    ).fetchall()

    ran = {r["func_name"] for r in ran_rows}
    tests = [
        {
            "name":        r["func_name"],
            "status":      r["status"],
            "result":      _j(r["result"], r["result"]),
            "description": r["description"] or "",
            "date":        r["date"],
            "duration":    r["duration"],
        }
        for r in ran_rows
    ]

    for t in conn.execute(
        "SELECT * FROM SensorTypeTests WHERE sensor_type = ? ORDER BY func_name",
        (stype,),
    ).fetchall():
        if t["func_name"] not in ran:
            tests.append({
                "name":         t["func_name"],
                "display_name": t["display_name"] or t["func_name"],
                "status":       "Pending",
                "result":       "",
                "description":  t["description"] or "",
                "date":         "",
                "duration":     0.0,
            })

    sensor["tests"] = tests
    return sensor


def add_sensor(sensor_name: str, sensor_type: str, sdf_path: str = "",
               topic: str = "", description: str = "",
               image_path: str = "", params: dict = None) -> int:
    init_db()
    with _connect() as conn:
        if conn.execute(
            "SELECT id FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone():
            raise ValueError(f"Sensor '{sensor_name}' already exists.")
        cursor = conn.execute(
            """
            INSERT INTO Sensors (sensor_name, sensor_type, sdf_path, topic, description, image_path, params)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (sensor_name, sensor_type, sdf_path, topic,
             description, image_path, json.dumps(params or {})),
        )
        return cursor.lastrowid


def update_sensor(sensor_name: str, description: str = None, image_path: str = None,
                  params: dict = None, sdf_path: str = None, topic: str = None) -> None:
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            raise KeyError(f"Sensor '{sensor_name}' not found.")
        conn.execute(
            """
            UPDATE Sensors SET description=?, image_path=?, sdf_path=?, topic=?, params=?
            WHERE sensor_name=?
            """,
            (
                description if description is not None else row["description"],
                image_path  if image_path  is not None else row["image_path"],
                sdf_path    if sdf_path    is not None else row["sdf_path"],
                topic       if topic       is not None else row["topic"],
                json.dumps(params) if params is not None else row["params"],
                sensor_name,
            ),
        )


def delete_sensor(sensor_name: str) -> None:
    init_db()
    with _connect() as conn:
        conn.execute("DELETE FROM Sensors WHERE sensor_name = ?", (sensor_name,))


def get_sensor_by_name(sensor_name: str) -> Optional[dict]:
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            return None
        return _attach_tests(conn, _sensor_row(row))


def get_all_sensors() -> list[dict]:
    init_db()
    with _connect() as conn:
        rows = conn.execute("SELECT * FROM Sensors ORDER BY sensor_name").fetchall()
        return [_attach_tests(conn, _sensor_row(r)) for r in rows]


def get_sensors_by_type(sensor_type: str) -> list[dict]:
    init_db()
    with _connect() as conn:
        rows = conn.execute(
            "SELECT * FROM Sensors WHERE sensor_type = ?", (sensor_type,)
        ).fetchall()
        return [_attach_tests(conn, _sensor_row(r)) for r in rows]


# ── TestResults ───────────────────────────────────────────────────────────────

def save_test_result(sensor_name: str, func_name: str, status: str,
                     result, description: str = "", duration: float = 0.0) -> int:
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
            INSERT INTO TestResults (sensor_id, func_name, status, result, description, date, duration)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (row["id"], func_name, status, result_str, description,
             datetime.now().strftime("%Y-%m-%d %H:%M:%S"), duration),
        )
        return cursor.lastrowid


def get_test_history(sensor_name: str, func_name: str) -> list[dict]:
    init_db()
    with _connect() as conn:
        row = conn.execute(
            "SELECT id FROM Sensors WHERE sensor_name = ?", (sensor_name,)
        ).fetchone()
        if not row:
            return []
        rows = conn.execute(
            "SELECT * FROM TestResults WHERE sensor_id=? AND func_name=? ORDER BY id DESC",
            (row["id"], func_name),
        ).fetchall()
    return [
        {
            "name":        r["func_name"],
            "status":      r["status"],
            "result":      _j(r["result"], r["result"]),
            "description": r["description"] or "",
            "date":        r["date"],
            "duration":    r["duration"],
        }
        for r in rows
    ]