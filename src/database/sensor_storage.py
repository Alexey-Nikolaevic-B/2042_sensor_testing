import sqlite3


DATABASE = 'sensor_storage.db'

def add_sensor(sensor_name: str, sensor_type: str, sdf_path: str) -> None:
    """
    Добавляет датчик в базу данных
    Если она не было создана - создает ее
    """

    with sqlite3.connect(DATABASE) as connection:
        cursor = connection.cursor()

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS Sensors (
            id INTEGER PRIMARY KEY,
            sensor_name TEXT NOT NULL UNIQUE,
            sensor_type TEXT NOT NULL,
            sdf_path TEXT NOT NULL
            )
            """
        )

        cursor.execute(
            "SELECT 1 FROM Sensors WHERE sensor_name = ?",
            (sensor_name,),
        )
        if cursor.fetchone() is not None:
            raise ValueError(f"Датчик с именем '{sensor_name}' уже существует")

        cursor.execute(
            "INSERT INTO Sensors (sensor_name, sensor_type, sdf_path) VALUES (?, ?, ?)",
            (sensor_name, sensor_type, sdf_path),
        )


def delete_sensor(sensor_name: str) -> None:
    """
    Удаляет датчик из базы данных
    """
    with sqlite3.connect(DATABASE) as connection:
        cursor = connection.cursor()

        cursor.execute(
            "DELETE FROM Sensors WHERE sensor_name = ?",
            (sensor_name,),
        )


def get_sensors() -> list[tuple[int, str, str, str]]:
    """
    Возвращает все датчики из базы данных
    """
    with sqlite3.connect(DATABASE) as connection:
        cursor = connection.cursor()
        cursor.execute('SELECT * FROM Sensors')
        results = cursor.fetchall()

    return results
