import random

sensor_library = {
    "camera": [
        {
            "name": "model",
            "test_world": "2",
            "topic":"/mono_camera/image_raw", # TODO: возможно нужно убарть
            "tests": {
                "Тест на угол обзора": (75.4, 75.4),
                "Тест на разрешение": ("640×480",    "640×480"),
                "Оценка Фокуса и Глубины Резкости (DoF, м)": (4, 3.9)
            }
        },
        {
            "name": "depth_camera",
            "test_world": "cubes",
            "topic":"/depth_camera/image_raw",
            "tests": {
                "Тест на угол обзора": (75, 73),
                "Тест на точность глубины": (3, 4),
                "Определение разрешающей способности камеры": ("5 см", "6 см")
            }
        }
    ],
    "tactile": [
        {
            "name": "Wacoh-Tech DynPick",
            "ros_launch": "test_scripts/launch_wacoh_dynpick.sh",
            "test_scene": "touch_scene",
            "tests": {
                "Тест на оценку пиковой силы": (1000, 990),
                "Тест на точность момента": ('0.09', '0.09'),
                "Тест на дрейф и шум": ('Стабильно', 'Стабильно'),
                "Тест на диапазон измерений": ('200-1000', '230-990')
            }
        },
    ],
    "rfid": [
        {
            "name": "UHF RFID Reader",
            "ros_pkg": "scenario_test_pkg",
            "launch_file": "scenario_rfid_angle_test.launch",
            "test_scene": "rfid",
            "tests": {
                "Определение максимальной дальности считывания": (7.5, 7.2),
                "Тестирование угла считывания": (45, 43),
                "Массовое считывание RFID-меток": ("5 с/7.3%", "5.2 с/96.0%"),
                "Определение минимального расстояния надежного считывания(м)": (0.15, 0.18)
            }
        },
    ]
}

def get_sensor_types():
    return list(sensor_library.keys())

def get_sensors_by_type(sensor_type: str):
    return sensor_library.get(sensor_type, [])

def get_sensor(sensor_type: str, sensor_name: str):
    sensors = sensor_library.get(sensor_type, [])
    for sensor in sensors:
        if sensor["name"] == sensor_name:
            return sensor
    return None

def add_sensor(sensor_type: str, sensor_data: dict):
    if sensor_type not in sensor_library:
        sensor_library[sensor_type] = []
    sensor_library[sensor_type].append(sensor_data)

def get_tests_by_sensor(sensor_type: str, sensor_name: str):
    sensor = get_sensor(sensor_type, sensor_name)
    if sensor:
        return sensor.get("tests", [])
    return []

def get_etalon_by_name(sensor_type: str, sensor_name: str, test_name: str):
    """
    Получает значение эталона для указанного теста по имени сенсора.
    """
    sensor = get_sensor(sensor_type, sensor_name)
    if sensor and "tests" in sensor:
        tests = sensor["tests"]
        if isinstance(tests, dict):  # Когда тесты представлены как словарь
            test_data = tests.get(test_name)
            if test_data and isinstance(test_data, tuple):
                return test_data[0]  # Возвращаем эталонное значение
        elif isinstance(tests, list):  # Когда тесты представлены как список
            for test in tests:
                if test_name in test:
                    # Проверяем, является ли значение теста кортежем
                    if isinstance(test[1], tuple):
                        return test[1][0]  # Первое значение — эталон
        return None  # Если тест не найден
    return None

def get_result_by_name(sensor_type: str, sensor_name: str, test_name: str):
    """
    Получает значение результата для указанного теста по имени сенсора.
    """
    sensor = get_sensor(sensor_type, sensor_name)
    if sensor and "tests" in sensor:
        tests = sensor["tests"]
        if isinstance(tests, dict):  # Когда тесты представлены как словарь
            test_data = tests.get(test_name)
            if test_data and isinstance(test_data, tuple):
                return test_data[1]  # Возвращаем значение результата
        elif isinstance(tests, list):  # Когда тесты представлены как список
            for test in tests:
                if test_name in test:
                    # Проверяем, является ли значение теста кортежем
                    if isinstance(test[1], tuple):
                        return test[1][1]  # Второе значение — результат
        return None  # Если тест не найден
    return None
