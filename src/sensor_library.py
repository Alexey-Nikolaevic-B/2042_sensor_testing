sensor_library = {
    "camera": [
        {
            "name": "depth_camera",
            "tests": {
                "depth_perception_test": (3, 4),
                "Определение разрешающей способности камеры": ("5 см", "6 см")
            }
        }
    ],
    "tactile": [ #Сюда пока не смотрите
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
    "rfid": [ #Сюда пока не смотрите
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
