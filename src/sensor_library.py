sensor_library = {
    "camera": [
        {
            "name": "depth_camera",
            "tests": {
                "depth_perception_test": (3, 4),
                "Определение разрешающей способности камеры": ("5 см", "6 см")
            }
        },
        {
            "name": "mono_camera",
            "tests": {
                "depth_perception_test": (3, 4),
                "Определение разрешающей способности камеры": ("5 см", "6 см")
            }
        }
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
