sensor_library = {
    "camera": [
        {
            "name": "depth_camera",
            "tests": {
                "depth_perception_test"
            }
        },
        {
            "name": "mono_camera",
            "tests": {
                "depth_perception_test"
            }
        }
    ],
    "rfid": [
        {
            "name": "rfid_antenna",
            "tests": {
                "change_distance_test",
                "min_stable_read_distance_test"
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
