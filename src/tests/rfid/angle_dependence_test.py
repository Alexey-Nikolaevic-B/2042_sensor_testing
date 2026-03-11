import math
import time
from typing import Dict, Any

from ..sensor_test import SensorTest, register_test


@register_test
class AngleDependenceTest(SensorTest):
    name             = "angle_dependence_test"
    display_name     = "Angle Dependence"
    description      = (
        "Places 5 RFID tags at angles 0°, 30°, 45°, 60°, 90° from the antenna axis "
        "and captures detections over 20 s. "
        "Passes when all 5 tags are detected regardless of angle."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        angles = [0, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
        radius = sensor.read_distance / 2

        with open(sensor.rfid_map_path, "w") as f:
            for tag_num, angle in enumerate(angles):
                x = radius * math.sin(angle)
                z = radius * math.cos(angle)
                f.write(f"fix{tag_num + 1} {tag_num + 1} {x} 0 {z}\n")

        world = sensor.test_to_world["angle_dependence_test"]
        if not simulator.open_scene(world, sensor.sensor_sdf_path):
            raise RuntimeError("Failed to open Gazebo scene")

        for i in range(len(angles)):
            if not simulator.wait_for_model_spawn(f"rfid_tag{i + 1}", 30):
                raise RuntimeError(f"rfid_tag{i + 1} did not spawn within 30 s")
            if progress_cb:
                progress_cb(int((i + 1) / len(angles) * 50))

        start_time  = time.time()
        tag_id2pose = sensor.capture_data(simulator=simulator, world_path=None, window=20)
        if progress_cb:
            progress_cb(100)

        duration  = time.time() - start_time
        is_passed = len(tag_id2pose) == len(angles)
        return {
            "passed":              is_passed,
            "duration":            duration,
            "tags_detected_count": len(tag_id2pose),
        }
