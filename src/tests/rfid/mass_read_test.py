import math
import time
from typing import Dict, Any

from ..sensor_test import SensorTest, register_test


@register_test
class MassReadTest(SensorTest):
    name             = "mass_read_test"
    display_name     = "Mass Read"
    description      = (
        "Places 75 RFID tags evenly around a circle of radius read_distance/2 "
        "and captures detections over a 20 s window. "
        "Passes when at least 75 % of all tags are detected."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        tags_count = 75
        radius     = sensor.read_distance / 2

        with open(sensor.rfid_map_path, "w") as f:
            for i in range(tags_count):
                x = radius * math.cos(2 * math.pi / tags_count * i)
                y = radius * math.sin(2 * math.pi / tags_count * i)
                f.write(f"fix{i + 1} {i + 1} {x} {y} 0\n")

        world = sensor.test_to_world["mass_read_test"]
        if not simulator.open_scene(world, sensor.sensor_sdf_path):
            raise RuntimeError("Failed to open Gazebo scene")

        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f"rfid_tag{i + 1}", 30):
                raise RuntimeError(f"rfid_tag{i + 1} did not spawn within 30 s")
            if progress_cb:
                progress_cb(int((i + 1) / tags_count * 50))

        start_time   = time.time()
        tag_id2pose  = sensor.capture_data(simulator=simulator, world_path=None, window=20)
        if progress_cb:
            progress_cb(100)

        duration  = time.time() - start_time
        is_passed = len(tag_id2pose) / tags_count >= 0.75
        return {
            "passed":               is_passed,
            "duration":             duration,
            "tags_detected_count":  len(tag_id2pose),
        }
