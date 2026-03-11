import math
import time
from typing import Dict, Any, Optional

from ..sensor_test import SensorTest, register_test


@register_test
class OverlapTagsTest(SensorTest):
    name             = "overlap_tags_test"
    display_name     = "Overlap Tags"
    description      = (
        "Tests RFID detection when tags are placed at decreasing angular spacing "
        "(0.2, 0.1, 0.05, 0.02 m arc distance). Records the smallest spacing at "
        "which all 5 tags are still detected. Passes if any spacing achieves 100 % detection."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        distances   = [0.2, 0.1, 0.05, 0.02]
        radius      = sensor.read_distance / 2
        tags_count  = 5
        dist_result: Optional[float] = None
        total_steps = len(distances)
        start_time  = time.time()

        for step, distance in enumerate(distances):
            with open(sensor.rfid_map_path, "w") as f:
                for i in range(tags_count):
                    x = radius * math.cos(distance / radius * i)
                    y = radius * math.sin(distance / radius * i)
                    f.write(f"fix{i + 1} {i + 1} {x} {y} 0\n")

            world = sensor.test_to_world["overlap_tags_test"]
            if not simulator.open_scene(world, sensor.sensor_sdf_path):
                raise RuntimeError("Failed to open Gazebo scene")

            for i in range(tags_count):
                if not simulator.wait_for_model_spawn(f"rfid_tag{i + 1}", 30):
                    raise RuntimeError(f"rfid_tag{i + 1} did not spawn within 30 s")

            tag_id2pose = sensor.capture_data(simulator=simulator, world_path=None, window=5)
            if len(tag_id2pose) == tags_count:
                dist_result = distance

            if progress_cb:
                progress_cb(int((step + 1) / total_steps * 100))

        duration  = time.time() - start_time
        is_passed = dist_result is not None
        return {
            "passed":      is_passed,
            "duration":    duration,
            "dist_result": dist_result,
        }
