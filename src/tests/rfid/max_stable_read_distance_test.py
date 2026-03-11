import time
from typing import Optional, Dict, Any

from ..sensor_test import SensorTest, register_test


@register_test
class MaxStableReadDistanceTest(SensorTest):
    name             = "max_stable_read_distance_test"
    display_name     = "Max Stable Read Distance"
    description      = (
        "Moves a single RFID tag from 0.5 m outward in 0.5 m steps up to the "
        "configured read distance, recording the farthest distance at which the "
        "tag is reliably detected. Passes when the measured max distance is "
        "within 0.5 m of the configured read distance."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        with open(sensor.rfid_map_path, "w") as f:
            f.write("fix1 1 0.5 0 0\n")

        world = sensor.test_to_world["max_stable_read_distance_test"]
        if not simulator.open_scene(world, sensor.sensor_sdf_path):
            raise RuntimeError("Failed to open Gazebo scene")

        tag_name = "rfid_tag1"
        if not simulator.wait_for_model_spawn(tag_name, 30):
            raise RuntimeError("RFID tag did not spawn within 30 s")

        current_dist   = 0.5
        reset_distance = sensor.read_distance * 5
        max_dist       = 0.0
        total_steps    = max(1, round((sensor.read_distance - 0.5) / 0.5) + 1)
        step           = 0
        start_time     = time.time()

        while current_dist <= sensor.read_distance + 1e-9:
            simulator.set_pose(tag_name, reset_distance, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag_name, current_dist, 0, 0)

            data = sensor.capture_data(simulator=simulator, world_path=None, window=3)
            if data is not None and tag_name in data:
                max_dist = current_dist

            step += 1
            if progress_cb:
                progress_cb(int(step / total_steps * 100))

            current_dist = round(current_dist + 0.5, 1)

        duration  = time.time() - start_time
        is_passed = abs(max_dist - sensor.read_distance) <= 0.5
        return {
            "passed":           is_passed,
            "duration":         duration,
            "max_read_distance": max_dist,
        }
