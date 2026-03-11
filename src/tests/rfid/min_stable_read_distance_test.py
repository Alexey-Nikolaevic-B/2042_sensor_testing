import time
from typing import Dict, Any

from ..sensor_test import SensorTest, register_test


@register_test
class MinStableReadDistanceTest(SensorTest):
    name             = "min_stable_read_distance_test"
    display_name     = "Min Stable Read Distance"
    description      = (
        "Moves a single RFID tag from 0.25 m toward the antenna in 0.01 m steps, "
        "recording the closest distance at which the tag is still reliably detected. "
        "Passes when the minimum detected distance is ≤ 0.05 m."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        with open(sensor.rfid_map_path, "w") as f:
            f.write("fix1 1 0.5 0 0\n")

        # BUG FIX: was using 'max_stable_read_distance_test' world key by mistake
        world = sensor.test_to_world["min_stable_read_distance_test"]
        if not simulator.open_scene(world, sensor.sensor_sdf_path):
            raise RuntimeError("Failed to open Gazebo scene")

        tag_name = "rfid_tag1"
        if not simulator.wait_for_model_spawn(tag_name, 30):
            raise RuntimeError("RFID tag did not spawn within 30 s")

        min_dist       = 0.25
        current_dist   = 0.25
        reset_distance = 25
        total_steps    = max(1, round(0.25 / 0.01))
        step           = 0
        start_time     = time.time()

        while current_dist >= 0:
            simulator.set_pose(tag_name, reset_distance, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag_name, current_dist, 0, 0)

            data = sensor.capture_data(simulator=simulator, world_path=None, window=2)
            if data is not None and tag_name in data:
                min_dist = current_dist

            step += 1
            if progress_cb:
                progress_cb(int(step / total_steps * 100))

            current_dist = round(current_dist - 0.01, 5)

        duration  = time.time() - start_time
        is_passed = min_dist <= 0.05
        return {
            "passed":           is_passed,
            "duration":         duration,
            "min_read_distance": min_dist,
        }
