import time
from typing import Dict, Any, Optional

from ..sensor_test import SensorTest, register_test


@register_test
class MoveTagsTest(SensorTest):
    name             = "move_tags_test"
    display_name     = "Move Tags"
    description      = (
        "Fires a single RFID tag through the antenna's read zone at increasing velocities "
        "(0.5, 1.0, 2.0 m/s). Records the highest velocity at which the tag is still "
        "detected in flight. Passes when at least one velocity produces a detection. "
        "Note: with the current rfid_tag_plugin (static tags) this test is expected to fail."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        from geometry_msgs.msg import Vector3

        velocities             = [Vector3(0.5, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)]
        result_velocity        = None
        dist_in_antenna_radius = 1.5
        start_distance         = -1 * sensor.read_distance * dist_in_antenna_radius
        start_time             = time.time()

        for step, velocity in enumerate(velocities):
            with open(sensor.rfid_map_path, "w") as f:
                f.write(f"fix1 1 {start_distance} 0 0\n")

            world = sensor.test_to_world["move_tags_test"]
            if not simulator.open_scene(world, sensor.sensor_sdf_path):
                raise RuntimeError("Failed to open Gazebo scene")

            if not simulator.wait_for_model_spawn("rfid_tag1", 30):
                raise RuntimeError("rfid_tag1 did not spawn within 30 s")

            simulator.set_pose(
                "rfid_tag1",
                x=start_distance, y=0, z=0,
                linear_velocity=velocity,
            )
            time_to_reach = dist_in_antenna_radius * sensor.read_distance / velocity.x
            tag_id2pose   = sensor.capture_data(
                simulator=simulator, world_path=None, window=time_to_reach * 2
            )

            if len(tag_id2pose) == 1:
                result_velocity = velocity

            if progress_cb:
                progress_cb(int((step + 1) / len(velocities) * 100))

            if result_velocity is not None and len(tag_id2pose) == 0:
                break

        duration  = time.time() - start_time
        is_passed = result_velocity is not None
        return {
            "passed":               is_passed,
            "duration":             duration,
            "max_detected_velocity": result_velocity,
        }
