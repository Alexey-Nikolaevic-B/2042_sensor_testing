import math
import time
from typing import Dict, Any

from ..sensor_test import SensorTest, register_test


@register_test
class AntennaRotationTest(SensorTest):
    name             = "antenna_rotation_test"
    display_name     = "Antenna Rotation"
    description      = (
        "Rotates the RFID antenna through 0°, 30°, 60°, 90° while 10 tags are "
        "arranged in a circle of radius read_distance/2. Records detection count "
        "at each angle. Passes when all 10 tags are detected at 0° orientation."
    )
    compatible_types = ["rfid"]

    def run(self, simulator, sensor, progress_cb=None) -> Dict[str, Any]:
        from geometry_msgs.msg import Quaternion

        angles     = [0, math.pi / 6, math.pi / 3, math.pi / 2]
        tags_count = 10
        radius     = sensor.read_distance / 2
        is_passed  = False
        angle2tags_count: Dict[float, int] = {}

        with open(sensor.rfid_map_path, "w") as f:
            for i in range(tags_count):
                x = radius * math.cos(2 * math.pi / tags_count * i)
                y = radius * math.sin(2 * math.pi / tags_count * i)
                f.write(f"fix{i + 1} {i + 1} {x} {y} 0\n")

        world = sensor.test_to_world["antenna_rotation_test"]
        if not simulator.open_scene(world, sensor.sensor_sdf_path):
            raise RuntimeError("Failed to open Gazebo scene")

        for i in range(tags_count):
            if not simulator.wait_for_model_spawn(f"rfid_tag{i + 1}", 30):
                raise RuntimeError(f"rfid_tag{i + 1} did not spawn within 30 s")

        start_time = time.time()

        antenna_model_name = getattr(sensor, "antenna_model_name", "rfid_antenna")

        for step, angle in enumerate(angles):
            quaternion = Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
            simulator.set_pose(antenna_model_name, x=0, y=0, z=0, quaternion=quaternion)
            time.sleep(0.01)

            tag_id2pose = sensor.capture_data(simulator=simulator, world_path=None, window=7)
            deg = math.degrees(angle)
            angle2tags_count[deg] = len(tag_id2pose)

            if len(tag_id2pose) == tags_count and angle == 0:
                is_passed = True

            if progress_cb:
                progress_cb(int((step + 1) / len(angles) * 100))

        duration = time.time() - start_time
        return {
            "passed":           is_passed,
            "duration":         duration,
            "angle2tags_count": angle2tags_count,
        }
