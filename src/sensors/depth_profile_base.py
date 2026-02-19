from __future__ import annotations

import os
import time
from typing import Any, Dict, Tuple

import numpy as np
import rospy

from .depth_camera import DepthCamera


class DepthProfileBase(DepthCamera):
    DEPTH_TOPIC = ""
    IMAGE_TOPIC = ""

    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 720
    UPDATE_RATE = 30

    HORIZONTAL_FOV_RAD = 1.518
    CLIP_NEAR = 0.28
    CLIP_FAR = 10.0

    TEST_DISTANCES: Tuple[float, ...] = (1.0, 3.0, 5.0)
    MAX_ABS_ERROR_M = 0.8

    def __init__(self, CONFIG):
        super().__init__(CONFIG)
        self.CONFIG = CONFIG

        self.image_width = int(self.IMAGE_WIDTH)
        self.image_height = int(self.IMAGE_HEIGHT)
        self.horizontal_fov = float(self.HORIZONTAL_FOV_RAD)
        self.clip_near = float(self.CLIP_NEAR)
        self.clip_far = float(self.CLIP_FAR)
        self.update_rate = int(self.UPDATE_RATE)

        worlds_root = CONFIG["WORLDS_PATH"]
        if not os.path.isabs(worlds_root):
            worlds_root = os.path.join(CONFIG["ROOT_PATH"], worlds_root)

        self.test_to_world = {
            "depth_perception_test": os.path.join(worlds_root, "camera_depth_perception.world")
        }

    def depth_perception_test(self, simulator) -> Dict[str, Any]:
        world_path = self.test_to_world["depth_perception_test"]

        cube_name = "close_green_cube"
        cube_z = 0.25
        reset_x = 50.0

        if not simulator.open_scene(world_path, self.sensor_sdf_path):
            raise RuntimeError(f"Failed to open scene: {world_path}")

        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)

        if not simulator.wait_for_model_spawn(cube_name, 30):
            raise RuntimeError(f"cube not spawned: {cube_name}")

        results = []
        measured_values = []

        for d in self.TEST_DISTANCES:
            simulator.set_pose(cube_name, reset_x, 0.0, cube_z)
            time.sleep(0.05)

            simulator.set_pose(cube_name, float(d) + 0.25, 0.0, cube_z)
            time.sleep(0.6)

            data = self.capture_data(simulator, world_path=None, timeout=1.5, focused_image=False)
            if data is None:
                raise RuntimeError(f"No depth message at distance={d}")
            if "error" in data:
                raise RuntimeError(f"Depth capture error at distance={d}: {data['error']}")

            z = data.get("focused_depth_m")
            if z is None or np.isnan(z) or z <= 0.0:
                raise RuntimeError(f"Invalid depth measurement at distance={d}: {z}")

            if not (self.CLIP_NEAR <= float(z) <= (self.CLIP_FAR + 0.5)):
                raise AssertionError(f"Depth out of clip range at distance={d}: z={z}, clip=({self.CLIP_NEAR}, {self.CLIP_FAR})")

            abs_err = abs(float(z) - float(d))
            rel_err = abs_err / float(d) * 100.0

            if abs_err > float(self.MAX_ABS_ERROR_M):
                raise AssertionError(
                    f"Depth absolute error too high at distance={d}: abs_err={abs_err:.4f}, max={self.MAX_ABS_ERROR_M}"
                )

            results.append(
                {
                    "distance": float(d),
                    "measured": float(z),
                    "abs_error": float(abs_err),
                    "rel_error": float(rel_err),
                }
            )
            measured_values.append(float(z))

        monotonic_ok = all(measured_values[i] < measured_values[i + 1] for i in range(len(measured_values) - 1))
        if not monotonic_ok:
            raise AssertionError(f"Depth monotonicity failed: {measured_values}")

        return {
            "id": "DEPTH",
            "metrics": {
                "distances_m": list(self.TEST_DISTANCES),
                "max_abs_error_m": float(self.MAX_ABS_ERROR_M),
                "measurements": results,
                "monotonic_increasing": True,
            },
        }
