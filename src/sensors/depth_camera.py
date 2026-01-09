import time
import rospy
import numpy as np

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion

from .sensor import register_sensor
from .mono_camera import MonoCamera


@register_sensor('camera', 'depth_camera')
class DepthCamera(MonoCamera):
    """Камера глубины"""
    def __init__(self, CONFIG):
        super().__init__(CONFIG)

        WORLDS_PATH = CONFIG["WORLDS_PATH"]
        SENSORS_PATH = CONFIG["SENSORS_PATH"]

        self.sensor_sdf_path = f'{SENSORS_PATH}{self.sensor_type}/{self.sensor_name}.sdf'

        self.test_to_world = {
            'depth_perception_test': f'{WORLDS_PATH}depth_camera/depth_perception_test.world'
        }


    def depth_perception_test(self, simulator):
        # TODO: если делать импорты снаружи, то почему-то не работает
        from cv_bridge import CvBridge, CvBridgeError
        """Тест точности измерения глубины на дистанциях 1м, 3м, 5м"""

        topic = '/depth_camera/depth/image_raw'
        world_path = self.test_to_world['depth_perception_test']

        cube_name = 'close_green_cube'
        cube_z = 0.25
        reset_x = 50.0
        distances = [1.0, 3.0, 5.0]

        if not simulator.open_scene(world_path, self.sensor_sdf_path):
            return None

        start = time.time()
        while time.time() - start < 30.0:
            try:
                ms = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
                if cube_name in ms.name:
                    break
            except rospy.ROSException:
                continue
        else:
            raise RuntimeError(f'cube not spawned: {cube_name}')

        rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        bridge = CvBridge()
        results = []

        for d in distances:
            state = ModelState()
            state.model_name = cube_name
            state.reference_frame = "world"

            state.pose = Pose(Point(reset_x, 0.0, cube_z), Quaternion(0, 0, 0, 1))
            resp = set_state(state)
            if not resp.success:
                raise RuntimeError(resp.status_message)

            time.sleep(0.5)

            state.pose = Pose(Point(d + 0.25, 0.0, cube_z), Quaternion(0, 0, 0, 1))
            resp = set_state(state)
            if not resp.success:
                raise RuntimeError(resp.status_message)

            time.sleep(0.5)

            msg = simulator.receive_sensor_data(topic)
            if msg is None:
                results.append({'distance': d, 'error': 'No message'})
                continue

            try:
                if msg.encoding == '32FC1':
                    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                elif msg.encoding == '16UC1':
                    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    depth_image = depth_image.astype(np.float32) * 0.001
                else:
                    results.append({'distance': d, 'error': f'Invalid encoding: {msg.encoding}'})
                    continue
            except CvBridgeError:
                results.append({'distance': d, 'error': 'CV Bridge Error'})
                continue

            h, w = depth_image.shape
            z = float(depth_image[h // 2, w // 2])

            if np.isnan(z) or z == 0.0:
                results.append({'distance': d, 'error': 'Invalid measurement'})
                continue

            abs_err = abs(z - d)
            rel_err = abs_err / d * 100.0

            results.append({
                'distance': d,
                'measured': z,
                'abs_error': abs_err,
                'rel_error': rel_err
            })

        return results
