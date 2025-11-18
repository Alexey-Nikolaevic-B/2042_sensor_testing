import sys

from .sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from .simulator.simulator_factory import SimulatorFactory
from .tests import _tests
from .mono_camera import capture

from .utils.utils import generate_world

from config import CONFIG

class Core:
    def __init__(self) -> None:
        self.simulator = SimulatorFactory.create_simulator('gazebo', CONFIG)

    def prepare_simulator(self) -> None:
        self.simulator.launch()

    def get_sensor_categories(self) -> list:
        return get_sensor_types()

    def get_sensors_by_model(self, sensor_model: str) -> list:
        return get_sensors_by_type(sensor_model)

    def get_sensor(self, sensor_category: str, sensor_model: str) -> dict:
        return get_sensor(sensor_category, sensor_model)
    
    def run_test(self, sensor_category: str, sensor: str) -> tuple:
        return _tests.run(self.simulator, CONFIG, sensor_category, sensor)
    
    def generate_base_scene(self, world_path, camera_model_path):
        generate_world(world_path, camera_model_path, CONFIG['BASE_WORLD_PATH'])

    def capture_data(self, ensor_category: str, sensor_model: str) -> None:
        world_path = 'resources/worlds/mono_camera/example.world'
        camera_model_path = 'resources/sensors/camera/mono_camera.sdf'
        generate_world(world_path, camera_model_path, CONFIG['BASE_WORLD_PATH'])
        capture(CONFIG, self.simulator)

    def kill(self) -> None:
        print('\nЗавершение процессов:')
        self.simulator.kill()
        sys.exit()