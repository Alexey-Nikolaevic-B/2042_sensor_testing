import logging

def ChangeDistanceTest(simulator, config, sensorType, sensor) -> None:
    """
    Run RFID change distance test.

    A fixed RFID reader remains stationary with constant orientation. A single RFID tag is placed
    in the reader's line of sight and moved straight away from the reader starting from a near
    distance (e.g., 0.5 m) in fixed steps (e.g., 0.5 m) up to a maximum test distance (e.g., 10 m).
    The function collects the reader output (e.g., ROS topics) to evaluate detection behavior vs. distance.
    """

    logger = logging.getLogger(__name__)

    testName = "rfid Change Distance Test"

    worldsPath = config['worldsPath']
    sensorsPath = config['sensorsPath']

    sensorName = sensor['name']
    world = ['rfid_change_distance']
    topic = '/detected_tags'

    worldPath = f'{worldsPath}depth_perception_test/{world}.world'
    rfidModelPath = f'{sensorsPath}{sensorType}/{sensorName}.sdf'

    simulatorIsRunning = simulator.open_scene(worldPath, rfidModelPath)
    if not simulatorIsRunning:
        errorMessage = f'simulator wasn\'t started during {testName}'
        logger.error(errorMessage)
        raise RuntimeError(errorMessage)