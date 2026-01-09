import json
import logging
from .sensors import Sensor


with open('log_config.json') as f_in:
    log_config = json.load(f_in)
logging.config.dictConfig(log_config)


logger = logging.getLogger(__name__)


def run_tests(simulator, sensor: Sensor):
    results = []

    test_functions = load_test_functions(sensor)

    # sorted - чтобы порядок был всегда одинаковым
    for test_name in sorted(test_functions):
        logger.info(f'Starting test: {test_name}')
        results.append({
            'test_name': test_name,
            'result': test_functions[test_name](simulator)
        })

    return results


def load_test_functions(sensor: Sensor):
    """Ищем методы для тестирования"""
    test_functions = dict()

    for attr in dir(sensor):
        if attr.endswith('_test'):
            test = getattr(sensor, attr)
            if callable(test):
                test_functions[attr] = test

    return test_functions
