from .sensors import Sensor


def load_test_functions(sensor: Sensor):
    """
    Возвращает словарь: название метода - метод
    для всех методов с постфиксом _test
    у переданного датчика sensor
    """
    test_functions = {}

    for attr in dir(sensor):
        if attr.endswith('_test'):
            test = getattr(sensor, attr)
            if callable(test):
                test_functions[attr] = test

    return test_functions
