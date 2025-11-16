import importlib.util
from pathlib import Path

def run(simulator, config, sensor_type: str, sensor: dict):
    result = []
    tests_to_run = list(sensor['tests'].keys())
    
    test_functions = load_test_functions()
    
    for test_name in tests_to_run:
        if test_name in test_functions:
            print(f"\n🔧 Running test: {test_name}")
            
            test_function = test_functions[test_name]
            test_result = test_function(simulator, config, sensor_type, sensor)
            result.append({
                'test_name': test_name,
                'result': test_result
            })
        else:
            print(f"\n⚠️  Warning: Test '{test_name}' not found")
        print('\n','='*60)
    return result

def load_test_functions():
    test_functions = {}
    tests_dir = Path(__file__).parent
    
    for file_path in tests_dir.glob("*_test.py"):
        module_name = file_path.stem
        spec = importlib.util.spec_from_file_location(module_name, file_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        for attr_name in dir(module):
            attr = getattr(module, attr_name)
            if callable(attr) and attr_name.endswith('_test') and not attr_name.startswith('_'):
                test_functions[attr_name] = attr
    
    return test_functions