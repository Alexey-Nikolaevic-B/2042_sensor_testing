import os
import time
import subprocess
import xml.etree.ElementTree as ET

def run_test(sensor_type: str, sensor: list):

    sensor_name = sensor['name']
    scene = sensor['test_world']

    world_path = f'resources/worlds/{scene}.world'
    camera_model_path = f'resources/sensors/{sensor_type}/{sensor_name}.sdf'
    base_world_path = 'catkin_ws/src/scenario_test_pkg/worlds/base_world.world'

    generate_world(world_path, camera_model_path, base_world_path)

    catkin_setup_dir = 'catkin_ws/devel/setup.bash'
    sensor_pkg = 'scenario_test_pkg'
    launch_file = 'scenario.launch'

    print(f"Запуск сцены {scene} для датчика {sensor_name}")
    run_gazebo(catkin_setup_dir, sensor_pkg, launch_file)

    if sensor_type == 'camera':
        test_result = run_camera_test()
    if sensor_type == 'tactile':
        test_result = run_tactile_test()
    if sensor_type == 'rfid':
        test_result = run_camera_test()


    time.sleep(10)
    kill_gazebo()

    return test_result

def run_camera_test():
    pass

def run_tactile_test():
    pass

def run_camera_test():
    pass

def generate_world(world_path, camera_model_path, base_world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    camera_tree = ET.parse(camera_model_path)
    camera_root = camera_tree.getroot()
    camera_model = camera_root.find('model')
    if camera_model is None:
        print("Error: No model found in camera SDF file")
        return
    world.append(camera_model)
    tree.write(base_world_path, encoding='utf-8', xml_declaration=True)

def run_gazebo(catkin_setup_dir, sensor_pkg, launch_file):
    roslaunch_cmd = f"source {catkin_setup_dir} && roslaunch {sensor_pkg} {launch_file}"
    print('--->', roslaunch_cmd)
    try:
        subprocess.Popen(["bash", "-c", roslaunch_cmd])
        time.sleep(10)
    except Exception as e:
        return f"Ошибка при запуске gazebo: {e}"

def kill_gazebo():
    try: # TODO: что-то он не ловит ошибки
        subprocess.run(["pkill", "-f", "gzserver"], check=False)
        subprocess.run(["pkill", "-f", "gzclient"], check=False)
        print("🔁 Gazebo был завершён принудительно.")
        time.sleep(5)
    except Exception as e:
        print("⚠️ Ошибка при завершении Gazebo:", e)

def get_sensor_data():
    pass 


