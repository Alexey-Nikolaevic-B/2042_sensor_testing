import time
import subprocess
import xml.etree.ElementTree as ET

class Gazebo: 
    def generate_world(self, world_path, camera_model_path, base_world_path):
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

    def launch(self, catkin_setup_dir, sensor_pkg, launch_file): #TODO: сделать получение ошибок из процесса
        roslaunch_cmd = f"source {catkin_setup_dir} && roslaunch {sensor_pkg} {launch_file}"
        try:
            self.gazebo_process = subprocess.Popen(["bash", "-c", roslaunch_cmd], 
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True)
            print(f"✅ Gazebo started with PID: {self.gazebo_process.pid}")
            # print("Process is running in background...")
            time.sleep(5)
            return f"Gazebo started successfully with PID: {self.gazebo_process.pid}" # TODO: это немного не тот PID
            
        except Exception as e:
            return f"Ошибка при запуске gazebo: {e}"

    def kill(self):
        try:
            subprocess.run(["pkill", "-f", "gzserver"], check=False)
            subprocess.run(["pkill", "-f", "gzclient"], check=False)
            print("💀 Gazebo was killed.")
        except Exception as e:
            print("⚠️ Ошибка при завершении Gazebo:", e)