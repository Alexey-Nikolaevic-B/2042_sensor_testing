import time
import subprocess

class Ros:
    def launch(self):
        subprocess.run(["bash", "-c", "pkill -f ros"])
        time.sleep(1)
        try:
            self.ros_process = subprocess.Popen(
                ["bash", "-c", "roscore"], 
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True
            )
            print(f"✅ Ros master started with PID: {self.ros_process.pid}") # TODO: это немного не тот PID
            print("Process is running in background...")
            time.sleep(2)
            return f"Ros started successfully with PID: {self.ros_process.pid}"  
            
        except Exception as e:
            return f"Ошибка при запуске gazebo: {e}"

    def kill(self):
        subprocess.run(["bash", "-c", "pkill -f ros"])
        print("💀 Ros was killed.")

        


