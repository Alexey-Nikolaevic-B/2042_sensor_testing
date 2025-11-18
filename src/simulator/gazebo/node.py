import os
import rospy

from std_srvs.srv import Trigger, TriggerRequest
import threading

from sensor_msgs.msg import Image

class Node:
    def __init__(self, CONFIG):
        self.TIMEOUT = CONFIG['MESSAGE_TIMEOUT']
        
    def launch(self):
        print(f"✅ Node \'sendor_data_receiver\' is working")
        self.ros_node = rospy.init_node('sendor_data_receiver', anonymous=True)

    def get_sensor_data(self, topic): 
        msg = rospy.wait_for_message(topic, Image, timeout=self.TIMEOUT)
        return msg
    
    def trigger_camera(self, camera_service):
        try:
            rospy.wait_for_service(camera_service, timeout=10)
            trigger = rospy.ServiceProxy(camera_service, Trigger)
            response = trigger(TriggerRequest())
            return response.success, response.message  # Return both success and message
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False, str(e)

    def synchronized_capture(self, topic):
        # Create a wrapper to capture return value
        result = [None]  # Use list to store result (mutable)
        
        def thread_target():
            result[0] = self.trigger_camera(topic)
        
        thread1 = threading.Thread(target=thread_target)
        thread1.start()
        thread1.join()  # Wait for thread to complete
        
        return result[0]  # Return the result
        
    def kill(self):
        rospy.signal_shutdown("Done")
        print("💀 Node was killed.")