#!/usr/bin/env python3
import time
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion

def quat_from_yaw(yaw):
    return Quaternion(0, 0, math.sin(yaw/2), math.cos(yaw/2))

rospy.init_node("move_tag")
rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

state = ModelState()
state.model_name = "rfid_tag_1"
state.reference_frame = "world"
state.pose.orientation = quat_from_yaw(0)

x = 0.5
while not rospy.is_shutdown() and x <= 10.0:
    state.pose = Pose(Point(x, 0.0, 0.05), quat_from_yaw(0))
    set_state(state)
    print("x =", x)
    x = round(x + 0.5, 2)
    time.sleep(0.3)
