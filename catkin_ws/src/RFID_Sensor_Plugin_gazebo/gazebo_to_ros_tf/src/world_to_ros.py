#!/usr/bin/env python3
import yaml
import rospy
import rospkg
import tf
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class GazeboRos:
    def __init__(self):
        rospack = rospkg.RosPack()
        rospack.list()
        path = rospack.get_path("gazebo_to_ros_tf")
        self.yamlpath = path + "/config/data.yaml"

        self.odomtopic = "/odom"
        self.odom_frame = "odom"
        self.baselink_frame = "base_link"

        with open(self.yamlpath) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            for key, value in data.items():
                if key == "odomtopic":
                    self.odomtopic = value
                elif key == "odom_frame_name":
                    self.odom_frame = value
                elif key == "baselink_frame_name":
                    self.baselink_frame = value

        rospy.init_node("odometry_publisher")

        rospy.Subscriber(self.odomtopic, Odometry, self.cb_pose, queue_size=1)

        self.odom_pub = rospy.Publisher(self.odom_frame, Odometry, queue_size=1)

        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.r = rospy.Rate(1000)

    def cb_pose(self, msg: Odometry):
        localx = msg.pose.pose.position.x
        localy = msg.pose.pose.position.y
        localz = msg.pose.pose.position.z

        orix = msg.pose.pose.orientation.x
        oriy = msg.pose.pose.orientation.y
        oriz = msg.pose.pose.orientation.z
        oriw = msg.pose.pose.orientation.w

        quaternion = (orix, oriy, oriz, oriw)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        yawdeg = yaw * (180 / 3.14)

        odom = Odometry()
        r = rospy.Time.now()
        odom.header.stamp = r
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.baselink_frame

        odom.pose.pose.position.x = localx
        odom.pose.pose.position.y = localy
        odom.pose.pose.position.z = localz

        odom.pose.pose.orientation.x = orix
        odom.pose.pose.orientation.y = oriy
        odom.pose.pose.orientation.z = oriz
        odom.pose.pose.orientation.w = oriw

        t = TransformStamped()
        t.header.stamp = r
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.baselink_frame

        t.transform.translation.x = localx
        t.transform.translation.y = localy
        t.transform.translation.z = localz

        t.transform.rotation.x = orix
        t.transform.rotation.y = oriy
        t.transform.rotation.z = oriz
        t.transform.rotation.w = oriw

        self.odom_broadcaster.sendTransform(t)
        self.odom_pub.publish(odom)

        self.last_time = r
        self.r.sleep()

    def listener(self):
        rospy.spin()


if __name__ == "__main__":
    gz = GazeboRos()
    gz.listener()
