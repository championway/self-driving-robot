#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped
from robotx_msgs.msg import BoolStamped
import numpy as np
pub_bool = BoolStamped()
pub_bool.data = True
def call_back(msg):
    # Global Frame: "/map"

    br = tf.TransformBroadcaster()
    # map <---> gazebo coordinate
    br.sendTransform((0.0, 0.0, 0.0),tf.transformations.quaternion_from_euler(0, 0, -np.pi/2.0),rospy.Time.now(),"/map","/gazebo")
    # robot <---> map
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),rospy.Time.now(),"/robot","/map")
    # lidar <---> map
    br.sendTransform((0.0, 0.0, 0.0),(0, 0, 0, 1.0),rospy.Time.now(),"/velodyne","robot")
    pub_bool.header.stamp = rospy.Time.now()
    pub.publish(pub_bool)
    print("Broadcast TF")

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("tf_publisher",anonymous=False)
    pub = rospy.Publisher("/tf_transform", BoolStamped, queue_size=1)
    rospy.Subscriber("/robot_pose", PoseStamped, call_back, queue_size=1)
    rospy.spin()