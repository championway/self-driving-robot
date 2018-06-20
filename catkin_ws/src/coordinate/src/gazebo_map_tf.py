#!/usr/bin/env python
import rospy
import roslib
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
def call_back(msg):
    # Global Frame: "/map"

    br = tf.TransformBroadcaster()
    # map <---> gazebo coordinate
    #br.sendTransform((0.0, 0.0, 0.0),tf.transformations.quaternion_from_euler(0, 0, -np.pi/2.0),rospy.Time.now(),"/map","/gazebo")
    # robot <---> map
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),rospy.Time.now(),"/base_footprint","/map")

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("tf_publisher",anonymous=False)
    rospy.Subscriber("/p3d_odom", Odometry, call_back, queue_size = 10)
    rospy.spin()