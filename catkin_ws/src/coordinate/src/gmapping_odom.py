#!/usr/bin/env python
import rospy
import tf
import roslib
from sensor_msgs.msg import PointCloud2

def call_back(msg):
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map','velodyne',msg.header.stamp)
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		print ("trans: ", trans)
		print ("rot: ", rot)

if __name__ == "__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("gmapping_odom",anonymous=False)
    rospy.Subscriber("/velodyne_points", PointCloud2, call_back, queue_size=1)
    rospy.spin()