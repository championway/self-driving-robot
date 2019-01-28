#!/usr/bin/env python
import rospy
import tf2_ros
import roslib
from sensor_msgs.msg import PointCloud2

if __name__ == "__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("gmapping_odom",anonymous=False)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom','velodyne', rospy.Time())
            print ("trans: ", trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print "PLZ DON'T PRINT THIS"
            continue
            