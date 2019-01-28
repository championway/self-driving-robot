#!/usr/bin/env python

import rospy

from robotx_msgs.msg import WaypointList
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def cb_list(msg):
	print "hello"
	length = len(msg.list)
	marker = Marker()
	marker.header = msg.header
	marker.type = Marker.POINTS
	marker.action = Marker.ADD
	marker.scale.x = marker.scale.y = marker.scale.z = 0.05
	for i in range(0, length):
		marker.points.append(Point(msg.list[i].x, msg.list[i].y, msg.list[i].z))
		marker.colors.append(ColorRGBA(0.0, 0.0, 1.0, 1.0))
	pub_marker.publish(marker)
	print "hello1"

if __name__ == "__main__":
	rospy.init_node("visual_waypoints", anonymous = False)
	pub_marker = rospy.Publisher("/visual/waypoints_marker", Marker, queue_size = 1)
	rospy.Subscriber("/waypointList", WaypointList, cb_list, queue_size = 1)
	rospy.spin()