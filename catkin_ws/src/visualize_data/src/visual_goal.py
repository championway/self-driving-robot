#!/usr/bin/env python

import rospy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from robotx_msgs.msg import WaypointList

def cb_wp_list(msg):
	length = len(msg.list)
	marker = Marker()
	marker.header = msg.header
	marker.type = Marker.POINTS
	marker.action = Marker.ADD
	marker.scale.x = marker.scale.y = marker.scale.z = 0.05
	marker.color.r = 1
	p = Point()
	for i in range(0, length):
		p.x = msg.list[i].x
		p.y = msg.list[i].y
		marker.colors.append(ColorRGBA(1, 0, 0, 1))
		marker.points.append(p)
	pub_marker.publish(marker)

if __name__ == "__main__":
	rospy.init_node('visual_goal_node', anonymous = False)
	rospy.Subscriber("/goal", WaypointList, cb_wp_list, queue_size = 10)
	pub_marker = rospy.Publisher('/visual/visual_goal', Marker, queue_size = 10)
	rospy.spin()