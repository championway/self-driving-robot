#!/usr/bin/env python

import rospy
import utm

from robotx_msgs.msg import Waypoint, WaypointList
from tf2_msgs.msg import TFMessage
from gps_receiver.srv import *

global x_, y_
x_ = 0
y_ = 0

global goal_x, goal_y
goal_x = 0
goal_y = 0

global status
status = False

global utm_parent 
utm_parent = 1 # 1 if use utm as parent, -1 if use odom as parent

def handle_set_goal(req):
	utm_x, utm_y, _, _ = utm.from_latlon(req.lat, req.lon)
	
	resp = set_goalResponse()
	if(not status):
		global x_, y_
		global utm_parent
		print x_, y_
		resp.x = utm_x - x_ * utm_parent
		resp.y = utm_y - y_ * utm_parent
		global goal_x, goal_y
		goal_x = resp.x
		goal_y = resp.y
		rospy.loginfo("x: %s, y: %s " %(resp.x, resp.y))
	return resp

def cb_tf(msg):
	global x_, y_
	global utm_parent
	x_ = msg.transforms[0].transform.translation.x
	y_ = msg.transforms[0].transform.translation.y
	utm_x0, utm_y0 = utm.to_latlon(x_ * utm_parent, y_ * utm_parent, 51, 'R') # Hsinchu zone number and symbol
	rospy.loginfo("[%s] x: %s; y: %s N: %s; E: %s "%(rospy.get_name(), x_, y_, utm_x0, utm_y0))
	status = True

def pub_data(event):
	if(goal_x == 0 and goal_y == 0):
		return
	else:
		wp_list = WaypointList()
		wp_list.header.stamp = rospy.Time.now()
		wp_list.header.frame_id = 'odom'
		wp = Waypoint()
		wp.x = goal_x
		wp.y = goal_y
		wp_list.list.append(wp)
		pub_wp_list.publish(wp_list)

def set_goal_server():
	rospy.init_node('set_goal_server_node')
	server = rospy.Service('set_goal', set_goal, handle_set_goal)
	sub_tf = rospy.Subscriber("/tf_static", TFMessage, cb_tf, queue_size = 1)
	global pub_wp_list
	pub_wp_list = rospy.Publisher("/goal", WaypointList, queue_size = 20)
	rospy.Timer(rospy.Duration(1), pub_data)
	rospy.spin()

if __name__ == "__main__":
	set_goal_server()
