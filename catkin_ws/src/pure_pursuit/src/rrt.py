#!/usr/bin/env python

import rospy
import numpy as np
from math import atan2, cos, sin
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObstaclePoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RRTPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obstacle_list", ObstaclePoseList, self.cb_obstacle, queue_size=1)
		self.sub_goal		  = rospy.Subscriber("/goal", WaypointList, self.cb_wplist, queue_size = 1)
		self.pub_rviz		  = rospy.Publisher("/wp_line", Marker, queue_size = 1)
		self.lock = False
		self.init_param()
	def init_param(self):
		# Dimension
		self.X_DIM = 40
		self.Y_DIM = 40
		# Goal point
		self.q_goal = np.array([self.X_DIM, self.Y_DIM])
		# Inital point
		self.init = np.zeros(2)
		# Vehicle size
		self.car_length = 0.6
		# Walk distance
		self.delta_d = 0.8
		# Tolerance distance
		self.epsilon = 1
		# Standard derivation
		self.sigma = 10;
		# obstacles list
		self.obstacle_list = None
		# Waypoints size
		self.waypoint_size = 1
		# Waypoint list
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = "odom"
		# q_list size
		self.q_size = 1
		# q_list
		self.q_list = np.array([[0., 0., 1]])
		# q_near
		self.q_near = np.array([0., 0.])
		# q_rand
		self.published = False

	def cb_wplist(self, wp_list_msg):
		length = len(wp_list_msg.list)
		self.X_DIM = wp_list_msg.list[length-1].x
		self.Y_DIM = wp_list_msg.list[length-1].y

	def cb_obstacle(self, obstacle_msg):
		self.init_param()
		self.obstacle_list = obstacle_msg
		if self.published is False and self.X_DIM is not None and not self.lock:
			self.lock = True
			print "Start RRT"
			self._rrt_process()
		else:
			self.waypoint_list.header.stamp = rospy.Time.now()
			self.pub_waypointList.publish(self.waypoint_list)

	def _rrt_process(self):
		while (np.linalg.norm(self.q_goal - self.q_near) > self.epsilon):
			
			# Generate q_rand
			#self.q_rand = np.array([np.random.uniform(-1,1)*self.X_DIM, np.random.uniform(-1,1)*self.Y_DIM])
			self.q_rand = np.array([np.random.normal(self.X_DIM, self.sigma),
					   	np.random.normal(self.Y_DIM, self.sigma)])
			self.q_near, index = self._near()
			if(self.q_near[0] > self.X_DIM):
				self.q_near[0] = self.X_DIM
			if(self.q_near[0] < 0):
				self.q_near[0] = 0
			if(self.q_near[1] > self.Y_DIM):
				self.q_near[1] = self.Y_DIM
			if(self.q_near[1] < 0):
				self.q_near[1] = 0
			if(self._is_hit_constrain(self.q_list[index], self.q_near) == 0):
				self.q_list = np.append(self.q_list, [[0, 0, 0]], axis = 0)
				self.q_list[self.q_size][0:2] = self.q_near
				self.q_list[self.q_size][2] = int(index+1)
				self.q_size = self.q_size + 1
		# end while 
		# Reverse tracking q_list
		waypoint_index_list = [self.q_size]
		child_index = int(self.q_size)
		parent_index = int(self.q_list[self.q_size-1][2])
		while (child_index != 1):
			self.waypoint_size = self.waypoint_size + 1
			waypoint_index_list.append(parent_index)
			temp = parent_index
			parent_index = int(self.q_list[temp-1][2])
			child_index = temp
		# end while
		self.waypoint_list.size = self.waypoint_size
		for i in range(self.waypoint_size, 0, -1):
			waypoint = Waypoint()
			waypoint.x = self.q_list[waypoint_index_list[i-1]-1][0]
			waypoint.y = self.q_list[waypoint_index_list[i-1]-1][1]
			print waypoint.x, waypoint.y
			self.waypoint_list.list.append(waypoint)
		self.waypoint_list.header.stamp = rospy.Time.now()
		self.pub_waypointList.publish(self.waypoint_list)
		#self.published = True
		print "q_list", self.q_list
		print "waypoint index", waypoint_index_list
		print "waypoint list"
		for i in range(0, self.waypoint_size):
			print self.waypoint_list.list[i].x, self.waypoint_list.list[i].y
		rospy.loginfo("[%s] RRT path planning finished" %(self.node_name))
		self.rviz()
		self.lock = False

	def rviz(self):
		marker = Marker()
		marker.header.frame_id = "velodyne"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.03
		marker.scale.y = 0.03
		marker.scale.z = 0.03
		marker.color.a = 1.0
		marker.color.r = 0
		marker.color.g = 1.0
		marker.color.b = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		marker.points = []
		for i in range(self.waypoint_size):
			p = Point()
			p.x = self.waypoint_list.list[i].x
			p.y = self.waypoint_list.list[i].y
			p.z = self.waypoint_list.list[i].z
			marker.points.append(p)
		self.pub_rviz.publish(marker)

	# Given q_list and q_rand, return q_near and parent index
	def _near(self):
		min_dis = 1e6
		index = 0
		L, _ = self.q_list.shape
		
		for i in range(1, L):
			if np.linalg.norm(self.q_rand - self.q_list[i][0:2]) < min_dis:
				min_dis = np.linalg.norm(self.q_rand - self.q_list[i][0:2])
				index = i
			# end if
		# end for (i)
		temp = self.q_rand - self.q_list[index][0:2] 
		q_near = self.q_list[index][0:2] + temp / np.linalg.norm(temp) * self.delta_d

		return q_near, index
	# end _near

	def in_hull(self, p, hull):
		if not isinstance(hull,Delaunay):
			hull = Delaunay(hull)
		return hull.find_simplex(p)>=0

	# Check if the line segment from parent node to child node hit constrain, return 1 for true and 0 otherwise
	def _is_hit_constrain(self, q_now, q_near):
		result = 0
		for obs_index in range(self.obstacle_list.size):
			a = [self.obstacle_list.list[obs_index].x_min_x, self.obstacle_list.list[obs_index].x_min_y]
			b = [self.obstacle_list.list[obs_index].y_min_x, self.obstacle_list.list[obs_index].y_min_y]
			c = [self.obstacle_list.list[obs_index].x_max_x, self.obstacle_list.list[obs_index].x_max_y]
			d = [self.obstacle_list.list[obs_index].y_max_x, self.obstacle_list.list[obs_index].y_max_y]
			obs_list = [a]
			if not b in obs_list :
				obs_list.append(b)
			if not c in obs_list :
				obs_list.append(c)
			if not d in obs_list :
				obs_list.append(d)
			obs_hull = np.array(obs_list)
			if(len(obs_list) < 3):
				continue
			new_node = np.array([[q_near[0], q_near[1]]])
			if self.in_hull(new_node, obs_hull):
				result = 1
				#print "FUCK"
				return result
		return result
	
if __name__ == "__main__":
	rospy.init_node("rrt_planning_node", anonymous = False)
	rrt_planning = RRTPlanning()
	rospy.spin()
