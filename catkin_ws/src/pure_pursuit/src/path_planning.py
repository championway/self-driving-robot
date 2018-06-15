#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2, cos, sin, sqrt
from scipy.spatial import Delaunay
from robotx_msgs.msg import ObstaclePoseList
from robotx_msgs.msg import Waypoint, WaypointList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PathPlanning(object):
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

	def line(self, p1, p2):
		# y = Ax + B
		A = (p2[1] - p1[1]) / (p2[0] - p1[0])
		B = -A * p1[0] + p1[1]
		return A, B

	def distance(self, p1, p2):
		return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def point_on_both_line(self, a1, a2, b1, b2, ans):
		if round(self.distance(a1, a2), 4) == round(self.distance(ans, a1) + self.distance(ans, a2), 4):
			if round(self.distance(b1, b2), 4) == round(self.distance(ans, b1) + self.distance(ans, b2), 4):
				return True
		return False

	def check_intersect(self, a1, a2, b1, b2):
		# if two lines are both perpendicular
		if a1[0] - a2[0] == 0 and b1[0] - b2[0] == 0:
			if a1[0] == b1[0]:
				return True	# if two line coincedent
			###### if they are on the same line ######
			return False	# if two lines parrellel
		if a1[0] - a2[0] == 0:	# a1a2 perpendicular
			L = self.line(b1, b2)
			x = a1[0]
			y = L[0]*x + L[1]
			ans = (x, y)
			return self.point_on_both_line(a1, a2, b1, b2, ans)
		if b1[0] - b2[0] == 0:	# b1b2 perpendicular
			L = self.line(a1, a2)
			x = b1[0]
			y = L[0]*x + L[1]
			ans = (x, y)
			return self.point_on_both_line(a1, a2, b1, b2, ans)
		# L1 : y = ax + b
		# L2 : y = cx + d
		# intersection(x, y):
		# x = -((b-d) / (a-c))
		# y = (ad-bc) / (a-c)
		L1 = self.line(a1, a2)
		L2 = self.line(b1, b2)
		if L1[0] == L2[0]:
			return False
			###### if they are on the same line ######
		x = -((L1[1] - L2[1]) / (L1[0] - L2[0]))
		y = (L1[0]*L2[1] - L1[1]*L2[0]) / (L1[0] - L2[0])
		ans = (x, y)
		return self.point_on_both_line(a1, a2, b1, b2, ans)

	def get_angle(self, p1, p2, p3):
		v0 = np.array(p2) - np.array(p1)
		v1 = np.array(p3) - np.array(p1)
		angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
		return abs(np.degrees(angle))

	def dis_point2line(self,):

	# dis -> shift distance, m -> slope, ,p -> shifted point
	# d > 0 right side, d < 0 left side, d = 0 on the line
	def shift_point(self, dis, m, d, p):
		if m == None:	# the slope is vertical
			if d > 0:	# point is above the line
				return [p[0], p[1] + dis]
			else:
				return [p[0], p[1] - dis]
		k = math.sqrt(dis**2 / (1 + m**2))
		if d < 0:
			k = -k
		new_point = [p[0]+k, p[1]+k*m]
		return new_point


	'''def cb_wplist(self, wp_list_msg):
		length = len(wp_list_msg.list)
		self.X_DIM = wp_list_msg.list[length-1].x
		self.Y_DIM = wp_list_msg.list[length-1].y'''

	def cb_obstacle(self, obstacle_msg):
		#self.init_param()
		self.obstacle_list = obstacle_msg
		np.
		if not self.lock:
			self.lock = True
			print "Start Path Planning"
			self._process()
		else:
			self.waypoint_list.header.stamp = rospy.Time.now()
			self.pub_waypointList.publish(self.waypoint_list)

	def _process(self):
		for obs_index in range(self.obstacle_list.size):
			collision = False
			p1 = [self.obstacle_list.list[obs_index].x_min_x, self.obstacle_list.list[obs_index].x_min_y]
			p2 = [self.obstacle_list.list[obs_index].y_min_x, self.obstacle_list.list[obs_index].y_min_y]
			p3 = [self.obstacle_list.list[obs_index].x_max_x, self.obstacle_list.list[obs_index].x_max_y]
			p4 = [self.obstacle_list.list[obs_index].y_max_x, self.obstacle_list.list[obs_index].y_max_y]
			obs_vertex = [p1]
			if not p2 in obs_vertex :
				obs_vertex.append(p2)
			if not p3 in obs_vertex :
				obs_vertex.append(p3)
			if not p4 in obs_vertex :
				obs_vertex.append(p4)
			for i,j in zip(obs_vertex[0::], obs_vertex[1::]):
				if self.check_intersect(i, j, p_now, p_next):
					collision = True
					break
			if self.check_intersect(obs_vertex[-1], obs_vertex[0], p_now, p_next):
				collision = True
			if collision:
				obs_angle_list = []
				angle_list = []
				for i in range(len(obs_vertex)):
					obs_angle_list.append([obs_vertex[i], self.get_angle(p_now, p_next, obs_vertex[i])])
					angle_list.append(obs_angle_list[i][1])
				angle_list.sort()
				for i in range(len(obs_vertex)):
					if obs_angle_list[i][1] == angle_list[-2]:
						p_hold = obs_angle_list[i]
						break
				m = None
				d = None
				if p_now[0] - p_next[0] == 0:	# if the line is vertical
					m = 0
					d = (p_hold[0] - p_now[0]) - (p_hold[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
				else:
					origin_m = self.line(p_now, p_next)
					if origin_m != 0:	# if the shift direction is not vertical
						# calculate the shift slope (vector)
						m = -(1.0 / origin_m)
						# determine the point is on which side of the line
						d = (p_hold[0] - p_now[0]) - (p_hold[1] - p_now[1])*(p_next[0] - p_now[0])/(p_next[1] - p_now[1])
					else:	# if the shift direction is vertical
						if p_hold[1] > p_now[1]:	# point is above the line
							d > 0
						else:
							d < 0
				p_hold = self.shift_point(self, self.car_length, m, d, p_hold)

			# find closest point
				# if distance from point to line < safe dis
					# then shift
					# check again
				# else
					# good!!!

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
	rospy.init_node("Path_planning_node", anonymous = False)
	rrt_planning = PathPlanning()
	rospy.spin()
