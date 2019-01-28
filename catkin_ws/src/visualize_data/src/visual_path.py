#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point

class VisualPath(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.pub_path = rospy.Publisher("/fusion_path", Path, queue_size = 20)
		self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.cb_odom, queue_size = 20)
		self.path = Path()
		rospy.loginfo("[%s] Initialized ..." %(self.node_name))

	def cb_odom(self, msg):
		self.path.header = msg.header
		pose_ = PoseStamped()
		pose_.header = msg.header
		pose_.pose = msg.pose.pose
		self.path.poses.append(pose_)
		self.pub_path.publish(self.path)

	
if __name__ == "__main__":
	rospy.init_node('visual_path_node', anonymous= False)
	visual = VisualPath()
	rospy.spin()	

