#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import math
import tf
"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class pub_rviz():
    def __init__(self):
        self.robot_pose = Point()
        self.finish = False
        self.frame_name = "gazebo"
        self.waypoint_list = rospy.get_param('~path')
        self.odom = Marker()
        self.initial_odom()
        self.lookahead = Marker()
        self.initial_lookahead()
        self.waypoint = Marker()
        self.initial_waypoint()
        # Init subscribers and publishers
        self.sub_lookahead = rospy.Subscriber("/pure_pursuit/lookahead", Point, self.lookaheadCB, queue_size=1)
        self.sub_model_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_stateCB, queue_size=1)
        self.sub_finish = rospy.Subscriber("/pure_pursuit/finished", Bool, self.finishCB, queue_size=1)
        self.odom_pub = rospy.Publisher("odometry_marker",Marker,queue_size=1)
        self.waypoint_pub = rospy.Publisher("waypoint_marker",Marker,queue_size=1)
        self.lookahead_pub = rospy.Publisher("lookahead_marker",Marker,queue_size=1)

    def initial_lookahead(self):
        #self.lookahead = Marker()
        self.lookahead.header.frame_id = self.frame_name
        self.lookahead.header.stamp = rospy.Time.now()
        self.lookahead.ns = "points_for_odometry"
        self.lookahead.action = Marker.ADD
        self.lookahead.pose.orientation.w = 1.0
        self.lookahead.id = 0
        self.lookahead.type = Marker.LINE_STRIP
        self.lookahead.scale.x = 0.03
        self.lookahead.scale.y = 0.03
        self.lookahead.color.b = 1.0
        self.lookahead.color.a = 1.0

    def initial_waypoint(self):
        #self.waypoint = Marker()
        self.waypoint.header.frame_id = self.frame_name
        self.waypoint.header.stamp = rospy.Time.now()
        self.waypoint.ns = "points_for_waypoint"
        self.waypoint.action = Marker.ADD
        self.waypoint.pose.orientation.w = 1.0
        self.waypoint.id = 0
        self.waypoint.type = Marker.LINE_STRIP
        self.waypoint.scale.x = 0.03
        self.waypoint.scale.y = 0.03
        self.waypoint.color.r = 1.0
        self.waypoint.color.a = 1.0
        for way in self.waypoint_list:
            wp = Point()
            wp.x, wp.y = way[:2]
            wp.z = 0
            self.waypoint.points.append(wp)

    def initial_odom(self):
        #self.odom = Marker()
        self.odom.header.frame_id = self.frame_name
        self.odom.header.stamp = rospy.Time.now()
        self.odom.ns = "points_for_odometry"
        self.odom.action = Marker.ADD
        self.odom.pose.orientation.w = 1.0
        self.odom.id = 0
        self.odom.type = Marker.LINE_STRIP
        self.odom.scale.x = 0.03
        self.odom.scale.y = 0.03
        self.odom.color.g = 1.0
        self.odom.color.a = 1.0

    def pub_to_rviz(self):
        #self.waypoint.points = self.odom.points
        #print self.waypoint.points
        self.waypoint_pub.publish(self.waypoint)
        self.lookahead_pub.publish(self.lookahead)

    def pub_odom(self):
        p = Point()
        p.x = self.robot_pose.x
        p.y = self.robot_pose.y
        p.z = self.robot_pose.z
        self.odom.points.append(p)
        self.odom_pub.publish(self.odom)

    def finishCB(self, msg):
        if msg.data:
            print "Finish"
            self.finish = True
            self.odom.points[:] = []
        else:
            self.finish = False

    def lookaheadCB(self, msg):
        p = Point()
        p.x = self.robot_pose.x
        p.y = self.robot_pose.y
        p.z = self.robot_pose.z
        q = Point()
        q.x = msg.x
        q.y = msg.y
        q.z = msg.z
        self.lookahead.points[:] = []
        self.lookahead.points.append(p)
        self.lookahead.points.append(q)
        #print q

    def model_stateCB(self, msg):
        self.robot_pose.x = msg.pose[1].position.x
        self.robot_pose.y = msg.pose[1].position.y
        self.robot_pose.z = msg.pose[1].position.z
        if not self.finish:
            self.pub_odom()    
            self.pub_to_rviz()

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("pub_rviz",anonymous=False)
    pub_rviz = pub_rviz()
    rospy.spin()