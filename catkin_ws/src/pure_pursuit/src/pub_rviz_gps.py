#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import math
import tf
import utm
"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class pub_rviz():
    def __init__(self):
        self.robot_pose = Point()
        self.finish = False
        self.frame_name = "gazebo"
        self.waypoint_list = [(297866.7060146949, 2743015.9757434796),(297842.4781992137, 2743046.7985285847),(297820.873462504, 2743014.9847823563),(297847.32052896597, 2742990.6662742873),(297806.29471554933, 2742986.3919879636)]
        self.odom = Marker()
        self.initial_odom()
        self.lookahead = Marker()
        self.initial_lookahead()
        self.waypoint = Marker()
        self.initial_waypoint()
        # Init subscribers and publishers
        self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=1)
        self.sub_gps = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_cb, queue_size=1)
        self.sub_lookahead = rospy.Subscriber("/pure_pursuit/lookahead", Point, self.lookaheadCB, queue_size=1)
        self.sub_finish = rospy.Subscriber("/pure_pursuit/finished", Bool, self.finishCB, queue_size=1)
        self.odom_pub = rospy.Publisher("odometry_marker",Marker,queue_size=1)
        self.waypoint_pub = rospy.Publisher("waypoint_marker",Marker,queue_size=1)
        self.lookahead_pub = rospy.Publisher("lookahead_marker",Marker,queue_size=1)

    def imu_cb(self, msg):
        quaternion_msg = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.euler = tf.transformations.euler_from_quaternion(quaternion_msg)

    def gps_cb(self, msg):
        x_and_y = utm.from_latlon(msg.latitude, msg.longitude)
        self.robot_pose.x = x_and_y[1] - 2743010.9432636807
        self.robot_pose.y = -(x_and_y[0] - 297839.6270080163)
        self.robot_pose.z = self.euler[2]
        if not self.finish:
            self.pub_odom()    
            self.pub_to_rviz()

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
            way_x = way[1] - 2743010.9432636807
            way_y = -(way[0] - 297839.6270080163)
            wp = Point()
            wp.x = way_x
            wp.y = way_y
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
        q.y = msg.x - 2743010.9432636807
        q.x = -(msg.y - 297839.6270080163)
        q.z = msg.z
        self.lookahead.points[:] = []
        self.lookahead.points.append(p)
        self.lookahead.points.append(q)
        #print q


if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("pub_rviz",anonymous=False)
    pub_rviz = pub_rviz()
    rospy.spin()