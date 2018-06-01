#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from nav_msgs.msg import Path
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import numpy as np
import math
import tf
import utm
from tf.transformations import quaternion_multiply, quaternion_from_euler

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class gazebo_pure_pursuit():
    def __init__(self):
        # Init attributes
        print ("start")
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.first_pose = PoseStamped()
        self.origin_pose = PoseStamped()
        self.origin_pose.header.frame_id = "map"
        self.origin_pose.pose.position.x = 0
        self.origin_pose.pose.position.y = 0
        self.origin_pose.pose.position.z = 0
        self.origin_pose.pose.orientation.x = 0
        self.origin_pose.pose.orientation.y = 0
        self.origin_pose.pose.orientation.z = 0
        self.origin_pose.pose.orientation.w = 1
        self.euler = None
        self.active = True
        self.start = True
        self.first = True
        # Init subscribers and publishers 
        #tss = TimeSynchronizer(Subscriber("/imu/data", Imu),Subscriber("/fix", NavSatFix))
        #tss.registerCallback(call_back)
        imu_sub = Subscriber("imu/data", Imu)
        gps_sub = Subscriber("/fix", NavSatFix)
        ats = ApproximateTimeSynchronizer((imu_sub, gps_sub),queue_size = 1, slop = 0.1)
        ats.registerCallback(self.call_back)
        #self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=1)
        #self.sub_gps = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_cb, queue_size=1)
        self.pub_gazebo = rospy.Publisher('/david/cmd_vel', Twist, queue_size=1)
        self.pub_origin_pose = rospy.Publisher('/orig_pose', PoseStamped, queue_size=1)
        self.pub_pose = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1)

    '''def imu_cb(self, msg):
        quaternion_msg = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.euler = tf.transformations.euler_from_quaternion(quaternion_msg)'''


    def call_back(self, imu_msg, gps_msg):
        if not self.active:
            return
        if self.first:
            x_y = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
            self.first_pose.pose.position.x = x_y[0]
            self.first_pose.pose.position.y = x_y[1]
            self.first = False
        # Rhe IMU sensor are not at the right direction originally, so we rotate 90 degree with z
        q_orig = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        q_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi/2.0)
        q_new = tf.transformations.quaternion_multiply(q_rot, q_orig)
        self.pose.pose.orientation.x = q_new[0]
        self.pose.pose.orientation.y = q_new[1]
        self.pose.pose.orientation.z = q_new[2]
        self.pose.pose.orientation.w = q_new[3]
        print(q_new[0],q_new[1],q_new[2],q_new[3])
        #self.pose.pose.orientation.w = self.pose.pose.orientation.w*math.cos(np.pi/2)
        #self.pose.pose.orientation.z = self.pose.pose.orientation.z*math.sin(np.pi/2)
        
        x_y = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.x = x_y[0] - self.first_pose.pose.position.x
        self.pose.pose.position.y = x_y[1] - self.first_pose.pose.position.y
        self.pose.pose.position.z = 0
        self.pub_origin_pose.publish(self.origin_pose)
        self.pub_pose.publish(self.pose)
        print(self.pose.pose.position.x, self.pose.pose.position.y)
        print("Finish")


if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("coordinate",anonymous=False)
    gazebo_pure_pursuit = gazebo_pure_pursuit()
    rospy.spin()