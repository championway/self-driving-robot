#!/usr/bin/env python
import roslib
roslib.load_manifest('bot')

import sys

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseArray, Pose

def get_location():

    rospy.init_node('get_robot_position')
    g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.wait_for_service("/gazebo/get_model_state")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:        
          state = g_get_state(model_name="bot")
            
        except Exception, e:  
          rospy.logerr('Error on calling service: %s',str(e))
          return

        pub_vehicle_pose_pair = rospy.Publisher("~vehicle_pose_pair", PoseArray, queue_size=1)
        vehicle_pose_pair_msg = PoseArray()

        vehicle_pose = Pose()                                       #ok
        vehicle_pose.position.x = state.pose.position.x             #ok
        vehicle_pose.position.y = state.pose.position.y             #ok    
        vehicle_pose.position.z = state.pose.position.z             #ok
        vehicle_pose.orientation.x = state.pose.orientation.x
        vehicle_pose.orientation.y = state.pose.orientation.y
        vehicle_pose.orientation.z = state.pose.orientation.z
        vehicle_pose.orientation.w = state.pose.orientation.w
        vehicle_pose_pair_msg.poses.append(vehicle_pose)
        pub_vehicle_pose_pair.publish(vehicle_pose_pair_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        get_location()
    except rospy.ROSInterruptException:
        pass