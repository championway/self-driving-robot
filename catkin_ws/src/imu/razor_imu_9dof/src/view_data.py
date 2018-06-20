#!/usr/bin/env python

import time
import rospy
import tf
from math import degrees
from sensor_msgs.msg import Imu

count = 0
sum_az = 0

def cb_imu(msg):
	global count, sum_az
	count += 1 
	a_z = msg.linear_acceleration.z
	sum_az += a_z
	avg_az = sum_az / count
	quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quat)
	r = degrees(euler[0])
	p = degrees(euler[1])
	y = degrees(euler[2])
	print '{0:5f} {1:5f} {2:5f} {3:5f}'.format(avg_az, r, p, y)
		
if __name__ == '__main__':
	rospy.init_node('imu_listener_node', anonymous = False)
	rospy.Subscriber('/imu', Imu, cb_imu, queue_size = 50)
	start = time.time()
	rospy.spin()
