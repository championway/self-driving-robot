#!/usr/bin/env python
import rospy
import tf
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

global x, y, theta, v_L, v_R, v_x, v_y, omega

x = 0
y = 0
theta = 0
v_L = 0
v_R = 0
v_x = 0
v_y = 0
omega = 0
pub_tf = False # Use estimate result as tf
if(pub_tf):
	br = tf.TransformBroadcaster()

def read_data(event):
	pub_odom = rospy.Publisher("/odom", Odometry, queue_size = 1)
	global str_
	str_ = str('')
	seq = 0
	while ard.inWaiting():
		str_ = ard.readline()
	split_str = str_.split(' ')
	if len(split_str) != 8:
		global x, y, theta
		if(pub_tf):
			br.sendTransform((x, y, 0),
				 	tf.transformations.quaternion_from_euler(0, 0, theta),
				 	rospy.Time.now(),
				 	'odom',
				 	'map')
		odom = Odometry()
		odom.header.seq = seq
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.pose.pose.position = Point(x, y, 0.0)
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]
		odom.pose.covariance[0] = 0.2 # X
		odom.pose.covariance[7] = 0.2 # Y
		odom.pose.covariance[35] = 0.05 # Theta
		seq = seq + 1
		pub_odom.publish(odom)
	else:
		try:
			x 	= float(split_str[0])
			y 	= float(split_str[1])
			theta   = float(split_str[2])
			v_L     = float(split_str[3])
			v_R     = float(split_str[4])
			v_x     = float(split_str[5])
			v_y     = float(split_str[6])
			omega   = float(split_str[7])
			if(pub_tf):
				br.sendTransform((x, y, 0),
				 		tf.transformations.quaternion_from_euler(0, 0, theta),
				 		rospy.Time.now(),
				 		'base_link',
				 		'odom')
			odom = Odometry()
			odom.header.seq = seq
			odom.header.stamp = rospy.Time.now()
			odom.header.frame_id = "odom"
			odom.child_frame_id = "base_link"
			odom.pose.pose.position = Point(x, y, 0.0)
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
			odom.pose.pose.orientation.x = odom_quat[0]
			odom.pose.pose.orientation.y = odom_quat[1]
			odom.pose.pose.orientation.z = odom_quat[2]
			odom.pose.pose.orientation.w = odom_quat[3]
			odom.twist.twist.linear.x = v_x
			odom.twist.twist.linear.y = v_y
			odom.twist.twist.angular.z = omega
			pub_odom.publish(odom)
			print("x: ", x,", y: " , y, ", theta: ", theta)
			seq = seq + 1
		except ValueError:
			pass

if __name__ == '__main__':
	rospy.init_node('whel_odom_node', anonymous = False)
	port = rospy.get_param("~port", "/dev/ttyACM0") # default port: /dev/ttyUSB0
	ard = serial.Serial(port, 9600)
	rospy.Timer(rospy.Duration.from_sec(0.1), read_data) # 10Hz
	rospy.spin()
