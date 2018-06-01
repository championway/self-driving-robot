#!/usr/bin/python
import rospy
import time
import math
import numpy
import tf
from Adafruit_LSM303 import Adafruit_LSM303
from Gyro_L3GD20 import Gyro_L3GD20

class AdafruitIMU(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.compass_accel = Adafruit_LSM303()
		self.gyro = Gyro_L3GD20()
		self.roll_past = 0
		self.pitch_past = 0
		self.yaw_past = 0
		self.compl_coeff = 0.97
		self.pub_timestep = self.setupParam("~pub_timestep", 0.02)
		self.br = tf.TransformBroadcaster()
		self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
		self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.publish)
		rospy.loginfo("[%s] Initialized..." %(self.node_name))
	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value
	def publish(self, event):
		compass_accel = self.compass_accel.read()
		compass = compass_accel[1]
		accel = compass_accel[0]
		gyro = self.gyro.read()
		acc_x = accel[0]
		acc_y = accel[1]
		acc_z = accel[2]
		roll = math.atan2(acc_y, acc_z)
		pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
		# Complementary filter
		roll = self.compl_coeff * roll + (1 - self.compl_coeff) * self.roll_past
		pitch = self.compl_coeff * pitch + (1 - self.compl_coeff) * self.pitch_past
		mag_x_tilt = compass[0] * math.cos(pitch) + compass[1] * math.sin(pitch)
		mag_y_tilt = compass[0] * math.sin(roll) * math.sin(pitch) + compass[2] * math.cos(roll) - compass[1] * math.sin(roll) * math.cos(pitch)
		yaw = math.atan2(mag_y_tilt, mag_x_tilt)
		yaw = self.compl_coeff * yaw + (1- self.compl_coeff) * self.yaw_past
		# Update
		self.roll_past = roll
		self.pitch_past = pitch
		self.yaw_past = yaw
		# compass[0] > mag_x
		# compass[1] > mag_z
		# compass[2] > mag_y
		#yaw = math.atan2(compass[2], compass[0])
		q = quaternion_from_euler(roll, pitch, yaw)
		imuMsg.orientation.x = q[0]
		imuMsg.orientation.y = q[1]
		imuMsg.orientation.z = q[2]
		imuMsg.orientation.w = q[3]
		imuMsg.header.stamp= rospy.Time.now()
		imuMsg.header.frame_id = 'imu'
		imuMsg.header.seq = seq
		seq = seq + 1
		self.pub_imu.publish(imuMsg)
		self.br.sendTransform((0, 0, 0),
					tf.transformations.quaternion_from_euler(roll, pitch, yaw),
					rospy.Time.now(),
					"IMU",
					"world")
		rospy.loginfo("[%s] Roll: %f, Pitch: %f, Yaw: %f" %(self.node_name, roll, pitch, yaw))
	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %(self.node_name))
if __name__ == "__main__":
	rospy.init_node("Adafruit_IMU_test", anonymous = False)
	Adafruit_IMU = AdafruitIMU()
	rospy.on_shutdown(Adafruit_IMU.on_shutdown)
	rospy.spin()