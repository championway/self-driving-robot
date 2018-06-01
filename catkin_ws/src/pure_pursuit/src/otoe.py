#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
import tf
import sys
import time
import threading
#from geometry_msgs.msg import pose

#pose.orientation.x = 0.000050324149
#pose.orientation.y = 0.0001345798
#pose.orientation.z = 0.606972714372
#pose.orientation.w = 0.794722658141
x = 0.0000824012931207
y = 0.0000285184803944
z = 0.963317311079
w = -0.268364957797

quaternion = (x, y, z, w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print roll
print pitch
print yaw