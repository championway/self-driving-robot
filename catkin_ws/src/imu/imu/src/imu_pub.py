#!/usr/bin/env python
import serial
import time
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
GRAVITY = 256.0
def imu_pose(imuMsg):
    imu_PoseStamped = PoseStamped()
    imu_PoseStamped.header.stamp = imuMsg.header.stamp
    imu_PoseStamped.header.frame_id = "imu"
    imu_PoseStamped.pose = Pose(Point(0., 0., 0.), Quaternion(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w))
    pose_pub.publish(imu_PoseStamped)

def ros_imu(info):
    for index, item in enumerate(info):
        info[index] = float(item)
    imuMsg = Imu()
    imuMsg.orientation_covariance = [
    0.0025 , 0 , 0,
    0, 0.0025, 0,
    0, 0, 0.0025
    ]
    imuMsg.angular_velocity_covariance = [
    0.02, 0 , 0,
    0 , 0.02, 0,
    0 , 0 , 0.02
    ]
    imuMsg.linear_acceleration_covariance = [
    0.04 , 0 , 0,
    0 , 0.04, 0,
    0 , 0 , 0.04
    ]
    Accel_magnitude = math.sprt(info[0]*info[0]+info[1]*info[1]+info[2]*info[2])
    Accel_magnitude = float(Accel_magnitude / GRAVITY)
    pitch = -math.atan2(info[1],math.sqrt(info[2]*info[2]+info[3]*info[3]))
    roll = -math.atan2(info[2],info[3])
    xh = info[7]*math.cos(pitch) + info[8]*math.sin(roll)*math.sin(pitch) + info[9]*math.sin(pitch)*math.cos(roll)
    yh = info[8]*math.cos(roll) + info[9]*math.sin(roll)
    yaw = math.atan2(-yh,xh)
    seq=0
    #print roll
    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp = rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    imu_pose(imuMsg)
    gpspub.publish(imuMsg)

def readlineCR(port):
    rv = ""
    while True:
        ch = port.read()
        if ch == '\r' or ch == '':
            info = rv.split(',')
            ros_imu(info)
            return info
        elif ch == '\n':
            continue
        else:
            rv += ch
    return None

if __name__ == '__main__':
    #init publisher
    rospy.init_node('imu_info_pub')
    pose_pub = rospy.Publisher("/imu_pose", PoseStamped, queue_size=1)
    gpspub = rospy.Publisher('/imu', Imu,queue_size=1)
    port = serial.Serial("/dev/ttyACM0", baudrate=152000, timeout=1)
    #rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            #print port.read()
            rcv = readlineCR(port)
            if rcv != None:
                print rcv
            #rate.sleep()
    except rospy.ROSInterruptException:
        pass