#!/usr/bin/env python
import serial
import time
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

def rv2NavSatFix(info):
    fix = NavSatFix()
    fix.latitude = float(info[3])
    if info[4] == 'S':
        fix.latitude = -fix.latitude
    fix.longitude = float(info[5])
    if info[6] == 'W':
        fix.longitude = -fix.longitude
    '''fix.status.status = NavSatStatus.STATUS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.altitude = float('NaN')
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN'''
    fix.header.frame_id = 'gps'
    fix.header.stamp = rospy.get_rostime()
    gpspub.publish(fix)

def readlineCR(port):
    rv = ""
    if port.read()=='$':
        while True:
            ch = port.read()
            rv += ch
            if ch=='\r' or ch=='':
                info = rv.split(',')
                if info[0] == 'GPRMC':
                    rv2NavSatFix(info)
                    return info
                else:
                    return None
    return None
def drawOdometry(self):
        #Draw hector slam odometry
        odom = Odometry()
        odom.header.stamp = self.wheel_state.time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = "base_link_2Dlidar"

        odom.pose.pose.position = Point(self.wheel_state.trans[0], self.wheel_state.trans[1], self.wheel_state.trans[2])
        odom.pose.pose.orientation = Quaternion(self.wheel_state.rot[0], self.wheel_state.rot[1], self.wheel_state.rot[2], self.wheel_state.rot[3])

        #print type(self.wheel_state.trans) 
        #print type(self.wheel_state.rot) 

        dt = self.wheel_state.time.secs - self.wheel_state_record.time.secs
        vx = (self.wheel_state.trans[0] - self.wheel_state_record.trans[0]) * dt
        vy = (self.wheel_state.trans[1] - self.wheel_state_record.trans[1]) * dt
        euler = tf.transformations.euler_from_quaternion(self.wheel_state.rot)
        euler_record = tf.transformations.euler_from_quaternion(self.wheel_state_record.rot)
        vth = (euler[2] - euler_record[2]) * dt

        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        self.pub_odom.publish(odom)
def drawOdometry(odom):
    try:
        driver.add_sentence(nmea_sentence.sentence, frame_id=nmea_sentence.header.frame_id, timestamp=nmea_sentence.header.stamp)
    except ValueError as e:
        rospy.logwarn("Value error, likely due to missing fields in the NMEA message. Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA sentences that caused it." % e)

if __name__ == '__main__':
    #init publisher
    rospy.init_node('vis')
    rospy.Subscriber("/odom", Odometry, drawOdometry)
    rospy.spin()