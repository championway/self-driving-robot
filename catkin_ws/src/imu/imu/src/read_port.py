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
    while True:
        ch = port.read()
        if ch == '\r' or ch == '':
            info = rv.split(',')
            return info
        elif ch == '\n':
            continue
        else:
            rv += ch
    return None

if __name__ == '__main__':
    #init publisher
    rospy.init_node('imu_info_pub')
    gpspub = rospy.Publisher('/fix', NavSatFix,queue_size=1)
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