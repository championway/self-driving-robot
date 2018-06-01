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

if __name__ == '__main__':
    #init publisher
    rospy.init_node('gps_info_pub')
    gpspub = rospy.Publisher('/fix', NavSatFix,queue_size=1)
    port = serial.Serial("/dev/ttyUSB0", baudrate=4800, timeout=1)
    #rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            rcv = readlineCR(port)
            if rcv != None:
                print rcv
            #rate.sleep()
    except rospy.ROSInterruptException:
        pass