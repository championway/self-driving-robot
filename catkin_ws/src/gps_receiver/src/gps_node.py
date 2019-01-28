#!/usr/bin/env python
import serial
import sys
import time
import calendar
import rospy
from sensor_msgs.msg import NavSatFix

# Convert Saxagesimal to decimal
def Sexagesimal2Decimal(data_in):
	length = len(data_in)
	# 6 bit min + 1 bit dot
	data_out = float(data_in[0 :length-7])
	min_ = float(data_in[length-7 :length])
	data_out += min_ / 60.
	return data_out

# Convert HHMMSS to time.time() format
def ToTimeFormat(in_str):
	utc_struct = time.gmtime()
	utc_list = list(utc_struct)
	hours   = int(in_str[0:2])
	minutes = int(in_str[2:4])
	seconds = int(in_str[4:6])
	utc_list[3] = hours
	utc_list[4] = minutes
	utc_list[5] = seconds
	return calendar.timegm(tuple(utc_list))

class GPSHandler(object):
	def __init__(self, port):
		self.node_name = rospy.get_name()
		self.port = port
		self.info = ""
		self.info_list = None
		self.seq = 0
		self.pub_fix = rospy.Publisher("fix", NavSatFix, queue_size = 10)
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
	def read_info(self, event):
		# Read one line 
		self.info = self.port.readline()
		# Split with ','
		self.info_list = self.info.split(',')
		if self.info_list[0] == '$GPGGA':
			self._gps_pub()
	def _gps_pub(self):
		if self.info_list[2] == '':
			rospy.loginfo("[%s] No data receiving ... Try go outside?" %(self.node_name))
			return
		# info format:
		# GPGGA UTC_time latitude N/S longitude E/W state  number_of_sat HDOP altitude others		
		fix = NavSatFix()
		fix.header.seq = self.seq
		# Use received data
		#fix.header.stamp = ToTimeFormat(self.info_list[1])
		# Use rospy.Time.now
		fix.header.stamp = rospy.Time.now()
		fix.header.frame_id = 'gps_'
		lat = float(Sexagesimal2Decimal(self.info_list[2]))
		longi = float(Sexagesimal2Decimal(self.info_list[4]))
		if self.info_list[3] == 'S':
			fix.latitude = -lat
		else:
			fix.latitude = lat

		if self.info_list[5] == 'W':
			fix.longitude = -longi
		else:
			fix.longitude = longi

		rospy.loginfo("[%s] %s %s %s %s" %(self.node_name, self.info_list[3], 
						   lat, self.info_list[5],
						   longi))
		# Covariance matrix
		# Refer to nmea_navsat_driver
		try:
			hdop = float(self.info_list[8])
		except ValueError:
			pass
		fix.position_covariance[0] = hdop ** 2
		fix.position_covariance[4] = hdop ** 2
		fix.position_covariance[8] = (2* hdop) ** 2
		fix.position_covariance_type = \
			NavSatFix.COVARIANCE_TYPE_APPROXIMATED
		fix.altitude = float(self.info_list[9]) + float(self.info_list[11])
		self.pub_fix.publish(fix)
		self.info = ""
		self.info_list = None
		self.seq += 1
if __name__ == "__main__":
	rospy.init_node("gps_node", anonymous = False)
	# Parameters
	port = rospy.get_param("~port", '/dev/ttyUSB0')
	flush_size = rospy.get_param("~flush_size", 20)
	try:
		gps_port = serial.Serial(port, 4800, timeout = 1)
	except:
		rospy.loginfo("[%s] Cann't find port: %s" %(rospy.get_name(), port))
		sys.exit(0)
	gps = GPSHandler(gps_port)
	# Flush first data
	rospy.loginfo("[%s] Flush first %s data" %(rospy.get_name(), flush_size))
	for i in range(0, flush_size):
		gps_port.readline()
	rospy.loginfo("[%s] Start to publish gps data" %(rospy.get_name()))
	rospy.Timer(rospy.Duration.from_sec(0.1), gps.read_info)
	rospy.spin()
