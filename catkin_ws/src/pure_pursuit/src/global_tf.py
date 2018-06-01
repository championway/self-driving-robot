#!/usr/bin/env python  
import roslib
import rospy
import tf
import math
from time import time, sleep
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
class data_transfer():
    def __init__(self): 

        self.world = Odometry()
        self.world.pose.pose.orientation.w = 1
        self.odomFilter = Odometry()
        # Thread
        self.count = 0
        self.threadC = threading.Condition()    

        #rospy.Subscriber('/odometry/filtered',Odometry,self.cbOdom)
        rospy.Subscriber('/p3d_odom',Odometry,self.cbfake)

        self.br = tf.TransformBroadcaster()
        #self.t1.start()
        #self.t2.start()
        self.global_tf = 'world'
        self.model_tf = 'gazebo'

    def cbfake(self,msg):
        self.threadC.acquire()
        self.world.pose.pose.position.x = msg.pose.pose.position.x
        self.world.pose.pose.position.y = msg.pose.pose.position.y
        self.world.pose.pose.position.z = msg.pose.pose.position.z
        self.world.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.world.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.world.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.world.pose.pose.orientation.w = msg.pose.pose.orientation.w    
        self.br.sendTransform((self.world.pose.pose.position.x,self.world.pose.pose.position.y,self.world.pose.pose.position.z)\
        ,(self.world.pose.pose.orientation.x,self.world.pose.pose.orientation.y,self.world.pose.pose.orientation.z,self.world.pose.pose.orientation.w),rospy.Time.now(),self.model_tf,self.global_tf)
        self.threadC.release()

if __name__ == '__main__':
     rospy.init_node('odom_fake')
     foo = data_transfer()
     rospy.spin()