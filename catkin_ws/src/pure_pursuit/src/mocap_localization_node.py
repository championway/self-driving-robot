#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
from duckietown_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseArray, Pose


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = "Mocap Localization" 

        # base tag id      
        self.base_tag_id = [191, 192, 193]
        # vehicle tag id    
        self.vehicle_tag_id = [194, 195]

        # base tag groundtruth point
        self.base_tag_point = np.array([[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0]], dtype='f')
        # base tag detection point     
        self.obser_tag_point = np.zeros((3, 3), dtype='f')
        self.test_tag_point = np.zeros((3, 4), dtype='f')

        # vehicle tag detection point
        self.vehicle_tag_point_pair = np.zeros((2, 4), dtype='f')

        # legal or illegal localization
        self.tag_detect_count = 0

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=1)
        # Publishers
        self.pub_vehicle_pose_pair = rospy.Publisher("~vehicle_pose_pair", PoseArray, queue_size=1)

    def processTagDetections(self,tag_detections_msg):
        print "-----------------------------------------------"
        # assign base tag coordination
        self.base_tag_point = np.array([[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0]], dtype='f')
        for tag_detection in tag_detections_msg.detections:
        	# extract base tag detection
            for index, tag_id in enumerate(self.base_tag_id):
                if tag_detection.id == tag_id:
                    self.obser_tag_point[index, 0] = tag_detection.pose.pose.position.x
                    self.obser_tag_point[index, 1] = tag_detection.pose.pose.position.y                   
                    self.obser_tag_point[index, 2] = tag_detection.pose.pose.position.z
                    self.test_tag_point[index, 0] = tag_detection.pose.pose.position.x
                    self.test_tag_point[index, 1] = tag_detection.pose.pose.position.y                   
                    self.test_tag_point[index, 2] = tag_detection.pose.pose.position.z
                    self.test_tag_point[index, 3] = 1
                    self.tag_detect_count += 1
            # extract vehicle tag detection
            for index, tag_id in enumerate(self.vehicle_tag_id):
                if tag_detection.id == tag_id:
                    self.vehicle_tag_point_pair[index, 0] = tag_detection.pose.pose.position.x
                    self.vehicle_tag_point_pair[index, 1] = tag_detection.pose.pose.position.y                   
                    self.vehicle_tag_point_pair[index, 2] = tag_detection.pose.pose.position.z
                    self.vehicle_tag_point_pair[index, 3] = 1 
                    self.tag_detect_count += 1 
        if(self.tag_detect_count != 5):
            self.tag_detect_count = 0
            return
        self.tag_detect_count = 0

        p_ct = (self.base_tag_point[0] + self.base_tag_point[1] + self.base_tag_point[2])/3
        p_cm = (self.obser_tag_point[0] + self.obser_tag_point[1] + self.obser_tag_point[2])/3

        for i in range(3):
            self.obser_tag_point[i] = self.obser_tag_point[i] - p_cm
            self.base_tag_point[i] = self.base_tag_point[i] - p_ct

        Mtd = np.vstack((self.base_tag_point[0], self.base_tag_point[1], self.base_tag_point[2])).transpose()
        Mmd = np.vstack((self.obser_tag_point[0], self.obser_tag_point[1], self.obser_tag_point[2])).transpose()

        H = np.dot(Mmd, Mtd.transpose())
        [U, D, V] = np.linalg.svd(H,full_matrices=1)
        R = np.dot(V,U.transpose())
        t = np.matrix(p_ct - np.dot(R,p_cm))

        temp = np.hstack((R,t.transpose()))
        zero = np.array([[0,0,0,1]])
        T = np.vstack((temp,zero))

        print "tag detection"
        print self.test_tag_point.transpose()
        print "tag detection after transformation"
        print np.dot(T,self.test_tag_point.transpose())

        print "vehicle tag detection"
        print self.vehicle_tag_point_pair.transpose()
        print "vehicle tag detection after transformation"
        print np.dot(T,self.vehicle_tag_point_pair.transpose())

        vehicle_point_pair = np.zeros((2, 4), dtype='f')
        vehicle_point_pair = np.dot(T,self.vehicle_tag_point_pair.transpose())
        vehicle_pose_pair_msg = PoseArray()
        for i in range(2):
            vehicle_pose = Pose()
            vehicle_pose.position.x = vehicle_point_pair[0,i]
            vehicle_pose.position.y = vehicle_point_pair[1,i]           
            vehicle_pose.position.z = vehicle_point_pair[2,i]
            vehicle_pose_pair_msg.poses.append(vehicle_pose)
        self.pub_vehicle_pose_pair.publish(vehicle_pose_pair_msg)

    def onShutdown(self):
        rospy.loginfo("[MocapLocalizationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('mocap_localization_node',anonymous=False)
    mocap_localization_node = MocapLocalizationNode()
    rospy.on_shutdown(mocap_localization_node.onShutdown)
    rospy.spin()
