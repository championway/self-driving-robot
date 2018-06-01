#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from robotx_msgs.msg import ObstaclePose, ObstaclePoseList
import math
from visualization_msgs.msg import Marker
from tf import TransformListener,TransformerROS

class map():
    def __init__(self):
        self.node_name = "Mocap Localization" 
        self.global_map = ObstaclePoseList()
        self.first = True   # process first time
        self.radius = 1   # to check whether the measurement obstacle is correspond to global map or not
        self.confi_threshold = 5 # confidence threshold, to determine whether to add point to map or not
        self.tf = TransformListener()
        self.transformer = TransformerROS()
        # Subscribers
        self.obstacle_list = rospy.Subscriber("/obstacle_list", ObstaclePoseList, self.call_back, queue_size=1)
        # Publishers
        self.rviz_pub = rospy.Publisher("/map_viz", Marker, queue_size=1)
        self.map_list = rospy.Publisher("/map_list", ObstaclePoseList, queue_size=1)

    def draw(self, ob_list):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        for i in range(0, ob_list.size):
            p = Point()
            p.x = ob_list.list[i].x
            p.y = ob_list.list[i].y
            p.z = ob_list.list[i].z
            marker.points.append(p)
        self.rviz_pub.publish(marker)

    def distance(self, a, b): # caculate distance between two 3d points
        return math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z))

    def update_obs_pos(self, measurement, prior): # update obstacle position by weighting measurement and original map
        posterior = ObstaclePose()
        posterior = prior
        posterior.x = measurement.x*0.3 + prior.x*0.7
        posterior.y = measurement.y*0.3 + prior.y*0.7
        posterior.z = measurement.z*0.3 + prior.z*0.7
        return posterior

    def call_back(self, msg):
        position, quaternion = self.tf.lookupTransform("/map", "/velodyne", rospy.Time(0))
        transpose_matrix = self.transformer.fromTranslationRotation(position, quaternion)
        
        #print type(position)
        #print type(quaternion)
        print ("Process Obstacle List")
        obs_list = ObstaclePoseList()
        obs_list = msg
        for i in range(0, obs_list.size):
            origin_p = np.array([obs_list.list[i].x, obs_list.list[i].y, obs_list.list[i].z, 1])
            new_p = np.dot(transpose_matrix, origin_p)
            obs_list.list[i].x = new_p[0]
            obs_list.list[i].y = new_p[1]
            obs_list.list[i].z = new_p[2]

        if self.first:  # record the first detection to be global map
            self.global_map = obs_list
            for i in range(0, self.global_map.size):
                self.global_map.list[i].confidence = 0
            self.first = False

        for i in range(0, self.global_map.size):
            self.global_map.list[i].occupy = False

        for i in range(0, obs_list.size):
            min_dis = 10000
            index = None
            for j in range(0, self.global_map.size):
                if not self.global_map.list[j].occupy:  # check whether the point has already been corresponded to another point
                    dis = self.distance(obs_list.list[i], self.global_map.list[j])
                    if dis < min_dis:
                        index = j
                        min_dis = dis
            if min_dis < self.radius:   # believe that measurement[i] is corresponded to the map[index]
                self.global_map.list[index].occupy = True
                self.global_map.list[index] = self.update_obs_pos(obs_list.list[i], self.global_map.list[index])
                if self.global_map.list[index].confidence < self.confi_threshold:
                    self.global_map.list[index].confidence = self.global_map.list[index].confidence + 1
            else:
                obs_list.list[i].confidence = 0
                self.global_map.list.append(obs_list.list[i])
                self.global_map.size = self.global_map.size + 1
        map_confi = ObstaclePoseList()
        map_confi.header = self.global_map.header
        for i in range(0, self.global_map.size):
            if self.global_map.list[i].confidence >= self.confi_threshold:
                map_confi.list.append(self.global_map.list[i])
                map_confi.size = map_confi.size + 1
        self.draw(map_confi)
        self.map_list.publish(map_confi)

    def onShutdown(self):
        rospy.loginfo("[Global_Map] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('map',anonymous=False)
    global_map = map()
    rospy.on_shutdown(map.onShutdown)
    rospy.spin()