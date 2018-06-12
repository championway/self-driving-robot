#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Path
import numpy as np
import math
import tf

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class gazebo_pure_pursuit():
    def __init__(self):
        # Init attributes
        self.default_speed = 2
        self.speed = self.default_speed
        self.steering_angle = 0
        self.robot_length = 0.22
        self.robot_pose = None#(-0.3, -0.1789, -0.0246)
        self.destination_pose = None
        self.stop_point = None
        self.waypoints = rospy.get_param('~path')
        #self.waypoints = [(0, 0),(2,2),(-1,1),(-3,3),(-3,0),(1,-2),(0,0)]
        self.current_waypoint_index = 0
        self.distance_from_path = None
        self.lookahead_distance = rospy.get_param("~lookahead")
        #self.lookahead_distance_adjust = self.lookahead_distance
        self.threshold_proximity = 0.2      # How close the robot needs to be to the final waypoint to stop driving
        self.active = True
        self.start = True
        # Init subscribers and publishers
        self.pub_gazebo = rospy.Publisher('/david/cmd_vel', Twist, queue_size=1)
        self.pub_lookahead = rospy.Publisher('/pure_pursuit/lookahead', Point, queue_size=1)
        self.pub_finish = rospy.Publisher('/pure_pursuit/finished', Bool, queue_size=1)
        while True:
            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                model_state = get_model_state('bot',"")
                msg = model_state
                #print msg
            except rospy.ServiceException, e:
                pass
            if not self.active:
                #break
                return 
            finish = Bool()
            finish.data = False
            self.pub_finish.publish(finish)
            quaternion_msg = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(quaternion_msg)
            #print euler[2]
            #print
            self.robot_pose = (msg.pose.position.x, msg.pose.position.y, euler[2])
            # print self.robot_pose

            #print 'robot_pose:', self.robot_pose#, 'waypoints:', self.waypoints
            self.destination_pose = self.pure_pursuit()

            if self.destination_pose == None:
                self.active = False
                print "approach destination  "
                self.gazebo_cmd(0,0)
                msg = Bool()
                msg.data = True
                self.pub_finish.publish(msg)

            else:
                self.publish_lookhead(self.destination_pose)
                self.speed = self.default_speed
                distance_to_destination= self.getDistance(self.robot_pose, self.destination_pose)
                angle_to_destination = -self.getAngle(self.robot_pose, self.destination_pose)       
                # self.steering_angle = np.arctan((2 * self.robot_length * np.sin(angle_to_destination)) / distance_to_destination)         
                # print "robot_head",euler[2]*180/math.pi,"angle_to_destination", angle_to_destination*180/math.pi
                #print angle_to_destination
                w = 10*((angle_to_destination + math.pi) / (2 * math.pi) - 0.5)
                '''if w > 0:
                    w = 1
                else:
                    w = -1'''
                #print angle_to_destination
                self.gazebo_cmd(self.speed,w)
    def publish_lookhead(self, lookahead):
        msg = Point()
        msg.x, msg.y = lookahead[:2]
        msg.z = 0
        self.pub_lookahead.publish(msg)

    def gazebo_cmd(self, v, w):
        model_state_msg = Twist()
        model_state_msg.linear.x = v
        model_state_msg.linear.y = 0
        model_state_msg.linear.z = 0

        model_state_msg.angular.x = 0
        model_state_msg.angular.y = 0
        model_state_msg.angular.z = w
        
        self.pub_gazebo.publish(model_state_msg)

    # calculate the distance between two points
    def distanceBtwnPoints(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # tell if the point(x,y) is on the line segment(x_start,y_start)-->(x_end,y_end)
    def isPointOnLineSegment(self, x, y, x_start, y_start, x_end, y_end):
        return round(self.distanceBtwnPoints(x_start, y_start, x, y) + self.distanceBtwnPoints(x, y, x_end, y_end), 4) == round(self.distanceBtwnPoints(x_start, y_start, x_end, y_end), 4)

    def getDistance(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns distance between them in map units
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]

        distance = np.sqrt([delta_x**2 + delta_y**2])
        return distance[0]

    def getAngle(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns angle between them relative to the front of the car in degrees
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]
        #print delta_x
        #rad_to_deg_conv = 180.0 / np.pi
        theta = start_pose[2] #* rad_to_deg_conv
        #print theta
        psi = np.arctan2(delta_y, delta_x) #* rad_to_deg_conv
        # print "between_angle", between_angle*180/math.pi
        #print theta-psi
        angle = theta - psi
        while angle > np.pi:
            angle = angle - 2*np.pi
        while angle < -np.pi:
            angle = angle + 2*np.pi
        return angle

    # Find a point on the line which is closest to the point(robot poisition)
    def closestPoint(self, point, start, end):
        # Initialize values
        x = float(point[0])
        y = float(point[1])
        x_start = float(start[0])
        y_start = float(start[1])
        x_end = float(end[0])
        y_end = float(end[1])
        x_closest, y_closest = None, None
        shortest_distance = self.distanceBtwnPoints(x, y, self.waypoints[self.current_waypoint_index][0], self.waypoints[self.current_waypoint_index][1])

        # ======== Distance from a point to a line ========
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        # For line (segment) equation ax + by + c = 0
        a = y_start - y_end
        b = x_end - x_start
        if a**2 + b**2 == 0:
            return (None, None, None)
        c = -b*y_start - a*x_start  # Equivalently: x_start*y_end - x_end*y_start
        x_closest = (b*(b*x - a*y) - a*c)/(a**2 + b**2)
        y_closest = (a*(-b*x + a*y) - b*c)/(a**2 + b**2)
        distance = self.distanceBtwnPoints(x, y, x_closest, y_closest)
        if not self.isPointOnLineSegment(x_closest, y_closest, x_start, y_start, x_end, y_end):
            return (None, None, None)
        return (x_closest, y_closest, distance)

    # Using lookahead to find out the waypoint
    def circleIntersect(self, point, start, end, lookahead_distance):
        # For circle equation (x - p)^2 + (y - q)^2 = r^2
        p = float(point[0])
        q = float(point[1])
        r = float(lookahead_distance)
        # Check line segments along path until intersection point is found or we run out of waypoints
        # For line (segment) equation y = mx + b
        x1 = float(start[0])
        y1 = float(start[1])
        x2 = float(end[0])
        y2 = float(end[1])
        #print x1, y1, x2, y2
        if x2 - x1 != 0:
            m = (y2 - y1)/(x2 - x1)
            b = y1 - m*x1

            # Quadratic equation to solve for x-coordinate of intersection point
            A = m**2 + 1
            B = 2*(m*b - m*q - p)
            C = q**2 - r**2 + p**2 - 2*b*q + b**2

            if B**2 - 4*A*C < 0:    # Circle does not intersect line
                #print "no solution"
                #print(self.distanceBtwnPoints(p, q, x1, y1))
                #self.lookahead_distance_adjust = self.lookahead_distance_adjust + 0.03
                #print start, " ", end
                return (None, None)
                #return self.circleIntersect(self.robot_pose, start, end, self.distanceBtwnPoints(p, q, self.waypoints[self.current_waypoint_index-1][0], self.waypoints[self.current_waypoint_index-1][1]))
            # Points of intersection (could be the same if circle is tangent to line)
            x_intersect1 = (-B + math.sqrt(B**2 - 4*A*C))/(2*A)
            x_intersect2 = (-B - math.sqrt(B**2 - 4*A*C))/(2*A)
            y_intersect1 = m*x_intersect1 + b
            y_intersect2 = m*x_intersect2 + b
        else:
            x_intersect1 = x1
            x_intersect2 = x1
            y_intersect1 = q - math.sqrt(abs(-x1**2 + 2*x1*p - p**2 + r**2))
            y_intersect2 = q + math.sqrt(abs(-x1**2 + 2*x1*p - p**2 + r**2))

        # See if intersection points are on this specific segment of the line
        if self.isPointOnLineSegment(x_intersect1, y_intersect1, x1, y1, x2, y2):
            #rospy.loginfo('is returning')
            #self.lookahead_distance_adjust = self.lookahead_distance
            return (x_intersect1, y_intersect1)
        elif self.isPointOnLineSegment(x_intersect2, y_intersect2, x1, y1, x2, y2):
            #rospy.loginfo('is returning2')
            #self.lookahead_distance_adjust = self.lookahead_distance
            return (x_intersect2, y_intersect2)
        #print x_intersect1, " ", x_intersect2
        #print y_intersect2, " ", y_intersect2
        #self.lookahead_distance_adjust = self.lookahead_distance_adjust - 0.03
        #return self.circleIntersect(self.robot_pose, start, end, self.distanceBtwnPoints(p, q, self.waypoints[self.current_waypoint_index-1][0], self.waypoints[self.current_waypoint_index-1][1]))
        #print x_intersect1, y_intersect1, x_intersect2, y_intersect2
        return (None, None)

    def pure_pursuit(self):
        x_robot, y_robot = self.robot_pose[:2]
        wp = self.waypoints
        cwpi = self.current_waypoint_index
        fake_robot_waypoint = (None, None)
        #print cwpi, " ", len(wp)-1
        if len(wp) == 0:
            print "No target waypoint"
            return None
            # if there is only one waypoint left
        elif cwpi == len(wp)-1:
            if self.threshold_proximity < self.lookahead_distance:
                self.lookahead_distance = self.threshold_proximity
                print "change threshold distance for safty"
            x_endpoint, y_endpoint = wp[-1]
            #print "last one"
            if self.distanceBtwnPoints(x_endpoint, y_endpoint, x_robot, y_robot) <= self.threshold_proximity:
                return None
        if self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) <= self.lookahead_distance :
            print "Arrived waypoint : %d"%(cwpi)
            if self.current_waypoint_index < len(self.waypoints)-1:
                self.current_waypoint_index = self.current_waypoint_index + 1
            else:
                return None
            if self.start:
                self.start = False
        if self.start:
            fake_robot_waypoint = (x_robot, y_robot)
        else:
            #i = self.closestPoint(self.robot_pose, wp[cwpi-1], wp[cwpi])
            #print (len(i), " ", wp[cwpi-1], wp[cwpi])
            #fake_robot_waypoint = i[:2]
            fake_robot_waypoint = self.closestPoint(self.robot_pose, wp[cwpi-1], wp[cwpi])[:2]
            if fake_robot_waypoint == (None, None):
                #print "nonononono"
                fake_robot_waypoint = (wp[cwpi-1][0], wp[cwpi-1][1])
        # insert new fake waypoint, and remove waypoints which have been visited
        waypoints_to_search = [fake_robot_waypoint] + wp[cwpi : ]

        if self.lookahead_distance < self.distance_from_path:
            x_intersect, y_intersect = self.circleIntersect(self.robot_pose, waypoints_to_search[0], waypoints_to_search[1], self.distance_from_path)
        else:
            x_intersect, y_intersect = self.circleIntersect(self.robot_pose, waypoints_to_search[0], waypoints_to_search[1], self.lookahead_distance)
        if (x_intersect, y_intersect) == (None, None):
            if self.start:
                if self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) <= self.lookahead_distance :
                    print "Arrived waypoint : %d"%(cwpi)
                    self.current_waypoint_index = self.current_waypoint_index + 1
                    self.start = False
            #rospy.loginfo("Oh-No")
            #rospy.loginfo(self.current_waypoint_index)
            return fake_robot_waypoint
        #print self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) - self.lookahead_distance
        '''if round(x_intersect,3) == round(wp[cwpi][0], 3) and round(y_intersect, 3) == round(wp[cwpi][1], 3) or self.distanceBtwnPoints(x_robot, y_robot, wp[cwpi][0], wp[cwpi][1]) <= self.lookahead_distance :
            print "Arrived waypoint : %d"%(cwpi)
            if self.current_waypoint_index < len(self.waypoints)-1:
                self.current_waypoint_index = self.current_waypoint_index + 1
            else:
                return None
            if self.start:
                self.start = False'''
        return (x_intersect, y_intersect)

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("gazebo_pure_pursuit",anonymous=False)
    gazebo_pure_pursuit = gazebo_pure_pursuit()
    rospy.spin()