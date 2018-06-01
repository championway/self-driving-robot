/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <iostream> 
using namespace gps_common;

static ros::Publisher odom_pub;
ros::Publisher marker_pub;
std::string frame_id, child_frame_id;
double rot_cov;
visualization_msgs::Marker line_list;
bool first = true;
float origin_x, origin_y, origin_z;
geometry_msgs::Point last_pos;


void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  ROS_INFO("********");
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }
  ROS_INFO("---------");
  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
  std::cout << "northing: " << northing;
  std::cout << "easting: " << easting;
  std::cout << "zone: " << zone;
  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;
    line_list.header.frame_id = odom.header.frame_id;
    line_list.header.stamp = odom.header.stamp;
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.scale.y = 0.1;
    line_list.color.g = 1.0f;
    line_list.color.a = 1.0;
    //odom.child_frame_id = child_frame_id;
    odom.child_frame_id = "/map";
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    if (first){
      origin_x = easting;
      origin_y = northing;
      origin_z = fix->altitude;
      p1.x = 0;
      p1.y = 0;
      p1.z = 0;
      first = false;

    }
    else{
      p1.x = last_pos.x;
      p1.y = last_pos.y;
      p1.z = last_pos.z;
    }
    p2.x = easting - origin_x;
    p2.y = northing - origin_y;
    p2.z = fix->altitude - origin_z;
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    last_pos.x = p2.x;
    last_pos.y = p2.y;
    last_pos.z = p2.z;

    odom.pose.pose.position.x = easting - origin_x;
    odom.pose.pose.position.y = northing - origin_y;
    odom.pose.pose.position.z = fix->altitude - origin_z;
    
    odom.pose.pose.orientation.x = 1;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;
    marker_pub.publish(line_list);
    odom_pub.publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  ROS_INFO("---------");
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);
  marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //ros::Subscriber fix_sub = node.subscribe("fix", 1, callback);
  ros::Subscriber sub = node.subscribe<sensor_msgs::NavSatFix> ("fix", 1, callback);
  ros::spin();
}

