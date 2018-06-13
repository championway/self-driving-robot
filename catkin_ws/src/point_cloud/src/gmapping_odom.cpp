#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pub_odom;
visualization_msgs::Marker path;
tf::TransformListener* lr;
ros::Time pcl_t;
nav_msgs::Odometry odom;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 tf_ros;
  sensor_msgs::PointCloud2 orig_ros;
  tf::StampedTransform trans;
  orig_ros = *input;
  pcl_t = input->header.stamp;
  try{
      //lr->waitForTransform("/velodyne", "/map", input->header.stamp, ros::Duration(10.0) );
      lr->lookupTransform("/map", "/velodyne", ros::Time(0), trans);
      //pcl_ros::transformPointCloud("viewed_tag_1", trans, orig_ros, tf_ros);
      //pcl_ros::transformPointCloud("/viewed_tag_1", *input, tf_ros, *lr);
      odom.header.stamp = ros::Time(0);
      odom.header.frame_id = "/odom";
      odom.pose.pose.position.x = trans.getOrigin().x();
      odom.pose.pose.position.y = trans.getOrigin().y();
      odom.pose.pose.position.z = trans.getOrigin().z();
      odom.pose.pose.orientation.x = trans.getRotation().getX();
      odom.pose.pose.orientation.y = trans.getRotation().getY();
      odom.pose.pose.orientation.z = trans.getRotation().getZ();
      odom.pose.pose.orientation.w = trans.getRotation().getW();
      pub_odom.publish(odom);
      //odometry();
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
    }
}

/*void odometry (MatrixXf pos)
{
    path.header.frame_id = "/odom";
    path.header.stamp = pcl_t;
    path.action = visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.id = 0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.scale.x = 0.05;
    path.scale.y = 0.05;
    path.color.g = 1.0;
    path.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = pos(0,0);
    p.y = pos(1,0);
    p.z = pos(2,0);
    path.points.push_back(p);
    rvis_publisher.publish(path);
}*/



int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "gmapping_odom");
     ros::NodeHandle nh;  
     tf::TransformListener listener(ros::Duration(10));
     lr = &listener;
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
     pub_odom = nh.advertise<nav_msgs::Odometry>("/gmapping_odom", 1);
     // Spin
     ros::spin ();
}