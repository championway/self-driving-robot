#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
//declare point cloud
PointCloudXYZ::Ptr cloud_XYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);

//declare ROS publisher
ros::Publisher pub_result;

//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::fromROSMsg (*input, *cloud_XYZ);
  copyPointCloud(*cloud_XYZ, *cloud_XYZRGB);
  //set color for point cloud
  for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++){
    if (cloud_XYZRGB->points[i].x < 0){
      cloud_XYZRGB->points[i].r = 255;
      cloud_XYZRGB->points[i].g = 0;
      cloud_XYZRGB->points[i].b = 0;
    }
    else{
      cloud_XYZRGB->points[i].r = 0;
      cloud_XYZRGB->points[i].g = 255;
      cloud_XYZRGB->points[i].b = 0;
    }
    
  }
  //point cloud clustering
  pub_result.publish(*cloud_XYZRGB);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_result = nh.advertise<PointCloudXYZRGB> ("/output", 1);
  // Spin
  ros::spin ();
}