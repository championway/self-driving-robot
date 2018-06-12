#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
using namespace message_filters;
using namespace pcl;
//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef boost::shared_ptr <nav_msgs::Odometry const> OdometryConstPtr;

//declare point cloud
PointCloudXYZ::Ptr input_XYZ (new PointCloudXYZ);
sensor_msgs::PointCloud2 tf_ros;
//PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB); 
//PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);

//declare point cloud variable
PointCloudXYZRGB::Ptr map (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr scene (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);


//declare ROS publisher
ros::Publisher pub_result;

//declare global variable
bool first = true;
bool lock = false;
Eigen::Quaterniond q;
float position[3];
geometry_msgs::Pose old_pos;
geometry_msgs::Pose new_pos;
tf::TransformListener* lr;

//declare function
//void callback(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function

void icp(void);

void callback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
  std::cout << "start" << std::endl;
  tf::StampedTransform trans;
  lr->lookupTransform("/odom", pcl_msg->header.frame_id, pcl_msg->header.stamp, trans);
  pcl_ros::transformPointCloud("/odom", trans, *pcl_msg, tf_ros);
  //lr->waitForTransform("/odom", pcl_msg->header.frame_id, pcl_msg->header.stamp, ros::Duration(10.0) );
  //pcl_ros::transformPointCloud("/odom", *pcl_msg, tf_ros, *lr);
  std::cout<< "finish tf transform " << std::endl;

  pcl::fromROSMsg (tf_ros, *input_XYZ);
  copyPointCloud(*input_XYZ, *scene);
  for (size_t i = 0; i < scene->points.size(); i++){
    scene->points[i].r = 255;
    scene->points[i].g = 0;
    scene->points[i].b = 0; 
  }
  //new_pos = odom_msg->pose.pose;
  if(first)
  {
    copyPointCloud (*scene, *map);
    //old_pos = odom_msg->pose.pose;
    /*std::vector<int> a;
    pcl::removeNaNFromPointCloud(*result, *result, a);*/
    first = false;
  }
  /*q.x() = odom_msg->pose.pose.orientation.x;
  q.y() = odom_msg->pose.pose.orientation.y;
  q.z() = odom_msg->pose.pose.orientation.z;
  q.w() = odom_msg->pose.pose.orientation.w;

  position[0] = odom_msg->pose.pose.position.x;
  position[1] = odom_msg->pose.pose.position.y;
  position[2] = odom_msg->pose.pose.position.z;*/
  //odom_msg.pose.pose.position
  //odom_msg.pose.pose.orientation

  if(!lock)
  {
    /*try
    {
      //lr->waitForTransform("/viewed_tag_1", input->header.frame_id, ros::Time::now(), ros::Duration(10.0) );
      lr->waitForTransform("/odom", pcl_msg->header.frame_id, pcl_msg->header.stamp, ros::Duration(10.0) );
      //lr.lookupTransform("viewed_tag_1", orig_ros.header.frame_id, ros::Time(0), trans);
      //pcl_ros::transformPointCloud("viewed_tag_1", trans, orig_ros, tf_ros);
      pcl_ros::transformPointCloud("/odom", *pcl_msg, tf_ros, *lr);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
    }*/
    lock = true;
    icp();
    
  }
  //old_pos = new_pos;
}

void icp()
{
  //define ICP
  /*pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);*/

  //ICP initial transform matrix
  /*Eigen::Matrix4f init_align;
  Eigen::Vector3d p;
  //Eigen::MatrixXf p(3, 1);
  p << position[0], position[1], position[2];
  q.normalized();
  Eigen::Matrix3d rot_inv;
  Eigen::Matrix3d rot = q.toRotationMatrix();
  rot_inv = rot;
  //rot_inv = rot.inverse();
  //p = -rot_inv*p;
  init_align <<     rot_inv(0,0), rot_inv(0,1), rot_inv(0,2), p(0),
                    rot_inv(1,0), rot_inv(1,1), rot_inv(1,2), p(1),
                    rot_inv(2,0), rot_inv(2,1), rot_inv(2,2), p(2),
                           0,        0,        0,           1;

  pcl::transformPointCloud (*scene, *scene, init_align);

  /*init_align <<     0.707097,    -0.707121,  0.00031099,   -43.8569,
                    0.707125,       0.7071, 0.000705772,    36.1607,
                -0.000718911, -0.000279344,           1,  -0.990627,
                           0,            0,           0,          1; */

  std::cout<< "Start ICP" << std::endl;
  // Remove NaN point
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_input, *map_input, indices);
  std::vector<int> indices1;
  pcl::removeNaNFromPointCloud(*scene_input, *scene_input, indices1);
  */
  //Start ICP algorithm using kd tree
  /*tree1->setInputCloud(scene); 
  tree2->setInputCloud(map); 
  icp.setSearchMethodSource(tree1);
  icp.setSearchMethodTarget(tree2);
  icp.setInputSource(scene);
  icp.setInputTarget(map);
  icp.setMaxCorrespondenceDistance(10);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations(20); 
  icp.align(*scene, init_align);
  Eigen::Matrix4f trans = icp.getFinalTransformation();*/
  *map += *scene;
  pub_result.publish(*map);
  lock = false;
  std::cout<<"finish"<<std::endl;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  tf::TransformListener listener(ros::Duration(10));
  lr = &listener;
  // Create a ROS subscriber for the input point cloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odometry/filtered", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_result = nh.advertise<PointCloudXYZRGB> ("/output", 1);
  // Spin
  ros::spin ();
}