#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
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
using namespace message_filters;
using namespace pcl;
//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef boost::shared_ptr <geometry_msgs::PoseStamped const> PoseStampedConstPtr;

//declare point cloud
PointCloudXYZ::Ptr input_XYZ (new PointCloudXYZ);
//PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB); 
//PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);

//declare point cloud variable
PointCloudXYZRGB::Ptr map (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr scene (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);


//declare ROS publisher
ros::Publisher pub_result;

//declare function
//void callback(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function

void callback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  pcl::fromROSMsg (*pcl_msg, *input_XYZ);
  copyPointCloud(*input_XYZ, *scene);
  
  //define ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);

  //ICP initial transform matrix
  Eigen::Matrix4f init_align;
  init_align <<     0.707097,    -0.707121,  0.00031099,   -43.8569,
                    0.707125,       0.7071, 0.000705772,    36.1607,
                -0.000718911, -0.000279344,           1,  -0.990627,
                           0,            0,           0,          1; 

  std::cout<< "Start ICP" << std::endl;
  // Remove NaN point
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_input, *map_input, indices);
  std::vector<int> indices1;
  pcl::removeNaNFromPointCloud(*scene_input, *scene_input, indices1);
  */
  //Start ICP algorithm using kd tree
  tree1->setInputCloud(scene); 
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
  Eigen::Matrix4f trans = icp.getFinalTransformation();
  *map += *scene;
  pub_result.publish(*map);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "/robot_pose", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_result = nh.advertise<PointCloudXYZRGB> ("/output", 1);
  // Spin
  ros::spin ();
}