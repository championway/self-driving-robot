#include <ros/ros.h>
#include <cmath>        // std::abs
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <robotx_msgs/ObstaclePose.h>
#include <robotx_msgs/ObstaclePoseList.h>
#include <robotx_msgs/BoolStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
using namespace Eigen;
using namespace message_filters;
//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef boost::shared_ptr <robotx_msgs::BoolStamped const> BoolStampedConstPtr;
//declare point cloud
PointCloudXYZ::Ptr cloud_inXYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr cloud_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_f (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_plane (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_out;
//declare ROS publisher
ros::Publisher pub_result;
ros::Publisher pub_marker;
ros::Publisher pub_obstacle;

tf::TransformListener* lr;
//declare global variable
bool lock = false;
//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function
void cluster_pointcloud(void); //point cloud clustering
void drawRviz(robotx_msgs::ObstaclePoseList); //draw marker in Rviz

void callback(const sensor_msgs::PointCloud2ConstPtr& input, const robotx_msgs::BoolStampedConstPtr& tf_bool)
{
  /*try{
    lr->waitForTransform("/map", "/robot", ros::Time::now(), ros::Duration(0.5) );
  }
  catch(tf::TransformException ex){
    ROS_ERROR("transform exception : %s", ex.what());
  }*/

  if (!lock){
    lock = true;
    //covert from ros type to pcl type
    pcl::fromROSMsg (*input, *cloud_inXYZ);
    copyPointCloud(*cloud_inXYZ, *cloud_in);
    //set color for point cloud
    for (size_t i = 0; i < cloud_in->points.size(); i++){
      cloud_in->points[i].r = 255;
      cloud_in->points[i].g = 255;
      cloud_in->points[i].b = 0;
    }
    //point cloud clustering
    if(tf_bool->data){
      cluster_pointcloud();
    }
    
  }
  else{
    std::cout << "lock" << std::endl;
  }
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cluster_pointcloud()
{
  
  std::cout<< "start processing point clouds" << std::endl;
  //========== Remove NaN point ==========
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

  //========== Downsample ==========
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.02f, 0.02f, 0.02f); //unit:cetimeter
  vg.filter (*cloud_filtered);
  std::cout << "Filtering successfully" << std::endl;
  
  //========== Planar filter ==========
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  //========== Point Cloud Clustering ==========

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);
  // Create cluster object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.2); // unit: meter
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int num_cluster = 0;
  int set_r=0, set_g=0, set_b=0; // declare cluster point cloud color
  int start_index = 0;
  robotx_msgs::ObstaclePoseList ob_list;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    num_cluster++;
    float min_z = 10000.0;
    robotx_msgs::ObstaclePose ob_pose;
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      result->points.push_back(cloud_filtered->points[*pit]);
      if(cloud_filtered->points[*pit].z < min_z)
      {
        min_z = cloud_filtered->points[*pit].z;
      }
      
    }
    MatrixXf a(cloud_cluster->points.size(),4);
    //a = MatrixXf::Random(cloud_cluster->points.size(),4);
    //std::cout << cloud_cluster->points.size() << std::endl;
    VectorXf b(cloud_cluster->points.size());
    //b = VectorXf::Random(cloud_cluster->points.size());
    Vector4f ans;
    std::cout << "Declare Matrix and Vector" << std::endl;
    for(int i=0; i < cloud_cluster->points.size(); i++)
    {
      a.row(i) << 2*cloud_cluster->points[i].x, 
                  2*cloud_cluster->points[i].y, 
                  2*cloud_cluster->points[i].z,
                  1;
      b(i) = pow(cloud_cluster->points[i].x, 2)+pow(cloud_cluster->points[i].y, 2)+pow(cloud_cluster->points[i].z, 2);
    }
    std::cout << "Start compute radius" << std::endl;
    ans = a.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
    //std::cout << a.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << std::endl;
    //std::cout<< "ANS:" << std::endl << ans << std::endl;
    //std::cout<< "b:" << b << std::endl;


    //pcl::compute3DCentroid(*cloud_cluster, centroid);
    //std::cout << centroid << std::endl;
    ob_pose.header.stamp = ros::Time::now();
    ob_pose.header.frame_id = cloud_cluster->header.frame_id;
    //ob_pose.x = centroid[0];
    //ob_pose.y = centroid[1];
    //ob_pose.z = centroid[2];
    ob_pose.x = ans(0);
    ob_pose.y = ans(1);
    ob_pose.z = ans(2);
    ob_pose.r = float(sqrt(ans(3)+pow(ans(0),2)+pow(ans(1),2)+pow(ans(2),2)));
    std::cout << ob_pose.r << std::endl;
    /*for(int i = 0; i < result->points.size(); i++)
    {
      result->points[i].r = 255;
      result->points[i].r = 255;
      result->points[i].r = 255;
    }*/
    ob_list.list.push_back(ob_pose);
    start_index = result->points.size();
  }

  //set obstacle list
  ob_list.header.stamp = ros::Time::now();
  ob_list.header.frame_id = cloud_in->header.frame_id;
  ob_list.size = num_cluster;
  pub_obstacle.publish(ob_list);
  drawRviz(ob_list);

  result->header.frame_id = cloud_in->header.frame_id;
  //writer.write<pcl::PointXYZRGB> ("result.pcd", *cloud_filtered, false);
  std::cout << "Finish" << std::endl << std::endl;
  pcl::toROSMsg(*result, ros_out);
  pub_result.publish(ros_out);
  lock = false;
  result->clear();
}

void drawRviz(robotx_msgs::ObstaclePoseList ob_list){
      visualization_msgs::Marker  marker;
      marker.header.frame_id = "velodyne";
      marker.header.stamp = ros::Time::now();
      marker.type = marker.SPHERE_LIST;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      // set marker color
      std_msgs::ColorRGBA c;
      for (int i = 0; i < ob_list.size;i++)
      {
        if (std::abs(ob_list.list[i].r-0.2)<=std::abs(ob_list.list[i].r-0.15) && std::abs(ob_list.list[i].r-0.2)<=std::abs(ob_list.list[i].r-0.1))
        {
          c.r = 1.0;
          c.g = 0.0;
          c.b = 0.0;
          c.a = 1.0;
        }
        else if (std::abs(ob_list.list[i].r-0.15)<=std::abs(ob_list.list[i].r-0.2) && std::abs(ob_list.list[i].r-0.15)<=std::abs(ob_list.list[i].r-0.1))
        {
          c.r = 0.0;
          c.g = 1.0;
          c.b = 0.0;
          c.a = 1.0;
        }
        else if (std::abs(ob_list.list[i].r-0.1)<=std::abs(ob_list.list[i].r-0.15) && std::abs(ob_list.list[i].r-0.1)<=std::abs(ob_list.list[i].r-0.2))
        {
          c.r = 0.0;
          c.g = 0.0;
          c.b = 1.0;
          c.a = 1.0;
        }
        else
        {
          c.r = 1.0;
          c.g = 1.0;
          c.b = 1.0;
          c.a = 1.0;
        }
        /*float dis = sqrt(ob_list.list[i].x*ob_list.list[i].x+ob_list.list[i].y*ob_list.list[i].y);
        if (dis <= 3.0)
        {
          c.r = 1.0;
          c.g = 0.0;
          c.a = 1.0;
        }
        else
        {
          c.r = 0.0;
          c.g = 1.0;
          c.a = 1.0;
        }*/
        geometry_msgs::Point p;
        p.x = ob_list.list[i].x;
        p.y = ob_list.list[i].y;
        p.z = ob_list.list[i].z;
        marker.colors.push_back(c);  
        marker.points.push_back(p);
      }
      pub_marker.publish(marker);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  tf::TransformListener listener(ros::Duration(1.0));
  lr = &listener;
  // Create a ROS subscriber for the input point cloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<robotx_msgs::BoolStamped> bool_sub(nh, "/tf_transform", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, robotx_msgs::BoolStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, bool_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, callback);
  // Create a ROS publisher for the output point cloud
  pub_obstacle = nh.advertise< robotx_msgs::ObstaclePoseList > ("/obstacle_list", 1);
  pub_marker = nh.advertise< visualization_msgs::Marker >("/obstacle_marker", 1);
  pub_result = nh.advertise<sensor_msgs::PointCloud2> ("/cluster_result", 1);
  // Spin
  ros::spin ();
}