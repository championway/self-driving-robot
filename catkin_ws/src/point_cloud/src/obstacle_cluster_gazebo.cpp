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
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
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
PointCloudXYZRGB::Ptr plane_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_h (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_f (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_plane (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr hold_plane (new PointCloudXYZRGB);
//PointCloudXYZRGB::Ptr wall (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_out;
//sensor_msgs::PointCloud2 ros_wall;
//declare ROS publisher
ros::Publisher pub_result;
//ros::Publisher pub_wall;
ros::Publisher pub_marker;
ros::Publisher pub_marker_line;
ros::Publisher pub_obstacle;


tf::TransformListener* lr;
//declare global variable
bool lock = false;
float low = -0.25;
float high = 1.5-low;
float thres_low = 0.03;
float thres_high = 1.5;
visualization_msgs::MarkerArray marker_array;
visualization_msgs::MarkerArray marker_array_line;
ros::Time pcl_t;

//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function
void cluster_pointcloud(void); //point cloud clustering
void drawRviz(robotx_msgs::ObstaclePoseList); //draw marker in Rviz
void drawRviz_line(robotx_msgs::ObstaclePoseList); //draw marker line list in Rviz

//void callback(const sensor_msgs::PointCloud2ConstPtr& input, const robotx_msgs::BoolStampedConstPtr& tf_bool)
void callback(const sensor_msgs::PointCloud2ConstPtr& input)
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
    //pcl_t = input->header.stamp;
    pcl::fromROSMsg (*input, *cloud_inXYZ);
    copyPointCloud(*cloud_inXYZ, *cloud_in);
    //set color for point cloud
    for (size_t i = 0; i < cloud_in->points.size(); i++){
      cloud_in->points[i].r = 255;
      cloud_in->points[i].g = 255;
      cloud_in->points[i].b = 0;
    }
    //point cloud clustering
    /*if(tf_bool->data){
      cluster_pointcloud();
    }*/
    cluster_pointcloud();
    
  }
  else{
    std::cout << "lock" << std::endl;
  }
}

//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cluster_pointcloud()
{
  
  std::cout<< "start processing point cloudssssss" << std::endl;
  copyPointCloud(*cloud_in, *cloud_filtered);
  
  //========== Remove NaN point ==========
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);*/

  //========== Downsample ==========
  /*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.02f, 0.02f, 0.02f); //unit:cetimeter
  vg.filter (*cloud_filtered);
  std::cout << "Filtering successfully" << std::endl;*/
  //copyPointCloud(*cloud_filtered, *wall);
  //wall->clear();

  //========== Outlier remove ==========
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> out_filter;
  out_filter.setInputCloud (cloud_filtered);
  out_filter.setMeanK (50);
  out_filter.setStddevMulThresh (1.0);
  out_filter.filter (*cloud_filtered);*/
  
  //========== Planar filter ==========
  /*pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  //pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.02);
  copyPointCloud(*cloud_filtered, *wall);
  wall->clear();
  int nr_points = (int) cloud_filtered->points.size ();
  const float nan_point = std::numeric_limits<float>::quiet_NaN();
  bool find_floor = false;
  int plane_z = 0;
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
    plane_z = 0;
    for (int i = 0; i < cloud_plane->points.size(); i++)
    {
      plane_z += cloud_plane->points[i].z;
    }
    plane_z =(float)plane_z/cloud_plane->points.size();
    //high = thres_high + low;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    //if (std::abs(coefficients->values[2])>0.2 && std::abs(coefficients->values[2]<0.85))
    if (std::abs(coefficients->values[2] < 0.2))
    {
      pcl::ModelCoefficients line; 
      pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices); 
      pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
      seg.setOptimizeCoefficients(true); 
      seg.setModelType(pcl::SACMODEL_LINE); 
      seg.setMethodType(pcl::SAC_RANSAC); 
      seg.setDistanceThreshold(1); 
      seg.setInputCloud(cloud_plane); 
      seg.segment(*line_inliers, line);
      std::cout<<line<<std::endl;
      //std::cout << "hold plane"<< std::endl;
      //plane_indices->indices.insert(plane_indices->indices.end(), inliers->indices.begin(), inliers->indices.end());
      //*wall += *cloud_plane;
    }
    //else if (plane_z > -0.3)
    //{
      //*hold_plane += *cloud_plane;
      //for (int i = 0; i < cloud_plane->points.size(); i++)
      //{
        //low += cloud_plane->points[i].z;
      //}
      //low =(float)low/cloud_plane->points.size() - thres_low;
      //high = thres_high + low;
      //find_floor = true;
    //}
  }
  //*cloud_filtered += *hold_plane;*/

  //========== Remove Higer and Lower Place ==========
  /*pcl::ExtractIndices<pcl::PointXYZRGB> extract_h_l_place;
  pcl::PointIndices::Ptr hl_indices (new pcl::PointIndices);
  //std::cout<< low << "," << high << std::endl;
  for (int i = 0; i < cloud_filtered->points.size(); i++)
  {
    if (cloud_filtered->points[i].z >= high || cloud_filtered->points[i].z <= low)
    {
      hl_indices->indices.push_back(i);
    }
  }
  extract_h_l_place.setInputCloud(cloud_filtered);
  extract_h_l_place.setIndices(hl_indices);
  extract_h_l_place.setNegative(true);
  extract_h_l_place.filter(*cloud_h);
  *cloud_filtered = *cloud_h;*/

  //========== Project To Ground ==========
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr c (new pcl::ModelCoefficients ());
  c->values.resize (4);
  c->values[0] = 0;
  c->values[1] = 0;
  c->values[2] = 1.0;
  c->values[3] = 0;
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (c);
  proj.filter (*cloud_filtered);
  /*for (size_t i = 0; i < wall->points.size(); i++){
    wall->points[i].r = 255;
    wall->points[i].g = 0;
    wall->points[i].b = 255;
  }*/
  
  //========== Wall Detector Node ==========
  /*pcl::ModelCoefficients line; 
  pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices); 
  pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
  seg.setOptimizeCoefficients(true); 
  seg.setModelType(pcl::SACMODEL_LINE); 
  seg.setMethodType(pcl::SAC_RANSAC); 
  seg.setDistanceThreshold(1); 
  seg.setInputCloud(cloud_filtered); 
  seg.segment(*line_inliers, line);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_line;
  extract_line.setInputCloud(cloud_filtered);
  extract_line.setIndices(line_inliers);
  extract_line.setNegative(false);
  extract_line.filter(*wall);
  std::cout<<line<<std::endl;*/
  //*cloud_filtered = *cloud_h;
  /*pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud_filtered));
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_line);

  ransac.setDistanceThreshold (0.01);
  ransac.computeModel();

  Eigen::VectorXf line_coefficients;
  ransac.getModelCoefficients(line_coefficients);
  std::cout<<line_coefficients<<std::endl;*/

  //========== Outlier remove ==========
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> out_filter;
  out_filter.setInputCloud (cloud_filtered);
  out_filter.setMeanK (50);
  out_filter.setStddevMulThresh (1.0);
  out_filter.filter (*cloud_filtered);*/

  //========== Point Cloud Clustering ==========
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);
  // Create cluster object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (1.2); // unit: meter
  ec.setMinClusterSize (2);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  int num_cluster = 0;
  int start_index = 0;
  robotx_msgs::ObstaclePoseList ob_list;
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    float x_min_x = 10000;
    float x_min_y = 10000;
    float y_min_x = 10000;
    float y_min_y = 10000;
    float x_max_x = -10000;
    float x_max_y = -10000;
    float y_max_x = -10000;
    float y_max_y = -10000;
    
    robotx_msgs::ObstaclePose ob_pose;
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      result->points.push_back(cloud_filtered->points[*pit]);
      if (cloud_filtered->points[*pit].x < x_min_x)
      {
        x_min_x = cloud_filtered->points[*pit].x;
        x_min_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].x > x_max_x)
      {
        x_max_x = cloud_filtered->points[*pit].x;
        x_max_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].y < y_min_y)
      {
        y_min_x = cloud_filtered->points[*pit].x;
        y_min_y = cloud_filtered->points[*pit].y;
      }
      if (cloud_filtered->points[*pit].y > y_max_y)
      {
        y_max_x = cloud_filtered->points[*pit].x;
        y_max_y = cloud_filtered->points[*pit].y;
      }
    }

    pcl::compute3DCentroid(*cloud_cluster, centroid);
    //std::cout << centroid << std::endl;
    ob_pose.header.stamp = ros::Time::now();
    ob_pose.header.frame_id = cloud_in->header.frame_id;
    ob_pose.x = centroid[0];
    ob_pose.y = centroid[1];
    ob_pose.z = centroid[2];
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::getMinMax3D (*cloud_cluster, min, max);
    ob_pose.min_x = min[0];
    ob_pose.max_x = max[0];
    ob_pose.min_y = min[1];
    ob_pose.max_y = max[1];
    ob_pose.min_z = min[2];
    ob_pose.max_z = max[2];
    ob_pose.x_min_x = x_min_x;
    ob_pose.x_min_y = x_min_y;
    ob_pose.x_max_x = x_max_x;
    ob_pose.x_max_y = x_max_y;
    ob_pose.y_min_x = y_min_x;
    ob_pose.y_min_y = y_min_y;
    ob_pose.y_max_x = y_max_x;
    ob_pose.y_max_y = y_max_y;
    std::cout<<ob_pose.y<<std::endl;
    std::cout<<ob_pose.x<<std::endl;
    std::cout<<"--------"<<std::endl;

    geometry_msgs::Point pose, velocity;
    ob_pose.r = 1;
    if(ob_pose.y < 2.5 && ob_pose.y > 0)
    {
      if (ob_pose.x > 1.5 || ob_pose.x < -1.5){
        ob_list.list.push_back(ob_pose);
        num_cluster++;
      }
      else{
        std::cout<<"no"<<std::endl;
      }
      std::cout<<"noaa"<<std::endl;
    }
    else{
      ob_list.list.push_back(ob_pose);
      num_cluster++;
    }
    start_index = result->points.size();
  }

  //set obstacle list
  ob_list.header.stamp = ros::Time::now();
  ob_list.header.frame_id = cloud_in->header.frame_id;
  ob_list.size = num_cluster;
  pub_obstacle.publish(ob_list);
  drawRviz(ob_list);
  drawRviz_line(ob_list);

  result->header.frame_id = cloud_in->header.frame_id;
  //writer.write<pcl::PointXYZRGB> ("result.pcd", *cloud_filtered, false);
  std::cout << "Finish" << std::endl << std::endl;
  pcl::toROSMsg(*result, ros_out);
  //pcl::toROSMsg(*wall, ros_wall);
  ros_out.header.stamp = ros::Time::now();
  //ros_wall.header.stamp = pcl_t;
  pub_result.publish(ros_out);
  //pub_wall.publish(ros_wall);
  lock = false;
  result->clear();
  hold_plane->clear();
  //wall->clear();
}

void drawRviz_line(robotx_msgs::ObstaclePoseList ob_list){
  marker_array_line.markers.resize(ob_list.size);
  //marker.lifetime = ros::Duration(0.5);
  //std::cout << "line" << ob_list.size << std::endl;
  for (int i = 0; i < ob_list.size; i++)
  {
    marker_array_line.markers[i].header.frame_id = "velodyne";
    marker_array_line.markers[i].id = i;
    marker_array_line.markers[i].header.stamp = ob_list.header.stamp;
    marker_array_line.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
    marker_array_line.markers[i].action = visualization_msgs::Marker::ADD;
    //marker_array.markers[i].pose.orientation.w = 1.0;
    marker_array_line.markers[i].points.clear();
    marker_array_line.markers[i].color.r = 1;
    marker_array_line.markers[i].color.g = 0;
    marker_array_line.markers[i].color.b = 0;
    marker_array_line.markers[i].color.a = 1;
    marker_array_line.markers[i].lifetime = ros::Duration(0.5);
    marker_array_line.markers[i].scale.x = (0.1);
    geometry_msgs::Point x_min;
    x_min.x = ob_list.list[i].x_min_x;
    x_min.y = ob_list.list[i].x_min_y;
    geometry_msgs::Point x_max;
    x_max.x = ob_list.list[i].x_max_x;
    x_max.y = ob_list.list[i].x_max_y;
    geometry_msgs::Point y_min;
    y_min.x = ob_list.list[i].y_min_x;
    y_min.y = ob_list.list[i].y_min_y;
    geometry_msgs::Point y_max;
    y_max.x = ob_list.list[i].y_max_x;
    y_max.y = ob_list.list[i].y_max_y;
    marker_array_line.markers[i].points.push_back(x_min);
    marker_array_line.markers[i].points.push_back(y_min);
    marker_array_line.markers[i].points.push_back(x_max);
    marker_array_line.markers[i].points.push_back(y_max);
    marker_array_line.markers[i].points.push_back(x_min);
    //std::cout<<x_max.x <<","<<x_min.x<<std::endl;
  }
  pub_marker_line.publish(marker_array_line);
}

void drawRviz(robotx_msgs::ObstaclePoseList ob_list){
      marker_array.markers.resize(ob_list.size);
      //marker.lifetime = ros::Duration(0.5);
      //std::cout << "cube" << ob_list.size << std::endl;
      std_msgs::ColorRGBA c;
      for (int i = 0; i < ob_list.size; i++)
      {
        marker_array.markers[i].header.frame_id = "velodyne";
        marker_array.markers[i].id = i;
        marker_array.markers[i].header.stamp = ob_list.header.stamp;
        marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].color.r = 1.0;
        marker_array.markers[i].color.g = 0;
        marker_array.markers[i].color.b = 0;
        marker_array.markers[i].color.a = 0.5;
        marker_array.markers[i].lifetime = ros::Duration(0.5);

        marker_array.markers[i].pose.position.x = ob_list.list[i].x;
        marker_array.markers[i].pose.position.y = ob_list.list[i].y;
        marker_array.markers[i].pose.position.z = ob_list.list[i].z;
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;
        
        marker_array.markers[i].scale.x = (ob_list.list[i].max_x-ob_list.list[i].min_x);
        marker_array.markers[i].scale.y = (ob_list.list[i].max_y-ob_list.list[i].min_y);
        marker_array.markers[i].scale.z = (ob_list.list[i].max_z-ob_list.list[i].min_z);
        if (marker_array.markers[i].scale.x ==0)
          marker_array.markers[i].scale.x=0.1;

        if (marker_array.markers[i].scale.y ==0)
          marker_array.markers[i].scale.y=0.1;

        if (marker_array.markers[i].scale.z ==0)
          marker_array.markers[i].scale.z=0.1;
      }
      /*for (int i = 0; i < ob_list.size; i++)
      {
        std::cout<< marker_array.markers[i].scale.x << std::endl;
      }
      std::cout << "==========" << std::endl;*/
      pub_marker.publish(marker_array);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  tf::TransformListener listener(ros::Duration(1.0));
  lr = &listener;
  // Create a ROS subscriber for the input point cloud
  /*message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<robotx_msgs::BoolStamped> bool_sub(nh, "/tf_transform", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, robotx_msgs::BoolStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), pcl_sub, bool_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));*/


  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, callback);
  // Create a ROS publisher for the output point cloud
  pub_obstacle = nh.advertise< robotx_msgs::ObstaclePoseList > ("/obstacle_list", 10);
  //pub_marker = nh.advertise< visualization_msgs::Marker >("/obstacle_marker", 1);
  pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker", 1);
  pub_marker_line = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker_line", 1);
  pub_result = nh.advertise<sensor_msgs::PointCloud2> ("/cluster_result", 1);
  //pub_wall = nh.advertise<sensor_msgs::PointCloud2> ("/wall", 1);
  // Spin
  ros::spin ();
}