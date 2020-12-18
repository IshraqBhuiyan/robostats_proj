#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <std_srvs/Empty.h>

#include "robostats_proj/get_map.h"
#include "robostats_proj/get_camera_transform.h"

tf::TransformListener *tf_listener;

laser_geometry::LaserProjection projector_;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(*scan_in, cloud);
  tf::StampedTransform to_world;

  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2); //Memory leak
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp(new pcl::PointCloud<pcl::PointXYZ>);

  try{
    tf_listener->waitForTransform("/world", ros::Time(0), (*scan_in).header.frame_id.c_str(), 
      (*scan_in).header.stamp, "/world", ros::Duration(2.0));
    tf_listener->lookupTransform("/world", ros::Time(0), (*scan_in).header.frame_id.c_str(), 
      (*scan_in).header.stamp, "/world", to_world);
  }catch(...){
    return;
  }

  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloudtemp);
  pcl_ros::transformPointCloud(*cloudtemp, *cloudtemp, to_world);

  *cloudtemp = *cloud_map + *cloudtemp;

  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(cloudtemp);
  grid.setLeafSize(0.001f, 0.001f, 0.001f);
  grid.filter(*cloud_map);

  // Do something with cloud.
  //delete temp_cloud;
}

bool reset_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
  cloud_map = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  return true;
}

bool camera_transform_cb(robostats_proj::get_camera_transform::Request& req,robostats_proj::get_camera_transform::Response& resp){
  tf::StampedTransform world_to_camera;

  try {
    tf_listener->waitForTransform("/laser_beam", "/world", ros::Time(0), ros::Duration(1.0) );
    tf_listener->lookupTransform("/laser_beam", "/world", ros::Time(0), world_to_camera);
  } catch (...) {
    return false;
  }
  tf::transformStampedTFToMsg(world_to_camera, resp.tf);
  return true;
}

bool pc_service_cb(robostats_proj::get_map::Request& req, robostats_proj::get_map::Response& resp){
  pcl::toROSMsg(*cloud_map, resp.cloud);
  resp.cloud.header.frame_id = "world";
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pc_utils");
  ros::NodeHandle nh("~");

  tf_listener = new tf::TransformListener();

  ros::Subscriber laser_sub = nh.subscribe("/laser_scan", 1, scanCallback);

  ros::ServiceServer reset_srv = nh.advertiseService("reset_pc", reset_cb);

  ros::ServiceServer camera_srv = nh.advertiseService("get_camera_transf", camera_transform_cb);
  ros::ServiceServer pc_srv = nh.advertiseService("get_model_cloud", pc_service_cb);

  ros::spin();

}