#include <ros/ros.h>

#include <algorithm>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");
  //将句柄传入loader 拿到use_n_scans参数
  Loader loader(Loader::getConfig(&nh_private));
  //初始化lidar odom
  Lidar lidar;
  Odom odom;
  //处理rosbag数据
  std::string input_bag_path;
  ROS_INFO("Loading Pointcloud Data...");
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  } else if (!loader.loadPointcloudFromROSBag(  //将lidar的rosbag数据读入 转到loader.cpp 传入bag_path Scan::Config 初始化的lidar
                 input_bag_path, Scan::getConfig(&nh_private), &lidar)) {
    ROS_FATAL("Error loading pointclouds from ROS bag.");
    exit(0);
  }

  bool transforms_from_csv;
  nh_private.param("transforms_from_csv", transforms_from_csv, false);
  std::string input_csv_path;
  ROS_INFO("Loading Transformation Data...                                ");
  if (transforms_from_csv) {
    if (!nh_private.getParam("input_csv_path", input_csv_path)) {
      ROS_FATAL("Could not find input_csv_path parameter, exiting");
      exit(EXIT_FAILURE);
    } else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
      ROS_FATAL("Error loading transforms from CSV.");
      exit(0);
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom)) { //odom的数据读入 转到loader.cpp 传入bag_path 以及初始化的odom
    ROS_FATAL("Error loading transforms from ROS bag.");
    exit(0);
  }

  if (lidar.getNumberOfScans() == 0) {
    ROS_FATAL("No data loaded, exiting");
    exit(0);
  }

  ROS_INFO("Interpolating Transformation Data...                          ");
  //设置每个scan里面的每个点的当前imu 的位姿 T_o0_ot_ 用于去畸变
  lidar.setOdomOdomTransforms(odom);
  
  //初始化aligner
  Aligner aligner(Aligner::getConfig(&nh_private));

  //求解aligner lidar与odom的相对变换 输出标定结果
  aligner.lidarOdomTransform(&lidar, &odom);

  return 0;
}
