// Copyright (c) 2023 Vulcan
// Email: vulcan@mail.com

#ifndef LIVOX_TOOLS__LIVOX_TOOLS_NODE_HPP_
#define LIVOX_TOOLS__LIVOX_TOOLS_NODE_HPP_

#include <mutex>
#include <deque>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.hpp"
#include "tf2/exceptions.h"
#include "sophus/se3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"


class livoxToolsNode : public rclcpp::Node
{
public:
  explicit livoxToolsNode(const rclcpp::NodeOptions & options);

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  bool find_closest_imu_data(const rclcpp::Time & cloud_stamp, sensor_msgs::msg::Imu & closest_imu_data);

  std::shared_ptr<sensor_msgs::msg::LaserScan> createLaserScan(
    const std_msgs::msg::Header &header,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud,
    double scan_angle_min_, double scan_angle_max_,
    double scan_angle_increment_, double range_min_, double range_max_,
    const std::string frame_id);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trans_point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  std::string cloud_topic_ = "/livox/pointcloud";
  std::string imu_topic_ = "/livox/imu";
  std::string trans_point_topic_ = "/livox/pointcloud/transform";
  std::string laserscan_topic_ = "/autocube/laserscan";
  std::string base_link_ = "autocube_link";

  double min_z_, max_z_;
  double blind_ = 0.05;
  int num_scans_ = 6;

  double scan_angle_min_ = -3.14;
  double scan_angle_max_ = 3.14;
  double scan_angle_increment_ = 0.01;
  double range_min_ = 0.1;
  double range_max_ = 10.0;

  int imu_quee_ = 200;
  std::deque<sensor_msgs::msg::Imu> imu_data_queue_;
  std::mutex imu_data_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Eigen::Vector3d lidar_init_rpy_;
  Sophus::SE3d lidar_pose_;
  bool lidar_pose_init_ = false;
  bool enable_log_ = false;
};

#endif  // LIVOX_TOOLS__LIVOX_TOOLS_NODE_HPP_
