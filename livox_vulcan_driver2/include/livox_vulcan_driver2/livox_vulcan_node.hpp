// Copyright (c) 2024
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#ifndef LIVOX_VULCAN_DRIVER2__LIVOX_VULCAN_NODE_HPP_
#define LIVOX_VULCAN_DRIVER2__LIVOX_VULCAN_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "livox_ros_msg/msg/custom_msg.hpp"
#include "livox_ros_msg/msg/custom_point.hpp"

// Accumulated point with its absolute timestamp
struct PointWithTime {
  LivoxLidarCartesianHighRawPoint point;
  uint64_t offset_time_ns;  // absolute point timestamp in ns, same scheme as livox_ros_driver2
};

class LivoxVulcanNode : public rclcpp::Node
{
public:
  explicit LivoxVulcanNode(const rclcpp::NodeOptions & options);
  ~LivoxVulcanNode();

private:
  // --- Livox SDK callbacks ---
  static void OnPointCloud(uint32_t handle, const uint8_t dev_type,
                           LivoxLidarEthernetPacket * data, void * client_data);
  static void OnImuData(uint32_t handle, const uint8_t dev_type,
                        LivoxLidarEthernetPacket * data, void * client_data);
  static void OnLidarInfoChange(const uint32_t handle, const LivoxLidarInfo * info,
                                void * client_data);
  static void OnWorkModeCb(livox_status status, uint32_t handle,
                           LivoxLidarAsyncControlResponse * response, void * client_data);

  // --- ROS publishers ---
  void publish_accumulated_cloud();
  void publish_custom_msg();
  void publish_imu_packet(LivoxLidarEthernetPacket * data);

  // --- Timestamp helpers (same scheme as livox_ros_driver2) ---
  static uint64_t GetPacketTimestamp(const LivoxLidarEthernetPacket * data);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<livox_ros_msg::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Accumulated point cloud for the current frame
  std::mutex cloud_mutex_;
  std::vector<PointWithTime> accumulated_points_;
  std::chrono::nanoseconds last_timestamp_ns_{0};
  uint32_t lidar_handle_{0};

  // Timer for periodic point cloud publishing (10 Hz)
  rclcpp::TimerBase::SharedPtr cloud_timer_;

  // Config
  std::string config_path_;
  std::string cloud_topic_;
  std::string custom_topic_;
  std::string imu_topic_;

  // Soft time sync: align LiDAR PTP time to system clock on first frame
  bool time_sync_soft_{false};
  bool first_frame_received_{false};
  int time_sync_wait_count_{50};
  int frame_count_{0};
  int64_t time_offset_ns_{0};
};

#endif  // LIVOX_VULCAN_DRIVER2__LIVOX_VULCAN_NODE_HPP_
