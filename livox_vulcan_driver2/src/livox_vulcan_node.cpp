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

#include "livox_vulcan_driver2/livox_vulcan_node.hpp"

#include <cstring>
#include <fstream>
#include <string>

using CustomMsg = livox_ros_msg::msg::CustomMsg;
using CustomPoint = livox_ros_msg::msg::CustomPoint;

// ---------------------------------------------------------------------------
// Parse Livox packet timestamp — same logic as livox_ros_driver2's
// PubHandler::GetEthPacketTimestamp, using union memcpy (no bswap).
// ---------------------------------------------------------------------------
uint64_t LivoxVulcanNode::GetPacketTimestamp(const LivoxLidarEthernetPacket * data)
{
  union {
    uint8_t  bytes[8];
    int64_t  stamp;
  } ts;
  memcpy(ts.bytes, data->timestamp, sizeof(ts.bytes));
  return static_cast<uint64_t>(ts.stamp);
}

// ---------------------------------------------------------------------------
// Static callback: point cloud data from Livox SDK
// ---------------------------------------------------------------------------
void LivoxVulcanNode::OnPointCloud(uint32_t handle, const uint8_t dev_type,
                                   LivoxLidarEthernetPacket * data, void * client_data)
{
  (void)handle;
  (void)dev_type;

  auto * node = static_cast<LivoxVulcanNode *>(client_data);
  if (node == nullptr) return;

  if (data == nullptr || data->data_type != kLivoxLidarCartesianCoordinateHighData) {
    return;
  }

  // Packet timestamp (same scheme as livox_ros_driver2)
  uint64_t pkt_timestamp_ns = GetPacketTimestamp(data);

  // point_interval in ns: same as livox_ros_driver2
  // data->time_interval is in 0.1 us units
  uint32_t point_interval_ns = static_cast<uint32_t>(data->time_interval) * 100 / data->dot_num;

  LivoxLidarCartesianHighRawPoint * pts =
    reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data);

  std::lock_guard<std::mutex> lock(node->cloud_mutex_);

  for (uint16_t i = 0; i < data->dot_num; ++i) {
    PointWithTime pwt;
    pwt.point = pts[i];
    pwt.offset_time_ns = pkt_timestamp_ns + i * point_interval_ns;
    node->accumulated_points_.push_back(pwt);
  }

  node->last_timestamp_ns_ = std::chrono::nanoseconds(pkt_timestamp_ns);
  node->lidar_handle_ = handle;
}

// ---------------------------------------------------------------------------
// Static callback: IMU data from Livox SDK (~200 Hz native rate)
// ---------------------------------------------------------------------------
void LivoxVulcanNode::OnImuData(uint32_t handle, const uint8_t dev_type,
                                LivoxLidarEthernetPacket * data, void * client_data)
{
  (void)handle;
  (void)dev_type;

  if (data == nullptr || data->data_type != kLivoxLidarImuData) {
    return;
  }

  auto * node = static_cast<LivoxVulcanNode *>(client_data);
  if (node == nullptr) return;

  node->publish_imu_packet(data);
}

// ---------------------------------------------------------------------------
// Lidar discovered callback
// ---------------------------------------------------------------------------
void LivoxVulcanNode::OnLidarInfoChange(const uint32_t handle, const LivoxLidarInfo * info,
                                        void * client_data)
{
  if (info == nullptr) return;

  auto * node = static_cast<LivoxVulcanNode *>(client_data);
  RCLCPP_INFO(node->get_logger(), "Lidar discovered - handle: %u, SN: %s", handle, info->sn);

  RCLCPP_INFO(node->get_logger(), "  LiDAR IP    : %s", info->lidar_ip);
  RCLCPP_INFO(node->get_logger(), "  Device type : %u", info->dev_type);

  EnableLivoxLidarPointSend(handle, nullptr, nullptr);
  EnableLivoxLidarImuData(handle, nullptr, nullptr);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, OnWorkModeCb, client_data);
}

void LivoxVulcanNode::OnWorkModeCb(livox_status status, uint32_t handle,
                                   LivoxLidarAsyncControlResponse * response, void * client_data)
{
  auto * node = static_cast<LivoxVulcanNode *>(client_data);
  if (response == nullptr) return;

  RCLCPP_INFO(node->get_logger(),
    "Set work mode result - handle: %u, status: %d, ret_code: %u",
    handle, status, response->ret_code);
}

// ---------------------------------------------------------------------------
// Timer callback: publish PointCloud2 + CustomMsg at 10 Hz
// ---------------------------------------------------------------------------
void LivoxVulcanNode::publish_accumulated_cloud()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  if (accumulated_points_.empty()) {
    return;
  }

  size_t num_points = accumulated_points_.size();
  uint64_t timebase_ns = accumulated_points_[0].offset_time_ns;

  // --- Publish PointCloud2 ---
  {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = rclcpp::Time(timebase_ns);
    msg->header.frame_id = "livox_frame";

    const uint32_t POINT_STEP = 16;
    msg->height = 1;
    msg->width = num_points;
    msg->is_bigendian = false;
    msg->point_step = POINT_STEP;
    msg->row_step = msg->width * msg->point_step;
    msg->is_dense = true;

    sensor_msgs::msg::PointField f;
    f.name = "x";         f.offset = 0;  f.datatype = sensor_msgs::msg::PointField::INT32;  f.count = 1;
    msg->fields.push_back(f);
    f.name = "y";         f.offset = 4;  f.datatype = sensor_msgs::msg::PointField::INT32;  f.count = 1;
    msg->fields.push_back(f);
    f.name = "z";         f.offset = 8;  f.datatype = sensor_msgs::msg::PointField::INT32;  f.count = 1;
    msg->fields.push_back(f);
    f.name = "reflectivity"; f.offset = 12; f.datatype = sensor_msgs::msg::PointField::UINT8; f.count = 1;
    msg->fields.push_back(f);
    f.name = "tag";       f.offset = 13; f.datatype = sensor_msgs::msg::PointField::UINT8; f.count = 1;
    msg->fields.push_back(f);

    msg->data.resize(num_points * POINT_STEP);
    for (size_t i = 0; i < num_points; ++i) {
      auto & pt = accumulated_points_[i].point;
      uint8_t * dst = &msg->data[i * POINT_STEP];
      memcpy(dst + 0,  &pt.x, 4);
      memcpy(dst + 4,  &pt.y, 4);
      memcpy(dst + 8,  &pt.z, 4);
      dst[12] = pt.reflectivity;
      dst[13] = pt.tag;
    }
    cloud_pub_->publish(std::move(msg));
  }

  // --- Publish CustomMsg (same timestamp scheme as livox_ros_driver2) ---
  {
    auto msg = std::make_unique<CustomMsg>();
    msg->header.stamp = rclcpp::Time(timebase_ns);
    msg->header.frame_id = "livox_frame";
    msg->timebase = timebase_ns;
    msg->point_num = num_points;
    msg->lidar_id = static_cast<uint8_t>(lidar_handle_ & 0xFF);
    msg->rsvd.fill(0);

    msg->points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      auto & pwt = accumulated_points_[i];
      CustomPoint cp;
      cp.offset_time  = static_cast<uint32_t>(pwt.offset_time_ns - timebase_ns);
      cp.x            = pwt.point.x * 1e-3f;
      cp.y            = pwt.point.y * 1e-3f;
      cp.z            = pwt.point.z * 1e-3f;
      cp.reflectivity = pwt.point.reflectivity;
      cp.tag          = pwt.point.tag;
      cp.line         = pwt.point.tag & 0x03;
      msg->points.push_back(cp);
    }

    custom_pub_->publish(std::move(msg));
  }

  accumulated_points_.clear();
}

// ---------------------------------------------------------------------------
// Publish IMU packet
// ---------------------------------------------------------------------------
void LivoxVulcanNode::publish_imu_packet(LivoxLidarEthernetPacket * data)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();

  uint64_t ts_raw = GetPacketTimestamp(data);
  msg->header.stamp = rclcpp::Time(ts_raw);
  msg->header.frame_id = "livox_imu";

  LivoxLidarImuRawPoint * imu_pts =
    reinterpret_cast<LivoxLidarImuRawPoint *>(data->data);

  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float acc_x = 0, acc_y = 0, acc_z = 0;
  for (uint16_t i = 0; i < data->dot_num; ++i) {
    gyro_x += imu_pts[i].gyro_x;
    gyro_y += imu_pts[i].gyro_y;
    gyro_z += imu_pts[i].gyro_z;
    acc_x  += imu_pts[i].acc_x;
    acc_y  += imu_pts[i].acc_y;
    acc_z  += imu_pts[i].acc_z;
  }
  float inv = 1.0f / static_cast<float>(data->dot_num);
  msg->angular_velocity.x = gyro_x * inv;
  msg->angular_velocity.y = gyro_y * inv;
  msg->angular_velocity.z = gyro_z * inv;
  msg->linear_acceleration.x = acc_x * inv;
  msg->linear_acceleration.y = acc_y * inv;
  msg->linear_acceleration.z = acc_z * inv;

  msg->orientation_covariance[0] = -1;
  msg->angular_velocity_covariance[0] = -1;
  msg->linear_acceleration_covariance[0] = -1;

  imu_pub_->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
LivoxVulcanNode::LivoxVulcanNode(const rclcpp::NodeOptions & options)
  : Node("livox_vulcan_node", options)
{
  if (!this->has_parameter("config_path")) {
    this->declare_parameter<std::string>("config_path", "");
  }
  config_path_ = this->get_parameter("config_path").as_string();

  if (!this->has_parameter("cloud_topic")) {
    this->declare_parameter<std::string>("cloud_topic", "livox/pointcloud");
  }
  if (!this->has_parameter("custom_topic")) {
    this->declare_parameter<std::string>("custom_topic", "livox/custom_msg");
  }
  if (!this->has_parameter("imu_topic")) {
    this->declare_parameter<std::string>("imu_topic", "livox/imu");
  }
  cloud_topic_  = this->get_parameter("cloud_topic").as_string();
  custom_topic_ = this->get_parameter("custom_topic").as_string();
  imu_topic_    = this->get_parameter("imu_topic").as_string();

  if (config_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No config_path provided; Livox SDK cannot initialize.");
    return;
  }

  cloud_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
  custom_pub_ = this->create_publisher<CustomMsg>(custom_topic_, 10);
  imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);

  cloud_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LivoxVulcanNode::publish_accumulated_cloud, this));

  // Disable SDK console logs before init (sets the global flag that
  // InitLogger checks, so the console sink is never created)
  extern bool is_console_log_enable;
  is_console_log_enable = false;

  if (!LivoxLidarSdkInit(config_path_.c_str())) {
    RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Livox SDK initialized.");

  // --- Startup info ---
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "  Config path : %s", config_path_.c_str());

  // Read host IP from config JSON (simple string match)
  {
    std::ifstream cfg(config_path_);
    std::string line;
    std::string host_ip;
    while (std::getline(cfg, line)) {
      std::size_t pos = line.find("\"host_ip\"");
      if (pos != std::string::npos) {
        pos = line.find(':', pos);
        if (pos != std::string::npos) {
          std::size_t start = line.find('"', pos) + 1;
          std::size_t end   = line.find('"', start);
          if (start != std::string::npos && end != std::string::npos) {
            host_ip = line.substr(start, end - start);
          }
        }
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(), "  Host IP     : %s", host_ip.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "  Cloud topic : %s  [sensor_msgs::PointCloud2]", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Custom topic: %s  [livox_ros_msg::CustomMsg]", custom_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  IMU topic   : %s  [sensor_msgs::Imu]", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "========================================");

  SetLivoxLidarPointCloudCallBack(OnPointCloud, this);
  SetLivoxLidarImuDataCallback(OnImuData, this);
  SetLivoxLidarInfoChangeCallback(OnLidarInfoChange, this);

  if (!LivoxLidarSdkStart()) {
    RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkStart failed.");
    LivoxLidarSdkUninit();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Livox SDK scanning started.");
}

LivoxVulcanNode::~LivoxVulcanNode()
{
  LivoxLidarSdkUninit();
}
