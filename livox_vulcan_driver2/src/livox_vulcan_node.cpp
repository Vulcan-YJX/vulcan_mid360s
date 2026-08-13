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

#include <csignal>
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
// Timer callback: cut one frame and hand it to two independent workers.
// No point conversion or ROS publishing is done while holding cloud_mutex_.
// ---------------------------------------------------------------------------
void LivoxVulcanNode::dispatch_accumulated_frame()
{
  auto frame = std::make_shared<PointCloudFrame>();
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_points_.empty()) {
      return;
    }
    frame->points.swap(accumulated_points_);
    frame->lidar_handle = lidar_handle_;
  }

  frame->timebase_ns = frame->points.front().offset_time_ns;

  // Soft time sync: wait for N frames to stabilize, then compute offset
  if (time_sync_soft_ && !first_frame_received_.load(std::memory_order_acquire)) {
    frame_count_++;
    if (frame_count_ >= time_sync_wait_count_) {
      int64_t sys_ns = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
      int64_t offset_ns = sys_ns - static_cast<int64_t>(frame->timebase_ns);
      time_offset_ns_.store(offset_ns, std::memory_order_relaxed);
      first_frame_received_.store(true, std::memory_order_release);
      RCLCPP_INFO(this->get_logger(),
        "Soft time sync (after %d frames): system=%ld ns  lidar=%lu ns  "
        "offset=%ld ns (%.3f ms)",
        frame_count_, sys_ns, frame->timebase_ns, offset_ns,
        offset_ns / 1e6);
    }
  }

  // Apply offset if soft sync is active
  frame->stamp_ns = frame->timebase_ns;
  if (time_sync_soft_ && first_frame_received_.load(std::memory_order_acquire)) {
    frame->stamp_ns = static_cast<uint64_t>(
      static_cast<int64_t>(frame->timebase_ns) +
      time_offset_ns_.load(std::memory_order_relaxed));
  }

  // Feed the latency-sensitive CustomMsg queue first. Each queue has its own
  // lock and capacity, so a slow PointCloud2 worker can only drop its own old
  // frames and can never delay the CustomMsg worker.
  {
    std::lock_guard<std::mutex> lock(custom_queue_mutex_);
    if (custom_queue_.size() >= kMaxPublisherQueueSize) {
      custom_queue_.pop_front();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "CustomMsg publisher is behind; dropping its oldest frame");
    }
    custom_queue_.push_back(frame);
  }
  custom_queue_cv_.notify_one();

  {
    std::lock_guard<std::mutex> lock(pointcloud_queue_mutex_);
    if (pointcloud_queue_.size() >= kMaxPublisherQueueSize) {
      pointcloud_queue_.pop_front();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "PointCloud2 publisher is behind; dropping its oldest frame");
    }
    pointcloud_queue_.push_back(std::move(frame));
  }
  pointcloud_queue_cv_.notify_one();
}

void LivoxVulcanNode::pointcloud_publisher_loop()
{
  while (publisher_threads_running_.load(std::memory_order_acquire)) {
    std::shared_ptr<const PointCloudFrame> frame;
    {
      std::unique_lock<std::mutex> lock(pointcloud_queue_mutex_);
      pointcloud_queue_cv_.wait(lock, [this] {
        return !publisher_threads_running_.load(std::memory_order_acquire) ||
               !pointcloud_queue_.empty();
      });
      if (!publisher_threads_running_.load(std::memory_order_acquire)) {
        break;
      }
      frame = std::move(pointcloud_queue_.front());
      pointcloud_queue_.pop_front();
    }
    publish_pointcloud_frame(*frame);
  }
}

void LivoxVulcanNode::custom_publisher_loop()
{
  while (publisher_threads_running_.load(std::memory_order_acquire)) {
    std::shared_ptr<const PointCloudFrame> frame;
    {
      std::unique_lock<std::mutex> lock(custom_queue_mutex_);
      custom_queue_cv_.wait(lock, [this] {
        return !publisher_threads_running_.load(std::memory_order_acquire) ||
               !custom_queue_.empty();
      });
      if (!publisher_threads_running_.load(std::memory_order_acquire)) {
        break;
      }
      frame = std::move(custom_queue_.front());
      custom_queue_.pop_front();
    }
    publish_custom_frame(*frame);
  }
}

void LivoxVulcanNode::publish_pointcloud_frame(const PointCloudFrame & frame)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = rclcpp::Time(frame.stamp_ns);
  msg->header.frame_id = "livox_frame";

  // PCL-compatible: float x/y/z (meters) + float intensity
  constexpr uint32_t POINT_STEP = 16;  // 4 floats = 16 bytes
  msg->height = 1;
  msg->width = static_cast<uint32_t>(frame.points.size());
  msg->is_bigendian = false;
  msg->point_step = POINT_STEP;
  msg->row_step = msg->width * msg->point_step;
  msg->is_dense = true;

  sensor_msgs::msg::PointField f;
  f.name = "x";
  f.offset = 0;
  f.datatype = sensor_msgs::msg::PointField::FLOAT32;
  f.count = 1;
  msg->fields.push_back(f);
  f.name = "y";
  f.offset = 4;
  msg->fields.push_back(f);
  f.name = "z";
  f.offset = 8;
  msg->fields.push_back(f);
  f.name = "intensity";
  f.offset = 12;
  msg->fields.push_back(f);

  msg->data.resize(frame.points.size() * POINT_STEP);
  for (size_t i = 0; i < frame.points.size(); ++i) {
    const auto & pt = frame.points[i].point;
    uint8_t * dst = &msg->data[i * POINT_STEP];
    float fx = pt.x * 1e-3f;
    float fy = pt.y * 1e-3f;
    float fz = pt.z * 1e-3f;
    float fi = static_cast<float>(pt.reflectivity);
    memcpy(dst + 0,  &fx, 4);
    memcpy(dst + 4,  &fy, 4);
    memcpy(dst + 8,  &fz, 4);
    memcpy(dst + 12, &fi, 4);
  }
  cloud_pub_->publish(std::move(msg));
}

void LivoxVulcanNode::publish_custom_frame(const PointCloudFrame & frame)
{
  auto msg = std::make_unique<CustomMsg>();
  msg->header.stamp = rclcpp::Time(frame.stamp_ns);
  msg->header.frame_id = "livox_frame";
  msg->timebase = frame.stamp_ns;
  msg->point_num = static_cast<uint32_t>(frame.points.size());
  msg->lidar_id = static_cast<uint8_t>(frame.lidar_handle & 0xFF);
  msg->rsvd.fill(0);

  msg->points.reserve(frame.points.size());
  for (const auto & pwt : frame.points) {
    CustomPoint cp;
    cp.offset_time  = static_cast<uint32_t>(pwt.offset_time_ns - frame.timebase_ns);
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

void LivoxVulcanNode::start_publisher_threads()
{
  publisher_threads_running_.store(true, std::memory_order_release);
  pointcloud_publisher_thread_ =
    std::thread(&LivoxVulcanNode::pointcloud_publisher_loop, this);
  custom_publisher_thread_ =
    std::thread(&LivoxVulcanNode::custom_publisher_loop, this);
}

void LivoxVulcanNode::stop_publisher_threads()
{
  publisher_threads_running_.store(false, std::memory_order_release);
  pointcloud_queue_cv_.notify_all();
  custom_queue_cv_.notify_all();

  if (pointcloud_publisher_thread_.joinable()) {
    pointcloud_publisher_thread_.join();
  }
  if (custom_publisher_thread_.joinable()) {
    custom_publisher_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(pointcloud_queue_mutex_);
    pointcloud_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(custom_queue_mutex_);
    custom_queue_.clear();
  }
}

// ---------------------------------------------------------------------------
// Publish IMU packet
// ---------------------------------------------------------------------------
void LivoxVulcanNode::publish_imu_packet(LivoxLidarEthernetPacket * data)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();

  uint64_t ts_raw = GetPacketTimestamp(data);
  uint64_t stamp_ns = ts_raw;
  if (time_sync_soft_ && first_frame_received_.load(std::memory_order_acquire)) {
    stamp_ns = static_cast<uint64_t>(
      static_cast<int64_t>(ts_raw) + time_offset_ns_.load(std::memory_order_relaxed));
  }
  msg->header.stamp = rclcpp::Time(stamp_ns);
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

  // --- AHRS orientation estimation ---
  {
    rclcpp::Time stamp = msg->header.stamp;
    if (!imu_initialised_) {
      last_imu_stamp_ = stamp;
      imu_initialised_ = true;
    } else {
      float dt = (stamp - last_imu_stamp_).seconds();
      last_imu_stamp_ = stamp;

      if (dt > 0.0f && dt <= 0.2f) {
        constexpr float RAD2DEG = 57.2957795f;
        constexpr float MS2_TO_G = 1.0f / 9.80665f;

        FusionVector gyro = {
          static_cast<float>(msg->angular_velocity.x) * RAD2DEG,
          static_cast<float>(msg->angular_velocity.y) * RAD2DEG,
          static_cast<float>(msg->angular_velocity.z) * RAD2DEG
        };
        FusionVector accel = {
          static_cast<float>(msg->linear_acceleration.x) * MS2_TO_G,
          static_cast<float>(msg->linear_acceleration.y) * MS2_TO_G,
          static_cast<float>(msg->linear_acceleration.z) * MS2_TO_G
        };
        float norm = sqrtf(accel.axis.x * accel.axis.x +
                           accel.axis.y * accel.axis.y +
                           accel.axis.z * accel.axis.z);
        if (norm > 0.0f) {
          accel.axis.x /= norm;
          accel.axis.y /= norm;
          accel.axis.z /= norm;
        }
        FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro, accel, dt);
        const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs_);
        msg->orientation.w = quat.element.w;
        msg->orientation.x = quat.element.x;
        msg->orientation.y = quat.element.y;
        msg->orientation.z = quat.element.z;
      }
    }
  }

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

  if (!this->has_parameter("time_sync_soft")) {
    this->declare_parameter<bool>("time_sync_soft", false);
  }
  time_sync_soft_ = this->get_parameter("time_sync_soft").as_bool();

  if (!this->has_parameter("time_sync_wait_count")) {
    this->declare_parameter<int>("time_sync_wait_count", 50);
  }
  time_sync_wait_count_ = this->get_parameter("time_sync_wait_count").as_int();

  FusionAhrsInitialise(&ahrs_);

  if (config_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No config_path provided; Livox SDK cannot initialize.");
    return;
  }

  // QoS queue depths follow livox_ros_driver2's scheme for a single lidar
  // with dedicated (non-shared) topics: kMinEthPacketQueueSize(32)/8 = 4 for
  // point cloud / custom msg, kMinEthPacketQueueSize(32)*2 = 64 for imu.
  // Publishers stay on the rclcpp default profile (Reliable/Volatile/KeepLast)
  // to match livox_ros_driver2's plain-depth create_publisher() calls.
  cloud_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 4);
  custom_pub_ = this->create_publisher<CustomMsg>(custom_topic_, 4);
  imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 64);

  start_publisher_threads();

  frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LivoxVulcanNode::dispatch_accumulated_frame, this));

  // Disable SDK console logs before init (sets the global flag that
  // InitLogger checks, so the console sink is never created)
  extern bool is_console_log_enable;
  is_console_log_enable = false;

  if (!LivoxLidarSdkInit(config_path_.c_str())) {
    RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed.");
    std::raise(SIGSEGV);
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

  RCLCPP_INFO(this->get_logger(), "  Soft sync   : %s", time_sync_soft_ ? "ON" : "OFF");
  RCLCPP_INFO(
    this->get_logger(), "  Cloud topic : %s  [sensor_msgs::PointCloud2]",
    cloud_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "  Custom topic: %s  [livox_ros_msg::CustomMsg]",
    custom_topic_.c_str());
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
  if (frame_timer_) {
    frame_timer_->cancel();
  }
  LivoxLidarSdkUninit();
  stop_publisher_threads();
}
