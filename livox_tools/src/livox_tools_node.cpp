// Copyright (c) 2023 Vulcan
// Email: vulcan@mail.com

#include "livox_tools/livox_tools_node.hpp"

livoxToolsNode::livoxToolsNode(const rclcpp::NodeOptions & options) : Node("livox_tools_node", options), 
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  this->get_parameter("cloud_topic", cloud_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("trans_point_topic", trans_point_topic_);
  this->get_parameter("laserscan_topic", laserscan_topic_);
  this->get_parameter("base_link", base_link_);
  this->get_parameter("min_z", min_z_);
  this->get_parameter("max_z", max_z_);
  this->get_parameter("enable_log", enable_log_);
  this->get_parameter("range_min", range_min_);
  this->get_parameter("range_max", range_max_);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&livoxToolsNode::parameter_callback, this, std::placeholders::_1));

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, 10,
    std::bind(&livoxToolsNode::pointcloud_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10,
    std::bind(&livoxToolsNode::imu_callback, this, std::placeholders::_1));
  
  trans_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(trans_point_topic_, 10);
  laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic_, 10);
}

void livoxToolsNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Orientation is already computed by livox_vulcan_driver2.
  // Just buffer the IMU message for point cloud de-skewing.
  imu_data_queue_.push_back(*msg);
  if (static_cast<int>(imu_data_queue_.size()) > imu_quee_) {
    imu_data_queue_.pop_front();
  }
}


void livoxToolsNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (enable_log_) RCLCPP_INFO(this->get_logger(), "Get point cloud: %d x %d",
    msg->width, msg->height);

  if (!lidar_pose_init_) {
    // Query the latest available transform (TimePointZero), not the cloud
    // timestamp: the static TF is latched and may not have a sample at the
    // exact point-cloud stamp, which spams TF_OLD_DATA.
    try {
      geometry_msgs::msg::TransformStamped lidar_transformStamped =
        tf_buffer_.lookupTransform(
          base_link_, msg->header.frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(0.2));
      const auto & lidar_t = lidar_transformStamped.transform.translation;
      const auto & lidar_quat = lidar_transformStamped.transform.rotation;
      Eigen::Quaterniond lidar_q(lidar_quat.w, lidar_quat.x, lidar_quat.y, lidar_quat.z);
      Eigen::Matrix3d R = lidar_q.toRotationMatrix();
      lidar_init_rpy_ = R.eulerAngles(0, 1, 2);
      Eigen::Vector3d lidar_trans(lidar_t.x, lidar_t.y, lidar_t.z);
      lidar_pose_ = Sophus::SE3d(lidar_q, lidar_trans);
      lidar_pose_init_ = true;
      RCLCPP_INFO(this->get_logger(),
        "TF %s <- %s acquired: t=(%.4f, %.4f, %.4f)",
        base_link_.c_str(), msg->header.frame_id.c_str(),
        lidar_t.x, lidar_t.y, lidar_t.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s <- %s: %s",
        base_link_.c_str(), msg->header.frame_id.c_str(), ex.what());
    }
    return;
  }
  {
    pcl::PointCloud<pcl::PointXYZI> cloud_in, cloud_trans;
    pcl::fromROSMsg(*msg, cloud_in);

    // Transform to base_link
    pcl::transformPointCloud(cloud_in, cloud_trans, lidar_pose_.matrix().cast<float>());

    // De-skew with IMU orientation
    sensor_msgs::msg::Imu closest_imu_data;
    if (find_closest_imu_data(msg->header.stamp, closest_imu_data)) {
      tf2::Quaternion q(
        closest_imu_data.orientation.x, closest_imu_data.orientation.y,
        closest_imu_data.orientation.z, closest_imu_data.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
      Eigen::Affine3f T = Eigen::Affine3f::Identity();
      T.rotate(Eigen::AngleAxisf(pitch - lidar_init_rpy_[1], Eigen::Vector3f::UnitY()));
      pcl::transformPointCloud(cloud_trans, cloud_trans, T.matrix());

      Eigen::Affine3f T_r = Eigen::Affine3f::Identity();
      T_r.rotate(Eigen::AngleAxisf(roll - lidar_init_rpy_[0], Eigen::Vector3f::UnitX()));
      pcl::transformPointCloud(cloud_trans, cloud_trans, T_r.matrix());
    }
    auto trans_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_trans, *trans_msg);
    trans_msg->header.stamp = msg->header.stamp;
    trans_msg->header.frame_id = base_link_;
    trans_point_pub_->publish(std::move(trans_msg));

    // LaserScan
    pcl::PassThrough<pcl::PointXYZI> lidar_pass;
    lidar_pass.setInputCloud(cloud_trans.makeShared());
    lidar_pass.setFilterFieldName("z");
    lidar_pass.setFilterLimits(min_z_, max_z_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cutlidar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    lidar_pass.filter(*cutlidar_cloud);
    auto scan_msg = createLaserScan(
        msg->header, cutlidar_cloud,
        scan_angle_min_, scan_angle_max_,
        scan_angle_increment_,
        range_min_, range_max_, base_link_
    );
    laserscan_publisher_->publish(*scan_msg);
  }
}

std::shared_ptr<sensor_msgs::msg::LaserScan> livoxToolsNode::createLaserScan(
    const std_msgs::msg::Header &header,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud,
    double scan_angle_min_,
    double scan_angle_max_,
    double scan_angle_increment_,
    double range_min_,
    double range_max_,
    const std::string frame_id)
{
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header = header;
    scan->header.frame_id = frame_id;
    scan->angle_min = scan_angle_min_;
    scan->angle_max = scan_angle_max_;
    scan->angle_increment = scan_angle_increment_;
    scan->range_min = range_min_;
    scan->range_max = range_max_;

    int num_readings = static_cast<int>((scan->angle_max - scan->angle_min) / scan->angle_increment);
    scan->ranges.resize(num_readings, scan->range_max);

    for (const auto &point : filtered_cloud->points) {
        double angle = std::atan2(point.y, point.x);
        double range = std::hypot(point.x, point.y);
        if (angle >= scan->angle_min && angle <= scan->angle_max) {
            int index = static_cast<int>((angle - scan->angle_min) / scan->angle_increment);
            if (range >= scan->range_min && range <= scan->range_max)
                scan->ranges[index] = std::min(scan->ranges[index], static_cast<float>(range));
        }
    }
    return scan;
}

bool livoxToolsNode::find_closest_imu_data(
  const rclcpp::Time & cloud_stamp, sensor_msgs::msg::Imu & closest_imu_data)
{
  if (imu_data_queue_.empty()) return false;
  auto to_ns = [](const builtin_interfaces::msg::Time & t) {
    return static_cast<int64_t>(t.sec) * 1'000'000'000LL + t.nanosec;
  };
  int64_t cloud_ns = to_ns(cloud_stamp);
  auto it = std::min_element(imu_data_queue_.begin(), imu_data_queue_.end(),
    [&](const sensor_msgs::msg::Imu & a, const sensor_msgs::msg::Imu & b) {
      return std::abs(cloud_ns - to_ns(a.header.stamp)) <
             std::abs(cloud_ns - to_ns(b.header.stamp));
    });
  if (it != imu_data_queue_.end()) { closest_imu_data = *it; return true; }
  return false;
}

rcl_interfaces::msg::SetParametersResult livoxToolsNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (p.get_name() == "min_z") min_z_ = p.as_double();
    if (p.get_name() == "max_z") max_z_ = p.as_double();
    if (p.get_name() == "enable_log") enable_log_ = p.as_bool();
  }
  return result;
}
