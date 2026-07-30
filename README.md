# livox_vulcan_driver2

Livox Mid-360s LiDAR ROS2 Humble 驱动，基于 Livox SDK2。

## 依赖

| 依赖 | 说明 |
|------|------|
| ROS2 Humble | rclcpp, sensor_msgs |
| [livox_ros_msg](./livox_ros_msg/) | 自定义消息 CustomMsg / CustomPoint |
| Livox SDK2 | `liblivox_lidar_sdk_shared.so`，系统安装于 `/usr/local/lib` |

## 构建

```bash
cd <workspace>
source /opt/ros/humble/setup.bash

# 先建 livox_ros_msg（纯消息包，无驱动代码）
colcon build --packages-select livox_ros_msg --symlink-install

# 再建驱动
colcon build --packages-select livox_vulcan_driver2 --symlink-install
source install/setup.bash
```

## 配置

### param.yaml — ROS2 参数

```yaml
/**:
  livox_vulcan_node:
    ros__parameters:
      config_path: ""            # Livox SDK JSON 配置路径
      cloud_topic: "livox/pointcloud"
      custom_topic: "livox/custom_msg"
      imu_topic: "livox/imu"
```

### livox_lidar_config.json — 网络配置

```json
{
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "host_ip": "10.10.10.101",
        "lidar_ip": ["10.10.10.10"],
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  }
}
```

- `host_ip` — **本机**连接雷达网卡的 IP，不能填 `127.0.0.1`
- `lidar_ip` — 雷达设备 IP（不填则走广播自动发现）

## 运行

```bash
source install/setup.bash
ros2 launch livox_vulcan_driver2 livox_mid360s.launch.py
```

启动日志会打印 IP、Topic 名称和数据格式：

```
[livox_vulcan_node] ========================================
[livox_vulcan_node]   Config path : .../livox_lidar_config.json
[livox_vulcan_node]   Host IP     : 10.10.10.101
[livox_vulcan_node]   Cloud topic : livox/pointcloud  [sensor_msgs::PointCloud2]
[livox_vulcan_node]   Custom topic: livox/custom_msg  [livox_ros_msg::CustomMsg]
[livox_vulcan_node]   IMU topic   : livox/imu         [sensor_msgs::Imu]
[livox_vulcan_node] ========================================
[livox_vulcan_node] Lidar discovered - handle: ..., SN: ...
[livox_vulcan_node]   LiDAR IP    : 10.10.10.10
[livox_vulcan_node]   Device type : 35
```

## 发布的 Topic

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `livox/pointcloud` | `sensor_msgs::PointCloud2` | 10 Hz | 点云，坐标 **mm** (int32) |
| `livox/custom_msg` | `livox_ros_msg::CustomMsg` | 10 Hz | 点云，坐标 **m** (float32)，含 timebase / offset_time |
| `livox/imu` | `sensor_msgs::Imu` | 200 Hz | 陀螺仪 + 加速度计 |

### CustomMsg 时间戳

与 `livox_ros_driver2` 一致：

- 包时间戳通过 union `memcpy` 直接按 `int64_t` 解读，不做字节序转换
- `timebase` = 帧内第一个点的绝对时间 (ns)
- `CustomPoint.offset_time` = 单点绝对时间 − timebase (ns)
- `point_interval` = `time_interval × 100 ÷ dot_num` (ns)

## 文件结构

```
livox_vulcan_driver2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── param.yaml
│   └── livox_lidar_config.json
├── launch/
│   └── livox_mid360s.launch.py
├── include/livox_vulcan_driver2/
│   └── livox_vulcan_node.hpp
└── src/
    ├── main.cpp
    └── livox_vulcan_node.cpp
```
