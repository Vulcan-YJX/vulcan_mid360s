#!/usr/bin/env python3
"""
Compare timestamps between:
  1. /livox/pointcloud  and  /livox/custom_msg  header stamps
  2. System time (node.now()) and LiDAR message stamp

Pairs messages by matching timestamps (both share the same timebase).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from livox_ros_msg.msg import CustomMsg


def _to_ns(stamp) -> int:
    return stamp.nanosec + stamp.sec * 1_000_000_000


class TimestampComparator(Node):
    def __init__(self):
        super().__init__("timestamp_comparator")

        self.cloud_sub = self.create_subscription(
            PointCloud2, "/livox/pointcloud", self.cloud_cb, 10
        )
        self.custom_sub = self.create_subscription(
            CustomMsg, "/livox/custom_msg", self.custom_cb, 10
        )

        # Match by timestamp key -> stored stamp
        self.cloud_map = {}
        self.custom_map = {}

        self.pair_count = 0
        self.pair_diff_sum = 0
        self.pair_diff_max = 0

        self.sys_count = 0
        self.sys_diff_sum = 0
        self.sys_diff_max = 0

        self.get_logger().info(
            "Comparing /livox/pointcloud <-> /livox/custom_msg  "
            "and  system_time <-> lidar_time ..."
        )

    def cloud_cb(self, msg: PointCloud2):
        lidar_ns = _to_ns(msg.header.stamp)
        sys_ns = _to_ns(self.get_clock().now().to_msg())
        diff_ns = abs(sys_ns - lidar_ns)

        self.sys_count += 1
        self.sys_diff_sum += diff_ns
        if diff_ns > self.sys_diff_max:
            self.sys_diff_max = diff_ns

        if self.sys_count % 10 == 0:
            avg = self.sys_diff_sum / self.sys_count
            self.get_logger().info(
                f"[sys-vs-lidar  #{self.sys_count}] "
                f"system={sys_ns}ns  lidar={lidar_ns}ns  "
                f"diff={diff_ns / 1e6:.3f}ms  "
                f"avg={avg / 1e6:.3f}ms  max={self.sys_diff_max / 1e6:.3f}ms"
            )

        # Store for pairwise matching
        self.cloud_map[lidar_ns] = msg.header.stamp
        self._try_match(lidar_ns)

    def custom_cb(self, msg: CustomMsg):
        lidar_ns = _to_ns(msg.header.stamp)
        self.custom_map[lidar_ns] = msg.header.stamp
        self._try_match(lidar_ns)

    def _try_match(self, key_ns: int):
        if key_ns in self.cloud_map and key_ns in self.custom_map:
            cloud_ns = _to_ns(self.cloud_map.pop(key_ns))
            custom_ns = _to_ns(self.custom_map.pop(key_ns))
            diff_ns = abs(cloud_ns - custom_ns)

            self.pair_count += 1
            self.pair_diff_sum += diff_ns
            if diff_ns > self.pair_diff_max:
                self.pair_diff_max = diff_ns

            if self.pair_count % 10 == 0:
                avg = self.pair_diff_sum / self.pair_count
                self.get_logger().info(
                    f"[msg-pair     #{self.pair_count}] "
                    f"cloud={cloud_ns}ns  custom={custom_ns}ns  "
                    f"diff={diff_ns}ns  avg={avg:.0f}ns  max={self.pair_diff_max}ns"
                )


def main():
    rclpy.init()
    node = TimestampComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
