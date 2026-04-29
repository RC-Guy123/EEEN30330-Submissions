#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')
        self.subscription = self.create_subscription(
            PointCloud2, '/points2', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/points2_filtered', 10)
        self.get_logger().info('PointCloudFilter running → /points2_filtered')

    def callback(self, msg: PointCloud2):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if np.isfinite(z) and 0.25 < z < 4.0:   # Clip to realistic range
                points.append([x, y, z])

        if len(points) > 200:
            filtered_msg = pc2.create_cloud_xyz32(msg.header, points)
            self.publisher.publish(filtered_msg)
            if len(points) % 1000 == 0:
                self.get_logger().info(f"Published {len(points)} valid points")
        else:
            self.get_logger().debug(f"Too few valid points: {len(points)}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()