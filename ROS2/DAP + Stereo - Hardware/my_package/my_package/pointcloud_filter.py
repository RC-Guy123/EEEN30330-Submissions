#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')
        self.create_subscription(PointCloud2, '/points2', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/points2_filtered', 10)
        self.get_logger().info('Advanced PointCloudFilter active')

    def callback(self, msg: PointCloud2):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            # Realistic range + remove obvious outliers
            if (0.25 < z < 4.0 and 
                abs(x) < 2.5 and 
                abs(y) < 2.0):
                points.append([x, y, z])

        if len(points) > 150:
            # Simple statistical outlier removal (optional but effective)
            points_np = np.array(points)
            if len(points_np) > 1000:
                z_mean = np.mean(points_np[:, 2])
                z_std = np.std(points_np[:, 2])
                mask = np.abs(points_np[:, 2] - z_mean) < 2.0 * z_std
                points_np = points_np[mask]

            filtered_msg = pc2.create_cloud_xyz32(msg.header, points_np.tolist())
            self.pub.publish(filtered_msg)

            if len(points_np) % 2000 == 0:
                self.get_logger().info(f"Clean points: {len(points_np)}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()