#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool


class LidarObstacleProcessor(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_processor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_topic', '/detected_obstacles')
        self.declare_parameter('obstacle_frame', 'lidar_link')
        self.declare_parameter('cluster_distance_threshold', 0.20)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('default_obstacle_radius', 0.20)
        self.declare_parameter('max_obstacles', 15)
        self.declare_parameter('max_detection_range', 3.0)
        self.declare_parameter('min_detection_range', 0.2)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.obstacle_frame = str(self.get_parameter('obstacle_frame').value)
        self.cluster_distance_threshold = float(
            self.get_parameter('cluster_distance_threshold').value
        )
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.default_obstacle_radius = float(
            self.get_parameter('default_obstacle_radius').value
        )
        self.max_obstacles = int(self.get_parameter('max_obstacles').value)
        self.max_detection_range = float(
            self.get_parameter('max_detection_range').value
        )
        self.min_detection_range = float(
            self.get_parameter('min_detection_range').value
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.obs_pub = self.create_publisher(
            PoseArray,
            self.obstacle_topic,
            10
        )

        self.debug_timer = self.create_timer(2.0, self.debug_status)

        self.get_logger().info('LiDAR obstacle processor started.')

        self.goal_reached = False

        self.goal_sub = self.create_subscription(
            Bool,
            "/goal_reached",
            self.goal_callback,
            10
        )

    def scan_callback(self, msg: LaserScan) -> None:
        if self.goal_reached:
            return
        points = self.laserscan_to_points(msg)
        clusters = self.cluster_points(points)
        obstacles = self.clusters_to_circles(clusters)
        self.get_logger().info(str(obstacles))

        out = PoseArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        for x, y, r in obstacles[:self.max_obstacles]:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = r
            pose.orientation.w = 1.0
            out.poses.append(pose)

        self.obs_pub.publish(out)

    def laserscan_to_points(self, msg: LaserScan) -> List[Tuple[float, float]]:
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r):
                if self.min_detection_range < r < min(msg.range_max, self.max_detection_range):
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))
            angle += msg.angle_increment

        return points

    def cluster_points(self, points: List[Tuple[float, float]]) -> List[List[Tuple[float, float]]]:
        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]
            dist = math.hypot(x2 - x1, y2 - y1)

            if dist < self.cluster_distance_threshold:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]

        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)

        return clusters

    def clusters_to_circles(
        self,
        clusters: List[List[Tuple[float, float]]]
    ) -> List[Tuple[float, float, float]]:
        obstacles = []

        for cluster in clusters:
            closest_x, closest_y = min(
                cluster,
                key=lambda p: math.hypot(p[0], p[1])
            )

            r_min = math.hypot(closest_x, closest_y)
            angle = math.atan2(closest_y, closest_x)

            radius = self.default_obstacle_radius
            cx = (r_min + radius) * math.cos(angle)
            cy = (r_min + radius) * math.sin(angle)

            obstacles.append((cx, cy, radius))

        return obstacles

    def debug_status(self) -> None:
        self.get_logger().info(
            f"alive: scan_topic={self.scan_topic}, obstacle_topic={self.obstacle_topic}, frame={self.obstacle_frame}"
        )

    def goal_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal reached — shutting down LiDAR obstacle processor.")
            self.goal_reached = True


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleProcessor()
    try:
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()