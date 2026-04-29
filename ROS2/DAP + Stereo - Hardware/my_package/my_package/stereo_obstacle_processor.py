#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
import math

class StereoObstacleProcessor(Node):
    def __init__(self):
        super().__init__('stereo_obstacle_processor')

        # Parameters
        self.declare_parameter('points_topic', '/points2_filtered')
        self.declare_parameter('obstacle_topic', '/detected_obstacles')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter('cluster_distance', 0.25)
        self.declare_parameter('min_cluster_size', 80)        # Important for stereo
        self.declare_parameter('robot_radius', 0.35)

        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.cluster_dist = self.get_parameter('cluster_distance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value

        # Subscribers & Publishers
        self.create_subscription(PointCloud2, 
                                self.get_parameter('points_topic').value, 
                                self.pc_callback, 10)
        
        self.obstacle_pub = self.create_publisher(PoseArray, 
                                                 self.get_parameter('obstacle_topic').value, 10)

        self.get_logger().info("Stereo Obstacle Processor started - waiting for points...")

    def pc_callback(self, msg: PointCloud2):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            cam_x = float(p[0])
            cam_y = float(p[1])
            cam_z = float(p[2])
            if self.min_range < cam_z < self.max_range:
                # === CAMERA TO BASE_LINK TRANSFORM ===
                # Rotate: camera +Z forward -> robot +X forward
                robot_x = cam_z * 0.95          # depth becomes forward
                robot_y = -cam_x         # camera right becomes robot left (negative Y)
                robot_z = -cam_y         # camera down becomes robot up (negative)
                
                points.append([robot_x, robot_y, robot_z])

        if len(points) < 200:
            return

        points = np.array(points)

        # Simple clustering (DBSCAN-like but lightweight)
        clusters = self.simple_cluster(points)

        # Convert to PoseArray (for navigation)
        obstacle_array = PoseArray()
        obstacle_array.header = Header(frame_id=self.get_parameter('frame_id').value, stamp=self.get_clock().now().to_msg())

        for cluster in clusters:
            if len(cluster) < self.min_cluster:
                continue
                
            cx = np.mean(cluster[:, 0])
            cy = np.mean(cluster[:, 1])
            dist = math.hypot(cx, cy)

            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            # Simple orientation pointing away from obstacle
            pose.orientation.w = 1.0
            obstacle_array.poses.append(pose)

        if obstacle_array.poses:
            self.obstacle_pub.publish(obstacle_array)
            self.get_logger().debug(f"Published {len(obstacle_array.poses)} obstacles")

    def simple_cluster(self, points: np.ndarray):
        """Very simple distance-based clustering"""
        clusters = []
        visited = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if visited[i]:
                continue
            cluster = [points[i]]
            visited[i] = True

            for j in range(i+1, len(points)):
                if not visited[j] and np.linalg.norm(points[i] - points[j]) < self.cluster_dist:
                    cluster.append(points[j])
                    visited[j] = True
            clusters.append(np.array(cluster))

        return clusters


def main(args=None):
    rclpy.init(args=args)
    node = StereoObstacleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()