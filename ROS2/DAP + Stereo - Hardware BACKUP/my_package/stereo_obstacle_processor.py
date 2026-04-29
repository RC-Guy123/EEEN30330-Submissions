import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool


class StereoObstacleProcessor(Node):
    def __init__(self):
        super().__init__('stereo_obstacle_processor')

        # Parameters - easy to tune
        self.declare_parameter('obstacle_topic', '/detected_obstacles')
        self.declare_parameter('cluster_distance_threshold', 0.22)
        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('default_obstacle_radius', 0.20)
        self.declare_parameter('max_obstacles', 10)
        self.declare_parameter('max_detection_range', 3.0)
        self.declare_parameter('min_detection_range', 0.25)
        self.declare_parameter('max_height', 0.65)
        self.declare_parameter('min_height', -0.15)

        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.cluster_threshold = self.get_parameter('cluster_distance_threshold').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.default_radius = self.get_parameter('default_obstacle_radius').value
        self.max_obstacles = self.get_parameter('max_obstacles').value
        self.max_range = self.get_parameter('max_detection_range').value
        self.min_range = self.get_parameter('min_detection_range').value
        self.max_h = self.get_parameter('max_height').value
        self.min_h = self.get_parameter('min_height').value

        # Subscriber
        self.create_subscription(
            PointCloud2,
            '/points2',
            self.pointcloud_callback,
            qos_profile_sensor_data
        )

        self.obs_pub = self.create_publisher(PoseArray, self.obstacle_topic, 10)

        self.get_logger().info(f'Stereo Obstacle Processor started → publishing to {self.obstacle_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        points = self.extract_points(msg)
        if len(points) < 10:
            return

        clusters = self.simple_cluster(points)
        obstacles = self.clusters_to_circles(clusters)

        # Publish
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "base_link"   # Important!

        for x, y, r in obstacles[:self.max_obstacles]:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.obs_pub.publish(pose_array)

        if len(obstacles) > 0:
            self.get_logger().info(f"Detected {len(obstacles)} obstacle(s)")

    def extract_points(self, msg: PointCloud2) -> List[Tuple[float, float]]:
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            dist = math.hypot(x, y)
            if not (self.min_range < dist <= self.max_range):
                continue
            if not (self.min_h < z < self.max_h):
                continue

            points.append((x, y))
        return points

    def simple_cluster(self, points):
        if not points:
            return []

        clusters = []
        current = [points[0]]

        for p in points[1:]:
            last = current[-1]
            if math.hypot(p[0]-last[0], p[1]-last[1]) < self.cluster_threshold:
                current.append(p)
            else:
                if len(current) >= self.min_cluster_size:
                    clusters.append(current)
                current = [p]

        if len(current) >= self.min_cluster_size:
            clusters.append(current)

        return clusters

    def clusters_to_circles(self, clusters):
        obstacles = []
        for cluster in clusters:
            cx = sum(pt[0] for pt in cluster) / len(cluster)
            cy = sum(pt[1] for pt in cluster) / len(cluster)
            obstacles.append((cx, cy, self.default_radius))
        return obstacles


def main(args=None):
    rclpy.init(args=args)
    node = StereoObstacleProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()