#!/usr/bin/env python3
import csv
import math
import os
from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    from scipy.io import savemat
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathLogger(Node):
    def __init__(self) -> None:
        super().__init__('path_logger')

        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odometry')
        self.declare_parameter('output_dir', os.path.expanduser('~/ros2_ws/data'))
        self.declare_parameter('csv_filename', 'trajectory.csv')
        self.declare_parameter('mat_filename', 'trajectory.mat')
        self.declare_parameter('save_mat', True)
        self.declare_parameter('min_sample_distance', 0.01)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.csv_filename = str(self.get_parameter('csv_filename').value)
        self.mat_filename = str(self.get_parameter('mat_filename').value)
        self.save_mat = bool(self.get_parameter('save_mat').value)
        self.min_sample_distance = float(self.get_parameter('min_sample_distance').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.times: List[float] = []
        self.xs: List[float] = []
        self.ys: List[float] = []
        self.yaws: List[float] = []
        self.vxs: List[float] = []
        self.vys: List[float] = []
        self.wzs: List[float] = []

        self.last_x = None
        self.last_y = None

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Logging path from {self.odom_topic}')
        self.get_logger().info(f'Output directory: {self.output_dir}')

    def odom_callback(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_sample_distance:
                return

        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        self.times.append(t)
        self.xs.append(x)
        self.ys.append(y)
        self.yaws.append(yaw)
        self.vxs.append(vx)
        self.vys.append(vy)
        self.wzs.append(wz)

        self.last_x = x
        self.last_y = y

    def save_csv(self) -> str:
        csv_path = os.path.join(self.output_dir, self.csv_filename)
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'])
            for row in zip(
                self.times, self.xs, self.ys, self.yaws,
                self.vxs, self.vys, self.wzs
            ):
                writer.writerow(row)
        return csv_path

    def save_mat_file(self) -> str:
        mat_path = os.path.join(self.output_dir, self.mat_filename)
        if not SCIPY_AVAILABLE:
            self.get_logger().warn('scipy not available; skipping .mat export')
            return ''

        savemat(mat_path, {
            'time': self.times,
            'x': self.xs,
            'y': self.ys,
            'yaw': self.yaws,
            'vx': self.vxs,
            'vy': self.vys,
            'wz': self.wzs,
        })
        return mat_path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        csv_path = node.save_csv()
        node.get_logger().info(f'Saved CSV: {csv_path}')

        if node.save_mat:
            mat_path = node.save_mat_file()
            if mat_path:
                node.get_logger().info(f'Saved MAT: {mat_path}')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()