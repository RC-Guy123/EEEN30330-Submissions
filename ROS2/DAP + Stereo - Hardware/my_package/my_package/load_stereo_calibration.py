#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class CalibrationLoader(Node):
    def __init__(self):
        super().__init__('calibration_loader')

        # Use the same paths as your rectify nodes
        self.left_yaml  = '/home/austin/ros2_ws/src/my_package/config/left.yaml'
        self.right_yaml = '/home/austin/ros2_ws/src/my_package/config/right.yaml'

        self.scale = 320.0 / 1640.0   # 320/1640

        self.left_pub  = self.create_publisher(CameraInfo, '/left/camera_info_fixed',  10)
        self.right_pub = self.create_publisher(CameraInfo, '/right/camera_info_fixed', 10)

        self.create_subscription(CameraInfo, '/left/camera_info',  self.left_cb,  10)
        self.create_subscription(CameraInfo, '/right/camera_info', self.right_cb, 10)

        self.get_logger().info(f"Calibration loader ready. Scale = {self.scale:.4f}")

    def load_yaml(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def scale_matrix(self, data, scale):
        """Scale ALL elements in the projection matrix (including Tx)"""
        return [x * scale for x in data]

    def left_cb(self, msg):
        calib = self.load_yaml(self.left_yaml)
        msg.p = self.scale_matrix(calib['projection_matrix']['data'], self.scale)
        msg.k = [x * self.scale for x in calib['camera_matrix']['data']]
        msg.d = calib['distortion_coefficients']['data']
        msg.r = calib['rectification_matrix']['data']
        msg.width = 320
        msg.height = 240
        msg.distortion_model = calib['distortion_model']
        self.left_pub.publish(msg)

    def right_cb(self, msg):
        calib = self.load_yaml(self.right_yaml)
        msg.p = self.scale_matrix(calib['projection_matrix']['data'], self.scale)
        msg.k = [x * self.scale for x in calib['camera_matrix']['data']]
        msg.d = calib['distortion_coefficients']['data']
        msg.r = calib['rectification_matrix']['data']
        msg.width = 320
        msg.height = 240
        msg.distortion_model = calib['distortion_model']
        self.right_pub.publish(msg)

def main():
    rclpy.init()
    node = CalibrationLoader()
    rclpy.spin(node)

if __name__ == '__main__':
    main()