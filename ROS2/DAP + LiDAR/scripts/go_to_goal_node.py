#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoToGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("go_to_goal_node")

        self.x = None
        self.y = None
        self.yaw = None
        self.goal_reached = False
        self.have_odom = False

        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 3.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("linear_gain_x", 0.8)
        self.declare_parameter("linear_gain_y", 0.8)
        self.declare_parameter("angular_gain", 2.5)

        self.declare_parameter("max_linear_speed_x", 0.4)
        self.declare_parameter("max_linear_speed_y", 0.4)
        self.declare_parameter("max_angular_speed", 1.5)

        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("yaw_tolerance", 0.10)

        self.declare_parameter("odom_topic", "/mecanum_drive_controller/odometry")
        self.declare_parameter("cmd_topic", "/mecanum_drive_controller/reference")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("command_frame_id", "chassis")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

        self.k_vx = float(self.get_parameter("linear_gain_x").value)
        self.k_vy = float(self.get_parameter("linear_gain_y").value)
        self.k_w = float(self.get_parameter("angular_gain").value)

        self.max_vx = float(self.get_parameter("max_linear_speed_x").value)
        self.max_vy = float(self.get_parameter("max_linear_speed_y").value)
        self.max_w = float(self.get_parameter("max_angular_speed").value)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.command_frame_id = str(self.get_parameter("command_frame_id").value)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10,
        )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info("Mecanum go-to-goal node started.")
        self.get_logger().info(
            f"  Goal position: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        self.get_logger().info(f"  Goal yaw: {self.goal_yaw:.2f} rad")
        self.get_logger().info(f"  Odom topic: {self.odom_topic}")
        self.get_logger().info(f"  Cmd topic:  {self.cmd_topic}")
        self.get_logger().info(
            f"  Gains: vx={self.k_vx:.2f}, vy={self.k_vy:.2f}, w={self.k_w:.2f}"
        )
        self.get_logger().info(
            f"  Limits: max_vx={self.max_vx:.2f}, max_vy={self.max_vy:.2f}, max_w={self.max_w:.2f}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info("Received first odometry message.")

        self.get_logger().info(
            f"ODOM -> x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}",
            throttle_duration_sec=1.0,
        )

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def world_error_to_robot_frame(self, dx_world: float, dy_world: float) -> tuple[float, float]:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        dx_robot = cos_yaw * dx_world + sin_yaw * dy_world
        dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world
        return dx_robot, dy_robot

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def make_cmd(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
    ) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame_id
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.angular.z = angular_z
        return msg

    def stop_robot(self, reason: str = "Stopping robot.") -> None:
        self.cmd_pub.publish(self.make_cmd())
        self.get_logger().info(reason, throttle_duration_sec=1.0)

    def control_loop(self) -> None:
        if not self.have_odom or self.x is None or self.y is None or self.yaw is None:
            self.get_logger().warning(
                f"Waiting for odometry on {self.odom_topic}...",
                throttle_duration_sec=2.0,
            )
            return

        if self.goal_reached:
            self.stop_robot("Goal already reached. Holding stop.")
            return

        dx_world = self.goal_x - self.x
        dy_world = self.goal_y - self.y
        distance = math.sqrt(dx_world * dx_world + dy_world * dy_world)

        dx_robot, dy_robot = self.world_error_to_robot_frame(dx_world, dy_world)
        yaw_error = wrap_to_pi(self.goal_yaw - self.yaw)

        self.get_logger().info(
            "CONTROL -> "
            f"goal=({self.goal_x:.2f}, {self.goal_y:.2f}, yaw={self.goal_yaw:.2f}) "
            f"pos=({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f}) "
            f"err_world=({dx_world:.2f}, {dy_world:.2f}) "
            f"err_robot=({dx_robot:.2f}, {dy_robot:.2f}) "
            f"dist={distance:.3f} "
            f"yaw_error={yaw_error:.3f}",
            throttle_duration_sec=0.5,
        )

        if distance < self.goal_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.goal_reached = True
            self.stop_robot("Goal reached.")
            return

        linear_x = self.k_vx * dx_robot
        linear_y = self.k_vy * dy_robot
        angular_z = self.k_w * yaw_error

        linear_x = self.clamp(linear_x, -self.max_vx, self.max_vx)
        linear_y = self.clamp(linear_y, -self.max_vy, self.max_vy)
        angular_z = self.clamp(angular_z, -self.max_w, self.max_w)

        if distance < self.goal_tolerance:
            linear_x = 0.0
            linear_y = 0.0

        self.get_logger().info(
            f"CMD -> linear_x={linear_x:.3f}, linear_y={linear_y:.3f}, angular_z={angular_z:.3f}",
            throttle_duration_sec=0.5,
        )

        self.cmd_pub.publish(self.make_cmd(linear_x, linear_y, angular_z))

    def destroy_node(self):
        try:
            self.cmd_pub.publish(self.make_cmd())
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()