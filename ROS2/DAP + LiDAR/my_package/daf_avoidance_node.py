#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import TwistStamped, PoseArray, PointStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, value))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DAFObstacleAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__('daf_obstacle_avoidance')

        # --- Parameters ---
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('reach_thresh_goal', 0.5)
        self.declare_parameter('r_robot', 0.55)

        # Goal attraction and isotropic damping
        self.declare_parameter('k1', 1.0)
        self.declare_parameter('k2', 3.0)

        # DAF obstacle-normal dissipation
        self.declare_parameter('k_daf', 1.0)
        self.declare_parameter('eps1', 0.6)
        self.declare_parameter('eps2', 1.8)
        self.declare_parameter('beta_max', 8.0)

        self.declare_parameter('goal', [7.0, 5.0])

        # fixed obstacles as string: "x1,y1,r1;x2,y2,r2;..."
        self.declare_parameter('obstacles', '')

        # motion constraints
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 0.2)
        self.declare_parameter('max_virtual_speed', 0.5)
        self.declare_parameter('heading_kp', 0.2)

        self.declare_parameter('use_ground_truth_pose', False)
        self.declare_parameter('ground_truth_pose_topic', '/ground_truth_pose')

        # live obstacle input
        self.declare_parameter('obstacle_topic', '/detected_obstacles')
        self.declare_parameter('use_live_obstacles', True)
        self.declare_parameter('obstacle_timeout', 0.5)
        self.declare_parameter('robot_base_frame', 'chassis')

        # odom / command topics
        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odometry')
        self.declare_parameter('cmd_topic', '/mecanum_drive_controller/reference')

        self.dt = float(self.get_parameter('dt').value)
        self.reach_thresh_goal = float(self.get_parameter('reach_thresh_goal').value)
        self.r_robot = float(self.get_parameter('r_robot').value)

        self.k1 = float(self.get_parameter('k1').value)
        self.k2 = float(self.get_parameter('k2').value)

        self.k_daf = float(self.get_parameter('k_daf').value)
        self.eps1 = float(self.get_parameter('eps1').value)
        self.eps2 = float(self.get_parameter('eps2').value)
        self.beta_max = float(self.get_parameter('beta_max').value)

        goal_list = self.get_parameter('goal').value
        self.goal = [float(goal_list[0]), float(goal_list[1])]

        obs_str = str(self.get_parameter('obstacles').value)
        self.obstacles = self._parse_obstacles_string(obs_str)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_virtual_speed = float(self.get_parameter('max_virtual_speed').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)

        self.obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.use_live_obstacles = bool(self.get_parameter('use_live_obstacles').value)
        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.use_ground_truth_pose = bool(self.get_parameter('use_ground_truth_pose').value)
        self.ground_truth_pose_topic = str(self.get_parameter('ground_truth_pose_topic').value)

        # --- State from odometry ---
        self.have_odom = False
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self.vx_world = 0.0
        self.vy_world = 0.0

        # internal virtual desired velocity in world frame
        self.vx_cmd_world = 0.0
        self.vy_cmd_world = 0.0

        self.reached = False
        self.collision = False
        self.last_obstacle_msg_time = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- ROS interfaces ---
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.gt_pose_sub = None
        if self.use_ground_truth_pose:
            self.gt_pose_sub = self.create_subscription(
                Pose,
                self.ground_truth_pose_topic,
                self.ground_truth_pose_callback,
                10
            )

        self.obs_sub = self.create_subscription(
            PoseArray,
            self.obstacle_topic,
            self.obstacle_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10
        )

        self.goal_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('DAF obstacle avoidance node started.')

    def _parse_obstacles_string(self, s: str) -> List[Tuple[float, float, float]]:
        s = s.strip()
        if not s:
            return []

        obs = []
        groups = s.split(';')
        for group in groups:
            parts = [p.strip() for p in group.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    "Parameter 'obstacles' must use format 'x1,y1,r1;x2,y2,r2;...'"
                )
            x, y, r = map(float, parts)
            obs.append((x, y, r))
        return obs

    @staticmethod
    def smoothstep01(x: float) -> float:
        x = clamp(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    def beta_daf(self, d: float) -> float:
        """
        Smooth DAF obstacle-normal damping gain.

        d = distance between obstacle boundary and robot boundary:
            dist(center, center) - (r_obs + r_robot)

        beta = 0 for d >= eps2
        beta = beta_max for d <= eps1
        beta transitions smoothly between eps2 and eps1
        """
        if d >= self.eps2:
            return 0.0
        if d <= self.eps1:
            return self.beta_max

        s = (self.eps2 - d) / (self.eps2 - self.eps1)
        return self.beta_max * self.smoothstep01(s)

    def odom_callback(self, msg: Odometry) -> None:
        if not self.use_ground_truth_pose:
            self.have_odom = True

            self.px = msg.pose.pose.position.x
            self.py = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # still use odom twist for damping
        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y

        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        self.vx_world = cy * vx_body - sy * vy_body
        self.vy_world = sy * vx_body + cy * vy_body

    def obstacle_callback(self, msg: PoseArray) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn('Obstacle message has empty frame_id.')
            return

        obstacles_world = []

        for pose in msg.poses:
            p_in = PointStamped()
            p_in.header = msg.header
            p_in.point.x = float(pose.position.x)
            p_in.point.y = float(pose.position.y)
            p_in.point.z = 0.0

            try:
                p_base = self.tf_buffer.transform(
                    p_in,
                    self.robot_base_frame,
                    timeout=Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not transform obstacle from {msg.header.frame_id} '
                    f'to {self.robot_base_frame}: {ex}'
                )
                return

            x_local = float(p_base.point.x)
            y_local = float(p_base.point.y)
            r = float(pose.position.z)  # 0.25 in testing

            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)

            x_world = self.px + cy * x_local - sy * y_local
            y_world = self.py + sy * x_local + cy * y_local

            obstacles_world.append((x_world, y_world, r))

        self.obstacles = obstacles_world
        self.last_obstacle_msg_time = self.get_clock().now()

    def control_loop(self) -> None:
        if not self.have_odom:
            if self.use_ground_truth_pose:
                self.get_logger().info(
                    f'Waiting for ground truth pose on {self.ground_truth_pose_topic}...'
                )
            else:
                self.get_logger().info(
                    f'Waiting for odometry on {self.odom_topic}...'
                )
            return

        if self.reached or self.collision:
            self.publish_stop()
            return

        if self.use_live_obstacles:
            if self.last_obstacle_msg_time is None:
                self.obstacles = []
            else:
                age = (self.get_clock().now() - self.last_obstacle_msg_time).nanoseconds * 1e-9
                if age > self.obstacle_timeout:
                    self.get_logger().warn('Obstacle data timed out; clearing obstacles.')
                    self.obstacles = []

        px, py = self.px, self.py
        vx, vy = self.vx_world, self.vy_world
        gx, gy = self.goal

        # Attractive term
        gradU_att_x = self.k1 * (px - gx)
        gradU_att_y = self.k1 * (py - gy)

        # Global isotropic damping
        F_damp_x = -self.k2 * vx
        F_damp_y = -self.k2 * vy

        # DAF obstacle-normal dissipation
        for ox, oy, r_obs in self.obstacles:
            dx = px - ox
            dy = py - oy
            dist = math.hypot(dx, dy)
            d = dist - (r_obs + self.r_robot)

            if dist > 1e-9:
                nx = dx / dist
                ny = dy / dist
            else:
                nx = 0.0
                ny = 0.0

            # Normal velocity relative to obstacle
            nv = nx * vx + ny * vy

            beta = self.beta_daf(d)

            # Only damp motion into the obstacle.
            # If nv > 0, robot is already moving away from obstacle.
            if nv < 0.0 and beta > 0.0:
                F_damp_x += -self.k_daf * beta * nx * nv
                F_damp_y += -self.k_daf * beta * ny * nv

            if dist < (r_obs + self.r_robot):
                self.collision = True
                self.get_logger().warn(
                    f'Obstacle collision at ({ox:.2f}, {oy:.2f}), dist={dist:.2f}'
                )

        # DAF acceleration: attraction + damping only
        ax = -gradU_att_x + F_damp_x
        ay = -gradU_att_y + F_damp_y

        # Integrate virtual desired velocity in world frame
        self.vx_cmd_world += ax * self.dt
        self.vy_cmd_world += ay * self.dt

        # Clamp virtual speed
        vmag = math.hypot(self.vx_cmd_world, self.vy_cmd_world)
        if vmag > self.max_virtual_speed and vmag > 1e-9:
            scale = self.max_virtual_speed / vmag
            self.vx_cmd_world *= scale
            self.vy_cmd_world *= scale

        self.get_logger().info(
            f'pos=({px:.2f},{py:.2f}) goal=({gx:.2f},{gy:.2f}) err=({gx-px:.2f},{gy-py:.2f})'
        )

        dist_to_goal = math.hypot(px - gx, py - gy)
        if dist_to_goal < self.reach_thresh_goal:
            self.reached = True
            self.publish_stop()

            msg = Bool()
            msg.data = True
            self.goal_pub.publish(msg)

            self.get_logger().info('Goal reached.')
            return

        # Convert desired world velocity to body-frame velocity for mecanum drive
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        vx_body = cy * self.vx_cmd_world + sy * self.vy_cmd_world
        vy_body = -sy * self.vx_cmd_world + cy * self.vy_cmd_world

        # Start with no heading control for paper-faithful DAF testing
        angular_speed = 0.0

        vx_body = clamp(vx_body, -self.max_linear_speed, self.max_linear_speed)
        vy_body = clamp(vy_body, -self.max_linear_speed, self.max_linear_speed)
        angular_speed = clamp(angular_speed, -self.max_angular_speed, self.max_angular_speed)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_base_frame
        cmd.twist.linear.x = vx_body
        cmd.twist.linear.y = vy_body
        cmd.twist.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

    def publish_stop(self) -> None:
        self.vx_cmd_world = 0.0
        self.vy_cmd_world = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_base_frame
        self.cmd_pub.publish(cmd)

    def ground_truth_pose_callback(self, msg: Pose) -> None:
        self.have_odom = True

        self.px = msg.position.x
        self.py = msg.position.y

        q = msg.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.get_logger().info(
            f'GT pose: x={self.px:.2f}, y={self.py:.2f}, yaw={self.yaw:.2f}'
        )

    @staticmethod
    def angle_wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DAFObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()