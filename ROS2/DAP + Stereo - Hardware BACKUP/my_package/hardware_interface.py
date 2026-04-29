import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from gpiozero import PWMOutputDevice


# ============================================
# BIDIRECTIONAL H-BRIDGE MOTOR USING GPIOZERO
# ============================================
class Motor:
    def __init__(self, pin_a, pin_b, frequency=1000):
        self.a = PWMOutputDevice(pin_a, frequency=frequency)
        self.b = PWMOutputDevice(pin_b, frequency=frequency)

    def set_speed(self, value):
        value = max(min(value, 1.0), -1.0)

        if abs(value) < 0.01:
            self.a.value = 0.0
            self.b.value = 0.0
            return

        # startup deadband compensation
        sign = 1 if value > 0 else -1
        value = sign * (0.18 + 0.82 * abs(value))

        if value > 0:
            self.a.value = abs(value)
            self.b.value = 0.0
        else:
            self.a.value = 0.0
            self.b.value = abs(value)


# ============================================
# MAIN HARDWARE NODE
# ============================================
class HardwareInterface(Node):

    def __init__(self):
        super().__init__('hardware_interface')

        # -------------------------------
        # FOUR MOTOR OUTPUTS
        # -------------------------------
        self.motors = [
            Motor(18, 19),  # front left
            Motor(12, 13),  # front right
            Motor(20, 21),  # rear left
            Motor(22, 23),  # rear right
        ]

        # -------------------------------
        # ROBOT GEOMETRY
        # -------------------------------
        self.r = 0.05
        self.L = 0.149
        self.W = 0.160
        self.k = self.L + self.W

        self.max_wheel_rad_s = 20.0

        # -------------------------------
        # ODOM / JOINT STATE PLACEHOLDERS
        # -------------------------------
        self.positions = [0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0]

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_wheels = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = time.time()

        # -------------------------------
        # ROS INTERFACES
        # -------------------------------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Full hardware_interface online.")

    # ---------------------------------
    # BODY CMD TO WHEEL RAD/S
    # ---------------------------------
    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.target_wheels[0] = (vx - vy - self.k * wz) / self.r
        self.target_wheels[1] = (vx + vy + self.k * wz) / self.r
        self.target_wheels[2] = (vx + vy - self.k * wz) / self.r
        self.target_wheels[3] = (vx - vy + self.k * wz) / self.r

        self.last_cmd_time = time.time()

    # ---------------------------------
    # MAIN LOOP
    # ---------------------------------
    def control_loop(self):
        now = time.time()
        dt = 0.02

        # safety timeout
        if now - self.last_cmd_time > 0.5:
            self.target_wheels = [0.0, 0.0, 0.0, 0.0]

        # send wheel commands to motors
        for i in range(4):
            norm = self.target_wheels[i] / self.max_wheel_rad_s
            self.motors[i].set_speed(norm)

            # placeholder virtual feedback until encoders wired
            self.velocities[i] = self.target_wheels[i]
            self.positions[i] += self.velocities[i] * dt

        self.publish_joint_states()
        self.update_odometry(dt)

    # ---------------------------------
    # JOINT STATES
    # ---------------------------------
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        js.position = self.positions
        js.velocity = self.velocities
        self.joint_pub.publish(js)

    # ---------------------------------
    # SIMPLE ODOM
    # ---------------------------------
    def update_odometry(self, dt):
        w1, w2, w3, w4 = self.velocities

        vx = self.r * (w1 + w2 + w3 + w4) / 4.0
        vy = self.r * (-w1 + w2 + w3 - w4) / 4.0
        wz = self.r * (-w1 + w2 - w3 + w4) / (4.0 * self.k)

        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += wz * dt

        self.publish_odom(vx, vy, wz)

    # ---------------------------------
    # ODOM + TF
    # ---------------------------------
    def publish_odom(self, vx, vy, wz):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()