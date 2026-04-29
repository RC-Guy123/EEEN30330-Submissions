import rclpy
from rclpy.node import Node
from gpiozero import PWMLED
from signal import pause


class PWMHWNode(Node):
    def __init__(self):
        super().__init__('pwm_hw_node')

        self.declare_parameter('freq', 1000)
        self.declare_parameter('duty', 40)

        freq = self.get_parameter('freq').value
        duty = self.get_parameter('duty').value

        # gpiozero PWM outputs on BCM pins
        self.pwm_left = PWMLED(18, frequency=freq)
        self.pwm_right = PWMLED(19, frequency=freq)

        pwm_value = max(0.0, min(1.0, duty / 100.0))

        self.pwm_left.value = pwm_value
        self.pwm_right.value = pwm_value

        self.get_logger().info(
            f'GPIOZero PWM running: GPIO18/GPIO19 at {freq} Hz, duty {duty}%'
        )

    def destroy_node(self):
        self.pwm_left.off()
        self.pwm_right.off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PWMHWNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()