import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class StereoCapture(Node):
    def __init__(self):
        super().__init__('stereo_capture')

        self.bridge = CvBridge()

        self.left_sub = self.create_subscription(
            Image, '/left/image_raw', self.left_cb, 10)

        self.right_sub = self.create_subscription(
            Image, '/right/image_raw', self.right_cb, 10)

        self.left_img = None
        self.right_img = None

        self.save_dir = os.path.expanduser('~/stereo_calib/images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.count = 0

        self.timer = self.create_timer(0.5, self.try_save)

    def left_cb(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def right_cb(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def try_save(self):
        if self.left_img is None or self.right_img is None:
            return

        cv2.imshow("left", self.left_img)
        cv2.imshow("right", self.right_img)

        key = cv2.waitKey(1)

        # press SPACE to capture pair
        if key == 32:
            cv2.imwrite(f"{self.save_dir}/left_{self.count:04d}.png", self.left_img)
            cv2.imwrite(f"{self.save_dir}/right_{self.count:04d}.png", self.right_img)
            self.get_logger().info(f"Saved pair {self.count}")
            self.count += 1

        # press q to quit
        if key == ord('q'):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = StereoCapture()
    rclpy.spin(node)

if __name__ == '__main__':
    main()