#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo

from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import time
import signal
import threading
from queue import Queue, Empty


class RPICamStereo(Node):
    def __init__(self):
        super().__init__('rpicam_stereo')

        self.bridge = CvBridge()

        # Publishers
        self.left_pub = self.create_publisher(Image, '/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)

        # Calibration services
        self.create_service(SetCameraInfo, '/left/set_camera_info', self.left_set_camera_info_callback)
        self.create_service(SetCameraInfo, '/right/set_camera_info', self.right_set_camera_info_callback)

        self.left_calib = None
        self.right_calib = None

        # === Configurable resolution for calibration vs normal use ===
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # MJPEG commands (much more stable than before)
        self.left_cmd = [
            "rpicam-vid", "-n", "--camera", "0",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(self.fps),
            "--codec", "mjpeg",
            "--nopreview",
            "--timeout", "0",
            "--output", "-"
        ]

        self.right_cmd = [
            "rpicam-vid", "-n", "--camera", "1",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(self.fps),
            "--codec", "mjpeg",
            "--nopreview",
            "--timeout", "0",
            "--output", "-"
        ]

        # Start processes
        self.left_proc = None
        self.right_proc = None
        self.left_queue = Queue(maxsize=5)
        self.right_queue = Queue(maxsize=5)

        self.start_cameras()

        # Timer for publishing (slower than capture threads)
        self.timer = self.create_timer(0.05, self.publish_frames)  # ~20 Hz check

        self.get_logger().info(f"RPICam Stereo node started: {self.width}x{self.height} @ {self.fps} fps")

    def start_cameras(self):
        try:
            self.left_proc = subprocess.Popen(
                self.left_cmd, stdout=subprocess.PIPE, bufsize=10**7, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            self.right_proc = subprocess.Popen(
                self.right_cmd, stdout=subprocess.PIPE, bufsize=10**7, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )

            # Start reader threads
            threading.Thread(target=self._frame_reader, args=(self.left_proc, self.left_queue, "left"), daemon=True).start()
            threading.Thread(target=self._frame_reader, args=(self.right_proc, self.right_queue, "right"), daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"Failed to start camera processes: {e}")

    def _frame_reader(self, proc, queue, side):
        """Non-blocking MJPEG frame reader thread"""
        buffer = b''
        while rclpy.ok() and proc.poll() is None:
            try:
                chunk = proc.stdout.read1(4096)
                if not chunk:
                    time.sleep(0.01)
                    continue

                buffer += chunk

                while True:
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9', start + 2)

                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end + 2]
                        buffer = buffer[end + 2:]

                        np_arr = np.frombuffer(jpg, np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            try:
                                queue.put_nowait(frame)
                            except:
                                pass  # queue full, drop frame
                        break
                    else:
                        break  # wait for more data
            except Exception as e:
                self.get_logger().warn(f"Error in {side} reader: {e}")
                time.sleep(0.1)

    def publish_frames(self):
        try:
            left_frame = self.left_queue.get_nowait()
            right_frame = self.right_queue.get_nowait()
        except Empty:
            return  # no new frames ready

        stamp = self.get_clock().now().to_msg()

        # Convert to ROS messages
        left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")

        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = "left_camera"
        right_msg.header.frame_id = "right_camera"

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Publish CameraInfo
        if self.left_calib is not None:
            info = self.left_calib
            info.header.stamp = stamp
            info.header.frame_id = "left_camera"
            self.left_info_pub.publish(info)
        else:
            self.left_info_pub.publish(self.make_dummy_info(stamp, "left_camera"))

        if self.right_calib is not None:
            info = self.right_calib
            info.header.stamp = stamp
            info.header.frame_id = "right_camera"
            self.right_info_pub.publish(info)
        else:
            self.right_info_pub.publish(self.make_dummy_info(stamp, "right_camera"))

    def make_dummy_info(self, stamp, frame_id):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"

        fx = self.width * 0.6   # rough focal length estimate
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        return info

    def left_set_camera_info_callback(self, request, response):
        self.left_calib = request.camera_info
        self.get_logger().info("Left camera calibration stored")
        response.success = True
        response.status_message = "Left calibration accepted"
        return response

    def right_set_camera_info_callback(self, request, response):
        self.right_calib = request.camera_info
        self.get_logger().info("Right camera calibration stored")
        response.success = True
        response.status_message = "Right calibration accepted"
        return response

    def destroy_node(self):
        if self.left_proc:
            self.left_proc.terminate()
        if self.right_proc:
            self.right_proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = RPICamStereo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()