from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # High resolution + lower FPS for better calibration
        Node(
            package='my_package',
            executable='rpicam_stereo_node',
            name='rpicam_stereo',
            output='screen',
            parameters=[
                {'width': 1640},
                {'height': 1232},
                {'fps': 8},           # Lower FPS = more stable for calibrator
            ],
        ),
    ])