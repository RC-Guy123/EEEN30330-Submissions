from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # === 1. Stereo Camera Driver ===
        Node(
            package='my_package',
            executable='rpicam_stereo_node',
            name='rpicam_stereo',
            output='screen',
            parameters=[{'width': 320, 'height': 240, 'fps': 20}],
        ),
        # === 2. Rectification (kept as-is) ===
        Node(
            package='image_proc',
            executable='rectify_node',
            name='left_rectify',
            output='screen',
            remappings=[('image', '/left/image_raw'), 
                       ('camera_info', '/left/camera_info'),
                       ('image_rect', '/left/image_rect')],
            parameters=[{'camera_info_url': 'file:///home/austin/ros2_ws/src/my_package/config/left.yaml'}]
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='right_rectify',
            output='screen',
            remappings=[('image', '/right/image_raw'), 
                       ('camera_info', '/right/camera_info'), 
                       ('image_rect', '/right/image_rect')],
            parameters=[{'camera_info_url': 'file:///home/austin/ros2_ws/src/my_package/config/right.yaml'}]
        ),
        # === NEW: Calibration Loader (scales your high-res calibration to 320x240) ===
        Node(
            package='my_package',                    # <-- Change if you put the script in another package
            executable='load_stereo_calibration',    # must match your Python file name without .py
            name='calibration_loader',
            output='screen',
        ),
        # === 3. Disparity Node - using fixed calibration ===
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',
            remappings=[
                ('left/image_rect',   '/left/image_rect'),
                ('right/image_rect',  '/right/image_rect'),
                ('left/camera_info',  '/left/camera_info_fixed'),   #  Important
                ('right/camera_info', '/right/camera_info_fixed'),  #  Important
            ],
            parameters=[{
                'approximate_sync': True,
                'queue_size': 15,
                'slop': 0.25,
                'max_disparity': 48.0,
                'min_disparity': 0,
                'uniqueness_ratio': 8.0,
                'texture_threshold': 10,
                'speckle_size': 150,
                'speckle_range': 2,
                'sgbm_mode': 2,
                'prefilter_size': 5,
                'prefilter_cap': 31,
                'correlation_window_size': 11,   # Odd number, 5-21
            }]
        ),
        # === 4. Point Cloud Node - using fixed calibration ===
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='point_cloud_node',
            output='screen',
            remappings=[
                ('left/image_rect_color', '/left/image_rect'),
                ('disparity',             '/disparity'),
                ('left/camera_info',      '/left/camera_info_fixed'),   # ← Important
                ('right/camera_info',     '/right/camera_info_fixed'),  # ← Important
            ],
            parameters=[{
                'approximate_sync': True,
                'queue_size': 10,
                'slop': 0.5,
            }]
        ),
        # === 5. Static TF ===
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['0.0', '0.0', '0.20', '1.57', '3.14', '1.57', 'base_link', 'left_camera'],
            output='screen'
        ),
        # === 6. Obstacle Processor & Filter ===
        Node(
            package='my_package',
            executable='stereo_obstacle_processor',
            name='stereo_obstacle_processor',
            output='screen',
        ),
        Node(
            package='my_package',
            executable='pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
        ),
        # === Obstacle Processor ===
        Node(
            package='my_package',
            executable='stereo_obstacle_processor',
            name='stereo_obstacle_processor',
            output='screen',
            parameters=[{
                'max_range': 3.0,
                'min_range': 0.25,
                'cluster_distance': 0.25,
                'min_cluster_size': 80,
            }]
        ),
    ])