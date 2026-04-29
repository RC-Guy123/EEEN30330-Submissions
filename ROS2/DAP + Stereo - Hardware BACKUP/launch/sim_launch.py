from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
# ----------------------------
# CONFIG
# ----------------------------
obstacles = [
    {"name": "obstacle_1", "x": 2.0, "y": -2.0, "radius": 0.8, "height": 1.0},
    {"name": "obstacle_2", "x": 5.0, "y": 2.0, "radius": 1.0, "height": 1.0},
]
goal_position = {"x": 7.0, "y": 5.0}
use_live_obstacles = True
def obstacles_to_string(obstacles):
    return ";".join(
        f'{obs["x"]},{obs["y"]},{obs["radius"]}'
        for obs in obstacles
    )
static_obstacles_param = obstacles_to_string(obstacles)
def make_cylinder_sdf(name: str, radius: float, height: float) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <pose>0 0 {height / 2.0} 0 0 0</pose>
    </link>
  </model>
</sdf>
"""
def make_obstacle_spawn_nodes():
    generated_dir = "/tmp/my_package_generated_obstacles"
    os.makedirs(generated_dir, exist_ok=True)
    nodes = []
    for obs in obstacles:
        sdf_text = make_cylinder_sdf(
            obs["name"],
            obs["radius"],
            obs.get("height", 1.0)
        )

        sdf_path = os.path.join(generated_dir, f'{obs["name"]}.sdf')
        with open(sdf_path, "w") as f:
            f.write(sdf_text)
        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "test_world",
                    "-file", sdf_path,
                    "-name", obs["name"],
                    "-x", str(obs["x"]),
                    "-y", str(obs["y"]),
                    "-z", "0.0",
                ],
            )
        )
    return nodes
def make_goal_spawn_node():
    pkg_share = FindPackageShare("my_package").find("my_package")
    goal_model = os.path.join(
        pkg_share,
        "models",
        "goal_marker",
        "model.sdf"
    )
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "test_world",
            "-file", goal_model,
            "-name", "goal_marker",
            "-x", str(goal_position["x"]),
            "-y", str(goal_position["y"]),
            "-z", "0.05",
        ],
    )
# ----------------------------
# ROBOT DESCRIPTION
# ----------------------------
def generate_launch_description():
    pkg_share = FindPackageShare("my_package")
    world_path = PathJoinSubstitution([pkg_share, "worlds", "test_world.sdf"])
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "mechbot.urdf"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "bridge.yaml"])
    set_libgl = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1"
    )
    robot_description = ParameterValue(
        Command(["cat ", urdf_path]),
        value_type=str
    )
    # ----------------------------
    # GAZEBO
    # ----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path]
        }.items(),
    )
    # ----------------------------
    # CORE ROS NODES
    # ----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": True},
        ],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "test_world",
            "-topic", "robot_description",
            "-name", "vehicle_blue",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.7",
        ],
    )
    # ----------------------------
    # CONTROLLERS (FIXED TIMING)
    # ----------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    stereo_obstacle_processor = Node(
        package="my_package",
        executable="stereo_obstacle_processor",
        name="stereo_obstacle_processor",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"obstacle_topic": "/detected_obstacles"},
            {"obstacle_frame": "stereo_link"},
        ],
    )
    # ----------------------------
    # STEREO PIPELINE
    # ----------------------------
    left_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        namespace="left",
        remappings=[
            ("image", "/left/image_raw"),
            ("camera_info", "/left/camera_info"),
            ("image_rect", "/left/image_rect"),
        ],
        output="screen",
    )
    right_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        namespace="right",
        remappings=[
            ("image", "/right/image_raw"),
            ("camera_info", "/right/camera_info"),
            ("image_rect", "/right/image_rect"),
        ],
        output="screen",
    )

    stereo_disparity = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        namespace="stereo",
        remappings=[
            ("left/image_rect", "/left/image_rect"),
            ("left/camera_info", "/left/camera_info"),
            ("right/image_rect", "/right/image_rect"),
            ("right/camera_info", "/right/camera_info"),
            ("disparity", "/disparity"),
        ],
        output="screen",
    )

    stereo_pointcloud = Node(
        package="stereo_image_proc",
        executable="point_cloud_node",
        namespace="stereo",
        remappings=[
            ("left/image_rect_color", "/left/image_rect"),
            ("left/camera_info", "/left/camera_info"),
            ("right/image_rect_color", "/right/image_rect"),
            ("right/camera_info", "/right/camera_info"),
            ("disparity", "/disparity"),
            ("points2", "/points2"),
        ],
        output="screen",
    )   
    
    daf_avoidance_node = Node(
        package="my_package",
        executable="daf_avoidance_node",
        name="daf_obstacle_avoidance",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"obstacle_topic": "/detected_obstacles"},
            {"use_live_obstacles": use_live_obstacles},
            {"obstacles": "" if use_live_obstacles else static_obstacles_param},
            {"goal": [goal_position["x"], goal_position["y"]]},
            {"obstacle_timeout": 0.5},
            {"robot_base_frame": "chassis"},
            {"use_ground_truth_pose": False},
            {"odom_topic": "/mecanum_drive_controller/odometry"},
        ],
    )
    path_logger = Node(
        package="my_package",
        executable="path_logger",
        name="path_logger",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    obstacle_spawn_nodes = make_obstacle_spawn_nodes()
    goal_spawn_node = make_goal_spawn_node()
    return LaunchDescription([
        path_logger,
        set_libgl,
        gazebo,
        TimerAction(period=2.0, actions=[robot_state_publisher]),
        TimerAction(period=3.0, actions=[spawn_robot]),

        TimerAction(period=4.0, actions=obstacle_spawn_nodes),
        TimerAction(period=4.0, actions=[goal_spawn_node]),

        TimerAction(period=4.0, actions=[ros_gz_bridge]),

        TimerAction(period=7.0, actions=[joint_state_broadcaster]),
        TimerAction(period=8.0, actions=[mecanum_controller]),

        TimerAction(period=5.0, actions=[left_rectify]),
        TimerAction(period=5.0, actions=[right_rectify]),
        TimerAction(period=5.0, actions=[stereo_disparity]),
        TimerAction(period=6.0, actions=[stereo_pointcloud]),

        TimerAction(period=6.0, actions=[stereo_obstacle_processor]),
        TimerAction(period=8.0, actions=[daf_avoidance_node]),
    ])