#!/usr/bin/env python3

"""
ROS 2 Launch file for olive_bot simulation in Gazebo Fortress (Ignition Gazebo)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    pkg_olive_sim = get_package_share_directory('olive_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set Gazebo resource path to include our models
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_olive_sim, 'models'),
            ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )

    # Paths to files
    urdf_file = os.path.join(pkg_olive_sim, 'urdf', 'olive_bot_fortress.urdf.xacro')
    world_file = os.path.join(pkg_olive_sim, 'worlds', 'fusion_test_world_fortress.sdf')
    rviz_config_file = os.path.join(pkg_olive_sim, 'rviz', 'olive_bot.rviz')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot spawn point'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot spawn point'
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of the robot spawn point'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )

    # Process the URDF file
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo Fortress (Ignition Gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world, ' -r'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn the robot in Gazebo Fortress
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'olive_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # Bridge for camera images
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Bridge for LiDAR point cloud
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/lidar/points/points', '/lidar/points')
        ],
        output='screen'
    )

    # Bridge for IMU
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    # Bridge for odometry and TF
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gz_resource_path)

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_world_cmd)

    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(bridge_camera)
    ld.add_action(bridge_lidar)
    ld.add_action(bridge_imu)
    ld.add_action(bridge_odom)
    ld.add_action(rviz_node)

    return ld
