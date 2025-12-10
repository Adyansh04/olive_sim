#!/usr/bin/env python3

"""
ROS 2 Launch file for olive_bot simulation in Gazebo with sensor fusion testing
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    pkg_olive_sim = get_package_share_directory('olive_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths to files
    urdf_file = os.path.join(pkg_olive_sim, 'urdf', 'olive_bot.urdf.xacro')
    world_file = os.path.join(pkg_olive_sim, 'worlds', 'fusion_test_world.sdf')
    rviz_config_file = os.path.join(pkg_olive_sim, 'rviz', 'olive_bot.rviz')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

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

    # Process the URDF file
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

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

    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'olive_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
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

    # Declare launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add nodes to launch description
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz_node)

    return ld
