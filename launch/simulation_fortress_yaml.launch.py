#!/usr/bin/env python3

"""
ROS 2 Launch file for olive_bot simulation in Gazebo Fortress (Ignition Gazebo)
Uses YAML configuration for world selection and spawn locations
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_world_config(context, *args, **kwargs):
    """Load world configuration from YAML and set up launch"""
    
    # Get package directories
    pkg_olive_sim = get_package_share_directory('olive_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Load YAML configuration
    config_file = os.path.join(pkg_olive_sim, 'config', 'worlds_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Get launch configurations
    world_name = LaunchConfiguration('world_name').perform(context)
    use_config_spawn = LaunchConfiguration('use_config_spawn').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Determine which world to use
    if world_name == 'default':
        world_name = config['default_world']
    
    # Get world configuration
    if world_name not in config['worlds']:
        print(f"Warning: World '{world_name}' not found in config. Using default.")
        world_name = config['default_world']
    
    world_config = config['worlds'][world_name]
    
    # Set world file path
    world_file = os.path.join(pkg_olive_sim, 'worlds', world_config['world_file'])
    
    # Determine spawn location
    if use_config_spawn == 'true':
        spawn_loc = world_config['spawn_location']
        x_pose = str(spawn_loc['x'])
        y_pose = str(spawn_loc['y'])
        z_pose = str(spawn_loc['z'])
        roll_pose = str(spawn_loc['roll'])
        pitch_pose = str(spawn_loc['pitch'])
        yaw_pose = str(spawn_loc['yaw'])
    else:
        # Use command line arguments
        x_pose = LaunchConfiguration('x_pose').perform(context)
        y_pose = LaunchConfiguration('y_pose').perform(context)
        z_pose = LaunchConfiguration('z_pose').perform(context)
        roll_pose = LaunchConfiguration('roll').perform(context)
        pitch_pose = LaunchConfiguration('pitch').perform(context)
        yaw_pose = LaunchConfiguration('yaw').perform(context)
    
    # Set Gazebo resource path
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
    rviz_config_file = os.path.join(pkg_olive_sim, 'rviz', 'olive_bot.rviz')
    
    # Process the URDF file
    robot_description_content = Command(['xacro ', urdf_file])
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
            'gz_args': [world_file, ' -r'],
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
            '-R', roll_pose,
            '-P', pitch_pose,
            '-Y', yaw_pose,
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
    
    # Print world information
    print(f"\n{'='*60}")
    print(f"Launching Olive Bot Simulation")
    print(f"{'='*60}")
    print(f"World: {world_config['name']}")
    print(f"Description: {world_config['description']}")
    print(f"Size: {world_config['size']}")
    print(f"Difficulty: {world_config['difficulty']}")
    print(f"Spawn Location: ({x_pose}, {y_pose}, {z_pose})")
    print(f"Spawn Orientation: roll={roll_pose}, pitch={pitch_pose}, yaw={yaw_pose}")
    print(f"{'='*60}\n")
    
    return [
        gz_resource_path,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge_camera,
        bridge_lidar,
        bridge_imu,
        bridge_odom,
        rviz_node
    ]


def generate_launch_description():
    """Generate launch description with world configuration"""
    
    # Declare launch arguments
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='default',
        description='World name from config file (fusion_test, warehouse, maze, office, industrial) or "default"'
    )
    
    declare_use_config_spawn_cmd = DeclareLaunchArgument(
        'use_config_spawn',
        default_value='true',
        description='Use spawn location from config file (true) or command line arguments (false)'
    )
    
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
    
    # Manual spawn position arguments (used when use_config_spawn=false)
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of robot spawn point (used when use_config_spawn=false)'
    )
    
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of robot spawn point (used when use_config_spawn=false)'
    )
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of robot spawn point (used when use_config_spawn=false)'
    )
    
    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll orientation of robot (used when use_config_spawn=false)'
    )
    
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch orientation of robot (used when use_config_spawn=false)'
    )
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw orientation of robot (used when use_config_spawn=false)'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add argument declarations
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_use_config_spawn_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    # Add opaque function to load config and create nodes
    ld.add_action(OpaqueFunction(function=load_world_config))
    
    return ld
