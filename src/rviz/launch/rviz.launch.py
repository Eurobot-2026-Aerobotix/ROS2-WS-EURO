#!/usr/bin/env python3
"""
Launch file for RViz visualization with map server and lifecycle manager
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(os.environ['HOME'], 'hybrid/src/rviz/map/map.yaml'),
        description='Full path to the map YAML file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('rviz'),
            'config',
            'default.rviz'
        ]),
        description='Full path to the RViz config file'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rviz'),
            'robot',
            'TORBAG.urdf'
        ]),
        description='Full path to the robot URDF file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    # Map server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )
    
    # Get the URDF file path
    urdf_file_path = os.path.join(
        get_package_share_directory('rviz'),
        'robot',
        'TORBAG.urdf'
    )
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Robot Pose Publisher (subscribes to /robot_pose_raw)
    robot_pose_publisher = Node(
        package='rviz',
        executable='robot_pose_publisher.py',
        name='robot_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz2 node (starts first)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Delay map server to start after RViz is ready
    delayed_map_server = TimerAction(
        period=2.0,  # Wait 2 seconds for RViz to be ready
        actions=[map_server]
    )
    
    # Delay lifecycle manager to start after map server
    delayed_lifecycle_manager = TimerAction(
        period=3.0,  # Wait 3 seconds total
        actions=[lifecycle_manager]
    )
    
    return LaunchDescription([
        map_file_arg,
        rviz_config_arg,
        urdf_file_arg,
        use_sim_time_arg,
        robot_state_publisher,
        robot_pose_publisher,
        rviz_node,
        delayed_map_server,
        delayed_lifecycle_manager
    ])
