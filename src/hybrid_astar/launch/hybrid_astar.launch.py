#!/usr/bin/env python3
"""
Launch file for Hybrid A* Path Planner Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('hybrid_astar'),
            'config',
            'hybrid_astar_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file'
    )
    
    # Hybrid A* Node
    hybrid_astar_node = Node(
        package='hybrid_astar',
        executable='hybrid_astar_node.py',
        name='hybrid_astar_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )
    
    # Web Handler Node (bridge between web dashboard and ROS2)
    web_handler_node = Node(
        package='hybrid_astar',
        executable='web_handler.py',
        output='screen',
        emulate_tty=True,
    )
    
    # Start web_handler after a short delay to ensure services from hybrid_astar_node are available
    delayed_web_handler = TimerAction(
        period=1.0,
        actions=[web_handler_node]
    )

    return LaunchDescription([
        params_file_arg,
        hybrid_astar_node,
        delayed_web_handler
    ])
