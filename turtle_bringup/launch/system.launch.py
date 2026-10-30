"""Starts turtlesim and all our servers, with the speed parameters from a YAML file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('turtle_bringup'), 'config', 'turtle_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with the goto_pose_server parameters'),
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim',
             output='screen'),
        Node(package='turtle_control', executable='goto_pose_server', name='goto_pose_server',
             parameters=[params_file], output='screen'),
        Node(package='turtle_control', executable='reset_service', name='reset_service',
             output='screen'),
        Node(package='turtle_control', executable='square_server', name='square_server',
             output='screen'),
    ])
