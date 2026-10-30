"""Starts the whole system, then triggers the square automatically. Optionally records a bag file.

  ros2 launch turtle_bringup square_demo.launch.py
  ros2 launch turtle_bringup square_demo.launch.py side_length:=4.0 record_bag:=true
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BAG_TOPICS = ['/turtle1/cmd_vel', '/turtle1/pose']


def _bag_recorder(context):
    if LaunchConfiguration('record_bag').perform(context).lower() not in ('true', '1', 'yes'):
        return []
    bag_name = 'square_bag_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    return [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_name, *BAG_TOPICS],
                           output='screen')]


def generate_launch_description():
    system = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('turtle_bringup'), 'launch', 'system.launch.py')))

    send_square = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '--feedback', '/draw_square',
             'turtle_interfaces/action/DrawSquare',
             ['{side_length: ', LaunchConfiguration('side_length'), '}']],
        output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='3.0',
                              description='Side of the square (0 = default 3.0)'),
        DeclareLaunchArgument('record_bag', default_value='false',
                              description='true = record /turtle1/cmd_vel and /turtle1/pose'),
        system,
        OpaqueFunction(function=_bag_recorder),
        send_square,
    ])
