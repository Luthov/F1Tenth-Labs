#!/usr/bin/env python3
import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
                get_package_share_directory('lab1_pkg'),
                'config',
                'lab1_params.yaml'
            )

    talker = Node(
                package='lab1_pkg',
                executable='talker.py',
                name='talker',
                parameters=[config]
            )
    relay = Node(
                package='lab1_pkg',
                executable='relay.py',
                name='relay'
            )
    ld.add_action(talker)
    ld.add_action(relay)

    return ld
