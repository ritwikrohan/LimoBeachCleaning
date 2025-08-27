#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# this is the function launch  system will look for


def generate_launch_description():

    package_description = "limo_description"

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        package_description), 'rviz', 'limo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [
            rviz_node
        ]
    )
