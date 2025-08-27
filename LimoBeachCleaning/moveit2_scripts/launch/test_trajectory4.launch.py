import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="limo_moveit_config").to_moveit_configs()

    moveit_cpp_node = Node(
    name="test_trajectory4",
    package="moveit2_scripts",
    executable="test_trajectory4",
    output="screen",
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {'use_sim_time': True},
    ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )
