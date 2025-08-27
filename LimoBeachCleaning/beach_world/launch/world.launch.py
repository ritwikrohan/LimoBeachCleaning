# #!/usr/bin/env python3

# from launch import LaunchDescription
# from launch.actions import (
#     IncludeLaunchDescription,
#     DeclareLaunchArgument,
# )
# from launch.substitutions import (
#     PathJoinSubstitution,
#     LaunchConfiguration,
# )
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node, SetParameter

# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     map_package = get_package_share_directory("beach_world")
#     world_file = PathJoinSubstitution([map_package, "worlds", "beach_world_test.sdf"])
#     world_cfg = LaunchConfiguration("world")
#     declare_world_arg = DeclareLaunchArgument(
#         "world", default_value=["-r ", world_file], description="SDF world file"
#     )

#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution(
#                 [
#                     get_package_share_directory("ros_gz_sim"),
#                     "launch",
#                     "gz_sim.launch.py",
#                 ]
#             )
#         ),
#         launch_arguments={"gz_args": world_file}.items(),
#         #launch_arguments={"gz_args": ['-r -v 4 -s --headless-rendering ' + world_cfg]}.items(),
#         #launch_arguments={"gz_args": ['-r -v 4 -s --headless-rendering '] + world_cfg}.items(),
#     )

#     ign_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         name="ign_bridge",
#         arguments=[
#             "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
#             "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
#             "/velodyne_points/points"
#             + "@sensor_msgs/msg/PointCloud2"
#             + "[ignition.msgs.PointCloudPacked",
#             "/camera/color/camera_info"
#             + "@sensor_msgs/msg/CameraInfo"
#             + "[ignition.msgs.CameraInfo",
#             "/camera/color/image_raw"
#             + "@sensor_msgs/msg/Image"
#             + "[ignition.msgs.Image",
#             "/camera/camera_info"
#             + "@sensor_msgs/msg/CameraInfo"
#             + "[ignition.msgs.CameraInfo",
#             "/camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
#             "/camera/depth/points"
#             + "@sensor_msgs/msg/PointCloud2"
#             + "[ignition.msgs.PointCloudPacked",
#         ],
#         remappings=[
#             ("/velodyne_points/points", "/velodyne_points"),
#             ("/camera/camera_info", "/camera/depth/camera_info"),
#             ("/camera/depth", "/camera/depth/image_raw"),
#         ],
#         output="screen",
#     )

#     return LaunchDescription(
#         [
#             declare_world_arg,
#             # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
#             SetParameter(name="use_sim_time", value=True),
#             gz_sim,
#             ign_bridge,
#         ]
#     )


#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context):
    # gz_gui = LaunchConfiguration("gz_gui").perform(context)
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)
    gz_world = LaunchConfiguration("gz_world").perform(context)

    gz_args = f"-r -v {gz_log_level} {gz_world}"
    if eval(gz_headless_mode):
        gz_args = "--headless-rendering -s " + gz_args
    # if gz_gui:
    #     gz_args = f"--gui-config {gz_gui} " + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return [gz_sim]


def generate_launch_description():
    # declare_gz_gui = DeclareLaunchArgument(
    #     "gz_gui",
    #     default_value=PathJoinSubstitution(
    #         [FindPackageShare("husarion_gz_worlds"), "config", "teleop.config"]
    #     ),
    #     description="Run simulation with specific GUI layout.",
    # )

    declare_gz_headless_mode = DeclareLaunchArgument(
        "gz_headless_mode",
        default_value="False",
        description="Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the amount of calculations.",
        choices=["True", "False"],
    )

    declare_gz_log_level = DeclareLaunchArgument(
        "gz_log_level",
        default_value="4",
        description="Adjust the level of console output.",
        choices=["0", "1", "2", "3", "4"],
    )

    declare_gz_world_arg = DeclareLaunchArgument(
        "gz_world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("beach_world"), "worlds", "beach_world_cloud.sdf"]
        ),
        description="Absolute path to SDF world file.",
    )

    return LaunchDescription(
        [
            # declare_gz_gui,
            declare_gz_headless_mode,
            declare_gz_log_level,
            declare_gz_world_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
