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

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def get_value(node: yaml.Node, key: str):
    try:
        value = node[key]
        if value == "None":
            value = ""
        return value

    except KeyError:
        return ""


def get_launch_description(name: str, package: str, namespace: str, component: yaml.Node):
    device_namespace = get_value(component, "device_namespace")
    robot_namespace = namespace

    if "ur" not in name and "kinova" not in name and "robotiq" not in name:
        if len(robot_namespace) and robot_namespace[0] != "/":
            robot_namespace = "/" + robot_namespace
        if len(device_namespace) and device_namespace[0] != "/":
            device_namespace = "/" + device_namespace

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, "/launch/gz_", name, ".launch.py"]),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "device_namespace": device_namespace,
            "gz_bridge_name": component["device_namespace"] + "_gz_bridge",
        }.items(),
    )


def get_launch_descriptions_from_yaml_node(
    node: yaml.Node, package: os.PathLike, namespace: str
) -> IncludeLaunchDescription:
    actions = []

    components_types_with_names = {
        "LDR01": "slamtec_rplidar",
        "LDR06": "slamtec_rplidar",
        "LDR10": "ouster_os",
        "LDR11": "ouster_os",
        "LDR12": "ouster_os",
        "LDR13": "ouster_os",
        "LDR14": "ouster_os",
        "LDR15": "ouster_os",
        "LDR20": "velodyne",
        "CAM01": "orbbec_astra",
        "CAM03": "stereolabs_zed",
        "CAM04": "stereolabs_zed",
        "CAM06": "stereolabs_zed",
        "MAN01": "ur",
        "MAN02": "ur",
        # "MAN03": "kinova_lite"  sim_isaac error
        "MAN04": "kinova_6dof",
        "MAN05": "kinova_6dof",
        "MAN06": "kinova_7dof",
        "MAN07": "kinova_7dof",
        "GRP02": "robotiq",
        # "GRP03": "robotiq", Waiting for release
        # https://github.com/PickNikRobotics/ros2_robotiq_gripper/blob/main/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro
    }

    for component in node["components"]:
        component_type = component["type"]
        if component_type in components_types_with_names:
            launch_description = get_launch_description(
                components_types_with_names[component_type], package, namespace, component
            )
            actions.append(launch_description)

    return actions


def launch_setup(context, *args, **kwargs):
    ros_components_description = get_package_share_directory("ros_components_description")

    components_config_path = LaunchConfiguration("components_config_path").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)

    components_config = None
    if components_config_path == "None":
        return []

    with open(os.path.join(components_config_path), 'r') as file:
        components_config = yaml.safe_load(file)

    actions = []
    if components_config != None:
        actions += get_launch_descriptions_from_yaml_node(
            components_config, ros_components_description, namespace
        )

    return actions


def generate_launch_description():
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value="None",
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in Panther's urdf."
            "Panther options are described here "
            "https://husarion.com/manuals/panther/panther-options/"
        ),
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    actions = [
        declare_components_config_path_arg,
        declare_namespace_arg,
        OpaqueFunction(function=launch_setup),
    ]

    return LaunchDescription(actions)
