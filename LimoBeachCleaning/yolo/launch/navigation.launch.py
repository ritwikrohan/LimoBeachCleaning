import os
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_name = 'yolo'
    rviz_path = 'rviz/nav2_config.rviz'

    nav2_pkg = 'nav2_bringup'
    nav_param_file = 'params/nav2_params.yaml'

    slam_pkg = 'slam_toolbox'
    slam_param_file = 'params/mapper_params_online_async.yaml'


    
    map_path = '/home/ritwik/asimovo_ws/src/yolo/params/final.yaml'
    nav_path = '/home/ritwik/asimovo_ws/src/yolo/params/nav2_params.yaml'
    slam_param_path = '/home/ritwik/asimovo_ws/src/yolo/params/mapper_params_online_async.yaml'


    rviz_file = os.path.join(get_package_share_directory(pkg_name),rviz_path)


    

    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(slam_pkg), 'launch'), '/online_async_launch.py']),
                launch_arguments={
                                  "use_sim_time": "True",
                                  "slam_params_file": slam_param_path}.items()
            )

    nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(nav2_pkg), 'launch'), '/bringup_launch.py']),
                launch_arguments={"map": map_path,
                                  "use_sim_time": "True",
                                  "params_file": nav_path}.items()
            )
    

    
    yolo_node = Node(
        package='yolo',
        executable='detection',
        name='yolo',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        output='screen'
    )

    


    return LaunchDescription(
        [
            #slam_launch,
            nav_launch,
            #yolo_node,
            rviz_node,
        ]
    )
