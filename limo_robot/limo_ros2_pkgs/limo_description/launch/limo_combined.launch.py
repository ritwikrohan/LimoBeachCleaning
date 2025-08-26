import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  # Correct import

def generate_launch_description():
    # Find the package shares for each of the launch files
    limo_moveit_config_share = get_package_share_directory('limo_moveit_config')
    moveit2_scripts_share = get_package_share_directory('moveit2_scripts')
    rtabmap_share = get_package_share_directory('limo_rtabmap')  # Adjust if your package name differs
    path_planner_share = get_package_share_directory('path_planner_server')  # Adjust the package name if necessary
    
    return LaunchDescription([
        # Optional: Set a namespace for all nodes 
        # PushRosNamespace('/limo_namespace'),  # Adjust for your use case if necessary

        # Include the move_group.launch.py from limo_moveit_config
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(limo_moveit_config_share, 'launch', 'move_group.launch.py')
            )
        ),

        # Include the moveit_rviz.launch.py from limo_moveit_config
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(limo_moveit_config_share, 'launch', 'moveit_rviz.launch.py')
            )
        ),
        
        # Run the basic_grasping_perception_node executable
        Node(
            package='simple_grasping',  # Replace with your actual package name
            executable='basic_grasping_perception_node',  # Name of your executable
            output='screen',
            parameters=[  # Pass parameters here if needed
                {'point_cloud_topic': '/camera/depth/image_rect_raw/points'},
                {'debug_topics': True},
                {'frame_id': 'base_footprint'}
            ]
        ),
        
        # Run the YOLO object detection executable
        Node(
            package='advanced_perception',  # Package name where YOLO is located
            executable='yolo_object_detection',  # Name of the YOLO executable
            output='screen',
        ),
        
        # Include the pick_server.launch.py from moveit2_scripts
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit2_scripts_share, 'launch', 'pick_server.launch.py')
            )
        ),
        
        # Include the pick_server.launch.py from moveit2_scripts
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit2_scripts_share, 'launch', 'pick_server2.launch.py')
            )
        ),

        # Include the start_rtab_map_loc.launch.py from limo_rtabmap
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_share, 'launch', 'start_rtab_map_loc.launch.py')
            )
        ),

        # Include the rtab_navigation.launch.py from path_planner_server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(path_planner_share, 'launch', 'rtab_navigation.launch.py')
            )
        ),
    ])

