import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True", description="Choose according to the use(True for simulation and False for Robot)")
    use_sim_time_arg_f = LaunchConfiguration('use_sim_time')

    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'config.rviz')

    # catrographer_node = Node(
    #         package='cartographer_ros', 
    #         executable='cartographer_node', 
    #         name='cartographer_node',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time_arg_f}],
    #         arguments=['-configuration_directory', cartographer_config_dir,
    #                    '-configuration_basename', configuration_basename])

    cartographer_node = Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_arg_f}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', configuration_basename],
        remappings=[('/odom', '/odometry/filtered')]  # Remap /odom to /odometry/filtered
        )

    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time_arg_f}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        )

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_arg_f}]) 

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
        # rviz2_node
    ]) 