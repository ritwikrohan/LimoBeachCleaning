from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import SetParameter, Node
from launch import LaunchDescription

def generate_launch_description():
    pkg_share_dir = FindPackageShare(package='husarion_office_gz').find('husarion_office_gz') + '/'
              
    cloud = LaunchConfiguration('cloud')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        join(pkg_share_dir,'worlds', 'meshes'))

    gazebo_launch_node_cloud_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 -s --headless-rendering ' + pkg_share_dir + 'worlds/husarion_world.sdf']}.items(),
                condition=IfCondition(cloud)
            )
    gazebo_launch_node_local_gz = IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments = {'gz_args': ['-r -v 4 ' + pkg_share_dir + 'worlds/husarion_world.sdf']}.items(),
                condition=UnlessCondition(cloud)
            )
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/velodyne_points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/depth@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/velodyne_points/points", "/velodyne_points"),
            ("/camera/camera_info", "/camera/depth/camera_info"),
            ("/camera/depth", "/camera/depth/image_raw"),
        ],
        output="screen",
        )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(name='cloud', default_value='False', description='Sets the rendering based on whether the simulation runs locally or on the cloud'),
        set_env_vars_resources,
        gazebo_launch_node_cloud_gz,
        gazebo_launch_node_local_gz,
        ign_bridge
    ])
