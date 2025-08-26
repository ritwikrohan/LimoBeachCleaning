import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from ament_index_python.packages import get_package_prefix


# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'limo_description'
    robot_name_in_model = 'limo_description'
    rviz_config_file_path = 'rviz/limo.rviz'
    urdf_file_path = 'urdf/limo_four_diff_arm.xacro'
    depth_camera_child_tf = ("limo_robot/base_footprint/sensor_depth")
    depth_camera_parent_tf = "camera_depth_optical_frame"
    pointcloud_rpy = ["1.57", "-1.57", "0",]
    
    # world_file = 'simple.world'
    world_file = 'turtlebot3_tc_office_grasp.world'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.1'
    spawn_yaw_val = '0.00'

    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  
    # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(pkg_share, 'worlds', world_file)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gazebo_models_path

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    

    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')

    declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

    declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', urdf_model]),'use_sim_time': use_sim_time}]
    )
    joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen'
    )

    # # Get Package Description and Directory #
    # package_description = "robot_description"
    # package_directory = get_package_share_directory(package_description)

    # # Load URDF File #
    # urdf_file = 'robot.xacro'
    # robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    # print("URDF Loaded !")

    # # Robot State Publisher (RSP) #
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher_node',
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{'use_sim_time': True, 
    #                  'robot_description': Command(['xacro ', robot_desc_path])}]
    # )

    # Spawn the Robot #
    declare_spawn_model_name = DeclareLaunchArgument("model_name", default_value="limo_robot",
                                                     description="Model Spawn Name")
    declare_spawn_x = DeclareLaunchArgument("x", default_value="-1.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="-4.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.2",
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="limo_spawn",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    
    pointcloud_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="point_cloud_tf",
        output="log",
        arguments=[
            "0",
            "0",
            "0",
        ]
        + pointcloud_rpy
        + [
            depth_camera_parent_tf,
            depth_camera_child_tf,
        ],
    )

    # ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            "/camera/depth/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
            "/camera/color/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
            "/camera/depth/image_rect_raw" + "@sensor_msgs/msg/Image" + "@ignition.msgs.Image",
            "/camera/color/image_raw" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
            "/camera/depth/image_rect_raw/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
            "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
        ],
        remappings=[
            # there are no remappings for this robot description
        ],
        output="screen",
    )

    # start_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['/camera/image_raw'],
    #     output='screen',
    # )

    controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("mycobot_gazebo"),
                        "launch",
                        "start_gripper_control.launch.py",
                    ]
                )
            ),
        )

    description_package_name = "limo_description"
    install_dir = get_package_prefix(description_package_name)

    mycobot_description_package_name = "mycobot_description"
    mycobot_description_install_dir = get_package_prefix(mycobot_description_package_name)

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir + \
            '/share' +':' + mycobot_description_install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = install_dir + \
            "/share" +':' + mycobot_description_install_dir + '/share' + ':' + gazebo_models_path

    print("IGN GAZEBO MODELS PATH=="+str(os.environ["IGN_GAZEBO_RESOURCE_PATH"]))


    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            declare_spawn_model_name,
            declare_use_sim_time_cmd,
            declare_use_joint_state_publisher_cmd,
            declare_namespace_cmd,
            declare_use_namespace_cmd,
            declare_rviz_config_file_cmd,
            declare_simulator_cmd,
            declare_urdf_model_path_cmd,
            declare_use_robot_state_pub_cmd,
            declare_use_rviz_cmd,
            declare_use_simulator_cmd,
            declare_world_cmd,
            start_robot_state_publisher_cmd,
            #joint_state_publisher_cmd,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
            ign_bridge,
            pointcloud_tf,
            #controller_launch,
        ]
    )
