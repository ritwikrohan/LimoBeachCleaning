from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'approx_sync':True,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'queue_size': 100,
          'topic_queue_size': 100,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
        #   'subscribe_scan':True,
        #   '"RGBD/NeighborLinkRefining':'true',
        #   'RGBD/ProximityBySpace':'true',
        #   'RGBD/ProximityByTime':'false',
        #   'RGBD/ProximityPathMaxNeighbors': '10',
        #   'Reg/Strategy': '1',
        #   'Vis/MinInliers': '12',
        #   'RGBD/OptimizeFromGraphEnd':'false',
        #   'RGBD/OptimizeMaxError': '3',
        #   'Grid/FromDepth' :'false',
        #   'Mem/STMSize': '30',
        #   'RGBD/LocalRadius':'5',
        #   'Icp/CorrespondenceRatio': '0.4',
        #   'Rtabmap/MemoryThr':'0',
        #   'Rtabmap/TimeThr':'0',
        }

    remappings=[
        #   ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        # Nodes to launch
        
        # SLAM mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])
