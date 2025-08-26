from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # load and START the controllers in launch file

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen')

    load_mycobot_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'mycobot_arm_controller'],
        output='screen')

    load_mycobot_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'gripper_controller'],
        output='screen')
    

    return LaunchDescription([load_joint_state_broadcaster, load_mycobot_arm_controller, load_mycobot_gripper_controller])
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot1,
                  on_exit=[
                    TimerAction(
                        period=100.0,
                        actions=[load_joint_state_broadcaster],
                    )
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_mycobot_arm_controller],
            )
        ),RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mycobot_arm_controller,
                on_exit=[load_mycobot_gripper_controller],
            )
        )
    ])
