#! /usr/bin/env bash

# We remove a folder that otherwise gives issues in ROS2 launches
sudo rm -r /home/user/.ros

# Check if the first argument is 'debug'
if [ "$1" = "debug" ]; then
    export ROS2_WS_PATH=/home/user/ros2_ws
else
    export ROS2_WS_PATH=/home/simulations/ros2_sims_ws
fi

# We set up the environment for ROS2
. /usr/share/gazebo/setup.sh
. ${ROS2_WS_PATH}/install/setup.bash

# We set up the environment for ROS2

export GAZEBO_RESOURCE_PATH=/home/user/ros2_ws/src/mycobot_sim/mycobot_ros2/mycobot_description:/home/kodama/TheConstruct/ros_worspaces/ur_ws/src/universal_robot_ros2/robotiq_85_gripper:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=/home/user/ros2_ws/src/mycobot_sim/mycobot_ros2/mycobot_description:/home/kodama/TheConstruct/ros_worspaces/ur_ws/src/universal_robot_ros2/robotnik_sensors:/home/kodama/TheConstruct/ros_worspaces/ur_ws/src/universal_robot_ros2/robotiq_85_gripper:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch mycobot_gazebo main_simple.launch.xml