#! /usr/bin/env bash

# We remove a folder that otherwise gives issues in ROS2 launches
sudo rm -r /home/user/.ros

# We set up the environment for ROS2
. /usr/share/gazebo/setup.sh
. /home/simulations/ros2_sims_ws/install/setup.bash
#export GAZEBO_RESOURCE_PATH=/home/simulations/ros2_sims_ws/src/t3_foxy/turtlebot3_simulations/turtlebot3_gazebo:${GAZEBO_RESOURCE_PATH}
export GAZEBO_RESOURCE_PATH=/home/simulations/ros2_sims_ws/src/universal_robot_ros2/Universal_Robots_ROS2_Description:${GAZEBO_RESOURCE_PATH}
#export GAZEBO_MODEL_PATH=/home/simulations/ros2_sims_ws/src/t3_foxy/turtlebot3_simulations/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=/home/simulations/ros2_sims_ws/src/universal_robot_ros2/robotnik_sensors:${GAZEBO_MODEL_PATH}

timeout 45 ros2 launch the_construct_office_gazebo empty_ur3e.launch.xml
rkill -9 $(pidof gzserver)
ros2 launch the_construct_office_gazebo empty_ur3e.launch.xml