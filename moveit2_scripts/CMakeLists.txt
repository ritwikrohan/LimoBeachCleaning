cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate the executable
add_executable(test_trajectory
    src/test_trajectory.cpp)
target_include_directories(test_trajectory
    PUBLIC include)
ament_target_dependencies(test_trajectory
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)
add_executable(test_trajectory2
    src/test_trajectory2.cpp)
target_include_directories(test_trajectory2
    PUBLIC include)
ament_target_dependencies(test_trajectory2
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(test_trajectory3
    src/test_trajectory3.cpp)
target_include_directories(test_trajectory3
    PUBLIC include)
ament_target_dependencies(test_trajectory3
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(test_gripper
    src/test_gripper.cpp)
target_include_directories(test_gripper
    PUBLIC include)
ament_target_dependencies(test_gripper
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(test_trajectory4
    src/test_trajectory4.cpp)
target_include_directories(test_trajectory4
    PUBLIC include)
ament_target_dependencies(test_trajectory4
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(pick_place
    src/pick_place.cpp)
target_include_directories(pick_place
    PUBLIC include)
ament_target_dependencies(pick_place
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)
add_executable(pick_server
    src/pick_server.cpp)
target_include_directories(pick_server
    PUBLIC include)
ament_target_dependencies(pick_server
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

add_executable(pick_server2
    src/pick_server2.cpp)
target_include_directories(pick_server2
    PUBLIC include)
ament_target_dependencies(pick_server2
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

install(TARGETS 
	test_trajectory
	test_trajectory2
	test_trajectory3
	test_trajectory4
	test_gripper
	pick_place
	pick_server
	pick_server2
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
