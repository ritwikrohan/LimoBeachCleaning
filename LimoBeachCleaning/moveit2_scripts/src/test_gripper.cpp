#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto move_group_node = rclcpp::Node::make_shared(
      "move_group_interface_tutorial",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // Close the gripper using a named target
  RCLCPP_INFO(LOGGER, "Closing the gripper.");
  if (move_group_gripper.setNamedTarget("close") &&
      move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper closed successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to close the gripper.");
  }

  // Wait for a few seconds
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Close Custom the gripper
  // Get current state of the gripper
  auto current_state_gripper = move_group_gripper.getCurrentState();
  std::vector<double> joint_group_positions_gripper;
  current_state_gripper->copyJointGroupPositions(
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER),
      joint_group_positions_gripper);

  RCLCPP_INFO(LOGGER, "Closing Custom the gripper.");
  joint_group_positions_gripper[2] =
      0.55; // Assuming the gripper's relevant joint is at index 2
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  if (move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper Custom closed successfully.");
  } else {
    RCLCPP_INFO(LOGGER, "Failed Custom closed.");
  }

  // Wait for a few seconds
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Close the gripper using a named target
  RCLCPP_INFO(LOGGER, "Closing the gripper.");
  if (move_group_gripper.setNamedTarget("open") &&
      move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper Opened successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to Open the gripper.");
  }

  rclcpp::shutdown();
  return 0;
}
