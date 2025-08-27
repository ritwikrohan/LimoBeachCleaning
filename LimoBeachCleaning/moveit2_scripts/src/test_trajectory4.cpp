// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node =
//       rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "ur_manipulator";
//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
//                                                             PLANNING_GROUP);

//   // We set parameters
//   move_group.setMaxVelocityScalingFactor(1.0);
//   move_group.setMaxAccelerationScalingFactor(1.0);
//   move_group.setPlanningTime(10.0);
//   move_group.setPoseReferenceFrame("base_footprint");

//   RCLCPP_INFO(LOGGER, "Planning frame: %s",
//               move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(LOGGER, "End effector link: %s",
//               move_group.getEndEffectorLink().c_str());

//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(),
//             move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   geometry_msgs::msg::Pose target_pose1;
//   target_pose1.position.x = 0.232;
//   target_pose1.position.y = -0.010;
//   target_pose1.position.z = 0.334;
//   // Set orientation if necessary
//   target_pose1.orientation.x = 0.0;
//   target_pose1.orientation.y = 0.0;
//   target_pose1.orientation.z = 0.0;
//   target_pose1.orientation.w = 1.0;

//   move_group.setPoseTarget(target_pose1);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success =
//       (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//     RCLCPP_INFO(LOGGER, "Moved to End Effector pose successfully.");
//   } else {
//     RCLCPP_ERROR(LOGGER, "Failed to move to End Effector pose.");
//     return 1;
//   }

//   // Execute the trajectory
//   if (move_group.execute(my_plan) ==
//       moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//     RCLCPP_INFO(LOGGER, "Moved to pose1 position executed successfully.");
//   } else {
//     RCLCPP_ERROR(LOGGER, "Failed to execute pose1 position.");
//   }

//   geometry_msgs::msg::Pose target_pose2;
//   target_pose2.position.x = -1.57;
//   target_pose2.position.y = 0.00;
//   target_pose2.position.z = 0.00;
//   // Set orientation if necessary
//   target_pose2.orientation.x = 0.0;
//   target_pose2.orientation.y = 0.0;
//   target_pose2.orientation.z = 0.0;
//   target_pose2.orientation.w = 1.0;

//   move_group.setPoseTarget(target_pose2);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success =
//       (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//     RCLCPP_INFO(LOGGER, "Moved to End Effector pose successfully.");
//   } else {
//     RCLCPP_ERROR(LOGGER, "Failed to move to End Effector pose.");
//     return 1;
//   }

//   // Execute the trajectory
//   if (move_group.execute(my_plan) ==
//       moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//     RCLCPP_INFO(LOGGER, "Moved to pose1 position executed successfully.");
//   } else {
//     RCLCPP_ERROR(LOGGER, "Failed to execute pose1 position.");
//   }

//   rclcpp::shutdown();
//   return 0;
// }

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <iterator>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// bool performCartesianPath(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<geometry_msgs::msg::Pose> &waypoints,
//     double jump_threshold, double eef_step,
//     moveit_msgs::msg::RobotTrajectory &trajectory) {
//   double fraction = move_group.computeCartesianPath(waypoints, eef_step,
//                                                     jump_threshold, trajectory);
//   if (fraction > 0.0) {
//     return move_group.execute(trajectory) ==
//            moveit::planning_interface::MoveItErrorCode::SUCCESS;
//   }
//   return false;
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setPlanningTime(10.0);
  move_group.setPoseReferenceFrame("base_footprint");

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // First target pose
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = 0.295151; //0.232;
  target_pose1.position.y = 0.000040;
  target_pose1.position.z = 0.334;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 1.0;

  move_group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;

  if (move_group.plan(my_plan1) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Planning to pose1 was successful.");
    if (move_group.execute(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Moved to pose1 position executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to execute pose1 position.");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to plan to End Effector pose1.");
    return 1;
  }

// Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");
  // geometry_msgs::msg::Pose target_pose1;

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
	// target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = -0.707;
  // target_pose1.orientation.z = 0.00;
  // target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.34;
  // target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.27;
  approach_waypoints.push_back(target_pose1);


	// target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = -0.707;
  // target_pose1.orientation.z = 0.00;
  // target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.34;
  // target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.231;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group.execute(trajectory_approach);


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

  std::vector<geometry_msgs::msg::Pose> retract_waypoints;
	// target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = -0.707;
  // target_pose1.orientation.z = 0.00;
  // target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.34;
  // target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.27;
  retract_waypoints.push_back(target_pose1);


	// target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = -0.707;
  // target_pose1.orientation.z = 0.00;
  // target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.34;
  // target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.334;
  retract_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retract;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  RCLCPP_INFO(LOGGER, "Hey I am here");

  double fraction2 = move_group.computeCartesianPath(
      retract_waypoints, eef_step, jump_threshold, trajectory_retract);

  move_group.execute(trajectory_retract);
moveit::core::RobotStatePtr current_state_arm =
      move_group.getCurrentState(10);
 const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
std::vector<double> joint_group_positions_arm;
  // std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  // current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
  //                                                joint_group_positions_gripper);

  move_group.setStartStateToCurrentState();
  // move_group_gripper.setStartStateToCurrentState();

  // Go Home
  // RCLCPP_INFO(LOGGER, "Going Home");
	RCLCPP_INFO(LOGGER, "Pregrasp Position");

  joint_group_positions_arm[0] = -3.13;  // Shoulder Pan
  joint_group_positions_arm[1] = -0.45259099172041023; // Shoulder Lift
  joint_group_positions_arm[2] = 0.699310610545014;  // Elbow
  joint_group_positions_arm[3] = -0.00019428551668579952; // Wrist 1
  joint_group_positions_arm[4] = -1.3239000773581084; // Wrist 2
  joint_group_positions_arm[5] = 0.6999047380940878;  // Wrist 3

  move_group.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group.execute(my_plan_arm);


 // Close the gripper using a named target
  RCLCPP_INFO(LOGGER, "opening the gripper.");
  if (move_group_gripper.setNamedTarget("open") &&
      move_group_gripper.move() == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Gripper opened successfully.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to open the gripper.");
  }

  // Wait for a few seconds
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  return 0;
}
