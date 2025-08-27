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
  target_pose1.position.x = 0.232;
  target_pose1.position.y = -0.010;
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


    const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group,
                                          joint_group_positions);

    // Go Home movement
    std::vector<double> home_position = {-1.57, 0.0, 0.7, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(home_position);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

    bool success =
        (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Moved to home position successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to move to home position.");
      return 1;
    }

    // Execute the trajectory
    if (move_group.execute(my_plan2) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Moved to home position executed successfully.");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to execute home position.");
    }

  rclcpp::shutdown();
  return 0;
}
