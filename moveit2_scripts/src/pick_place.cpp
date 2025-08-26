#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class GetPoseClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetPoseClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("get_pose_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Find>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&GetPoseClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  const std::vector<grasping_msgs::msg::GraspableObject>& get_result() const {
    return result_;
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GetPoseClient::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::vector<grasping_msgs::msg::GraspableObject> result_;

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFind::SharedPtr,
                         const std::shared_ptr<const Find::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }

  void result_callback(const GoalHandleFind::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "\033[1;32mResult received\033[0m");
    result_ = result.result->objects;
  }
}; // class GetPoseClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetPoseClient>();
  double x_pose = std::numeric_limits<double>::quiet_NaN();
  double y_pose = std::numeric_limits<double>::quiet_NaN();

  double error_x = -0.002615;
  double error_y= 0.000043;
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  // Access the result using the getter method
  const auto& result = action_client->get_result();
  for (const auto& object : result) {
    if (object.object.primitives[0].type == 1 &&
        object.object.primitives[0].dimensions[0] < 0.05 &&
        object.object.primitives[0].dimensions[1] < 0.05 &&
        object.object.primitives[0].dimensions[2] < 0.1) {
    //   RCLCPP_INFO(action_client->get_logger(), "X: %f",
    //               object.object.primitive_poses[0].position.x);
    //   RCLCPP_INFO(action_client->get_logger(), "Y: %f",
    //               object.object.primitive_poses[0].position.y);
        RCLCPP_INFO(action_client->get_logger(), "\033[1;32mX Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.x);
        RCLCPP_INFO(action_client->get_logger(), "\033[1;32mY Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.y);
        RCLCPP_INFO(action_client->get_logger(), "\033[1;32mZ Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.z);
        x_pose = object.object.primitive_poses[0].position.x;
        y_pose = object.object.primitive_poses[0].position.y;
    }
    else {
            // Assign NaN if object dimensions condition is not met
            x_pose = std::numeric_limits<double>::quiet_NaN();
            y_pose = std::numeric_limits<double>::quiet_NaN();
        }
  }

   // Check if x_pose and y_pose are still None
  if (std::isnan(x_pose) || std::isnan(y_pose)) {
    RCLCPP_ERROR(action_client->get_logger(), "No valid object detected. Exiting.");
    rclcpp::shutdown();
    return 1;  // Exit with an error code
  }

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

	

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);
  move_group_arm.setPlanningTime(10.0);
  move_group_arm.setPoseReferenceFrame("base_footprint");

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  RCLCPP_INFO(action_client->get_logger(), "\033[1;32mX Pose : %f\033[0m", x_pose);
  RCLCPP_INFO(action_client->get_logger(), "\033[1;32mY Pose : %f\033[0m", y_pose);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 1.00;
  target_pose1.position.x = x_pose;
  target_pose1.position.y = y_pose;
  target_pose1.position.z = 0.334;
  move_group_arm.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.x += error_x;
  target_pose1.position.y += error_y;
  target_pose1.position.z -= 0.07;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.04;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Retreat
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.1;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.1;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] = -3.14; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}
