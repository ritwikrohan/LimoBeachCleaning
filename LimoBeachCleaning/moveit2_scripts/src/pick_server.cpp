// #include "grasping_msgs/action/find_graspable_objects.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <memory>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_service_node");
// static const double ERROR_X = -0.002615; // Error in x position
// static const double ERROR_Y = 0.000043;   // Error in y position

// class GetPoseClient : public rclcpp::Node {
// public:
//     using Find = grasping_msgs::action::FindGraspableObjects;
//     using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

//     explicit GetPoseClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
//         : Node("get_pose_client", node_options), goal_done_(false) {
//         this->client_ptr_ = rclcpp_action::create_client<Find>(
//             this->get_node_base_interface(), this->get_node_graph_interface(),
//             this->get_node_logging_interface(),
//             this->get_node_waitables_interface(), "find_objects");

//         // this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
//         //     std::bind(&GetPoseClient::send_goal, this));
//     }

//     void send_goal() {
//         // this->timer_->cancel();
//         this->goal_done_ = false;

//         if (!this->client_ptr_) {
//             RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
//             return;
//         }

//         if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
//             RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//             this->goal_done_ = true;
//             return;
//         }

//         auto goal_msg = Find::Goal();
//         goal_msg.plan_grasps = false;

//         RCLCPP_INFO(this->get_logger(), "Sending goal");

//         auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
//         send_goal_options.goal_response_callback =
//             std::bind(&GetPoseClient::goal_response_callback, this, std::placeholders::_1);
//         send_goal_options.feedback_callback =
//             std::bind(&GetPoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
//         send_goal_options.result_callback =
//             std::bind(&GetPoseClient::result_callback, this, std::placeholders::_1);

//         // this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//         auto goal_handle_future =
//         this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//     }

//     bool is_goal_done() const { return this->goal_done_; }

//     const std::vector<grasping_msgs::msg::GraspableObject>& get_result() const {
//         return result_;
//     }

//     void check_result() {
//         if (this->goal_done_) {
//             // Result processing can happen here or elsewhere depending on your design
//         }
//     }

// private:
//     rclcpp_action::Client<Find>::SharedPtr client_ptr_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     bool goal_done_;
//     std::vector<grasping_msgs::msg::GraspableObject> result_;

//     void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
//         RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//         if (!goal_handle) {
//             RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//         }
//     }

//     void feedback_callback(GoalHandleFind::SharedPtr, 
//         const std::shared_ptr<const Find::Feedback> feedback) {
//         RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
//     }

//     // void result_callback(const GoalHandleFind::WrappedResult &result) {
//     //     this->goal_done_ = true;
//     //     switch (result.code) {
//     //     case rclcpp_action::ResultCode::SUCCEEDED:
//     //         result_ = result.result->objects;
//     //         RCLCPP_INFO(this->get_logger(), "\033[1;32mResult received\033[0m");
//     //         break;
//     //     case rclcpp_action::ResultCode::ABORTED:
//     //         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//     //         return;
//     //     case rclcpp_action::ResultCode::CANCELED:
//     //         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//     //         return;
//     //     default:
//     //         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//     //         return;
//     //     }
//     // }
//     void result_callback(const GoalHandleFind::WrappedResult &result) {
//     this->goal_done_ = true;
//     switch (result.code) {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//       return;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//       return;
//     default:
//       RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//       return;
//     }

//     // RCLCPP_INFO(this->get_logger(), "Result received");
//     RCLCPP_INFO(this->get_logger(), "\033[1;32mResult received\033[0m");
//     result_ = result.result->objects;
//   }
// };

// class MotionService : public rclcpp::Node {
// public:
//     MotionService() : Node("motion_service_node") {
//         service_ = this->create_service<std_srvs::srv::Empty>(
//             "execute_motion", std::bind(&MotionService::execute_motion_callback, this, std::placeholders::_1, std::placeholders::_2));
//     action_client_ = std::make_shared<GetPoseClient>();
//     }


//     void on_action_complete() {
//         if (action_client_ && action_client_->is_goal_done()) {
//             detected_objects_ = action_client_->get_result();
//             double x_pose = std::numeric_limits<double>::quiet_NaN();
//             double y_pose = std::numeric_limits<double>::quiet_NaN();

//             for (const auto& object : detected_objects_) {
//                 if (object.object.primitives[0].type == 1 &&
//                     object.object.primitives[0].dimensions[0] < 0.05 &&
//                     object.object.primitives[0].dimensions[1] < 0.05 &&
//                     object.object.primitives[0].dimensions[2] < 0.1) {

//                     x_pose = object.object.primitive_poses[0].position.x;
//                     y_pose = object.object.primitive_poses[0].position.y;
//                     RCLCPP_INFO(LOGGER, "Detected Object: X=%f, Y=%f", x_pose, y_pose);
//                 }
//             }

//             if (!(std::isnan(x_pose) || std::isnan(y_pose))) {
//                 execute_moveit_logic(x_pose, y_pose);
//             } else {
//                 RCLCPP_ERROR(LOGGER, "No valid object detected. Exiting service.");
//             }
//         }
//     }

// private:
//     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
//     std::shared_ptr<GetPoseClient> action_client_;
//     std::vector<grasping_msgs::msg::GraspableObject> detected_objects_;

//     // void execute_motion_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//     //                              const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
//     //     RCLCPP_INFO(LOGGER, "Service called to execute motion.");
//     //     // action_client_ = std::make_shared<GetPoseClient>();
//     //     // std::unique_ptr<GetPoseClient> action_client_;  // Change this
//     //     // action_client_ = std::make_unique<GetPoseClient>();
//     //     // action_client_->send_goal(); // Send the action goal
//     //     // RCLCPP_INFO(this->get_logger(), "Before calling send_goal");
//     //     // action_client_->send_goal();
//     //     // RCLCPP_INFO(this->get_logger(), "After calling send_goal");

//     // }
//     void execute_motion_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//                              const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
//     RCLCPP_INFO(this->get_logger(), "Service called to execute motion.");

//     // Check if action_client_ is initialized
//     if (!action_client_) {
//         RCLCPP_ERROR(this->get_logger(), "Action client is not initialized.");
//         return;
//     }

//     // Attempt to send the goal
//     try {
//         action_client_->send_goal(); // Attempt to send the action goal
//         RCLCPP_INFO(this->get_logger(), "Goal sent to action client.");
//     } catch (const std::exception &e) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to send goal: %s", e.what());
//     }
// }


    

//     void execute_moveit_logic(double x_pose, double y_pose) {
//         auto node_ptr = this->shared_from_this();
//         static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
//         static const std::string PLANNING_GROUP_GRIPPER = "gripper";
//         double error_x = -0.002615;
//         double error_y= 0.000043;

//         moveit::planning_interface::MoveGroupInterface move_group_arm(node_ptr,PLANNING_GROUP_ARM);
//         moveit::planning_interface::MoveGroupInterface move_group_gripper(node_ptr,PLANNING_GROUP_GRIPPER);

//           move_group_arm.setMaxVelocityScalingFactor(1.0);
//           move_group_arm.setMaxAccelerationScalingFactor(1.0);
//           move_group_arm.setPlanningTime(10.0);
//           move_group_arm.setPoseReferenceFrame("base_footprint");

//           const moveit::core::JointModelGroup *joint_model_group_arm =
//               move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
//           const moveit::core::JointModelGroup *joint_model_group_gripper =
//               move_group_gripper.getCurrentState()->getJointModelGroup(
//                   PLANNING_GROUP_GRIPPER);
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           // Get Current State
//           moveit::core::RobotStatePtr current_state_arm =
//               move_group_arm.getCurrentState(10);
//           moveit::core::RobotStatePtr current_state_gripper =
//               move_group_gripper.getCurrentState(10);

//           std::vector<double> joint_group_positions_arm;
//           std::vector<double> joint_group_positions_gripper;
//           current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                                     joint_group_positions_arm);
//           current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
//                                                         joint_group_positions_gripper);

//           move_group_arm.setStartStateToCurrentState();
//           move_group_gripper.setStartStateToCurrentState();

//           RCLCPP_INFO(LOGGER, "Pregrasp Position");
//           RCLCPP_INFO(LOGGER, "\033[1;32mX Pose : %f\033[0m", x_pose);
//           RCLCPP_INFO(LOGGER, "\033[1;32mY Pose : %f\033[0m", y_pose);

//           geometry_msgs::msg::Pose target_pose1;
//           target_pose1.orientation.x = 0.0;
//           target_pose1.orientation.y = 0.0;
//           target_pose1.orientation.z = 0.00;
//           target_pose1.orientation.w = 1.00;
//           target_pose1.position.x = x_pose;
//           target_pose1.position.y = y_pose;
//           target_pose1.position.z = 0.334;
//           move_group_arm.setPoseTarget(target_pose1);

//           moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
//           bool success_arm = (move_group_arm.plan(my_plan_arm) ==
//                         moveit::core::MoveItErrorCode::SUCCESS);

//           move_group_arm.execute(my_plan_arm);
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           // Open Gripper

//           RCLCPP_INFO(LOGGER, "Open Gripper!");

//           move_group_gripper.setNamedTarget("open");

//           moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
//           bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                                   moveit::core::MoveItErrorCode::SUCCESS);

//           move_group_gripper.execute(my_plan_gripper);
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           // Approach
//           RCLCPP_INFO(LOGGER, "Approach to object!");

//           std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//           target_pose1.position.x += error_x;
//           target_pose1.position.y += error_y;
//           target_pose1.position.z -= 0.07;
//           approach_waypoints.push_back(target_pose1);

//           target_pose1.position.z -= 0.04;
//           approach_waypoints.push_back(target_pose1);

//           moveit_msgs::msg::RobotTrajectory trajectory_approach;
//           const double jump_threshold = 0.0;
//           const double eef_step = 0.01;

//           double fraction = move_group_arm.computeCartesianPath(
//               approach_waypoints, eef_step, jump_threshold, trajectory_approach);

//           move_group_arm.execute(trajectory_approach);
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           // Close Gripper

//           RCLCPP_INFO(LOGGER, "Close Gripper!");

//           move_group_gripper.setNamedTarget("close");

//           success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                             moveit::core::MoveItErrorCode::SUCCESS);

//           move_group_gripper.execute(my_plan_gripper);

//           // Retreat
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           RCLCPP_INFO(LOGGER, "Retreat from object!");

//           std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
//           target_pose1.position.z += 0.1;
//           retreat_waypoints.push_back(target_pose1);

//           target_pose1.position.z += 0.1;
//           retreat_waypoints.push_back(target_pose1);

//           moveit_msgs::msg::RobotTrajectory trajectory_retreat;

//           fraction = move_group_arm.computeCartesianPath(
//               retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

//           move_group_arm.execute(trajectory_retreat);

//           std::this_thread::sleep_for(std::chrono::seconds(1));

//           RCLCPP_INFO(LOGGER, "Rotating Arm");

//           current_state_arm = move_group_arm.getCurrentState(10);
//           current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                                     joint_group_positions_arm);

//           joint_group_positions_arm[0] = -3.14; // Shoulder Pan

//           move_group_arm.setJointValueTarget(joint_group_positions_arm);

//           success_arm = (move_group_arm.plan(my_plan_arm) ==
//                         moveit::core::MoveItErrorCode::SUCCESS);

//           move_group_arm.execute(my_plan_arm);
//           std::this_thread::sleep_for(std::chrono::seconds(1));
//           // Open Gripper

//           RCLCPP_INFO(LOGGER, "Release Object!");

//           move_group_gripper.setNamedTarget("open");

//           success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                             moveit::core::MoveItErrorCode::SUCCESS);

//           move_group_gripper.execute(my_plan_gripper);
//             }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto motion_service = std::make_shared<MotionService>();

//     // Create the executor for the node
//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(motion_service);

//     // Start executing and processing callbacks
//     executor.spin();  // This handles all callbacks and keeps the node alive

//     // Finally, shutdown ROS 2
//     rclcpp::shutdown();
//     return 0;
// }


#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
class MotionServiceNode : public rclcpp::Node {
public:
    using FindGraspableObjects = grasping_msgs::action::FindGraspableObjects;
    using GoalHandleFindGraspableObjects = rclcpp_action::ClientGoalHandle<FindGraspableObjects>;
    // using Find = grasping_msgs::action::FindGraspableObjects;
    // using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

    MotionServiceNode() : Node("motion_service_node"), goal_done_(false) {
        client_ptr_ = rclcpp_action::create_client<FindGraspableObjects>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_logging_interface(), this->get_node_waitables_interface(), "find_objects");

        // Service to execute motions
        service_ = create_service<std_srvs::srv::Empty>(
            "execute_motion", std::bind(&MotionServiceNode::execute_motion_callback, this, std::placeholders::_1, std::placeholders::_2));

    }

private:
    rclcpp_action::Client<FindGraspableObjects>::SharedPtr client_ptr_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    
    bool goal_done_;
    double detected_x_pose_;
    double detected_y_pose_;
    std::vector<grasping_msgs::msg::GraspableObject> result_;

    void send_goal() {
        auto goal_msg = FindGraspableObjects::Goal();
        goal_msg.plan_grasps = false;

        auto send_goal_options = rclcpp_action::Client<FindGraspableObjects>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MotionServiceNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MotionServiceNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MotionServiceNode::result_callback, this, std::placeholders::_1);

        // Send the goal to the action server
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void execute_moveit_logic(std::vector<grasping_msgs::msg::GraspableObject> &result_2){

        auto node_ptr = shared_from_this();

        std::thread([node_ptr, result_2]() {
                                                double x_pose = std::numeric_limits<double>::quiet_NaN();
                                        double y_pose = std::numeric_limits<double>::quiet_NaN();

                                        double error_x = -0.002615;
                                        double error_y= 0.000043;
                                        //   while (!action_client->is_goal_done()) {
                                        //     rclcpp::spin_some(action_client);
                                        //   }

                                        // Access the result using the getter method
                                        //   const auto& result = result_2;
                                        for (const auto& object : result_2) {
                                            if (object.object.primitives[0].type == 1 &&
                                                object.object.primitives[0].dimensions[0] < 0.05 &&
                                                object.object.primitives[0].dimensions[1] < 0.05 &&
                                                object.object.primitives[0].dimensions[2] < 0.1) {
                                            //   RCLCPP_INFO(LOGGER, "X: %f",
                                            //               object.object.primitive_poses[0].position.x);
                                            //   RCLCPP_INFO(LOGGER, "Y: %f",
                                            //               object.object.primitive_poses[0].position.y);
                                                RCLCPP_INFO(LOGGER, "\033[1;32mX Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.x);
                                                RCLCPP_INFO(LOGGER, "\033[1;32mY Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.y);
                                                RCLCPP_INFO(LOGGER, "\033[1;32mZ Pose of the cube: %f\033[0m", object.object.primitive_poses[0].position.z);
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
                                            RCLCPP_ERROR(LOGGER, "No valid object detected. Exiting.");
                                            // rclcpp::shutdown();
                                            // return 1;  // Exit with an error code
                                        }

                                        //   rclcpp::NodeOptions node_options;
                                        //   node_options.automatically_declare_parameters_from_overrides(true);
                                        //   auto move_group_node =
                                        //       rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

                                        //   rclcpp::executors::SingleThreadedExecutor executor;
                                        //   executor.add_node(move_group_node);
                                        //   std::thread([&executor]() { executor.spin(); }).detach();

                                            

                                        static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
                                        static const std::string PLANNING_GROUP_GRIPPER = "gripper";

                                        moveit::planning_interface::MoveGroupInterface move_group_arm(
                                            node_ptr, PLANNING_GROUP_ARM);
                                        moveit::planning_interface::MoveGroupInterface move_group_gripper(
                                            node_ptr, PLANNING_GROUP_GRIPPER);

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
                                        RCLCPP_INFO(LOGGER, "\033[1;32mX Pose : %f\033[0m", x_pose);
                                        RCLCPP_INFO(LOGGER, "\033[1;32mY Pose : %f\033[0m", y_pose);

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

                                        joint_group_positions_arm[1] = -0.5; // Shoulder Pan

                                        move_group_arm.setJointValueTarget(joint_group_positions_arm);

                                        success_arm = (move_group_arm.plan(my_plan_arm) ==
                                                        moveit::core::MoveItErrorCode::SUCCESS);

                                        move_group_arm.execute(my_plan_arm);
                                        // std::this_thread::sleep_for(std::chrono::seconds(1));
                                        // Open Gripper

                                        // RCLCPP_INFO(LOGGER, "Release Object!");

                                        // move_group_gripper.setNamedTarget("open");

                                        // success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                                        //                     moveit::core::MoveItErrorCode::SUCCESS);

                                        // move_group_gripper.execute(my_plan_gripper);
    }).detach(); 
    }

    void execute_motion_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Service called to execute motion.");

        // Call send_goal to fetch the most recent poses
        this->send_goal(); 

        // Motion will be executed after poses are retrieved from result_callback
    }

    void goal_response_callback(const GoalHandleFindGraspableObjects::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }

    void feedback_callback(GoalHandleFindGraspableObjects::SharedPtr, const std::shared_ptr<const FindGraspableObjects::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
    }

    void result_callback(const GoalHandleFindGraspableObjects::WrappedResult &result) {
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
    this->result_ = result.result->objects;
    this->execute_moveit_logic(this->result_);
  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto motion_service_node = std::make_shared<MotionServiceNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motion_service_node);

    // Start executing and processing callbacks
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


    