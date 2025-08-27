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
        // client_ptr_ = rclcpp_action::create_client<FindGraspableObjects>(
        //     this->get_node_base_interface(), this->get_node_graph_interface(),
        //     this->get_node_logging_interface(), this->get_node_waitables_interface(), "find_objects");

        // Service to execute motions
        service_ = create_service<std_srvs::srv::Empty>(
            "execute_motion_2", std::bind(&MotionServiceNode::execute_motion_callback, this, std::placeholders::_1, std::placeholders::_2));

    }

private:
    // rclcpp_action::Client<FindGraspableObjects>::SharedPtr client_ptr_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    
    bool goal_done_;
    double detected_x_pose_;
    double detected_y_pose_;
    std::vector<grasping_msgs::msg::GraspableObject> result_;

    // void send_goal() {
    //     auto goal_msg = FindGraspableObjects::Goal();
    //     goal_msg.plan_grasps = false;

    //     auto send_goal_options = rclcpp_action::Client<FindGraspableObjects>::SendGoalOptions();
    //     send_goal_options.goal_response_callback =
    //         std::bind(&MotionServiceNode::goal_response_callback, this, std::placeholders::_1);
    //     send_goal_options.feedback_callback =
    //         std::bind(&MotionServiceNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    //     send_goal_options.result_callback =
    //         std::bind(&MotionServiceNode::result_callback, this, std::placeholders::_1);

    //     // Send the goal to the action server
    //     client_ptr_->async_send_goal(goal_msg, send_goal_options);
    // }

    void execute_moveit_logic(){

        auto node_ptr = shared_from_this();

        std::thread([node_ptr]() {
                                          

                                    
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

                                         RCLCPP_INFO(LOGGER, "Rotating Arm");

                                        current_state_arm = move_group_arm.getCurrentState(10);
                                        current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                                                    joint_group_positions_arm);

                                        joint_group_positions_arm[1] = -1.0; // Shoulder Pan

                                        move_group_arm.setJointValueTarget(joint_group_positions_arm);

                                        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
                                        bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                                                        moveit::core::MoveItErrorCode::SUCCESS);

                                        move_group_arm.execute(my_plan_arm);

                                        // RCLCPP_INFO(LOGGER, "Pregrasp Position");
                                        // RCLCPP_INFO(LOGGER, "\033[1;32mX Pose : %f\033[0m", x_pose);
                                        // RCLCPP_INFO(LOGGER, "\033[1;32mY Pose : %f\033[0m", y_pose);

                                        // Open Gripper

                                        RCLCPP_INFO(LOGGER, "Open Gripper!");

                                        move_group_gripper.setNamedTarget("open");

                                        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
                                        bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                                                                moveit::core::MoveItErrorCode::SUCCESS);

                                        move_group_gripper.execute(my_plan_gripper);
                                        std::this_thread::sleep_for(std::chrono::seconds(1));
                                        // Approach

                                        // std::this_thread::sleep_for(std::chrono::seconds(1));

                                       // Move to the "home" position
                                       RCLCPP_INFO(LOGGER, "Moving to Home Position");
                                       move_group_arm.setNamedTarget("home");
                                       moveit::planning_interface::MoveGroupInterface::Plan home_plan;
                                       bool success_home = (move_group_arm.plan(home_plan) ==
                                                            moveit::core::MoveItErrorCode::SUCCESS);
                                       if (success_home) {
                                           move_group_arm.execute(home_plan);
                                       } else {
                                           RCLCPP_ERROR(LOGGER, "Failed to plan to home position");
                                       }

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
        this->execute_moveit_logic(); 

        // Motion will be executed after poses are retrieved from result_callback
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


    