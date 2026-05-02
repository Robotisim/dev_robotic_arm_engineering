/**
 * @file moveit_pose_controller_node.cpp
 * @brief ROS2 node to control robot end effector pose using MoveIt2
 *
 * This node provides a service interface to send goal poses to MoveIt2's
 * move_group action server for the panda_arm planning group.
 */

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_options.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/workspace_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using MoveGroupAction = moveit_msgs::action::MoveGroup;
using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroupAction>;

/**
 * @class MoveItPoseController
 * @brief Controls robot end effector using MoveIt2 move group action
 */
class MoveItPoseController : public rclcpp::Node {
public:
	MoveItPoseController() : Node("moveit_pose_controller") {
		// Declare parameters
		this->declare_parameter<std::string>("move_action_name", "/move_action");
		this->declare_parameter<std::string>("planning_group", "panda_arm");
		this->declare_parameter<std::string>("planning_frame", "panda_link0");
		this->declare_parameter<std::string>("end_effector_link", "panda_link8");

		// Goal pose parameters (default values)
		this->declare_parameter<double>("goal_x", 0.4);
		this->declare_parameter<double>("goal_y", 0.0);
		this->declare_parameter<double>("goal_z", 0.4);
		this->declare_parameter<double>("goal_qx", 0.0);
		this->declare_parameter<double>("goal_qy", 0.0);
		this->declare_parameter<double>("goal_qz", 0.0);
		this->declare_parameter<double>("goal_qw", 1.0);

		// Planning parameters
		this->declare_parameter<double>("allowed_planning_time", 5.0);
		this->declare_parameter<int>("num_planning_attempts", 5);
		this->declare_parameter<double>("max_velocity_scaling", 0.2);
		this->declare_parameter<double>("max_acceleration_scaling", 0.2);
		this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
		this->declare_parameter<bool>("plan_only", false);

		// Get parameters
		action_name_ = this->get_parameter("move_action_name").as_string();
		planning_group_ = this->get_parameter("planning_group").as_string();
		planning_frame_ = this->get_parameter("planning_frame").as_string();
		end_effector_link_ = this->get_parameter("end_effector_link").as_string();

		// Create action client
		action_client_ = rclcpp_action::create_client<MoveGroupAction>(this, action_name_);

		// Create service to trigger goal execution
		execute_service_ = this->create_service<std_srvs::srv::Trigger>(
		    "execute_pose_goal",
		    std::bind(&MoveItPoseController::execute_goal_callback, this, std::placeholders::_1, std::placeholders::_2));

		RCLCPP_INFO(this->get_logger(), "MoveIt Pose Controller initialized");
		RCLCPP_INFO(this->get_logger(), "  Action: %s", action_name_.c_str());
		RCLCPP_INFO(this->get_logger(), "  Planning group: %s", planning_group_.c_str());
		RCLCPP_INFO(this->get_logger(), "  End effector: %s", end_effector_link_.c_str());
		RCLCPP_INFO(this->get_logger(), "Call service 'execute_pose_goal' to send goal");
	}

	/**
	 * @brief Send a pose goal to MoveIt2
	 * @param pose Target pose for the end effector
	 * @return true if goal was sent successfully
	 */
	bool send_pose_goal(geometry_msgs::msg::Pose const& pose) {
		// Wait for action server
		if (!action_client_->wait_for_action_server(10s)) {
			RCLCPP_ERROR(this->get_logger(), "MoveIt action server not available after 10 seconds");
			return false;
		}

		RCLCPP_INFO(this->get_logger(), "Action server available");

		// Build the goal message
		auto goal_msg = MoveGroupAction::Goal();

		// Set motion plan request
		auto& request = goal_msg.request;
		request.workspace_parameters.header.stamp = this->now();
		request.workspace_parameters.header.frame_id = planning_frame_;
		request.workspace_parameters.min_corner.x = -1.0;
		request.workspace_parameters.min_corner.y = -1.0;
		request.workspace_parameters.min_corner.z = -1.0;
		request.workspace_parameters.max_corner.x = 1.0;
		request.workspace_parameters.max_corner.y = 1.0;
		request.workspace_parameters.max_corner.z = 1.0;

		// Start state (empty = current state)
		request.start_state.is_diff = true;

		// Group name
		request.group_name = planning_group_;

		// Goal constraints
		moveit_msgs::msg::Constraints goal_constraints;

		// Position constraint
		moveit_msgs::msg::PositionConstraint pos_constraint;
		pos_constraint.header.frame_id = planning_frame_;
		pos_constraint.link_name = end_effector_link_;
		pos_constraint.weight = 1.0;

		shape_msgs::msg::SolidPrimitive primitive;
		primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
		primitive.dimensions.resize(1);
		primitive.dimensions[0] = 0.01; // tolerance radius in meters

		pos_constraint.constraint_region.primitives.push_back(primitive);

		geometry_msgs::msg::Pose constraint_pose;
		constraint_pose.position = pose.position;
		constraint_pose.orientation.w = 1.0;
		pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);

		goal_constraints.position_constraints.push_back(pos_constraint);

		// Orientation constraint
		moveit_msgs::msg::OrientationConstraint orient_constraint;
		orient_constraint.header.frame_id = planning_frame_;
		orient_constraint.link_name = end_effector_link_;
		orient_constraint.orientation = pose.orientation;
		orient_constraint.absolute_x_axis_tolerance = 0.1;
		orient_constraint.absolute_y_axis_tolerance = 0.1;
		orient_constraint.absolute_z_axis_tolerance = 0.1;
		orient_constraint.weight = 1.0;

		goal_constraints.orientation_constraints.push_back(orient_constraint);

		request.goal_constraints.push_back(goal_constraints);

		// Planning parameters
		request.allowed_planning_time = this->get_parameter("allowed_planning_time").as_double();
		request.num_planning_attempts = this->get_parameter("num_planning_attempts").as_int();
		request.max_velocity_scaling_factor = this->get_parameter("max_velocity_scaling").as_double();
		request.max_acceleration_scaling_factor = this->get_parameter("max_acceleration_scaling").as_double();
		request.planner_id = this->get_parameter("planner_id").as_string();

		// Planning options
		goal_msg.planning_options.plan_only = this->get_parameter("plan_only").as_bool();
		goal_msg.planning_options.planning_scene_diff.is_diff = true;
		goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

		RCLCPP_INFO(this->get_logger(),
		            "Sending pose goal: [%.3f, %.3f, %.3f] q[%.3f, %.3f, %.3f, %.3f]",
		            pose.position.x,
		            pose.position.y,
		            pose.position.z,
		            pose.orientation.x,
		            pose.orientation.y,
		            pose.orientation.z,
		            pose.orientation.w);

		// Send goal with callbacks
		auto send_goal_options = rclcpp_action::Client<MoveGroupAction>::SendGoalOptions();

		send_goal_options.goal_response_callback =
		    std::bind(&MoveItPoseController::goal_response_callback, this, std::placeholders::_1);

		send_goal_options.feedback_callback =
		    std::bind(&MoveItPoseController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

		send_goal_options.result_callback = std::bind(&MoveItPoseController::result_callback, this, std::placeholders::_1);

		goal_in_progress_ = true;
		action_client_->async_send_goal(goal_msg, send_goal_options);

		return true;
	}

private:
	// Action client
	rclcpp_action::Client<MoveGroupAction>::SharedPtr action_client_;

	// Service
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;

	// Parameters
	std::string action_name_;
	std::string planning_group_;
	std::string planning_frame_;
	std::string end_effector_link_;

	// State
	bool goal_in_progress_ = false;

	/**
	 * @brief Service callback to execute a pose goal from parameters
	 */
	void execute_goal_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
		(void)request;

		if (goal_in_progress_) {
			response->success = false;
			response->message = "Goal already in progress";
			RCLCPP_WARN(this->get_logger(), "Goal already in progress");
			return;
		}

		// Get goal pose from parameters
		geometry_msgs::msg::Pose goal_pose;
		goal_pose.position.x = this->get_parameter("goal_x").as_double();
		goal_pose.position.y = this->get_parameter("goal_y").as_double();
		goal_pose.position.z = this->get_parameter("goal_z").as_double();
		goal_pose.orientation.x = this->get_parameter("goal_qx").as_double();
		goal_pose.orientation.y = this->get_parameter("goal_qy").as_double();
		goal_pose.orientation.z = this->get_parameter("goal_qz").as_double();
		goal_pose.orientation.w = this->get_parameter("goal_qw").as_double();

		bool success = send_pose_goal(goal_pose);

		response->success = success;
		response->message = success ? "Goal sent to MoveIt" : "Failed to send goal";
	}

	/**
	 * @brief Callback when goal is accepted/rejected
	 */
	void goal_response_callback(GoalHandleMoveGroup::SharedPtr const& goal_handle) {
		if (!goal_handle) {
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			goal_in_progress_ = false;
		} else {
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
		}
	}

	/**
	 * @brief Callback for action feedback
	 */
	void feedback_callback(GoalHandleMoveGroup::SharedPtr,
	                       const std::shared_ptr<const MoveGroupAction::Feedback> feedback) {
		RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->state.c_str());
	}

	/**
	 * @brief Callback when action completes
	 */
	void result_callback(GoalHandleMoveGroup::WrappedResult const& result) {
		goal_in_progress_ = false;

		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				RCLCPP_INFO(this->get_logger(), "Goal execution succeeded!");
				if (result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
					RCLCPP_INFO(this->get_logger(), "MoveIt reports SUCCESS");
				} else {
					RCLCPP_WARN(this->get_logger(), "MoveIt error code: %d", result.result->error_code.val);
				}
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
				break;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
		}
	}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MoveItPoseController>();

	RCLCPP_INFO(node->get_logger(), "MoveIt Pose Controller node started");

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
