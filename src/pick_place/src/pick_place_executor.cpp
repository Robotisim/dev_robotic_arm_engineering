#include "pick_place/pick_place_executor.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pick_place {

	namespace {

		double positionDistance(geometry_msgs::msg::PoseStamped const& a, geometry_msgs::msg::PoseStamped const& b) {
			double dx = a.pose.position.x - b.pose.position.x;
			double dy = a.pose.position.y - b.pose.position.y;
			double dz = a.pose.position.z - b.pose.position.z;
			return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
		}

		std::string chooseArmGroupName(std::vector<std::string> const& available_groups,
		                               std::string const& requested_group) {
			auto has_group = [&](std::string const& name) {
				return std::find(available_groups.begin(), available_groups.end(), name) != available_groups.end();
			};

			if (has_group(requested_group)) {
				return requested_group;
			}

			if (has_group("panda_arm")) {
				return "panda_arm";
			}

			if (has_group("arm")) {
				return "arm";
			}

			for (auto const& name : available_groups) {
				if (name != "hand") {
					return name;
				}
			}

			return {};
		}

	} // namespace

	PickPlaceExecutor::PickPlaceExecutor(rclcpp::NodeOptions const& options) : Node("pick_place_executor", options) {
		RCLCPP_INFO(this->get_logger(), "PickPlaceExecutor Node initialized");

		this->declare_parameter("pick_place_executor.base_frame", "panda_link0");
		this->declare_parameter("pick_place_executor.arm_group_name", "arm");
		this->declare_parameter("pick_place_executor.hand_group_name", "hand");
		this->declare_parameter("pick_place_executor.home_named_target", "ready");
		this->declare_parameter("pick_place_executor.detection_stability_count", 5);
		this->declare_parameter("pick_place_executor.detection_stability_dist", 0.02);
		this->declare_parameter("pick_place_executor.approach_offset_z", 0.1);
		this->declare_parameter("pick_place_executor.cartesian_step", 0.01);
		this->declare_parameter("pick_place_executor.min_cartesian_fraction", 0.9);
		this->declare_parameter("pick_place_executor.execute_enabled", false);
		this->declare_parameter("pick_place_executor.execute_pick_only", true);
		this->declare_parameter("pick_place_executor.place_pose.x", 0.4);
		this->declare_parameter("pick_place_executor.place_pose.y", -0.3);
		this->declare_parameter("pick_place_executor.place_pose.z", 0.2);
		this->declare_parameter("pick_place_executor.place_pose.qx", 1.0);
		this->declare_parameter("pick_place_executor.place_pose.qy", 0.0);
		this->declare_parameter("pick_place_executor.place_pose.qz", 0.0);
		this->declare_parameter("pick_place_executor.place_pose.qw", 0.0);

		base_frame_ = this->get_parameter("pick_place_executor.base_frame").as_string();
		arm_group_name_ = this->get_parameter("pick_place_executor.arm_group_name").as_string();
		hand_group_name_ = this->get_parameter("pick_place_executor.hand_group_name").as_string();
		home_named_target_ = this->get_parameter("pick_place_executor.home_named_target").as_string();
		detection_stability_count_ = this->get_parameter("pick_place_executor.detection_stability_count").as_int();
		detection_stability_dist_ = this->get_parameter("pick_place_executor.detection_stability_dist").as_double();
		approach_offset_z_ = this->get_parameter("pick_place_executor.approach_offset_z").as_double();
		cartesian_step_ = this->get_parameter("pick_place_executor.cartesian_step").as_double();
		min_cartesian_fraction_ = this->get_parameter("pick_place_executor.min_cartesian_fraction").as_double();
		execute_enabled_ = this->get_parameter("pick_place_executor.execute_enabled").as_bool();
		execute_pick_only_ = this->get_parameter("pick_place_executor.execute_pick_only").as_bool();

		place_pose_.header.frame_id = base_frame_;
		place_pose_.pose.position.x = this->get_parameter("pick_place_executor.place_pose.x").as_double();
		place_pose_.pose.position.y = this->get_parameter("pick_place_executor.place_pose.y").as_double();
		place_pose_.pose.position.z = this->get_parameter("pick_place_executor.place_pose.z").as_double();
		place_pose_.pose.orientation.x = this->get_parameter("pick_place_executor.place_pose.qx").as_double();
		place_pose_.pose.orientation.y = this->get_parameter("pick_place_executor.place_pose.qy").as_double();
		place_pose_.pose.orientation.z = this->get_parameter("pick_place_executor.place_pose.qz").as_double();
		place_pose_.pose.orientation.w = this->get_parameter("pick_place_executor.place_pose.qw").as_double();

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

		std::vector<std::string> available_groups;
		std::string resolved_arm_group_name;
		try {
			robot_model_loader::RobotModelLoader model_loader(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}),
			                                                  "robot_description");
			auto model = model_loader.getModel();
			if (model) {
				available_groups = model->getJointModelGroupNames();
				resolved_arm_group_name = chooseArmGroupName(available_groups, arm_group_name_);
			}
		} catch (std::exception const& ex) {
			RCLCPP_WARN(this->get_logger(), "Robot model load for group detection failed: %s", ex.what());
		}

		if (!resolved_arm_group_name.empty()) {
			if (resolved_arm_group_name != arm_group_name_) {
				RCLCPP_WARN(this->get_logger(),
				            "Requested arm group '%s' not available; using '%s'",
				            arm_group_name_.c_str(),
				            resolved_arm_group_name.c_str());
			}
			arm_group_name_ = resolved_arm_group_name;
		}

		try {
			auto node_handle = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
			arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_handle, arm_group_name_);
			arm_move_group_->setPoseReferenceFrame(base_frame_);
			arm_move_group_->setPlanningTime(5.0);
			hand_move_group_ =
			    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_handle, hand_group_name_);
			hand_move_group_->setPlanningTime(3.0);
		} catch (std::exception const& ex) {
			arm_move_group_.reset();
			hand_move_group_.reset();
			RCLCPP_ERROR(
			    this->get_logger(), "MoveGroup initialization failed for group '%s': %s", arm_group_name_.c_str(), ex.what());
			RCLCPP_ERROR(this->get_logger(), "Planning is disabled until arm group configuration is corrected.");
		}

		object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		    "/pick_place/object_pose", 10, std::bind(&PickPlaceExecutor::objectPoseCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Subscribed to /pick_place/object_pose");
		RCLCPP_INFO(this->get_logger(), "Transform target base frame: %s", base_frame_.c_str());
		RCLCPP_INFO(this->get_logger(), "Arm planning group: %s", arm_group_name_.c_str());
		RCLCPP_INFO(this->get_logger(), "Hand planning group: %s", hand_group_name_.c_str());
		if (!available_groups.empty()) {
			std::string joined;
			for (size_t i = 0; i < available_groups.size(); ++i) {
				joined += available_groups[i];
				if (i + 1 < available_groups.size()) {
					joined += ", ";
				}
			}
			RCLCPP_INFO(this->get_logger(), "Available MoveIt groups: %s", joined.c_str());
		}
		RCLCPP_INFO(this->get_logger(),
		            "Detection stability: count=%ld, distance_threshold=%.3f m",
		            detection_stability_count_,
		            detection_stability_dist_);
		RCLCPP_INFO(this->get_logger(),
		            "Approach offset: %.3f m | Cartesian step: %.3f m | Min Cartesian fraction: %.2f",
		            approach_offset_z_,
		            cartesian_step_,
		            min_cartesian_fraction_);
		RCLCPP_INFO(this->get_logger(),
		            "Execution mode: execute_enabled=%s, execute_pick_only=%s",
		            execute_enabled_ ? "true" : "false",
		            execute_pick_only_ ? "true" : "false");

		setState(State::DETECTING);
	}

	void PickPlaceExecutor::objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
		last_object_pose_ = msg;
		geometry_msgs::msg::PoseStamped transformed_pose;

		try {
			transformed_pose = tf_buffer_->transform(*msg, base_frame_, tf2::durationFromSec(0.1));
		} catch (tf2::TransformException const& ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(),
			                     *this->get_clock(),
			                     1000,
			                     "Transform failed (%s -> %s): %s",
			                     msg->header.frame_id.c_str(),
			                     base_frame_.c_str(),
			                     ex.what());
			return;
		}

		RCLCPP_INFO_THROTTLE(this->get_logger(),
		                     *this->get_clock(),
		                     1000,
		                     "Object pose transformed to %s | xyz: [%.3f, %.3f, %.3f]",
		                     base_frame_.c_str(),
		                     transformed_pose.pose.position.x,
		                     transformed_pose.pose.position.y,
		                     transformed_pose.pose.position.z);

		if (!has_last_transformed_pose_) {
			last_transformed_pose_ = transformed_pose;
			has_last_transformed_pose_ = true;
			stable_detection_count_ = 1;
			has_stable_detection_ = false;

			RCLCPP_INFO(this->get_logger(), "Detection stability progress: 1/%ld", detection_stability_count_);
			return;
		}

		double dist = positionDistance(transformed_pose, last_transformed_pose_);

		if (dist <= detection_stability_dist_) {
			stable_detection_count_++;
			RCLCPP_INFO_THROTTLE(this->get_logger(),
			                     *this->get_clock(),
			                     500,
			                     "Detection stability progress: %ld/%ld (dist=%.4f m)",
			                     stable_detection_count_,
			                     detection_stability_count_,
			                     dist);
		} else {
			stable_detection_count_ = 1;
			has_stable_detection_ = false;
			RCLCPP_WARN(this->get_logger(),
			            "Detection stability reset: dist %.4f m > threshold %.4f m",
			            dist,
			            detection_stability_dist_);
		}

		last_transformed_pose_ = transformed_pose;

		if (!has_stable_detection_ && stable_detection_count_ >= detection_stability_count_) {
			has_stable_detection_ = true;
			stable_object_pose_ = transformed_pose;
			RCLCPP_INFO(this->get_logger(),
			            "Stable detection achieved in %s | xyz: [%.3f, %.3f, %.3f]",
			            base_frame_.c_str(),
			            stable_object_pose_.pose.position.x,
			            stable_object_pose_.pose.position.y,
			            stable_object_pose_.pose.position.z);

			if (!planning_busy_ && !sequence_planned_once_) {
				planning_busy_ = true;
				runPlanningSequence(stable_object_pose_);
				planning_busy_ = false;
			}
		}
	}

	void PickPlaceExecutor::runPlanningSequence(geometry_msgs::msg::PoseStamped const& object_pose_base) {
		if (!arm_move_group_) {
			RCLCPP_WARN(this->get_logger(), "Skipping planning sequence: arm MoveGroup is not initialized.");
			return;
		}

		if (state_ != State::DETECTING && state_ != State::IDLE) {
			RCLCPP_WARN(this->get_logger(), "Planning sequence request ignored: current state is %s", stateToString(state_));
			return;
		}

		setState(State::PICKING);
		bool execute_pick_stages = execute_enabled_;
		bool execute_place_stages = execute_enabled_ && !execute_pick_only_;

		RCLCPP_INFO(this->get_logger(),
		            "Sequence execution flags | pick=%s place=%s",
		            execute_pick_stages ? "true" : "false",
		            execute_place_stages ? "true" : "false");

		geometry_msgs::msg::PoseStamped pre_grasp_pose = computePreGraspPose(object_pose_base);
		if (!planToPoseTarget(pre_grasp_pose, "pre_grasp_plan", execute_pick_stages)) {
			setState(State::IDLE);
			return;
		}

		if (!planCartesianMove(object_pose_base.pose, "cartesian_descent_plan", execute_pick_stages)) {
			setState(State::IDLE);
			return;
		}

		if (!planGripperAction("close", execute_pick_stages)) {
			setState(State::IDLE);
			return;
		}

		setState(State::PLACING);

		if (!planCartesianMove(pre_grasp_pose.pose, "cartesian_retreat_plan", execute_place_stages)) {
			setState(State::IDLE);
			return;
		}

		if (!planToPoseTarget(place_pose_, "place_pose_plan", execute_place_stages)) {
			setState(State::IDLE);
			return;
		}

		if (!planGripperAction("open", execute_place_stages)) {
			setState(State::IDLE);
			return;
		}

		if (!planToNamedTarget(home_named_target_, "home_named_target_plan", execute_place_stages)) {
			setState(State::IDLE);
			return;
		}

		sequence_planned_once_ = true;
		if (execute_pick_stages || execute_place_stages) {
			RCLCPP_INFO(this->get_logger(), "Phase 5 sequence completed");
		} else {
			RCLCPP_INFO(this->get_logger(), "Phase 4 planning sequence completed (plan-only, no execution)");
		}

		resetDetectionStability();
		setState(State::IDLE);
	}

	geometry_msgs::msg::PoseStamped
	PickPlaceExecutor::computePreGraspPose(geometry_msgs::msg::PoseStamped const& object_pose_base) const {
		geometry_msgs::msg::PoseStamped pre_grasp_pose = object_pose_base;
		pre_grasp_pose.header.frame_id = base_frame_;
		pre_grasp_pose.pose.position.z += approach_offset_z_;

		RCLCPP_INFO(this->get_logger(),
		            "Pre-grasp pose computed | xyz: [%.3f, %.3f, %.3f]",
		            pre_grasp_pose.pose.position.x,
		            pre_grasp_pose.pose.position.y,
		            pre_grasp_pose.pose.position.z);

		return pre_grasp_pose;
	}

	bool PickPlaceExecutor::planToPoseTarget(geometry_msgs::msg::PoseStamped const& target_pose,
	                                         std::string const& stage_name,
	                                         bool execute_after_plan) {
		arm_move_group_->setStartStateToCurrentState();
		arm_move_group_->setPoseTarget(target_pose);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = static_cast<bool>(arm_move_group_->plan(plan));
		arm_move_group_->clearPoseTargets();

		if (success) {
			RCLCPP_INFO(this->get_logger(), "Planning success: %s", stage_name.c_str());
		} else {
			RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", stage_name.c_str());
			return false;
		}

		if (execute_after_plan) {
			return executeArmPlan(plan, stage_name);
		}

		return success;
	}

	bool PickPlaceExecutor::planCartesianMove(geometry_msgs::msg::Pose const& target_pose,
	                                          std::string const& stage_name,
	                                          bool execute_after_plan) {
		arm_move_group_->setStartStateToCurrentState();
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(target_pose);

		moveit_msgs::msg::RobotTrajectory trajectory;
		double fraction = arm_move_group_->computeCartesianPath(waypoints, cartesian_step_, 0.0, trajectory);

		if (fraction < min_cartesian_fraction_) {
			RCLCPP_ERROR(this->get_logger(),
			             "Cartesian planning failed: %s | fraction=%.2f < required %.2f",
			             stage_name.c_str(),
			             fraction,
			             min_cartesian_fraction_);
			return false;
		}

		if (fraction >= min_cartesian_fraction_) {
			RCLCPP_INFO(this->get_logger(), "Cartesian planning success: %s | fraction=%.2f", stage_name.c_str(), fraction);
			if (!execute_after_plan) {
				return true;
			}

			moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
			cartesian_plan.trajectory = trajectory;
			return executeArmPlan(cartesian_plan, stage_name);
		}

		return true;
	}

	bool PickPlaceExecutor::planToNamedTarget(std::string const& target_name,
	                                          std::string const& stage_name,
	                                          bool execute_after_plan) {
		arm_move_group_->setStartStateToCurrentState();
		arm_move_group_->setNamedTarget(target_name);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = static_cast<bool>(arm_move_group_->plan(plan));

		if (success) {
			RCLCPP_INFO(this->get_logger(), "Planning success: %s (%s)", stage_name.c_str(), target_name.c_str());
		} else {
			RCLCPP_ERROR(this->get_logger(), "Planning failed: %s (%s)", stage_name.c_str(), target_name.c_str());
			return false;
		}

		if (execute_after_plan) {
			return executeArmPlan(plan, stage_name);
		}

		return success;
	}

	bool PickPlaceExecutor::planGripperAction(std::string const& action_name, bool execute_action) {
		if (!execute_action) {
			RCLCPP_INFO(
			    this->get_logger(), "Gripper action integrated in state machine (plan-only): %s", action_name.c_str());
			return true;
		}

		return executeHandNamedTarget(action_name, "gripper_" + action_name);
	}

	bool PickPlaceExecutor::executeArmPlan(moveit::planning_interface::MoveGroupInterface::Plan const& plan,
	                                       std::string const& stage_name) {
		auto result = arm_move_group_->execute(plan);
		if (result == moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "Execution success: %s", stage_name.c_str());
			return true;
		}

		RCLCPP_ERROR(this->get_logger(), "Execution failed: %s (error_code=%d)", stage_name.c_str(), result.val);
		return false;
	}

	bool PickPlaceExecutor::executeHandNamedTarget(std::string const& target_name, std::string const& stage_name) {
		if (!hand_move_group_) {
			RCLCPP_ERROR(this->get_logger(), "Gripper execution failed: hand MoveGroup is not initialized");
			return false;
		}

		hand_move_group_->setStartStateToCurrentState();
		hand_move_group_->setNamedTarget(target_name);
		moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
		bool planned = static_cast<bool>(hand_move_group_->plan(hand_plan));
		if (!planned) {
			RCLCPP_ERROR(this->get_logger(), "Gripper planning failed: %s (%s)", stage_name.c_str(), target_name.c_str());
			return false;
		}

		auto result = hand_move_group_->execute(hand_plan);
		if (result == moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "Gripper execution success: %s (%s)", stage_name.c_str(), target_name.c_str());
			return true;
		}

		RCLCPP_ERROR(this->get_logger(),
		             "Gripper execution failed: %s (%s), error_code=%d",
		             stage_name.c_str(),
		             target_name.c_str(),
		             result.val);
		return false;
	}

	void PickPlaceExecutor::resetDetectionStability() {
		has_last_transformed_pose_ = false;
		has_stable_detection_ = false;
		stable_detection_count_ = 0;
	}

	void PickPlaceExecutor::setState(State new_state) {
		if (state_ == new_state) {
			return;
		}

		RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", stateToString(state_), stateToString(new_state));
		state_ = new_state;
	}

	char const* PickPlaceExecutor::stateToString(State state) const {
		switch (state) {
			case State::IDLE:
				return "IDLE";
			case State::DETECTING:
				return "DETECTING";
			case State::PICKING:
				return "PICKING";
			case State::PLACING:
				return "PLACING";
			default:
				return "UNKNOWN";
		}
	}

} // namespace pick_place
