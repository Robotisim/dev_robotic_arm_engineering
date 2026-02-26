#ifndef PICK_PLACE__PICK_PLACE_EXECUTOR_HPP_
#define PICK_PLACE__PICK_PLACE_EXECUTOR_HPP_

#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace pick_place {

	class PickPlaceExecutor : public rclcpp::Node {
	public:
		explicit PickPlaceExecutor(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

	private:
		enum class State { IDLE, DETECTING, PICKING, PLACING };

		void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
		void runPlanningSequence(geometry_msgs::msg::PoseStamped const& object_pose_base);
		geometry_msgs::msg::PoseStamped computePreGraspPose(geometry_msgs::msg::PoseStamped const& object_pose_base) const;
		bool planToPoseTarget(geometry_msgs::msg::PoseStamped const& target_pose,
		                      std::string const& stage_name,
		                      bool execute_after_plan);
		bool planCartesianMove(geometry_msgs::msg::Pose const& target_pose,
		                       std::string const& stage_name,
		                       bool execute_after_plan);
		bool planToNamedTarget(std::string const& target_name, std::string const& stage_name, bool execute_after_plan);
		void planGripperAction(std::string const& action_name);
		bool executeArmPlan(moveit::planning_interface::MoveGroupInterface::Plan const& plan,
		                    std::string const& stage_name);
		bool executeHandNamedTarget(std::string const& target_name, std::string const& stage_name);
		void resetDetectionStability();
		void setState(State new_state);
		char const* stateToString(State state) const;

		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
		geometry_msgs::msg::PoseStamped::SharedPtr last_object_pose_;
		geometry_msgs::msg::PoseStamped last_transformed_pose_;
		geometry_msgs::msg::PoseStamped stable_object_pose_;
		bool has_last_transformed_pose_{false};
		bool has_stable_detection_{false};
		int64_t stable_detection_count_{0};
		int64_t detection_stability_count_{5};
		double detection_stability_dist_{0.02};
		double approach_offset_z_{0.1};
		double cartesian_step_{0.01};
		double min_cartesian_fraction_{0.9};
		bool execute_enabled_{false};
		bool execute_pick_only_{true};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_;
		geometry_msgs::msg::PoseStamped place_pose_;
		State state_{State::IDLE};
		bool planning_busy_{false};
		bool sequence_planned_once_{false};
		std::string base_frame_;
		std::string arm_group_name_;
		std::string hand_group_name_;
		std::string home_named_target_;
	};

} // namespace pick_place

#endif // PICK_PLACE__PICK_PLACE_EXECUTOR_HPP_
