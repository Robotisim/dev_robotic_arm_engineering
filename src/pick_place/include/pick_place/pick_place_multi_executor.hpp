#ifndef PICK_PLACE__PICK_PLACE_MULTI_EXECUTOR_HPP_
#define PICK_PLACE__PICK_PLACE_MULTI_EXECUTOR_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace pick_place {

	class PickPlaceMultiExecutor : public rclcpp::Node {
	public:
		explicit PickPlaceMultiExecutor(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

	private:
		enum class State { IDLE, WAITING_FOR_SNAPSHOT, RUNNING, COMPLETED };

		void objectPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
		void rerunServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
		bool executeSequence();
		bool runPlanningSequenceForObject(geometry_msgs::msg::PoseStamped const& object_pose_base,
		                                  geometry_msgs::msg::PoseStamped const& place_pose_base,
		                                  size_t object_index);
		bool loadStaticObstaclesFromSdf();
		bool transformPoseToBase(geometry_msgs::msg::PoseStamped const& input,
		                         geometry_msgs::msg::PoseStamped& output) const;

		geometry_msgs::msg::PoseStamped computePreGraspPose(geometry_msgs::msg::PoseStamped const& object_pose_base) const;
		bool planToPoseTarget(geometry_msgs::msg::PoseStamped const& target_pose,
		                      std::string const& stage_name,
		                      bool execute_after_plan);
		bool planCartesianMove(geometry_msgs::msg::Pose const& target_pose,
		                       std::string const& stage_name,
		                       bool execute_after_plan);
		bool planToNamedTarget(std::string const& target_name, std::string const& stage_name, bool execute_after_plan);
		bool planGripperAction(std::string const& action_name, bool execute_action);
		bool executeArmPlan(moveit::planning_interface::MoveGroupInterface::Plan const& plan,
		                    std::string const& stage_name);
		bool executeHandNamedTarget(std::string const& target_name, std::string const& stage_name);
		void setState(State new_state);
		char const* stateToString(State state) const;

		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_poses_sub_;
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rerun_service_;

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

		std::vector<geometry_msgs::msg::PoseStamped> object_poses_snapshot_;
		std::vector<geometry_msgs::msg::PoseStamped> place_poses_;

		std::string object_pose_topic_;
		std::string base_frame_;
		std::string arm_group_name_;
		std::string end_effector_link_;
		std::string hand_group_name_;
		std::string home_named_target_;

		int64_t max_objects_{3};
		double approach_offset_z_{0.1};
		double cartesian_step_{0.01};
		double min_cartesian_fraction_{0.9};
		double min_allowed_z_{0.0};
		bool enforce_min_z_{true};
		bool use_current_ee_orientation_for_pick_{false};
		bool use_fixed_pick_orientation_{true};
		geometry_msgs::msg::Quaternion fixed_pick_orientation_;
		bool execute_enabled_{false};
		bool execute_pick_only_{false};
		bool continue_on_failure_{true};

		bool publish_static_env_obstacles_{true};
		std::string static_env_sdf_path_;
		double static_env_offset_x_{0.0};
		double static_env_offset_y_{-1.70};
		double static_env_offset_z_{-1.0};
		double static_env_offset_roll_{0.0};
		double static_env_offset_pitch_{0.0};
		double static_env_offset_yaw_{0.0};
		bool table_proxy_enabled_{true};
		double table_proxy_size_x_{1.50};
		double table_proxy_size_y_{0.80};
		double table_proxy_size_z_{0.08};
		double table_proxy_center_z_offset_{1.0};

		bool snapshot_locked_{false};
		bool sequence_completed_once_{false};
		bool planning_busy_{false};
		State state_{State::IDLE};
	};

} // namespace pick_place

#endif // PICK_PLACE__PICK_PLACE_MULTI_EXECUTOR_HPP_
