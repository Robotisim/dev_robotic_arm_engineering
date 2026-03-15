#include "pick_place/pick_place_multi_executor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <regex>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>

namespace pick_place {

	namespace {

		struct StaticObstacleSpec {
			std::string name;
			geometry_msgs::msg::Pose pose;
			geometry_msgs::msg::Vector3 size;
		};

		std::string trim(std::string value) {
			auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
			while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
				value.erase(value.begin());
			}
			while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
				value.pop_back();
			}
			return value;
		}

		bool startsWith(std::string const& value, std::string const& prefix) {
			return value.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), value.begin());
		}

		std::vector<double> parseDoubles(std::string const& text) {
			std::vector<double> values;
			std::istringstream stream(text);
			double v = 0.0;
			while (stream >> v) {
				values.push_back(v);
			}
			return values;
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
			if (has_group("panda_arm_hand")) {
				return "panda_arm_hand";
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

		geometry_msgs::msg::Pose applyEnvOffset(geometry_msgs::msg::Pose const& env_pose,
		                                        double tx,
		                                        double ty,
		                                        double tz,
		                                        double roll,
		                                        double pitch,
		                                        double yaw) {
			tf2::Quaternion q_env(
			    env_pose.orientation.x, env_pose.orientation.y, env_pose.orientation.z, env_pose.orientation.w);
			tf2::Vector3 p_env(env_pose.position.x, env_pose.position.y, env_pose.position.z);

			tf2::Transform t_env(q_env, p_env);
			tf2::Quaternion q_offset;
			q_offset.setRPY(roll, pitch, yaw);
			tf2::Transform t_offset(q_offset, tf2::Vector3(tx, ty, tz));

			tf2::Transform t_base = t_offset * t_env;
			geometry_msgs::msg::Pose out;
			out.position.x = t_base.getOrigin().x();
			out.position.y = t_base.getOrigin().y();
			out.position.z = t_base.getOrigin().z();
			out.orientation = tf2::toMsg(t_base.getRotation());
			return out;
		}

		std::vector<StaticObstacleSpec> parseStaticObstaclesFromSdf(std::string const& sdf_text,
		                                                            bool table_proxy_enabled,
		                                                            double table_proxy_size_x,
		                                                            double table_proxy_size_y,
		                                                            double table_proxy_size_z,
		                                                            double table_proxy_center_z_offset,
		                                                            double static_env_offset_x,
		                                                            double static_env_offset_y,
		                                                            double static_env_offset_z,
		                                                            double static_env_offset_roll,
		                                                            double static_env_offset_pitch,
		                                                            double static_env_offset_yaw) {
			std::vector<StaticObstacleSpec> specs;
			std::unordered_set<std::string> excluded_names = {"target_cube", "target_cylinder", "target_cuboid"};

			std::regex model_re("<model\\s+name='([^']+)'>([\\s\\S]*?)</model>");
			std::regex pose_re("<pose>([^<]+)</pose>");
			std::regex static_re("<static>\\s*true\\s*</static>");
			std::regex box_size_re("<box>\\s*<size>([^<]+)</size>\\s*</box>");

			auto model_begin = std::sregex_iterator(sdf_text.begin(), sdf_text.end(), model_re);
			auto model_end = std::sregex_iterator();

			for (auto it = model_begin; it != model_end; ++it) {
				std::string model_name = (*it)[1].str();
				std::string model_body = (*it)[2].str();
				if (excluded_names.count(model_name) > 0 || startsWith(model_name, "target_")) {
					continue;
				}

				if (!std::regex_search(model_body, static_re)) {
					continue;
				}

				std::smatch pose_match;
				if (!std::regex_search(model_body, pose_match, pose_re)) {
					continue;
				}
				auto pose_values = parseDoubles(trim(pose_match[1].str()));
				if (pose_values.size() < 6) {
					continue;
				}

				std::smatch box_match;
				if (!std::regex_search(model_body, box_match, box_size_re)) {
					continue;
				}
				auto size_values = parseDoubles(trim(box_match[1].str()));
				if (size_values.size() < 3) {
					continue;
				}

				geometry_msgs::msg::Pose env_pose;
				env_pose.position.x = pose_values[0];
				env_pose.position.y = pose_values[1];
				env_pose.position.z = pose_values[2];
				tf2::Quaternion q_env;
				q_env.setRPY(pose_values[3], pose_values[4], pose_values[5]);
				env_pose.orientation = tf2::toMsg(q_env);

				StaticObstacleSpec spec;
				spec.name = model_name;
				spec.pose = applyEnvOffset(env_pose,
				                           static_env_offset_x,
				                           static_env_offset_y,
				                           static_env_offset_z,
				                           static_env_offset_roll,
				                           static_env_offset_pitch,
				                           static_env_offset_yaw);
				spec.size.x = size_values[0];
				spec.size.y = size_values[1];
				spec.size.z = size_values[2];
				specs.push_back(spec);
			}

			if (table_proxy_enabled) {
				std::regex include_re("<include>([\\s\\S]*?)</include>");
				std::regex table_uri_re("<uri>\\s*model://table\\s*</uri>");
				std::regex include_pose_re("<pose>([^<]+)</pose>");
				auto include_begin = std::sregex_iterator(sdf_text.begin(), sdf_text.end(), include_re);
				auto include_end = std::sregex_iterator();

				for (auto it = include_begin; it != include_end; ++it) {
					std::string include_body = (*it)[1].str();
					if (!std::regex_search(include_body, table_uri_re)) {
						continue;
					}

					geometry_msgs::msg::Pose env_pose;
					env_pose.orientation.w = 1.0;

					std::smatch pose_match;
					if (std::regex_search(include_body, pose_match, include_pose_re)) {
						auto pose_values = parseDoubles(trim(pose_match[1].str()));
						if (pose_values.size() >= 6) {
							env_pose.position.x = pose_values[0];
							env_pose.position.y = pose_values[1];
							env_pose.position.z = pose_values[2] + table_proxy_center_z_offset;
							tf2::Quaternion q_table;
							q_table.setRPY(pose_values[3], pose_values[4], pose_values[5]);
							env_pose.orientation = tf2::toMsg(q_table);
						}
					}

					StaticObstacleSpec table_spec;
					table_spec.name = "table_proxy";
					table_spec.pose = applyEnvOffset(env_pose,
					                                 static_env_offset_x,
					                                 static_env_offset_y,
					                                 static_env_offset_z,
					                                 static_env_offset_roll,
					                                 static_env_offset_pitch,
					                                 static_env_offset_yaw);
					table_spec.size.x = table_proxy_size_x;
					table_spec.size.y = table_proxy_size_y;
					table_spec.size.z = table_proxy_size_z;
					specs.push_back(table_spec);
					break;
				}
			}

			return specs;
		}

	} // namespace

	PickPlaceMultiExecutor::PickPlaceMultiExecutor(rclcpp::NodeOptions const& options)
	    : Node("pick_place_multi_executor", options) {
		RCLCPP_INFO(this->get_logger(), "PickPlaceMultiExecutor node initialized");

		this->declare_parameter("pick_place_multi_executor.object_pose_topic", "/segmentation/object_poses");
		this->declare_parameter("pick_place_multi_executor.base_frame", "panda_link0");
		this->declare_parameter("pick_place_multi_executor.arm_group_name", "panda_arm");
		this->declare_parameter("pick_place_multi_executor.end_effector_link", "panda_hand_tcp");
		this->declare_parameter("pick_place_multi_executor.hand_group_name", "hand");
		this->declare_parameter("pick_place_multi_executor.home_named_target", "ready");
		this->declare_parameter("pick_place_multi_executor.max_objects", 3);
		this->declare_parameter("pick_place_multi_executor.approach_offset_z", 0.1);
		this->declare_parameter("pick_place_multi_executor.cartesian_step", 0.01);
		this->declare_parameter("pick_place_multi_executor.min_cartesian_fraction", 0.9);
		this->declare_parameter("pick_place_multi_executor.enforce_min_z", true);
		this->declare_parameter("pick_place_multi_executor.min_allowed_z", 0.0);
		this->declare_parameter("pick_place_multi_executor.use_current_ee_orientation_for_pick", false);
		this->declare_parameter("pick_place_multi_executor.use_fixed_pick_orientation", true);
		this->declare_parameter("pick_place_multi_executor.pick_pose.qx", 1.0);
		this->declare_parameter("pick_place_multi_executor.pick_pose.qy", 0.0);
		this->declare_parameter("pick_place_multi_executor.pick_pose.qz", 0.0);
		this->declare_parameter("pick_place_multi_executor.pick_pose.qw", 0.0);
		this->declare_parameter("pick_place_multi_executor.execute_enabled", false);
		this->declare_parameter("pick_place_multi_executor.execute_pick_only", false);
		this->declare_parameter("pick_place_multi_executor.continue_on_failure", true);

		this->declare_parameter("pick_place_multi_executor.place_poses_x", std::vector<double>{0.35, 0.42, 0.49});
		this->declare_parameter("pick_place_multi_executor.place_poses_y", std::vector<double>{-0.30, -0.30, -0.30});
		this->declare_parameter("pick_place_multi_executor.place_poses_z", std::vector<double>{0.20, 0.20, 0.20});
		this->declare_parameter("pick_place_multi_executor.place_poses_qx", std::vector<double>{1.0, 1.0, 1.0});
		this->declare_parameter("pick_place_multi_executor.place_poses_qy", std::vector<double>{0.0, 0.0, 0.0});
		this->declare_parameter("pick_place_multi_executor.place_poses_qz", std::vector<double>{0.0, 0.0, 0.0});
		this->declare_parameter("pick_place_multi_executor.place_poses_qw", std::vector<double>{0.0, 0.0, 0.0});

		this->declare_parameter("pick_place_multi_executor.publish_static_env_obstacles", true);
		this->declare_parameter("pick_place_multi_executor.static_env_sdf_path",
		                        "src/dev_robotic_arm_engineering/src/panda/worlds/multi_object_sequential_env.sdf");
		this->declare_parameter("pick_place_multi_executor.static_env_offset_x", 0.0);
		this->declare_parameter("pick_place_multi_executor.static_env_offset_y", -1.70);
		this->declare_parameter("pick_place_multi_executor.static_env_offset_z", -1.0);
		this->declare_parameter("pick_place_multi_executor.static_env_offset_roll", 0.0);
		this->declare_parameter("pick_place_multi_executor.static_env_offset_pitch", 0.0);
		this->declare_parameter("pick_place_multi_executor.static_env_offset_yaw", 0.0);
		this->declare_parameter("pick_place_multi_executor.table_proxy_enabled", true);
		this->declare_parameter("pick_place_multi_executor.table_proxy_size_x", 0.80);
		this->declare_parameter("pick_place_multi_executor.table_proxy_size_y", 1.20);
		this->declare_parameter("pick_place_multi_executor.table_proxy_size_z", 0.08);
		this->declare_parameter("pick_place_multi_executor.table_proxy_center_z_offset", 1.0);

		object_pose_topic_ = this->get_parameter("pick_place_multi_executor.object_pose_topic").as_string();
		base_frame_ = this->get_parameter("pick_place_multi_executor.base_frame").as_string();
		arm_group_name_ = this->get_parameter("pick_place_multi_executor.arm_group_name").as_string();
		end_effector_link_ = this->get_parameter("pick_place_multi_executor.end_effector_link").as_string();
		hand_group_name_ = this->get_parameter("pick_place_multi_executor.hand_group_name").as_string();
		home_named_target_ = this->get_parameter("pick_place_multi_executor.home_named_target").as_string();
		max_objects_ = this->get_parameter("pick_place_multi_executor.max_objects").as_int();
		approach_offset_z_ = this->get_parameter("pick_place_multi_executor.approach_offset_z").as_double();
		cartesian_step_ = this->get_parameter("pick_place_multi_executor.cartesian_step").as_double();
		min_cartesian_fraction_ = this->get_parameter("pick_place_multi_executor.min_cartesian_fraction").as_double();
		enforce_min_z_ = this->get_parameter("pick_place_multi_executor.enforce_min_z").as_bool();
		min_allowed_z_ = this->get_parameter("pick_place_multi_executor.min_allowed_z").as_double();
		use_current_ee_orientation_for_pick_ =
		    this->get_parameter("pick_place_multi_executor.use_current_ee_orientation_for_pick").as_bool();
		use_fixed_pick_orientation_ = this->get_parameter("pick_place_multi_executor.use_fixed_pick_orientation").as_bool();
		execute_enabled_ = this->get_parameter("pick_place_multi_executor.execute_enabled").as_bool();
		execute_pick_only_ = this->get_parameter("pick_place_multi_executor.execute_pick_only").as_bool();
		continue_on_failure_ = this->get_parameter("pick_place_multi_executor.continue_on_failure").as_bool();

		fixed_pick_orientation_.x = this->get_parameter("pick_place_multi_executor.pick_pose.qx").as_double();
		fixed_pick_orientation_.y = this->get_parameter("pick_place_multi_executor.pick_pose.qy").as_double();
		fixed_pick_orientation_.z = this->get_parameter("pick_place_multi_executor.pick_pose.qz").as_double();
		fixed_pick_orientation_.w = this->get_parameter("pick_place_multi_executor.pick_pose.qw").as_double();

		publish_static_env_obstacles_ =
		    this->get_parameter("pick_place_multi_executor.publish_static_env_obstacles").as_bool();
		static_env_sdf_path_ = this->get_parameter("pick_place_multi_executor.static_env_sdf_path").as_string();
		static_env_offset_x_ = this->get_parameter("pick_place_multi_executor.static_env_offset_x").as_double();
		static_env_offset_y_ = this->get_parameter("pick_place_multi_executor.static_env_offset_y").as_double();
		static_env_offset_z_ = this->get_parameter("pick_place_multi_executor.static_env_offset_z").as_double();
		static_env_offset_roll_ = this->get_parameter("pick_place_multi_executor.static_env_offset_roll").as_double();
		static_env_offset_pitch_ = this->get_parameter("pick_place_multi_executor.static_env_offset_pitch").as_double();
		static_env_offset_yaw_ = this->get_parameter("pick_place_multi_executor.static_env_offset_yaw").as_double();
		table_proxy_enabled_ = this->get_parameter("pick_place_multi_executor.table_proxy_enabled").as_bool();
		table_proxy_size_x_ = this->get_parameter("pick_place_multi_executor.table_proxy_size_x").as_double();
		table_proxy_size_y_ = this->get_parameter("pick_place_multi_executor.table_proxy_size_y").as_double();
		table_proxy_size_z_ = this->get_parameter("pick_place_multi_executor.table_proxy_size_z").as_double();
		table_proxy_center_z_offset_ =
		    this->get_parameter("pick_place_multi_executor.table_proxy_center_z_offset").as_double();

		auto place_x = this->get_parameter("pick_place_multi_executor.place_poses_x").as_double_array();
		auto place_y = this->get_parameter("pick_place_multi_executor.place_poses_y").as_double_array();
		auto place_z = this->get_parameter("pick_place_multi_executor.place_poses_z").as_double_array();
		auto place_qx = this->get_parameter("pick_place_multi_executor.place_poses_qx").as_double_array();
		auto place_qy = this->get_parameter("pick_place_multi_executor.place_poses_qy").as_double_array();
		auto place_qz = this->get_parameter("pick_place_multi_executor.place_poses_qz").as_double_array();
		auto place_qw = this->get_parameter("pick_place_multi_executor.place_poses_qw").as_double_array();

		size_t place_count = std::min({place_x.size(),
		                               place_y.size(),
		                               place_z.size(),
		                               place_qx.size(),
		                               place_qy.size(),
		                               place_qz.size(),
		                               place_qw.size()});
		for (size_t i = 0; i < place_count; ++i) {
			geometry_msgs::msg::PoseStamped pose;
			pose.header.frame_id = base_frame_;
			pose.pose.position.x = place_x[i];
			pose.pose.position.y = place_y[i];
			pose.pose.position.z = place_z[i];
			pose.pose.orientation.x = place_qx[i];
			pose.pose.orientation.y = place_qy[i];
			pose.pose.orientation.z = place_qz[i];
			pose.pose.orientation.w = place_qw[i];
			place_poses_.push_back(pose);
		}

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
			arm_move_group_->setEndEffectorLink(end_effector_link_);
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

		if (!place_poses_.empty()) {
			RCLCPP_INFO(this->get_logger(), "Configured %zu place targets", place_poses_.size());
		} else {
			RCLCPP_ERROR(this->get_logger(), "No place targets configured. Sequence execution will fail.");
		}

		if (!loadStaticObstaclesFromSdf()) {
			RCLCPP_WARN(this->get_logger(), "Static obstacle load failed. Planning will continue without those collisions.");
		}

		object_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
		    object_pose_topic_, 10, std::bind(&PickPlaceMultiExecutor::objectPosesCallback, this, std::placeholders::_1));

		rerun_service_ = this->create_service<std_srvs::srv::Trigger>(
		    "/pick_place/rerun_multi",
		    std::bind(&PickPlaceMultiExecutor::rerunServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

		RCLCPP_INFO(this->get_logger(), "Subscribed to %s", object_pose_topic_.c_str());
		RCLCPP_INFO(this->get_logger(), "Service ready: /pick_place/rerun_multi");
		RCLCPP_INFO(this->get_logger(), "Object order strategy: PoseArray order as published");
		RCLCPP_INFO(
		    this->get_logger(), "Failure strategy: continue_on_failure=%s", continue_on_failure_ ? "true" : "false");

		setState(State::WAITING_FOR_SNAPSHOT);
	}

	bool PickPlaceMultiExecutor::loadStaticObstaclesFromSdf() {
		if (!publish_static_env_obstacles_) {
			RCLCPP_INFO(this->get_logger(), "Static environment obstacles disabled by parameter");
			return true;
		}

		std::ifstream input(static_env_sdf_path_);
		if (!input.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "Cannot open SDF file: %s", static_env_sdf_path_.c_str());
			return false;
		}

		std::string sdf_text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
		if (sdf_text.empty()) {
			RCLCPP_ERROR(this->get_logger(), "SDF file is empty: %s", static_env_sdf_path_.c_str());
			return false;
		}

		auto specs = parseStaticObstaclesFromSdf(sdf_text,
		                                         table_proxy_enabled_,
		                                         table_proxy_size_x_,
		                                         table_proxy_size_y_,
		                                         table_proxy_size_z_,
		                                         table_proxy_center_z_offset_,
		                                         static_env_offset_x_,
		                                         static_env_offset_y_,
		                                         static_env_offset_z_,
		                                         static_env_offset_roll_,
		                                         static_env_offset_pitch_,
		                                         static_env_offset_yaw_);
		if (specs.empty()) {
			RCLCPP_WARN(this->get_logger(), "No static obstacles parsed from SDF: %s", static_env_sdf_path_.c_str());
			return false;
		}

		std::vector<std::string> ids;
		std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
		collision_objects.reserve(specs.size());

		for (auto const& spec : specs) {
			moveit_msgs::msg::CollisionObject object;
			object.id = "env_" + spec.name;
			object.header.frame_id = base_frame_;

			shape_msgs::msg::SolidPrimitive primitive;
			primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
			primitive.dimensions = {spec.size.x, spec.size.y, spec.size.z};

			object.primitives.push_back(primitive);
			object.primitive_poses.push_back(spec.pose);
			object.operation = moveit_msgs::msg::CollisionObject::ADD;

			ids.push_back(object.id);
			collision_objects.push_back(object);
		}

		planning_scene_interface_.removeCollisionObjects(ids);
		planning_scene_interface_.applyCollisionObjects(collision_objects);

		RCLCPP_INFO(this->get_logger(),
		            "Loaded %zu static obstacles from %s (pick targets excluded)",
		            collision_objects.size(),
		            static_env_sdf_path_.c_str());
		return true;
	}

	bool PickPlaceMultiExecutor::transformPoseToBase(geometry_msgs::msg::PoseStamped const& input,
	                                                 geometry_msgs::msg::PoseStamped& output) const {
		if (input.header.frame_id == base_frame_ || input.header.frame_id.empty()) {
			output = input;
			output.header.frame_id = base_frame_;
			return true;
		}

		try {
			output = tf_buffer_->transform(input, base_frame_, tf2::durationFromSec(0.2));
			return true;
		} catch (tf2::TransformException const& ex) {
			RCLCPP_WARN(this->get_logger(),
			            "Pose transform failed (%s -> %s): %s",
			            input.header.frame_id.c_str(),
			            base_frame_.c_str(),
			            ex.what());
			return false;
		}
	}

	void PickPlaceMultiExecutor::objectPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
		if (snapshot_locked_ || planning_busy_) {
			return;
		}
		if (msg->poses.empty()) {
			RCLCPP_WARN_THROTTLE(
			    this->get_logger(), *this->get_clock(), 2000, "PoseArray is empty; waiting for object poses");
			return;
		}
		if (place_poses_.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Cannot capture snapshot because no place targets are configured");
			return;
		}

		size_t capture_limit = static_cast<size_t>(std::max<int64_t>(1, max_objects_));
		capture_limit = std::min(capture_limit, msg->poses.size());
		object_poses_snapshot_.clear();
		object_poses_snapshot_.reserve(capture_limit);

		for (size_t i = 0; i < capture_limit; ++i) {
			geometry_msgs::msg::PoseStamped pose;
			pose.header = msg->header;
			pose.pose = msg->poses[i];

			geometry_msgs::msg::PoseStamped pose_base;
			if (!transformPoseToBase(pose, pose_base)) {
				continue;
			}
			object_poses_snapshot_.push_back(pose_base);
		}

		if (object_poses_snapshot_.empty()) {
			RCLCPP_WARN(this->get_logger(), "No valid transformed poses captured from PoseArray");
			return;
		}

		if (place_poses_.size() < object_poses_snapshot_.size()) {
			RCLCPP_ERROR(this->get_logger(),
			             "Insufficient place targets (%zu) for captured objects (%zu)",
			             place_poses_.size(),
			             object_poses_snapshot_.size());
			return;
		}

		snapshot_locked_ = true;
		setState(State::RUNNING);
		RCLCPP_INFO(this->get_logger(),
		            "Captured one-time snapshot: %zu objects in %s. Starting sequence.",
		            object_poses_snapshot_.size(),
		            base_frame_.c_str());

		planning_busy_ = true;
		(void)executeSequence();
		planning_busy_ = false;
	}

	bool PickPlaceMultiExecutor::executeSequence() {
		if (!arm_move_group_) {
			RCLCPP_ERROR(this->get_logger(), "Cannot execute sequence: arm MoveGroup is not initialized");
			setState(State::IDLE);
			return false;
		}
		if (object_poses_snapshot_.empty()) {
			RCLCPP_WARN(this->get_logger(), "No snapshot available for execution");
			setState(State::WAITING_FOR_SNAPSHOT);
			return false;
		}

		size_t success_count = 0;
		for (size_t i = 0; i < object_poses_snapshot_.size(); ++i) {
			auto const& object_pose = object_poses_snapshot_[i];
			auto const& place_pose = place_poses_[i];
			RCLCPP_INFO(this->get_logger(), "Executing object %zu/%zu", i + 1, object_poses_snapshot_.size());

			bool ok = runPlanningSequenceForObject(object_pose, place_pose, i);
			if (ok) {
				success_count++;
				continue;
			}

			RCLCPP_ERROR(this->get_logger(), "Object %zu failed", i + 1);
			(void)planGripperAction("open", execute_enabled_);
			(void)planToNamedTarget(home_named_target_, "recover_home", execute_enabled_);
			if (!continue_on_failure_) {
				break;
			}
		}

		sequence_completed_once_ = true;
		setState(State::COMPLETED);
		RCLCPP_INFO(this->get_logger(),
		            "Sequence finished: %zu/%zu objects succeeded",
		            success_count,
		            object_poses_snapshot_.size());
		setState(State::IDLE);
		return success_count > 0;
	}

	bool PickPlaceMultiExecutor::runPlanningSequenceForObject(geometry_msgs::msg::PoseStamped const& object_pose_base,
	                                                          geometry_msgs::msg::PoseStamped const& place_pose_base,
	                                                          size_t object_index) {
		bool execute_pick_stages = execute_enabled_;
		bool execute_place_stages = execute_enabled_ && !execute_pick_only_;

		std::string index_suffix = "obj_" + std::to_string(object_index + 1);

		if (!planGripperAction("open", execute_pick_stages)) {
			return false;
		}

		geometry_msgs::msg::PoseStamped pick_pose = object_pose_base;
		pick_pose.header.frame_id = base_frame_;
		if (use_fixed_pick_orientation_) {
			pick_pose.pose.orientation = fixed_pick_orientation_;
		} else if (use_current_ee_orientation_for_pick_) {
			auto current_pose = arm_move_group_->getCurrentPose(end_effector_link_);
			if (!current_pose.header.frame_id.empty()) {
				pick_pose.pose.orientation = current_pose.pose.orientation;
			}
		}

		// Keep tool Z-axis pointing downward in base frame.
		{
			tf2::Quaternion q_target(pick_pose.pose.orientation.x,
			                         pick_pose.pose.orientation.y,
			                         pick_pose.pose.orientation.z,
			                         pick_pose.pose.orientation.w);
			q_target.normalize();
			tf2::Matrix3x3 m_target(q_target);
			tf2::Vector3 z_axis_target = m_target.getColumn(2);
			if (z_axis_target.z() > 0.0) {
				tf2::Quaternion flip_x(1.0, 0.0, 0.0, 0.0);
				tf2::Quaternion q_flipped = q_target * flip_x;
				q_flipped.normalize();
				pick_pose.pose.orientation = tf2::toMsg(q_flipped);
			}
		}

		if (enforce_min_z_ && pick_pose.pose.position.z < min_allowed_z_) {
			RCLCPP_ERROR(this->get_logger(),
			             "[%s] Pick pose rejected: z=%.3f below min_allowed_z=%.3f",
			             index_suffix.c_str(),
			             pick_pose.pose.position.z,
			             min_allowed_z_);
			return false;
		}

		auto pre_grasp_pose = computePreGraspPose(pick_pose);
		if (enforce_min_z_ && pre_grasp_pose.pose.position.z < min_allowed_z_) {
			RCLCPP_ERROR(this->get_logger(),
			             "[%s] Pre-grasp pose rejected: z=%.3f below min_allowed_z=%.3f",
			             index_suffix.c_str(),
			             pre_grasp_pose.pose.position.z,
			             min_allowed_z_);
			return false;
		}

		if (!planToPoseTarget(pre_grasp_pose, "pre_grasp_" + index_suffix, execute_pick_stages)) {
			return false;
		}
		if (!planCartesianMove(pick_pose.pose, "descend_" + index_suffix, execute_pick_stages)) {
			return false;
		}
		if (!planGripperAction("close", execute_pick_stages)) {
			return false;
		}
		if (!planCartesianMove(pre_grasp_pose.pose, "retreat_" + index_suffix, execute_place_stages)) {
			return false;
		}
		if (!planToPoseTarget(place_pose_base, "place_pose_" + index_suffix, execute_place_stages)) {
			return false;
		}
		if (!planGripperAction("open", execute_place_stages)) {
			return false;
		}
		if (!planToNamedTarget(home_named_target_, "home_" + index_suffix, execute_place_stages)) {
			return false;
		}

		return true;
	}

	geometry_msgs::msg::PoseStamped
	PickPlaceMultiExecutor::computePreGraspPose(geometry_msgs::msg::PoseStamped const& object_pose_base) const {
		geometry_msgs::msg::PoseStamped pre_grasp_pose = object_pose_base;
		pre_grasp_pose.header.frame_id = base_frame_;
		pre_grasp_pose.pose.position.z += approach_offset_z_;
		return pre_grasp_pose;
	}

	bool PickPlaceMultiExecutor::planToPoseTarget(geometry_msgs::msg::PoseStamped const& target_pose,
	                                              std::string const& stage_name,
	                                              bool execute_after_plan) {
		if (enforce_min_z_ && target_pose.pose.position.z < min_allowed_z_) {
			RCLCPP_ERROR(this->get_logger(),
			             "[%s] Target pose rejected: z=%.3f below min_allowed_z=%.3f",
			             stage_name.c_str(),
			             target_pose.pose.position.z,
			             min_allowed_z_);
			return false;
		}

		arm_move_group_->setStartStateToCurrentState();
		arm_move_group_->setPoseTarget(target_pose, end_effector_link_);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = static_cast<bool>(arm_move_group_->plan(plan));
		arm_move_group_->clearPoseTargets();

		if (!success) {
			RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", stage_name.c_str());
			return false;
		}

		if (execute_after_plan) {
			return executeArmPlan(plan, stage_name);
		}
		return true;
	}

	bool PickPlaceMultiExecutor::planCartesianMove(geometry_msgs::msg::Pose const& target_pose,
	                                               std::string const& stage_name,
	                                               bool execute_after_plan) {
		if (enforce_min_z_ && target_pose.position.z < min_allowed_z_) {
			RCLCPP_ERROR(this->get_logger(),
			             "[%s] Cartesian target rejected: z=%.3f below min_allowed_z=%.3f",
			             stage_name.c_str(),
			             target_pose.position.z,
			             min_allowed_z_);
			return false;
		}

		arm_move_group_->setStartStateToCurrentState();
		std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
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

		if (!execute_after_plan) {
			return true;
		}

		moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
		cartesian_plan.trajectory = trajectory;
		return executeArmPlan(cartesian_plan, stage_name);
	}

	bool PickPlaceMultiExecutor::planToNamedTarget(std::string const& target_name,
	                                               std::string const& stage_name,
	                                               bool execute_after_plan) {
		arm_move_group_->setStartStateToCurrentState();
		arm_move_group_->setNamedTarget(target_name);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = static_cast<bool>(arm_move_group_->plan(plan));
		if (!success) {
			RCLCPP_ERROR(this->get_logger(), "Planning failed: %s (%s)", stage_name.c_str(), target_name.c_str());
			return false;
		}

		if (execute_after_plan) {
			return executeArmPlan(plan, stage_name);
		}
		return true;
	}

	bool PickPlaceMultiExecutor::planGripperAction(std::string const& action_name, bool execute_action) {
		if (!execute_action) {
			RCLCPP_INFO(this->get_logger(), "Gripper action (plan-only): %s", action_name.c_str());
			return true;
		}
		return executeHandNamedTarget(action_name, "gripper_" + action_name);
	}

	bool PickPlaceMultiExecutor::executeArmPlan(moveit::planning_interface::MoveGroupInterface::Plan const& plan,
	                                            std::string const& stage_name) {
		auto result = arm_move_group_->execute(plan);
		if (result == moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "Execution success: %s", stage_name.c_str());
			return true;
		}
		RCLCPP_ERROR(this->get_logger(), "Execution failed: %s (error_code=%d)", stage_name.c_str(), result.val);
		return false;
	}

	bool PickPlaceMultiExecutor::executeHandNamedTarget(std::string const& target_name, std::string const& stage_name) {
		if (!hand_move_group_) {
			RCLCPP_ERROR(this->get_logger(), "Gripper execution failed: hand MoveGroup is not initialized");
			return false;
		}

		hand_move_group_->setStartStateToCurrentState();
		hand_move_group_->setNamedTarget(target_name);
		moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
		bool planned = static_cast<bool>(hand_move_group_->plan(hand_plan));
		if (!planned) {
			RCLCPP_WARN(this->get_logger(),
			            "Gripper named-target planning failed: %s (%s). Trying direct joint fallback.",
			            stage_name.c_str(),
			            target_name.c_str());

			auto const active_joints = hand_move_group_->getActiveJoints();
			if (active_joints.empty()) {
				RCLCPP_ERROR(this->get_logger(), "Gripper fallback failed: no active joints in hand group");
				return false;
			}

			double finger_target = 0.04;
			if (target_name == "close") {
				finger_target = 0.0;
			} else if (target_name == "open") {
				finger_target = 0.04;
			}

			std::map<std::string, double> joint_targets;
			for (auto const& joint_name : active_joints) {
				joint_targets[joint_name] = finger_target;
			}

			bool set_target_ok = hand_move_group_->setJointValueTarget(joint_targets);
			if (!set_target_ok) {
				RCLCPP_ERROR(this->get_logger(), "Gripper fallback failed: setJointValueTarget returned false");
				return false;
			}

			planned = static_cast<bool>(hand_move_group_->plan(hand_plan));
			if (!planned) {
				RCLCPP_ERROR(this->get_logger(),
				             "Gripper planning failed after fallback: %s (%s)",
				             stage_name.c_str(),
				             target_name.c_str());
				return false;
			}
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

	void PickPlaceMultiExecutor::rerunServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
		(void)request;
		if (planning_busy_) {
			response->success = false;
			response->message = "Planning already in progress";
			return;
		}
		if (object_poses_snapshot_.empty()) {
			response->success = false;
			response->message = "No captured snapshot available yet";
			return;
		}

		setState(State::RUNNING);
		planning_busy_ = true;
		bool ok = executeSequence();
		planning_busy_ = false;

		response->success = ok;
		response->message = ok ? "Rerun accepted and sequence executed" : "Rerun executed with no successful objects";
	}

	void PickPlaceMultiExecutor::setState(State new_state) {
		if (state_ == new_state) {
			return;
		}
		RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", stateToString(state_), stateToString(new_state));
		state_ = new_state;
	}

	char const* PickPlaceMultiExecutor::stateToString(State state) const {
		switch (state) {
			case State::IDLE:
				return "IDLE";
			case State::WAITING_FOR_SNAPSHOT:
				return "WAITING_FOR_SNAPSHOT";
			case State::RUNNING:
				return "RUNNING";
			case State::COMPLETED:
				return "COMPLETED";
			default:
				return "UNKNOWN";
		}
	}

} // namespace pick_place
