#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/action/task.hpp>
#include <arm_course_interfaces/msg/objects.hpp>
#include <filesystem>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <set>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <yaml-cpp/yaml.h>

using namespace arm_course;
using Task = arm_course_interfaces::action::Task;
using Handle = rclcpp_action::ServerGoalHandle<Task>;
namespace mpi = moveit::planning_interface;

class MotionServer {
  rclcpp::Node::SharedPtr n_;
  mpi::MoveGroupInterface arm_;
  mpi::PlanningSceneInterface scene_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_reader_;
  JointReader reader_;
  std::string robot_, base_, tool_, gripper_, gripper_action_, store_;
  bool allow_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> policy_owner_{false};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ownership_;
  std::thread worker_;
  std::mutex truth_mutex_;
  arm_course_interfaces::msg::Objects truth_;
  rclcpp::Subscription<arm_course_interfaces::msg::Objects>::SharedPtr
      truth_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp_action::Server<Task>::SharedPtr server_;
  std::shared_ptr<Handle> active_;
  moveit::core::RobotStatePtr preview_state_;
  void start_state() {
    if (preview_state_)
      arm_.setStartState(*preview_state_);
    else
      arm_.setStartStateToCurrentState();
  }
  void check() {
    if (!rclcpp::ok() || (active_ && active_->is_canceling()))
      throw std::runtime_error("Task cancelled");
  }
  void stage(const std::string &name) {
    check();
    std_msgs::msg::String m;
    m.data = name;
    stage_pub_->publish(m);
    if (active_) {
      auto f = std::make_shared<Task::Feedback>();
      f->stage = name;
      active_->publish_feedback(f);
    }
    RCLCPP_INFO(n_->get_logger(), "stage=%s", name.c_str());
  }
  void validate(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    const auto &tr = trajectory.joint_trajectory;
    if (tr.points.empty())
      throw std::runtime_error("Planner returned an empty trajectory");
    auto model = arm_.getRobotModel();
    double previous = -1;
    for (const auto &p : tr.points) {
      if (p.positions.size() != tr.joint_names.size())
        throw std::runtime_error("Malformed trajectory");
      double t = rclcpp::Duration(p.time_from_start).seconds();
      if (t <= previous)
        throw std::runtime_error("Nonmonotonic trajectory time");
      previous = t;
      for (size_t i = 0; i < tr.joint_names.size(); ++i) {
        const auto &b = model->getVariableBounds(tr.joint_names[i]);
        double q = p.positions[i];
        if (!std::isfinite(q) ||
            (b.position_bounded_ &&
             (q < b.min_position_ - 1e-8 || q > b.max_position_ + 1e-8)))
          throw std::runtime_error("Planned point exceeds URDF limits");
        if (i < p.velocities.size() &&
            (!std::isfinite(p.velocities[i]) ||
             std::abs(p.velocities[i]) > b.max_velocity_ + 1e-5))
          throw std::runtime_error("Planned velocity exceeds model limit");
      }
    }
  }
  void run_plan(mpi::MoveGroupInterface::Plan &plan, bool execute) {
    validate(plan.trajectory);
    check();
    if (!execute && preview_state_) {
      const auto &tr = plan.trajectory.joint_trajectory;
      preview_state_->setVariablePositions(tr.joint_names,
                                           tr.points.back().positions);
      preview_state_->update();
    }
    if (execute) {
      auto future =
          std::async(std::launch::async, [&]() { return arm_.execute(plan); });
      auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(180);
      while (future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        if (!rclcpp::ok() || active_->is_canceling() ||
            std::chrono::steady_clock::now() > deadline) {
          arm_.stop();
          future.wait();
          throw std::runtime_error("Execution cancelled or timed out");
        }
      }
      if (future.get() != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("MoveIt execution failed");
    }
  }
  void plan(bool execute) {
    mpi::MoveGroupInterface::Plan p;
    if (arm_.plan(p) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Motion planning failed");
    run_plan(p, execute);
  }
  void named(const std::string &name, bool execute) {
    stage("named:" + name);
    start_state();
    arm_.clearPoseTargets();
    if (!arm_.setNamedTarget(name))
      throw std::runtime_error("Unknown named pose: " + name);
    plan(execute);
  }
  void joints(const std::vector<double> &q, bool execute) {
    stage("joint_goal");
    start_state();
    arm_.clearPoseTargets();
    if (q.size() != arm_.getJointNames().size() ||
        !std::all_of(q.begin(), q.end(),
                     [](double v) { return std::isfinite(v); }) ||
        !arm_.setJointValueTarget(q))
      throw std::runtime_error("Joint goal is invalid or outside hard limits");
    plan(execute);
  }
  void pose(geometry_msgs::msg::PoseStamped p, bool execute,
            bool cartesian = false) {
    stage(cartesian ? "cartesian_segment" : "pose_goal");
    if (p.header.frame_id != base_)
      throw std::runtime_error("Goal frame must be " + base_);
    if (!std::isfinite(p.pose.position.x) ||
        !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z))
      throw std::runtime_error("Non-finite pose");
    start_state();
    arm_.clearPoseTargets();
    if (cartesian) {
      moveit_msgs::msg::RobotTrajectory tr;
      double fraction = arm_.computeCartesianPath({p.pose}, 0.005, tr, true);
      if (fraction < 0.995)
        throw std::runtime_error("Cartesian path incomplete: " +
                                 std::to_string(fraction));
      mpi::MoveGroupInterface::Plan plan;
      plan.trajectory = tr;
      run_plan(plan, execute);
    } else {
      if (robot_ == "roboflex")
        arm_.setPositionTarget(p.pose.position.x, p.pose.position.y,
                               p.pose.position.z, tool_);
      else {
        double norm = std::sqrt(p.pose.orientation.x * p.pose.orientation.x +
                                p.pose.orientation.y * p.pose.orientation.y +
                                p.pose.orientation.z * p.pose.orientation.z +
                                p.pose.orientation.w * p.pose.orientation.w);
        if (!std::isfinite(norm) || norm < 0.99 || norm > 1.01)
          throw std::runtime_error(
              "Panda orientation must be a unit quaternion");
        arm_.setPoseTarget(p, tool_);
      }
      plan(execute);
    }
  }
  void gripper(double value, bool execute) {
    stage("gripper:" + std::to_string(value));
    const auto &bounds = arm_.getRobotModel()->getVariableBounds(gripper_);
    if (!std::isfinite(value) || value < bounds.min_position_ ||
        value > bounds.max_position_)
      throw std::runtime_error("Gripper target exceeds URDF hard limits");
    if (robot_ == "panda")
      value = std::clamp(value, 0.001, 0.038);
    if (!execute) {
      if (preview_state_) {
        preview_state_->setVariablePosition(gripper_, value);
        preview_state_->update();
      }
      return;
    }
    auto start = reader_.get({gripper_});
    trajectory_msgs::msg::JointTrajectory tr;
    tr.joint_names = {gripper_};
    trajectory_msgs::msg::JointTrajectoryPoint first, last;
    first.positions = {start[0]};
    first.time_from_start = rclcpp::Duration::from_seconds(0);
    last.positions = {value};
    last.time_from_start = rclcpp::Duration::from_seconds(
        std::max(1.0, std::abs(value - start[0]) / 0.04));
    tr.points = {first, last};
    execute_trajectory(n_, gripper_action_, tr,
                       [this]() { return active_->is_canceling(); });
  }
  void collision_pairs(const std::string &id,
                       const std::vector<std::string> &others, bool allow) {
    if (!scene_reader_->wait_for_service(std::chrono::seconds(3)))
      throw std::runtime_error("Planning scene service unavailable");
    auto request =
        std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = scene_reader_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      scene_reader_->remove_pending_request(future);
      throw std::runtime_error("Cannot read existing collision rules");
    }
    collision_detection::AllowedCollisionMatrix matrix(
        future.get()->scene.allowed_collision_matrix);
    for (const auto &other : others)
      matrix.setEntry(id, other, allow);
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    matrix.getMessage(diff.allowed_collision_matrix);
    if (!scene_.applyPlanningScene(diff))
      throw std::runtime_error("Could not update target contact rules");
  }
  void contact(const std::string &id, bool allow) {
    collision_pairs(id, {"panda_leftfinger", "panda_rightfinger"}, allow);
  }
  geometry_msgs::msg::Pose object_pose(const std::string &id) {
    std::lock_guard<std::mutex> lock(truth_mutex_);
    if ((n_->now() - rclcpp::Time(truth_.header.stamp)).seconds() > 1)
      throw std::runtime_error("Simulator outcome stream is stale");
    for (const auto &o : truth_.objects)
      if (o.id == id)
        return o.pose;
    throw std::runtime_error("Object absent from simulator evaluator: " + id);
  }
  void pick(const Task::Goal &g) {
    if (robot_ != "panda")
      throw std::runtime_error(
          "Automatic perception/grasp projects are simulation-only; use a "
          "real-robot joint recipe in Module 3");
    if (g.object_id.empty() || !std::isfinite(g.object_size) ||
        !std::isfinite(g.grasp_yaw) || g.object_size < 0.01 ||
        g.object_size > 0.065)
      throw std::runtime_error("Invalid grasp object size/ID");
    if (g.pose.header.frame_id != base_ ||
        g.destination.header.frame_id != base_)
      throw std::runtime_error("Pick and destination must use " + base_);
    for (const auto &p : {g.pose.pose.position, g.destination.pose.position})
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::runtime_error("Non-finite pick/destination");
    auto grasp = g.pose;
    grasp.header.frame_id = base_;
    Eigen::Quaterniond q(
        Eigen::AngleAxisd(g.grasp_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    grasp.pose.orientation.x = q.x();
    grasp.pose.orientation.y = q.y();
    grasp.pose.orientation.z = q.z();
    grasp.pose.orientation.w = q.w();
    auto pre = grasp;
    pre.pose.position.z += 0.12;
    moveit_msgs::msg::CollisionObject object;
    object.id = g.object_id;
    object.header.frame_id = base_;
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {g.object_size, g.object_size, g.object_size};
    object.primitives.push_back(box);
    auto op = g.pose.pose;
    op.orientation.w = 1;
    object.primitive_poses.push_back(op);
    if (!scene_.applyCollisionObject(object))
      throw std::runtime_error("Could not add grasp object");
    bool attached = false, contact_allowed = false;
    std::vector<std::string> supports;
    try {
      gripper(0.04, g.execute);
      pose(pre, g.execute);
      if (!g.execute) {
        stage("pick_preview_pregrasp_only");
        return;
      }
      contact(g.object_id, true);
      contact_allowed = true;
      pose(grasp, true, true);
      const double jaw_width =
          g.object_size *
          (std::abs(std::cos(g.grasp_yaw)) + std::abs(std::sin(g.grasp_yaw)));
      if (jaw_width > 0.072)
        throw std::runtime_error(
            "Selected grasp yaw exceeds usable jaw opening");
      gripper(std::max(0.001, jaw_width / 2 - 0.005), true);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      arm_.attachObject(g.object_id, tool_,
                        {"panda_leftfinger", "panda_rightfinger"});
      attached = true;
      pose(pre, true, true);
      stage("verify_lift");
      auto lifted = object_pose(g.object_id);
      if (lifted.position.z < g.pose.pose.position.z + 0.04)
        throw std::runtime_error("Physics grasp failed: object did not lift; "
                                 "scene attachment is not a grasp");
      auto place = g.destination;
      if (place.header.frame_id != base_)
        throw std::runtime_error("Destination must be in " + base_);
      place.pose.orientation = grasp.pose.orientation;
      auto above = place;
      above.pose.position.z += 0.12;
      pose(above, true);
      // Permit only the placed object to touch its support in the final
      // descent. Preserve all robot and unrelated-object collision rules.
      for (const auto &[id, item] :
           scene_.getObjectPoses(scene_.getKnownObjectNames())) {
        if (id == g.object_id)
          continue;
        const auto &p = item.position;
        if (id == "workbench" ||
            (std::hypot(p.x - place.pose.position.x,
                        p.y - place.pose.position.y) < 0.035 &&
             p.z < place.pose.position.z))
          supports.push_back(id);
      }
      collision_pairs(g.object_id, supports, true);
      pose(place, true, true);
      gripper(0.04, true);
      arm_.detachObject(g.object_id);
      attached = false;
      contact(g.object_id, false);
      contact_allowed = false;
      pose(above, true, true);
      collision_pairs(g.object_id, supports, false);
      supports.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      auto actual = object_pose(g.object_id);
      double e = std::hypot(actual.position.x - place.pose.position.x,
                            actual.position.y - place.pose.position.y);
      if (e > 0.05 ||
          std::abs(actual.position.z - place.pose.position.z) > 0.025)
        throw std::runtime_error(
            "Placement outside 5 cm XY / 2.5 cm Z tolerance");
      stage("placement_verified");
    } catch (...) {
      if (attached)
        arm_.detachObject(g.object_id);
      if (contact_allowed)
        contact(g.object_id, false);
      if (!supports.empty())
        collision_pairs(g.object_id, supports, false);
      throw;
    }
  }
  void recipe(const std::string &path, bool execute) {
    auto yaml = YAML::LoadFile(path);
    if (yaml["robot"].as<std::string>() != robot_)
      throw std::runtime_error("Recipe robot does not match server");
    if (!yaml["steps"].IsSequence() || yaml["steps"].size() > 100)
      throw std::runtime_error("Recipe requires at most 100 steps");
    if (!execute) {
      preview_state_ = arm_.getCurrentState(5);
      if (!preview_state_)
        throw std::runtime_error("No current state for sequential preview");
    }
    for (const auto &s : yaml["steps"]) {
      check();
      if (s["named"])
        named(s["named"].as<std::string>(), execute);
      else if (s["joints"])
        joints(s["joints"].as<std::vector<double>>(), execute);
      else if (s["saved_pose"])
        load_pose(s["saved_pose"].as<std::string>(), execute);
      else if (s["gripper"])
        gripper(s["gripper"].as<double>(), execute);
      else if (s["wait"]) {
        double wait = s["wait"].as<double>();
        if (!std::isfinite(wait) || wait < 0 || wait > 10)
          throw std::runtime_error("Wait must be 0..10 seconds");
        auto end = std::chrono::steady_clock::now() +
                   std::chrono::duration<double>(wait);
        while (execute && std::chrono::steady_clock::now() < end) {
          check();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      } else
        throw std::runtime_error("Unknown recipe step");
    }
    preview_state_.reset();
  }
  void load_pose(const std::string &name, bool execute) {
    auto store = YAML::LoadFile(store_);
    if (store["robot"].as<std::string>() != robot_ ||
        store["joint_names"].as<std::vector<std::string>>() !=
            arm_.getJointNames())
      throw std::runtime_error("Pose store robot/joint order mismatch");
    const auto poses = store["poses"];
    if (!poses.IsMap() || !poses[name])
      throw std::runtime_error("Unknown saved pose: " + name);
    if (!poses[name].IsSequence())
      throw std::runtime_error("Saved pose must contain a joint-angle list: " + name);
    joints(poses[name].as<std::vector<double>>(), execute);
  }
  void run(std::shared_ptr<Handle> h) {
    active_ = h;
    auto result = std::make_shared<Task::Result>();
    auto start = std::chrono::steady_clock::now();
    try {
      const auto &g = *h->get_goal();
      if (g.execute && !allow_)
        throw std::runtime_error("Server execution disabled; start with "
                                 "allow_execution:=true for deliberate motion");
      if (g.operation == "named")
        named(g.target, g.execute);
      else if (g.operation == "joints")
        joints(g.joints, g.execute);
      else if (g.operation == "pose" || g.operation == "cartesian")
        pose(g.pose, g.execute, g.operation == "cartesian");
      else if (g.operation == "gripper") {
        if (g.joints.size() != 1)
          throw std::runtime_error("Gripper needs one value");
        gripper(g.joints[0], g.execute);
      } else if (g.operation == "recipe")
        recipe(g.target, g.execute);
      else if (g.operation == "save") {
        stage("save_pose");
        auto q = reader_.get(arm_.getJointNames());
        YAML::Node store;
        if (std::filesystem::exists(store_))
          store = YAML::LoadFile(store_);
        store["robot"] = robot_;
        store["joint_names"] = arm_.getJointNames();
        std::vector<double> v(q.data(), q.data() + q.size());
        store["poses"][g.target] = v;
        std::ofstream f(store_);
        if (!f)
          throw std::runtime_error("Cannot write pose store");
        f << store;
      } else if (g.operation == "load")
        load_pose(g.target, g.execute);
      else if (g.operation == "pick")
        pick(g);
      else
        throw std::runtime_error("Unsupported operation");
      result->duration = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
      result->success = true;
      result->detail =
          g.execute ? "Command sequence complete; hardware joint arrival is "
                      "unmeasured"
                    : "Preview/data operation complete; no motor command";
      if (g.operation == "pick" && g.execute)
        result->detail = "Gazebo object lift and placement verified";
      if (h->is_canceling()) {
        result->success = false;
        h->canceled(result);
      } else
        h->succeed(result);
    } catch (const std::exception &e) {
      result->detail = e.what();
      result->duration = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
      if (h->is_canceling())
        h->canceled(result);
      else
        h->abort(result);
      RCLCPP_ERROR(n_->get_logger(), "%s", e.what());
    }
    preview_state_.reset();
    result->duration =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    std_msgs::msg::String idle;
    idle.data = "idle";
    stage_pub_->publish(idle);
    active_.reset();
    busy_ = false;
  }

public:
  explicit MotionServer(const rclcpp::Node::SharedPtr &n)
      : n_(n), arm_(n, parameter<std::string>(n, "arm_group", "arm")),
        reader_(n) {
    robot_ = parameter<std::string>(n, "robot", "roboflex");
    base_ = parameter<std::string>(n, "base_frame", "base_link");
    tool_ = parameter<std::string>(n, "tool_frame", "gripper_drive_link");
    gripper_ = parameter<std::string>(n, "gripper_joint", "joint_gripper");
    gripper_action_ = parameter<std::string>(
        n, "gripper_action", "/gripper_controller/follow_joint_trajectory");
    store_ = parameter<std::string>(n, "pose_store",
                                    "/tmp/roboflex_course_poses.yaml");
    scene_reader_ = n_->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene");
    allow_ = parameter<bool>(n, "allow_execution", false);
    arm_.setPoseReferenceFrame(base_);
    arm_.setEndEffectorLink(tool_);
    arm_.setPlanningTime(10);
    arm_.setNumPlanningAttempts(5);
    arm_.setMaxVelocityScalingFactor(0.25);
    arm_.setMaxAccelerationScalingFactor(0.25);
    arm_.setGoalPositionTolerance(0.002);
    arm_.setGoalOrientationTolerance(0.02);
    stage_pub_ = n->create_publisher<std_msgs::msg::String>(
        "/learning/stage", rclcpp::QoS(1).transient_local());
    stage("idle");
    if (robot_ == "panda") {
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = base_;
      table.id = "workbench";
      table.operation = table.ADD;
      shape_msgs::msg::SolidPrimitive box;
      box.type = box.BOX;
      box.dimensions = {1.0, 1.2, 0.05};
      geometry_msgs::msg::Pose p;
      p.position.x = 0.6;
      p.position.z = -0.03;
      p.orientation.w = 1;
      table.primitives.push_back(box);
      table.primitive_poses.push_back(p);
      if (!scene_.applyCollisionObject(table))
        throw std::runtime_error(
            "Could not install workbench collision geometry");
    }
    truth_sub_ = n->create_subscription<arm_course_interfaces::msg::Objects>(
        "/learning/ground_truth", 10,
        [this](arm_course_interfaces::msg::Objects::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(truth_mutex_);
          truth_ = *m;
        });
    ownership_ = n->create_service<std_srvs::srv::SetBool>(
        "/learning/policy_control",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          if (req->data) {
            if (robot_ != "panda" || !allow_ || busy_.exchange(true)) {
              response->message = "Simulation arm busy or wrong robot";
              return;
            }
            policy_owner_ = true;
          } else {
            if (!policy_owner_.exchange(false)) {
              response->message = "Policy does not own arm";
              return;
            }
            busy_ = false;
          }
          response->success = true;
          std_msgs::msg::String s;
          s.data = req->data ? "policy" : "idle";
          stage_pub_->publish(s);
          response->message =
              req->data ? "Policy owns arm" : "Task executor owns arm";
        });
    server_ = rclcpp_action::create_server<Task>(
        n, "/learning/task",
        [this](const auto &, std::shared_ptr<const Task::Goal>) {
          if (busy_.exchange(true))
            return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Handle>) {
          arm_.stop();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Handle> h) {
          if (worker_.joinable())
            worker_.join();
          worker_ = std::thread([this, h]() { run(h); });
        });
    RCLCPP_INFO(n->get_logger(), "Task server robot=%s execution=%s",
                robot_.c_str(), allow_ ? "enabled" : "preview only");
  }
  ~MotionServer() {
    arm_.stop();
    if (worker_.joinable())
      worker_.join();
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>(
      "motion_server",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  try {
    Spinner spin(n);
    MotionServer server(n);
    while (rclcpp::ok())
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(n->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
