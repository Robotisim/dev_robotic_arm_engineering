#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
    : Node("trajectory_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node created: trajectory_publisher_node");

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryPublisher::timer_callback, this));

        // All Panda joints
        joints_ = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
                   "panda_joint5", "panda_joint6", "panda_joint7"};

        // Target positions, 
        goal_positions_ = {2, 2, 2, 1.5, 2, 1, 0.5}; // Set desired position for joints
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joints_;
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = goal_positions_;
        point.time_from_start.sec = 2; // Time to reach the position

        message.points.push_back(point);

        // Log the values being published
        RCLCPP_INFO(this->get_logger(), "Publishing joint trajectory for joints: %s", 
                    joints_[0].c_str());
        RCLCPP_INFO(this->get_logger(), "Target position: %f", goal_positions_[0]);
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joints_;
    std::vector<double> goal_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting TrajectoryPublisher node...");
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
