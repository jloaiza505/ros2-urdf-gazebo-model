#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RobotSimNode : public rclcpp::Node {
public:
  RobotSimNode()
  : Node("robot_sim_node"), start_time_sec_(this->get_clock()->now().seconds()) {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RobotSimNode::publish_joint_states, this));

    RCLCPP_INFO(this->get_logger(), "robot_sim_node started. Publishing /joint_states at 50 Hz (RViz mode)");
  }

private:
  void publish_joint_states() {
    const double t = this->get_clock()->now().seconds() - start_time_sec_;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name = {"joint1", "joint2"};
    msg.position = {
      0.6 * std::sin(0.7 * t),
      0.4 * std::sin(1.2 * t)
    };
    msg.velocity = {
      0.6 * 0.7 * std::cos(0.7 * t),
      0.4 * 1.2 * std::cos(1.2 * t)
    };

    joint_state_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double start_time_sec_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
