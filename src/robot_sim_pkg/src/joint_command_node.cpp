#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointCommandNode : public rclcpp::Node {
public:
  JointCommandNode()
  : Node("joint_command_node"), start_time_sec_(this->get_clock()->now().seconds()) {
    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&JointCommandNode::publish_command, this));

    RCLCPP_INFO(this->get_logger(), "joint_command_node started. Publishing controller commands at 50 Hz");
  }

private:
  void publish_command() {
    const double t = this->get_clock()->now().seconds() - start_time_sec_;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      0.6 * std::sin(0.5 * t),
      0.4 * std::sin(0.8 * t)
    };

    command_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double start_time_sec_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
