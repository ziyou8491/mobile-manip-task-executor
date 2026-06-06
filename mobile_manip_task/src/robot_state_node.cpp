#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RobotStateNode : public rclcpp::Node
{
public:
  RobotStateNode()
  : Node("robot_state_node")
  {
    this->declare_parameter<std::string>("initial_state", "READY");
    this->declare_parameter<double>("publish_rate", 1.0);

    robot_state_ = this->get_parameter("initial_state").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/robot_state",
      10
    );

    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (publish_rate <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_rate <= 0, reset to 1.0 Hz");
      publish_rate = 1.0;
    }

    auto period_ms = static_cast<int>(1000.0 / publish_rate);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RobotStateNode::publish_robot_state, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Robot state node started. Initial state: %s",
      robot_state_.c_str()
    );
  }

private:
  void publish_robot_state()
  {
    auto msg = std_msgs::msg::String();
    msg.data = robot_state_;

    publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published robot state: %s",
      msg.data.c_str()
    );
  }

  std::string robot_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}