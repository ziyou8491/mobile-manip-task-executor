#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class FakeVisionNode : public rclcpp::Node
{
public:
  FakeVisionNode()
  : Node("fake_vision_node")
  {
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<std::string>("frame_id", "camera_frame");

    this->declare_parameter<double>("object_x", 1.0);
    this->declare_parameter<double>("object_y", 0.2);
    this->declare_parameter<double>("object_z", 0.7);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_object_pose",
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
      std::bind(&FakeVisionNode::publish_object_pose, this)
    );

    RCLCPP_INFO(this->get_logger(), "Fake vision node started.");
  }

private:
  void publish_object_pose()
  {
    auto msg = geometry_msgs::msg::PoseStamped();

    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    msg.pose.position.x = this->get_parameter("object_x").as_double();
    msg.pose.position.y = this->get_parameter("object_y").as_double();
    msg.pose.position.z = this->get_parameter("object_z").as_double();

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published object pose: frame=%s, x=%.2f, y=%.2f, z=%.2f",
      msg.header.frame_id.c_str(),
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z
    );
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeVisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}