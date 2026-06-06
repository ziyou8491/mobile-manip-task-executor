#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "mobile_manip_interfaces/msg/task_state.hpp"

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : Node("task_executor_node")
  {
    object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_object_pose",
      10,
      std::bind(&TaskExecutorNode::objectPoseCallback, this, std::placeholders::_1)
    );

    robot_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_state",
      10,
      std::bind(&TaskExecutorNode::robotStateCallback, this, std::placeholders::_1)
    );

    task_state_pub_ = this->create_publisher<mobile_manip_interfaces::msg::TaskState>(
      "/task_state",
      10
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TaskExecutorNode::publishTaskState, this)
    );

    current_task_state_ = "IDLE";
    current_robot_state_ = "UNKNOWN";
    has_object_pose_ = false;

    RCLCPP_INFO(this->get_logger(), "Task executor node started.");
  }

private:
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_object_pose_ = *msg;
    has_object_pose_ = true;

    current_task_state_ = "OBJECT_DETECTED";
    task_message_ = "Received object pose in frame: " + msg->header.frame_id;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Received object pose: frame=%s, x=%.2f, y=%.2f, z=%.2f",
      msg->header.frame_id.c_str(),
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    );
  }

  void robotStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_robot_state_ = msg->data;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Received robot state: %s",
      current_robot_state_.c_str()
    );
  }

  void publishTaskState()
  {
    auto msg = mobile_manip_interfaces::msg::TaskState();

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.state = current_task_state_;

    if (!has_object_pose_) {
      msg.state = "WAITING_OBJECT";
      msg.message = "Waiting for detected object pose. Robot state: " + current_robot_state_;
    } else {
      msg.message = task_message_ + ". Robot state: " + current_robot_state_;
    }

    task_state_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published task state: %s",
      msg.state.c_str()
    );
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;

  rclcpp::Publisher<mobile_manip_interfaces::msg::TaskState>::SharedPtr task_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped latest_object_pose_;

  std::string current_task_state_;
  std::string current_robot_state_;
  std::string task_message_;

  bool has_object_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}