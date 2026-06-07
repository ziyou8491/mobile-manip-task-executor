#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mobile_manip_interfaces/msg/task_state.hpp"
#include "mobile_manip_interfaces/srv/set_mode.hpp"

using namespace std::chrono_literals;

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : Node("task_executor_node")
  {
    // subscription and publisher
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
      500ms,
      std::bind(&TaskExecutorNode::publishTaskState, this)
    );

    // service
    set_mode_srv_ = this->create_service<mobile_manip_interfaces::srv::SetMode>(
      "/set_mode",
      std::bind(&TaskExecutorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/emergency_stop",
      std::bind(&TaskExecutorNode::emergencyStopCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    clear_estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/clear_estop",
      std::bind(&TaskExecutorNode::clearEstopCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    current_task_state_ = "IDLE";
    current_robot_state_ = "UNKNOWN";
    task_message_ = "Task executor initialized.";
    mode_ = "manual";
    has_object_pose_ = false;
    estop_active_ = false;

    RCLCPP_INFO(this->get_logger(), "Task executor node started.");
  }

private:
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_object_pose_ = *msg;
    has_object_pose_ = true;

    if (!estop_active_){
      current_task_state_ = "OBJECT_DETECTED";
      task_message_ = "Received object pose in frame: " + msg->header.frame_id;
    }

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

    if (estop_active_) {
      msg.state = "ESTOP";
      msg.message = "Emergency stop active. Mode: " + mode_ +
        ". Robot state: " + current_robot_state_;
    } else if (!has_object_pose_){
      msg.state = "WAITING_OBJECT";
      msg.message = "Waiting for detected object pose. Mode: " + mode_ + 
        ". Robot state: " + current_robot_state_;
    } else {
      msg.state = current_task_state_;
      msg.message = task_message_ + " Mode: " + mode_ + 
        ". Robot state: " + current_robot_state_;
    }

    task_state_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published task state: %s, mode=%s, estop=%s",
      msg.state.c_str(),
      mode_.c_str(),
      estop_active_ ? "true" : "false"
    );
  }

  void setModeCallback(
    const std::shared_ptr<mobile_manip_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<mobile_manip_interfaces::srv::SetMode::Response> response
  )
  {
    if (request->mode != "manual" && request->mode != "auto"){
      response->success = false;
      response->message = "Invalid mode. Use 'manual' or 'auto'.";
      RCLCPP_WARN(this->get_logger(), "Rejected invalid mode: %s", request->mode.c_str());
      return;
    }

    if (estop_active_){
      response->success = false;
      response->message = "Cannot change mode while emergency stop is active.";
      RCLCPP_WARN(this->get_logger(), "Mode switch rejected because ESTOP is active.");
      return;
    }

    mode_ = request->mode;
    current_task_state_ = "MODE_CHANGED";
    task_message_ = "Mode switched to: " + mode_;

    response->success = true;
    response->message = task_message_;

    RCLCPP_WARN(this->get_logger(), "Mode switched to: %s", mode_.c_str());
  }

  void emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  )
  {
    (void)request;

    estop_active_ = true;
    current_task_state_ = "ESTOP";
    task_message_ = "Emergency stop activated.";

    response->success = true;
    response->message = task_message_;

    RCLCPP_ERROR(this->get_logger(), "Emergency stop activated.");
  }

  void clearEstopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  )
  {
    (void) request;

    if (!estop_active_){
      response->success = true;
      response->message = "Emergency stop is not active.";
      return;
    }

    estop_active_ = false;
    current_task_state_ = has_object_pose_? "OBJECT_DETECTED" : "IDLE";
    task_message_ = "Emergency stop cleared. Mode remains: " + mode_;

    response->success = true;
    response->message = task_message_;

    RCLCPP_WARN(this->get_logger(), "Emergency stop cleared.");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
  rclcpp::Publisher<mobile_manip_interfaces::msg::TaskState>::SharedPtr task_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<mobile_manip_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_estop_srv_;

  geometry_msgs::msg::PoseStamped latest_object_pose_;

  std::string current_task_state_;
  std::string current_robot_state_;
  std::string task_message_;
  std::string mode_;

  bool has_object_pose_;
  bool estop_active_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}