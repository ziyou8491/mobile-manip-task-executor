#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mobile_manip_interfaces/msg/task_state.hpp"
#include "mobile_manip_interfaces/srv/set_mode.hpp"
#include "mobile_manip_interfaces/action/execute_object_task.hpp"

using namespace std::chrono_literals;

class TaskExecutorNode : public rclcpp::Node
{
public:
  using ExecuteObjectTask = mobile_manip_interfaces::action::ExecuteObjectTask;
  using GoalHandleExecuteObjectTask = rclcpp_action::ServerGoalHandle<ExecuteObjectTask>;

  TaskExecutorNode()
  : Node("task_executor_node")
  {

    current_task_state_ = "IDLE";
    current_robot_state_ = "UNKNOWN";
    task_message_ = "Task executor initialized.";
    mode_ = "manual";
    has_object_pose_ = false;
    estop_active_ = false;
    action_in_progress_ = false;

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

    // action
    action_server_ = rclcpp_action::create_server<ExecuteObjectTask>(
      this,
      "/execute_object_task",
      std::bind(&TaskExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TaskExecutorNode::handleCancel, this, std::placeholders::_1),
      std::bind(&TaskExecutorNode::handleAccepted, this, std::placeholders::_1)
    );


    RCLCPP_INFO(this->get_logger(), "Task executor node started.");
  }

private:

  // subscription callback
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      latest_detected_object_pose_ = *msg;
      has_object_pose_ = true;

      if (!estop_active_ && !action_in_progress_ && shouldUpdateStateFromDetection(current_task_state_)){
        current_task_state_ = "OBJECT_DETECTED";
        task_message_ = "Received object pose in frame: " + msg->header.frame_id;
      }
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
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_robot_state_ = msg->data;
    }
    

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Received robot state: %s",
      msg->data.c_str()
    );
  }

  // publisher
  void publishTaskState()
  {
    std::string task_state;
    std::string message;
    std::string mode;
    std::string robot_state;

    bool has_object_pose;
    bool estop_active;
    bool action_in_progress;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      task_state = current_task_state_;
      message = task_message_;
      mode = mode_;
      robot_state = current_robot_state_;

      has_object_pose = has_object_pose_;
      estop_active = estop_active_;
      action_in_progress = action_in_progress_;
    }

    auto msg = mobile_manip_interfaces::msg::TaskState();

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    if (estop_active) {
      msg.state = "ESTOP";
      msg.message = 
        "Emergency stop active. Mode: " + mode +
        ". Action in progress: " + std::string(action_in_progress ? "true" : "false") +
        ". Robot state: " + robot_state;
    } else {
      msg.state = task_state;
      msg.message = message + 
        " Mode: " + mode + 
        ". Action in progress: " + std::string(action_in_progress ? "true" : "false") +
        ". Object pose available: " + std::string(has_object_pose ? "true" : "false") +
        ". Robot state: " + robot_state;
    }

    task_state_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published task state: %s, mode=%s, estop=%s, action_in_progress=%s",
      msg.state.c_str(),
      mode.c_str(),
      estop_active ? "true" : "false",
      action_in_progress ? "true" : "false"
    );
  }

  // service callback
  void setModeCallback(
    const std::shared_ptr<mobile_manip_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<mobile_manip_interfaces::srv::SetMode::Response> response
  )
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

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

    if (action_in_progress_){
      response->success = false;
      response->message = "Cannot change mode while another action is in progress.";
      RCLCPP_WARN(this->get_logger(), "Mode switch rejected because another action is in progress.");
      return;
    }

    mode_ = request->mode;
    current_task_state_ = "MODE_CHANGED";
    task_message_ = "Mode switched to: " + mode_;

    response->success = true;
    response->message = task_message_;

    RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", mode_.c_str());
  }

  void emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  )
  {
    (void)request;

    const std::string message = "Emergency stop activated.";

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      estop_active_ = true;
      current_task_state_ = "ESTOP";
      task_message_ = message;      
    }

    response->success = true;
    response->message = message;

    RCLCPP_ERROR(this->get_logger(), "Emergency stop activated.");
  }

  void clearEstopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
  )
  {
    (void) request;

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!estop_active_){
      response->success = true;
      response->message = "Emergency stop is not active.";
      return;
    }

    if (action_in_progress_){
      response->success = false;
      response->message = "Cannot clear emergency stop while an action is still in progress. Wait until the action aborts";
      return;
    }

    estop_active_ = false;
    current_task_state_ = has_object_pose_? "OBJECT_DETECTED" : "IDLE";
    task_message_ = "Emergency stop cleared. Mode remains: " + mode_;

    response->success = true;
    response->message = task_message_;

    RCLCPP_WARN(this->get_logger(), "Emergency stop cleared.");
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteObjectTask::Goal> goal
  ){
    (void)uuid;

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (estop_active_) {
      RCLCPP_WARN(this->get_logger(), "Rejected action goal because ESTOP is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (action_in_progress_) {
      RCLCPP_WARN(this->get_logger(), "Rejected action goal because another action is already in progress.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (mode_ != "auto") {
      RCLCPP_WARN(this->get_logger(), "Rejected action goal because mode is not auto.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->task_type != "inspect") {
      RCLCPP_WARN(this->get_logger(), "Rejected action goal because only task_type = 'inspect' is supported now.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->approach_distance <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Rejected action goal because approach_distance <= 0.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    action_in_progress_ = true;
    current_task_state_ = "GOAL_ACCEPTED";
    task_message_ = "Action goal accepted";

    RCLCPP_INFO(
      this->get_logger(), "Accepted action goal: task_type=%s, approach_distance=%.2f", 
      goal->task_type.c_str(), goal->approach_distance
    );

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteObjectTask> goal_handle
  ) {
    (void)goal_handle;
    RCLCPP_WARN(this->get_logger(), "Received request to cancel action goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteObjectTask> goal_handle)
  {
    std::thread{
      std::bind(&TaskExecutorNode::executeTask, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void executeTask(const std::shared_ptr<GoalHandleExecuteObjectTask> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto task_object_pose = goal->object_pose;
    auto feedback = std::make_shared<ExecuteObjectTask::Feedback>();
    auto result = std::make_shared<ExecuteObjectTask::Result>();
    
    if (task_object_pose.pose.position.x > 2.0) {
      setTaskState("FAILED", "Object is outside the mock reachable range.");
      result->success = false;
      result->message = "Object is outside the mock reachable range.";
      goal_handle->abort(result);
      finishTask();
      return;
    }

    auto run_step = [this, &goal_handle, &feedback, &result](
      const std::string & state,
      const std::string & message,
      float progress,
      int duration_ms
    ) -> bool
    {
      setTaskState(state, message);
      feedback->progress = progress;
      feedback->current_state = state;
      goal_handle->publish_feedback(feedback);

      const int tick_ms = 100;

      for (int elapsed = 0; elapsed < duration_ms; elapsed += tick_ms){
        if (goal_handle->is_canceling()) {
          setTaskState("CANCELED", "Task canceled by client.");
          result->success = false;
          result->message = "Task canceled by client.";
          goal_handle->canceled(result);
          finishTask();
          return false;
        }

        if (isEstopActive()) {
          setTaskState("ESTOP", "Task aborted because emergency stop is active.");
          result->success = false;
          result->message = "Task aborted because emergency stop is active.";
          goal_handle->abort(result);
          finishTask();
          return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(tick_ms));
      }

      return true;
    };

    if (!run_step("OBJECT_DETECTED", "Object task goal accepted.", 0.10f, 500)) {
      return;
    }

    if (!run_step("NAV_GOAL_COMPUTED", "Mock navigation goal computed from object pose.", 0.25f, 500)) {
      return;
    }

    if (!run_step("NAVIGATING", "Mock Nav2 navigation is running.", 0.45f, 3000)) {
      return;
    }

    if (!run_step("NAV_REACHED", "Mock mobile base reached the manipulation area.", 0.60f, 500)) {
      return;
    }

    if (!run_step("ARM_PLANNING", "Mock MoveIt2 arm planning is running.", 0.75f, 1000)){
      return;
    }

    if (!run_step("ARM_EXECUTING", "Mock arm execution is running.", 0.90f, 3000)) {
      return;
    }

    setTaskState("SUCCEEDED", "Mock mobile manipulation task completed.");

    feedback->progress = 1.0f;
    feedback->current_state = "SUCCEEDED";
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Mock mobile manipulation task completed successfully.";
    goal_handle->succeed(result);

    finishTask();

    RCLCPP_INFO(this->get_logger(), "Action task succeeded.");
  }

  void setTaskState(const std::string & state, const std::string &message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_state_ = state;
    task_message_ = message;
  }

  void finishTask()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    action_in_progress_ = false;
  }

  bool isEstopActive()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return estop_active_;
  }

  bool shouldUpdateStateFromDetection(const std::string & state) const
  {
    return state == "IDLE" ||
           state == "WAITING_OBJECT" ||
           state == "OBJECT_DETECTED" ||
           state == "MODE_CHANGED";
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
  rclcpp::Publisher<mobile_manip_interfaces::msg::TaskState>::SharedPtr task_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<mobile_manip_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_estop_srv_;

  rclcpp_action::Server<ExecuteObjectTask>::SharedPtr action_server_;

  geometry_msgs::msg::PoseStamped latest_detected_object_pose_;

  std::string current_task_state_;
  std::string current_robot_state_;
  std::string task_message_;
  std::string mode_;

  bool has_object_pose_;
  bool estop_active_;
  bool action_in_progress_;

  std::mutex state_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}