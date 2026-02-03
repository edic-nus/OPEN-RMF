#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "lift_control/action/command.hpp"

using namespace std::chrono_literals;

class Lift_State : public rclcpp::Node
{
  public:
    using Command = lift_control::action::Command;
    using GoalHandleCommand = rclcpp_action::ServerGoalHandle<Command>;

    // Lift status codes as per README
    enum LiftStatus {
      FIRST_LEVEL = 1,      // 001: Stationary at First Level
      BETWEEN_LEVELS = 2,   // 010: Stationary Between Levels
      MOVING_DOWN = 3,      // 011: Moving to First Level
      SECOND_LEVEL = 4,     // 100: Stationary at Second Level
      MOVING_UP = 6         // 110: Moving to Second Level
    };
    
    // Command codes
    enum CommandType {
      STOP = 0,
      GO_UP = 1,
      GO_DOWN = 2
    };
    
    explicit Lift_State(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("lift_state", options), count_(0)
    {
      //Initialize publisher
      publisher_ = this->create_publisher<std_msgs::msg::String>("lift_state", 10);
      
      //Initialize timer
      timer_ = this->create_wall_timer(
      2000ms, std::bind(&Lift_State::timer_callback, this));
      
      //Initialize action server
      using namespace std::placeholders;
      this->action_server_ = rclcpp_action::create_server<Command>(
        this,
        "lift_action",
        std::bind(&Lift_State::handle_goal, this, _1, _2),
        std::bind(&Lift_State::handle_cancel, this, _1),
        std::bind(&Lift_State::handle_accepted, this, _1));

      // Report Initialisation
      RCLCPP_INFO(this->get_logger(), "Lift State node initialized. Status: FIRST_LEVEL");
    }

  private:
    // Publisher Members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    // Action Server Members
    rclcpp_action::Server<Command>::SharedPtr action_server_;

    // Lift Status Members
    LiftStatus current_status_ = FIRST_LEVEL;
    std::mutex status_mutex_;

    // Publisher Callback
    void timer_callback()
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      auto message = std_msgs::msg::String();
      message.data = get_status_string(current_status_);
      RCLCPP_INFO(this->get_logger(), "Lift Status: %s", message.data.c_str());
      publisher_->publish(message);
    }

    std::string get_status_string(LiftStatus status)
    {
      switch(status) {
        case FIRST_LEVEL:
          return "001: Stationary at First Level";
        case BETWEEN_LEVELS:
          return "010: Stationary Between Levels";
        case MOVING_DOWN:
          return "011: Moving to First Level";
        case SECOND_LEVEL:
          return "100: Stationary at Second Level";
        case MOVING_UP:
          return "110: Moving to Second Level";
        default:
          return "Unknown Status";
      }
    }

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Command::Goal> goal)
    {
      (void)uuid;
      std::lock_guard<std::mutex> lock(status_mutex_);
      
      RCLCPP_INFO(this->get_logger(), "Received command: %d (0=Stop, 1=Up, 2=Down)", 
                  goal->command);
      
      // Validate command based on current status
      if (goal->command < STOP || goal->command > GO_DOWN) {
        RCLCPP_WARN(this->get_logger(), "Invalid command: %d", goal->command);
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      // Check if command is allowed in current state
      bool command_allowed = false;
      std::string rejection_reason;
      
      switch(current_status_) {
        case FIRST_LEVEL:
          command_allowed = (goal->command == GO_UP);
          rejection_reason = "At First Level - Only GO_UP accepted";
          break;
          
        case SECOND_LEVEL:
          command_allowed = (goal->command == GO_DOWN);
          rejection_reason = "At Second Level - Only GO_DOWN accepted";
          break;
          
        case BETWEEN_LEVELS:
          command_allowed = (goal->command == GO_UP || goal->command == GO_DOWN);
          rejection_reason = "Between Levels - Only GO_UP or GO_DOWN accepted";
          break;
          
        case MOVING_UP:
        case MOVING_DOWN:
          command_allowed = (goal->command == STOP);
          rejection_reason = "Lift is Moving - Only STOP accepted";
          break;
      }
      
      if (!command_allowed) {
        RCLCPP_WARN(this->get_logger(), "Command rejected: %s", rejection_reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel command");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCommand> goal_handle)
  {
    using namespace std::placeholders;
    // Spin up a new thread to avoid blocking the executor
    std::thread{std::bind(&Lift_State::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing command");
    rclcpp::Rate loop_rate(2);  // 2 Hz for status updates
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Command::Feedback>();
    auto result = std::make_shared<Command::Result>();
    
    CommandType command = static_cast<CommandType>(goal->command);
    
    // Handle STOP command
    if (command == STOP) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = BETWEEN_LEVELS;
      feedback->status = 1;  // Reached (stopped)
      result->completion = 1;  // Reached
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Lift stopped between levels");
      return;
    }
    
    // Handle GO_UP and GO_DOWN commands
    int steps = 10;  // Simulated steps to complete movement
    LiftStatus target_status;
    LiftStatus moving_status;
    
    if (command == GO_UP) {
      target_status = SECOND_LEVEL;
      moving_status = MOVING_UP;
      RCLCPP_INFO(this->get_logger(), "Moving UP to Second Level");
    } else {  // GO_DOWN
      target_status = FIRST_LEVEL;
      moving_status = MOVING_DOWN;
      RCLCPP_INFO(this->get_logger(), "Moving DOWN to First Level");
    }
    
    // Update status to moving
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = moving_status;
    }
    
    // Simulate movement
    for (int i = 0; i <= steps && rclcpp::ok(); ++i) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = BETWEEN_LEVELS;
        result->completion = 0;  // Still moving when cancelled
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Command canceled - Lift stopped between levels");
        return;
      }
      
      // Publish feedback
      feedback->status = 0;  // Moving
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Progress: %d%%", (i * 100) / steps);
      
      loop_rate.sleep();
    }
    
    // Movement complete
    if (rclcpp::ok()) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = target_status;
      result->completion = 1;  // Reached
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Reached target level: %s", 
                  get_status_string(target_status).c_str());
    }
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Lift_State)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lift_State>());
  rclcpp::shutdown();
  return 0;
}