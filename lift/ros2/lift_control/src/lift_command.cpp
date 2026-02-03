#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lift_control/action/command.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Lift_Commander : public rclcpp::Node
{
public:
  using Command = lift_control::action::Command;
  using GoalHandleCommand = rclcpp_action::ClientGoalHandle<Command>;

  Lift_Commander()
    : Node("lift_commander"), goal_in_progress_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "lift_state", 10, std::bind(&Lift_Commander::topic_callback, this, _1));
    
    client_ptr_ = rclcpp_action::create_client<Command>(this, "lift_action");
    
    RCLCPP_INFO(this->get_logger(), "READY TO SEND COMMANDS");
  }

  void send_command(int command)
  {
    if (goal_in_progress_) {
      RCLCPP_WARN(this->get_logger(), "Goal in progress");
      return;
    }

    auto goal_msg = Command::Goal();
    goal_msg.command = command;
    
    const char* cmd_str[] = {"STOP", "GO UP", "GO DOWN"};
    RCLCPP_INFO(this->get_logger(), "Command: %s", cmd_str[command]);

    auto send_goal_options = rclcpp_action::Client<Command>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&Lift_Commander::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&Lift_Commander::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&Lift_Commander::result_callback, this, _1);
    
    goal_in_progress_ = true;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp_action::Client<Command>::SharedPtr client_ptr_;
  bool goal_in_progress_;

  void topic_callback(const std_msgs::msg::String & msg)
  {
    RCLCPP_INFO(this->get_logger(), "State: %s", msg.data.c_str());
  }

  void goal_response_callback(const GoalHandleCommand::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "REJECTED");
      goal_in_progress_ = false;
      RCLCPP_INFO(this->get_logger(), "=== LIFT CONTROL ===");
      RCLCPP_INFO(this->get_logger(), "0=STOP 1=UP 2=DOWN q=QUIT");
    } else {
      RCLCPP_INFO(this->get_logger(), "ACCEPTED");
    }
  }

  void feedback_callback(
    GoalHandleCommand::SharedPtr,
    const std::shared_ptr<const Command::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), feedback->status == 0 ? "MOVING" : "REACHED");
  }

  void result_callback(const GoalHandleCommand::WrappedResult & result)
  {
    goal_in_progress_ = false;
    
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), result.result->completion == 1 ? 
                  "SUCCESS - Reached" : "SUCCESS - Stopped");
    } else {
      RCLCPP_ERROR(this->get_logger(), "FAILED");
    }
    RCLCPP_INFO(this->get_logger(), "=== LIFT CONTROL ===");
    RCLCPP_INFO(this->get_logger(), "0=STOP 1=UP 2=DOWN q=QUIT");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Lift_Commander>();
  
  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  std::cout << "\n=== LIFT CONTROL ===\n";
  std::cout << "0=STOP 1=UP 2=DOWN q=QUIT\n\n";
  
  std::string input;
  while (rclcpp::ok()) {
    std::cout << "> ";
    std::getline(std::cin, input);
    
    if (input.empty()) continue;
    
    if (input == "q" || input == "Q") {
      std::cout << "Shutting down...\n";
      rclcpp::shutdown();
      break;
    }
    
    try {
      int command = std::stoi(input);
      if (command >= 0 && command <= 2) {
        node->send_command(command);
      } else {
        std::cout << "Invalid! Use 0, 1, 2, or q\n";
      }
    } catch (...) {
      std::cout << "Invalid input\n";
    }
  }
  
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  
  return 0;
}