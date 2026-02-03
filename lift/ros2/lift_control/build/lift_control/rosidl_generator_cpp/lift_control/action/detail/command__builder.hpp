// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lift_control:action/Command.idl
// generated code does not contain a copyright notice

#ifndef LIFT_CONTROL__ACTION__DETAIL__COMMAND__BUILDER_HPP_
#define LIFT_CONTROL__ACTION__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lift_control/action/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_Goal_command
{
public:
  Init_Command_Goal_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lift_control::action::Command_Goal command(::lift_control::action::Command_Goal::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_Goal>()
{
  return lift_control::action::builder::Init_Command_Goal_command();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_Result_completion
{
public:
  Init_Command_Result_completion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lift_control::action::Command_Result completion(::lift_control::action::Command_Result::_completion_type arg)
  {
    msg_.completion = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_Result>()
{
  return lift_control::action::builder::Init_Command_Result_completion();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_Feedback_status
{
public:
  Init_Command_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lift_control::action::Command_Feedback status(::lift_control::action::Command_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_Feedback>()
{
  return lift_control::action::builder::Init_Command_Feedback_status();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_SendGoal_Request_goal
{
public:
  explicit Init_Command_SendGoal_Request_goal(::lift_control::action::Command_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::lift_control::action::Command_SendGoal_Request goal(::lift_control::action::Command_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_SendGoal_Request msg_;
};

class Init_Command_SendGoal_Request_goal_id
{
public:
  Init_Command_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_SendGoal_Request_goal goal_id(::lift_control::action::Command_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Command_SendGoal_Request_goal(msg_);
  }

private:
  ::lift_control::action::Command_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_SendGoal_Request>()
{
  return lift_control::action::builder::Init_Command_SendGoal_Request_goal_id();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_SendGoal_Response_stamp
{
public:
  explicit Init_Command_SendGoal_Response_stamp(::lift_control::action::Command_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::lift_control::action::Command_SendGoal_Response stamp(::lift_control::action::Command_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_SendGoal_Response msg_;
};

class Init_Command_SendGoal_Response_accepted
{
public:
  Init_Command_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_SendGoal_Response_stamp accepted(::lift_control::action::Command_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Command_SendGoal_Response_stamp(msg_);
  }

private:
  ::lift_control::action::Command_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_SendGoal_Response>()
{
  return lift_control::action::builder::Init_Command_SendGoal_Response_accepted();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_GetResult_Request_goal_id
{
public:
  Init_Command_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lift_control::action::Command_GetResult_Request goal_id(::lift_control::action::Command_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_GetResult_Request>()
{
  return lift_control::action::builder::Init_Command_GetResult_Request_goal_id();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_GetResult_Response_result
{
public:
  explicit Init_Command_GetResult_Response_result(::lift_control::action::Command_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::lift_control::action::Command_GetResult_Response result(::lift_control::action::Command_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_GetResult_Response msg_;
};

class Init_Command_GetResult_Response_status
{
public:
  Init_Command_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_GetResult_Response_result status(::lift_control::action::Command_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Command_GetResult_Response_result(msg_);
  }

private:
  ::lift_control::action::Command_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_GetResult_Response>()
{
  return lift_control::action::builder::Init_Command_GetResult_Response_status();
}

}  // namespace lift_control


namespace lift_control
{

namespace action
{

namespace builder
{

class Init_Command_FeedbackMessage_feedback
{
public:
  explicit Init_Command_FeedbackMessage_feedback(::lift_control::action::Command_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::lift_control::action::Command_FeedbackMessage feedback(::lift_control::action::Command_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lift_control::action::Command_FeedbackMessage msg_;
};

class Init_Command_FeedbackMessage_goal_id
{
public:
  Init_Command_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_FeedbackMessage_feedback goal_id(::lift_control::action::Command_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Command_FeedbackMessage_feedback(msg_);
  }

private:
  ::lift_control::action::Command_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::lift_control::action::Command_FeedbackMessage>()
{
  return lift_control::action::builder::Init_Command_FeedbackMessage_goal_id();
}

}  // namespace lift_control

#endif  // LIFT_CONTROL__ACTION__DETAIL__COMMAND__BUILDER_HPP_
