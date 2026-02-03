// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lift_control:action/Command.idl
// generated code does not contain a copyright notice

#ifndef LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_HPP_
#define LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_Goal __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_Goal __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Goal_
{
  using Type = Command_Goal_<ContainerAllocator>;

  explicit Command_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0l;
    }
  }

  explicit Command_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0l;
    }
  }

  // field types and members
  using _command_type =
    int32_t;
  _command_type command;

  // setters for named parameter idiom
  Type & set__command(
    const int32_t & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_Goal
    std::shared_ptr<lift_control::action::Command_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_Goal
    std::shared_ptr<lift_control::action::Command_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Goal_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Goal_

// alias to use template instance with default allocator
using Command_Goal =
  lift_control::action::Command_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control


#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_Result __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_Result __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Result_
{
  using Type = Command_Result_<ContainerAllocator>;

  explicit Command_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->completion = 0l;
    }
  }

  explicit Command_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->completion = 0l;
    }
  }

  // field types and members
  using _completion_type =
    int32_t;
  _completion_type completion;

  // setters for named parameter idiom
  Type & set__completion(
    const int32_t & _arg)
  {
    this->completion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_Result
    std::shared_ptr<lift_control::action::Command_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_Result
    std::shared_ptr<lift_control::action::Command_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Result_ & other) const
  {
    if (this->completion != other.completion) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Result_

// alias to use template instance with default allocator
using Command_Result =
  lift_control::action::Command_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control


#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_Feedback __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Feedback_
{
  using Type = Command_Feedback_<ContainerAllocator>;

  explicit Command_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0l;
    }
  }

  explicit Command_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0l;
    }
  }

  // field types and members
  using _status_type =
    int32_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const int32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_Feedback
    std::shared_ptr<lift_control::action::Command_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_Feedback
    std::shared_ptr<lift_control::action::Command_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Feedback_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Feedback_

// alias to use template instance with default allocator
using Command_Feedback =
  lift_control::action::Command_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "lift_control/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_SendGoal_Request __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_SendGoal_Request_
{
  using Type = Command_SendGoal_Request_<ContainerAllocator>;

  explicit Command_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Command_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    lift_control::action::Command_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const lift_control::action::Command_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_SendGoal_Request
    std::shared_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_SendGoal_Request
    std::shared_ptr<lift_control::action::Command_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_SendGoal_Request_

// alias to use template instance with default allocator
using Command_SendGoal_Request =
  lift_control::action::Command_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_SendGoal_Response __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_SendGoal_Response_
{
  using Type = Command_SendGoal_Response_<ContainerAllocator>;

  explicit Command_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Command_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_SendGoal_Response
    std::shared_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_SendGoal_Response
    std::shared_ptr<lift_control::action::Command_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_SendGoal_Response_

// alias to use template instance with default allocator
using Command_SendGoal_Response =
  lift_control::action::Command_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control

namespace lift_control
{

namespace action
{

struct Command_SendGoal
{
  using Request = lift_control::action::Command_SendGoal_Request;
  using Response = lift_control::action::Command_SendGoal_Response;
};

}  // namespace action

}  // namespace lift_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_GetResult_Request __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_GetResult_Request_
{
  using Type = Command_GetResult_Request_<ContainerAllocator>;

  explicit Command_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Command_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_GetResult_Request
    std::shared_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_GetResult_Request
    std::shared_ptr<lift_control::action::Command_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_GetResult_Request_

// alias to use template instance with default allocator
using Command_GetResult_Request =
  lift_control::action::Command_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control


// Include directives for member types
// Member 'result'
// already included above
// #include "lift_control/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_GetResult_Response __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_GetResult_Response_
{
  using Type = Command_GetResult_Response_<ContainerAllocator>;

  explicit Command_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Command_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    lift_control::action::Command_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const lift_control::action::Command_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_GetResult_Response
    std::shared_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_GetResult_Response
    std::shared_ptr<lift_control::action::Command_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_GetResult_Response_

// alias to use template instance with default allocator
using Command_GetResult_Response =
  lift_control::action::Command_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control

namespace lift_control
{

namespace action
{

struct Command_GetResult
{
  using Request = lift_control::action::Command_GetResult_Request;
  using Response = lift_control::action::Command_GetResult_Response;
};

}  // namespace action

}  // namespace lift_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "lift_control/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lift_control__action__Command_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__lift_control__action__Command_FeedbackMessage __declspec(deprecated)
#endif

namespace lift_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_FeedbackMessage_
{
  using Type = Command_FeedbackMessage_<ContainerAllocator>;

  explicit Command_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Command_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    lift_control::action::Command_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const lift_control::action::Command_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lift_control::action::Command_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const lift_control::action::Command_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lift_control::action::Command_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lift_control__action__Command_FeedbackMessage
    std::shared_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lift_control__action__Command_FeedbackMessage
    std::shared_ptr<lift_control::action::Command_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_FeedbackMessage_

// alias to use template instance with default allocator
using Command_FeedbackMessage =
  lift_control::action::Command_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace lift_control

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace lift_control
{

namespace action
{

struct Command
{
  /// The goal message defined in the action definition.
  using Goal = lift_control::action::Command_Goal;
  /// The result message defined in the action definition.
  using Result = lift_control::action::Command_Result;
  /// The feedback message defined in the action definition.
  using Feedback = lift_control::action::Command_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = lift_control::action::Command_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = lift_control::action::Command_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = lift_control::action::Command_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Command Command;

}  // namespace action

}  // namespace lift_control

#endif  // LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_HPP_
