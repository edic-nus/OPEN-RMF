// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lift_control:action/Command.idl
// generated code does not contain a copyright notice

#ifndef LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_H_
#define LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_Goal
{
  int32_t command;
} lift_control__action__Command_Goal;

// Struct for a sequence of lift_control__action__Command_Goal.
typedef struct lift_control__action__Command_Goal__Sequence
{
  lift_control__action__Command_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_Result
{
  int32_t completion;
} lift_control__action__Command_Result;

// Struct for a sequence of lift_control__action__Command_Result.
typedef struct lift_control__action__Command_Result__Sequence
{
  lift_control__action__Command_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_Feedback
{
  int32_t status;
} lift_control__action__Command_Feedback;

// Struct for a sequence of lift_control__action__Command_Feedback.
typedef struct lift_control__action__Command_Feedback__Sequence
{
  lift_control__action__Command_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "lift_control/action/detail/command__struct.h"

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  lift_control__action__Command_Goal goal;
} lift_control__action__Command_SendGoal_Request;

// Struct for a sequence of lift_control__action__Command_SendGoal_Request.
typedef struct lift_control__action__Command_SendGoal_Request__Sequence
{
  lift_control__action__Command_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} lift_control__action__Command_SendGoal_Response;

// Struct for a sequence of lift_control__action__Command_SendGoal_Response.
typedef struct lift_control__action__Command_SendGoal_Response__Sequence
{
  lift_control__action__Command_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} lift_control__action__Command_GetResult_Request;

// Struct for a sequence of lift_control__action__Command_GetResult_Request.
typedef struct lift_control__action__Command_GetResult_Request__Sequence
{
  lift_control__action__Command_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "lift_control/action/detail/command__struct.h"

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_GetResult_Response
{
  int8_t status;
  lift_control__action__Command_Result result;
} lift_control__action__Command_GetResult_Response;

// Struct for a sequence of lift_control__action__Command_GetResult_Response.
typedef struct lift_control__action__Command_GetResult_Response__Sequence
{
  lift_control__action__Command_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "lift_control/action/detail/command__struct.h"

/// Struct defined in action/Command in the package lift_control.
typedef struct lift_control__action__Command_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  lift_control__action__Command_Feedback feedback;
} lift_control__action__Command_FeedbackMessage;

// Struct for a sequence of lift_control__action__Command_FeedbackMessage.
typedef struct lift_control__action__Command_FeedbackMessage__Sequence
{
  lift_control__action__Command_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lift_control__action__Command_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIFT_CONTROL__ACTION__DETAIL__COMMAND__STRUCT_H_
