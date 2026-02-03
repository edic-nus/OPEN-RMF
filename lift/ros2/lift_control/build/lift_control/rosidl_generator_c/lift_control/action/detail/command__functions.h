// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lift_control:action/Command.idl
// generated code does not contain a copyright notice

#ifndef LIFT_CONTROL__ACTION__DETAIL__COMMAND__FUNCTIONS_H_
#define LIFT_CONTROL__ACTION__DETAIL__COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lift_control/msg/rosidl_generator_c__visibility_control.h"

#include "lift_control/action/detail/command__struct.h"

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_Goal
 * )) before or use
 * lift_control__action__Command_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__init(lift_control__action__Command_Goal * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Goal__fini(lift_control__action__Command_Goal * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Goal *
lift_control__action__Command_Goal__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Goal__destroy(lift_control__action__Command_Goal * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__are_equal(const lift_control__action__Command_Goal * lhs, const lift_control__action__Command_Goal * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__copy(
  const lift_control__action__Command_Goal * input,
  lift_control__action__Command_Goal * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__Sequence__init(lift_control__action__Command_Goal__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Goal__Sequence__fini(lift_control__action__Command_Goal__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Goal__Sequence *
lift_control__action__Command_Goal__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Goal__Sequence__destroy(lift_control__action__Command_Goal__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__Sequence__are_equal(const lift_control__action__Command_Goal__Sequence * lhs, const lift_control__action__Command_Goal__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Goal__Sequence__copy(
  const lift_control__action__Command_Goal__Sequence * input,
  lift_control__action__Command_Goal__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_Result
 * )) before or use
 * lift_control__action__Command_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__init(lift_control__action__Command_Result * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Result__fini(lift_control__action__Command_Result * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Result *
lift_control__action__Command_Result__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Result__destroy(lift_control__action__Command_Result * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__are_equal(const lift_control__action__Command_Result * lhs, const lift_control__action__Command_Result * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__copy(
  const lift_control__action__Command_Result * input,
  lift_control__action__Command_Result * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__Sequence__init(lift_control__action__Command_Result__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Result__Sequence__fini(lift_control__action__Command_Result__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Result__Sequence *
lift_control__action__Command_Result__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Result__Sequence__destroy(lift_control__action__Command_Result__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__Sequence__are_equal(const lift_control__action__Command_Result__Sequence * lhs, const lift_control__action__Command_Result__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Result__Sequence__copy(
  const lift_control__action__Command_Result__Sequence * input,
  lift_control__action__Command_Result__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_Feedback
 * )) before or use
 * lift_control__action__Command_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__init(lift_control__action__Command_Feedback * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Feedback__fini(lift_control__action__Command_Feedback * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Feedback *
lift_control__action__Command_Feedback__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Feedback__destroy(lift_control__action__Command_Feedback * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__are_equal(const lift_control__action__Command_Feedback * lhs, const lift_control__action__Command_Feedback * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__copy(
  const lift_control__action__Command_Feedback * input,
  lift_control__action__Command_Feedback * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__Sequence__init(lift_control__action__Command_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Feedback__Sequence__fini(lift_control__action__Command_Feedback__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_Feedback__Sequence *
lift_control__action__Command_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_Feedback__Sequence__destroy(lift_control__action__Command_Feedback__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__Sequence__are_equal(const lift_control__action__Command_Feedback__Sequence * lhs, const lift_control__action__Command_Feedback__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_Feedback__Sequence__copy(
  const lift_control__action__Command_Feedback__Sequence * input,
  lift_control__action__Command_Feedback__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_SendGoal_Request
 * )) before or use
 * lift_control__action__Command_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__init(lift_control__action__Command_SendGoal_Request * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Request__fini(lift_control__action__Command_SendGoal_Request * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_SendGoal_Request *
lift_control__action__Command_SendGoal_Request__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Request__destroy(lift_control__action__Command_SendGoal_Request * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__are_equal(const lift_control__action__Command_SendGoal_Request * lhs, const lift_control__action__Command_SendGoal_Request * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__copy(
  const lift_control__action__Command_SendGoal_Request * input,
  lift_control__action__Command_SendGoal_Request * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__Sequence__init(lift_control__action__Command_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Request__Sequence__fini(lift_control__action__Command_SendGoal_Request__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_SendGoal_Request__Sequence *
lift_control__action__Command_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Request__Sequence__destroy(lift_control__action__Command_SendGoal_Request__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__Sequence__are_equal(const lift_control__action__Command_SendGoal_Request__Sequence * lhs, const lift_control__action__Command_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Request__Sequence__copy(
  const lift_control__action__Command_SendGoal_Request__Sequence * input,
  lift_control__action__Command_SendGoal_Request__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_SendGoal_Response
 * )) before or use
 * lift_control__action__Command_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__init(lift_control__action__Command_SendGoal_Response * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Response__fini(lift_control__action__Command_SendGoal_Response * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_SendGoal_Response *
lift_control__action__Command_SendGoal_Response__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Response__destroy(lift_control__action__Command_SendGoal_Response * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__are_equal(const lift_control__action__Command_SendGoal_Response * lhs, const lift_control__action__Command_SendGoal_Response * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__copy(
  const lift_control__action__Command_SendGoal_Response * input,
  lift_control__action__Command_SendGoal_Response * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__Sequence__init(lift_control__action__Command_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Response__Sequence__fini(lift_control__action__Command_SendGoal_Response__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_SendGoal_Response__Sequence *
lift_control__action__Command_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_SendGoal_Response__Sequence__destroy(lift_control__action__Command_SendGoal_Response__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__Sequence__are_equal(const lift_control__action__Command_SendGoal_Response__Sequence * lhs, const lift_control__action__Command_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_SendGoal_Response__Sequence__copy(
  const lift_control__action__Command_SendGoal_Response__Sequence * input,
  lift_control__action__Command_SendGoal_Response__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_GetResult_Request
 * )) before or use
 * lift_control__action__Command_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__init(lift_control__action__Command_GetResult_Request * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Request__fini(lift_control__action__Command_GetResult_Request * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_GetResult_Request *
lift_control__action__Command_GetResult_Request__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Request__destroy(lift_control__action__Command_GetResult_Request * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__are_equal(const lift_control__action__Command_GetResult_Request * lhs, const lift_control__action__Command_GetResult_Request * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__copy(
  const lift_control__action__Command_GetResult_Request * input,
  lift_control__action__Command_GetResult_Request * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__Sequence__init(lift_control__action__Command_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Request__Sequence__fini(lift_control__action__Command_GetResult_Request__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_GetResult_Request__Sequence *
lift_control__action__Command_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Request__Sequence__destroy(lift_control__action__Command_GetResult_Request__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__Sequence__are_equal(const lift_control__action__Command_GetResult_Request__Sequence * lhs, const lift_control__action__Command_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Request__Sequence__copy(
  const lift_control__action__Command_GetResult_Request__Sequence * input,
  lift_control__action__Command_GetResult_Request__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_GetResult_Response
 * )) before or use
 * lift_control__action__Command_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__init(lift_control__action__Command_GetResult_Response * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Response__fini(lift_control__action__Command_GetResult_Response * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_GetResult_Response *
lift_control__action__Command_GetResult_Response__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Response__destroy(lift_control__action__Command_GetResult_Response * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__are_equal(const lift_control__action__Command_GetResult_Response * lhs, const lift_control__action__Command_GetResult_Response * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__copy(
  const lift_control__action__Command_GetResult_Response * input,
  lift_control__action__Command_GetResult_Response * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__Sequence__init(lift_control__action__Command_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Response__Sequence__fini(lift_control__action__Command_GetResult_Response__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_GetResult_Response__Sequence *
lift_control__action__Command_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_GetResult_Response__Sequence__destroy(lift_control__action__Command_GetResult_Response__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__Sequence__are_equal(const lift_control__action__Command_GetResult_Response__Sequence * lhs, const lift_control__action__Command_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_GetResult_Response__Sequence__copy(
  const lift_control__action__Command_GetResult_Response__Sequence * input,
  lift_control__action__Command_GetResult_Response__Sequence * output);

/// Initialize action/Command message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lift_control__action__Command_FeedbackMessage
 * )) before or use
 * lift_control__action__Command_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__init(lift_control__action__Command_FeedbackMessage * msg);

/// Finalize action/Command message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_FeedbackMessage__fini(lift_control__action__Command_FeedbackMessage * msg);

/// Create action/Command message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lift_control__action__Command_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_FeedbackMessage *
lift_control__action__Command_FeedbackMessage__create();

/// Destroy action/Command message.
/**
 * It calls
 * lift_control__action__Command_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_FeedbackMessage__destroy(lift_control__action__Command_FeedbackMessage * msg);

/// Check for action/Command message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__are_equal(const lift_control__action__Command_FeedbackMessage * lhs, const lift_control__action__Command_FeedbackMessage * rhs);

/// Copy a action/Command message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__copy(
  const lift_control__action__Command_FeedbackMessage * input,
  lift_control__action__Command_FeedbackMessage * output);

/// Initialize array of action/Command messages.
/**
 * It allocates the memory for the number of elements and calls
 * lift_control__action__Command_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__Sequence__init(lift_control__action__Command_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_FeedbackMessage__Sequence__fini(lift_control__action__Command_FeedbackMessage__Sequence * array);

/// Create array of action/Command messages.
/**
 * It allocates the memory for the array and calls
 * lift_control__action__Command_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
lift_control__action__Command_FeedbackMessage__Sequence *
lift_control__action__Command_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Command messages.
/**
 * It calls
 * lift_control__action__Command_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
void
lift_control__action__Command_FeedbackMessage__Sequence__destroy(lift_control__action__Command_FeedbackMessage__Sequence * array);

/// Check for action/Command message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__Sequence__are_equal(const lift_control__action__Command_FeedbackMessage__Sequence * lhs, const lift_control__action__Command_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Command messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lift_control
bool
lift_control__action__Command_FeedbackMessage__Sequence__copy(
  const lift_control__action__Command_FeedbackMessage__Sequence * input,
  lift_control__action__Command_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIFT_CONTROL__ACTION__DETAIL__COMMAND__FUNCTIONS_H_
