// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__FUNCTIONS_H_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "tangram_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tangram_msgs/msg/detail/tangram_poses__struct.h"

/// Initialize msg/TangramPoses message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tangram_msgs__msg__TangramPoses
 * )) before or use
 * tangram_msgs__msg__TangramPoses__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__init(tangram_msgs__msg__TangramPoses * msg);

/// Finalize msg/TangramPoses message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
void
tangram_msgs__msg__TangramPoses__fini(tangram_msgs__msg__TangramPoses * msg);

/// Create msg/TangramPoses message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tangram_msgs__msg__TangramPoses__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
tangram_msgs__msg__TangramPoses *
tangram_msgs__msg__TangramPoses__create();

/// Destroy msg/TangramPoses message.
/**
 * It calls
 * tangram_msgs__msg__TangramPoses__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
void
tangram_msgs__msg__TangramPoses__destroy(tangram_msgs__msg__TangramPoses * msg);

/// Check for msg/TangramPoses message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__are_equal(const tangram_msgs__msg__TangramPoses * lhs, const tangram_msgs__msg__TangramPoses * rhs);

/// Copy a msg/TangramPoses message.
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
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__copy(
  const tangram_msgs__msg__TangramPoses * input,
  tangram_msgs__msg__TangramPoses * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
const rosidl_type_hash_t *
tangram_msgs__msg__TangramPoses__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
const rosidl_runtime_c__type_description__TypeDescription *
tangram_msgs__msg__TangramPoses__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
const rosidl_runtime_c__type_description__TypeSource *
tangram_msgs__msg__TangramPoses__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tangram_msgs__msg__TangramPoses__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/TangramPoses messages.
/**
 * It allocates the memory for the number of elements and calls
 * tangram_msgs__msg__TangramPoses__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__Sequence__init(tangram_msgs__msg__TangramPoses__Sequence * array, size_t size);

/// Finalize array of msg/TangramPoses messages.
/**
 * It calls
 * tangram_msgs__msg__TangramPoses__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
void
tangram_msgs__msg__TangramPoses__Sequence__fini(tangram_msgs__msg__TangramPoses__Sequence * array);

/// Create array of msg/TangramPoses messages.
/**
 * It allocates the memory for the array and calls
 * tangram_msgs__msg__TangramPoses__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
tangram_msgs__msg__TangramPoses__Sequence *
tangram_msgs__msg__TangramPoses__Sequence__create(size_t size);

/// Destroy array of msg/TangramPoses messages.
/**
 * It calls
 * tangram_msgs__msg__TangramPoses__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
void
tangram_msgs__msg__TangramPoses__Sequence__destroy(tangram_msgs__msg__TangramPoses__Sequence * array);

/// Check for msg/TangramPoses message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__Sequence__are_equal(const tangram_msgs__msg__TangramPoses__Sequence * lhs, const tangram_msgs__msg__TangramPoses__Sequence * rhs);

/// Copy an array of msg/TangramPoses messages.
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
ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
bool
tangram_msgs__msg__TangramPoses__Sequence__copy(
  const tangram_msgs__msg__TangramPoses__Sequence * input,
  tangram_msgs__msg__TangramPoses__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__FUNCTIONS_H_
