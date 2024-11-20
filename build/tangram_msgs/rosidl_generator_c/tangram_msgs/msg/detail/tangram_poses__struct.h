// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_H_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'poses'
#include "tangram_msgs/msg/detail/tangram_pose__struct.h"

/// Struct defined in msg/TangramPoses in the package tangram_msgs.
typedef struct tangram_msgs__msg__TangramPoses
{
  tangram_msgs__msg__TangramPose__Sequence poses;
} tangram_msgs__msg__TangramPoses;

// Struct for a sequence of tangram_msgs__msg__TangramPoses.
typedef struct tangram_msgs__msg__TangramPoses__Sequence
{
  tangram_msgs__msg__TangramPoses * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tangram_msgs__msg__TangramPoses__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_H_
