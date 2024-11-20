// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_H_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'BIG_TRIANGLE'.
enum
{
  tangram_msgs__msg__TangramPose__BIG_TRIANGLE = 1l
};

/// Constant 'MEDIUM_TRIANGLE'.
enum
{
  tangram_msgs__msg__TangramPose__MEDIUM_TRIANGLE = 2l
};

/// Constant 'SMALL_TRIANGLE'.
enum
{
  tangram_msgs__msg__TangramPose__SMALL_TRIANGLE = 3l
};

/// Constant 'PARALELOGRAM'.
enum
{
  tangram_msgs__msg__TangramPose__PARALELOGRAM = 4l
};

/// Constant 'SQUARE'.
enum
{
  tangram_msgs__msg__TangramPose__SQUARE = 5l
};

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

/// Struct defined in msg/TangramPose in the package tangram_msgs.
typedef struct tangram_msgs__msg__TangramPose
{
  geometry_msgs__msg__Pose2D pose;
  bool flip;
  int32_t type;
} tangram_msgs__msg__TangramPose;

// Struct for a sequence of tangram_msgs__msg__TangramPose.
typedef struct tangram_msgs__msg__TangramPose__Sequence
{
  tangram_msgs__msg__TangramPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tangram_msgs__msg__TangramPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_H_
