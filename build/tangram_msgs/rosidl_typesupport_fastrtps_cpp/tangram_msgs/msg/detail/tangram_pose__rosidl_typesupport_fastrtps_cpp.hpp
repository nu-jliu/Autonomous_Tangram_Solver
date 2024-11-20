// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tangram_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "tangram_msgs/msg/detail/tangram_pose__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace tangram_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tangram_msgs
cdr_serialize(
  const tangram_msgs::msg::TangramPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tangram_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  tangram_msgs::msg::TangramPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tangram_msgs
get_serialized_size(
  const tangram_msgs::msg::TangramPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tangram_msgs
max_serialized_size_TangramPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace tangram_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tangram_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tangram_msgs, msg, TangramPose)();

#ifdef __cplusplus
}
#endif

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
