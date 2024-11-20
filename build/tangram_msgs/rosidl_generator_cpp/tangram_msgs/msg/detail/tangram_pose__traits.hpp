// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__TRAITS_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tangram_msgs/msg/detail/tangram_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace tangram_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TangramPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: flip
  {
    out << "flip: ";
    rosidl_generator_traits::value_to_yaml(msg.flip, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TangramPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: flip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flip: ";
    rosidl_generator_traits::value_to_yaml(msg.flip, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TangramPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tangram_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tangram_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tangram_msgs::msg::TangramPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  tangram_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tangram_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tangram_msgs::msg::TangramPose & msg)
{
  return tangram_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tangram_msgs::msg::TangramPose>()
{
  return "tangram_msgs::msg::TangramPose";
}

template<>
inline const char * name<tangram_msgs::msg::TangramPose>()
{
  return "tangram_msgs/msg/TangramPose";
}

template<>
struct has_fixed_size<tangram_msgs::msg::TangramPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<tangram_msgs::msg::TangramPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<tangram_msgs::msg::TangramPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__TRAITS_HPP_
