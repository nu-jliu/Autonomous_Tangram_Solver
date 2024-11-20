// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__TRAITS_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tangram_msgs/msg/detail/tangram_poses__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'poses'
#include "tangram_msgs/msg/detail/tangram_pose__traits.hpp"

namespace tangram_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TangramPoses & msg,
  std::ostream & out)
{
  out << "{";
  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TangramPoses & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TangramPoses & msg, bool use_flow_style = false)
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
  const tangram_msgs::msg::TangramPoses & msg,
  std::ostream & out, size_t indentation = 0)
{
  tangram_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tangram_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tangram_msgs::msg::TangramPoses & msg)
{
  return tangram_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tangram_msgs::msg::TangramPoses>()
{
  return "tangram_msgs::msg::TangramPoses";
}

template<>
inline const char * name<tangram_msgs::msg::TangramPoses>()
{
  return "tangram_msgs/msg/TangramPoses";
}

template<>
struct has_fixed_size<tangram_msgs::msg::TangramPoses>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tangram_msgs::msg::TangramPoses>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tangram_msgs::msg::TangramPoses>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__TRAITS_HPP_
