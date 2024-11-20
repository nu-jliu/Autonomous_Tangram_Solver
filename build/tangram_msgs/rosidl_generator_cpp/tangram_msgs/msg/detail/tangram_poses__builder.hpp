// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__BUILDER_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tangram_msgs/msg/detail/tangram_poses__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tangram_msgs
{

namespace msg
{

namespace builder
{

class Init_TangramPoses_poses
{
public:
  Init_TangramPoses_poses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tangram_msgs::msg::TangramPoses poses(::tangram_msgs::msg::TangramPoses::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tangram_msgs::msg::TangramPoses msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tangram_msgs::msg::TangramPoses>()
{
  return tangram_msgs::msg::builder::Init_TangramPoses_poses();
}

}  // namespace tangram_msgs

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__BUILDER_HPP_
