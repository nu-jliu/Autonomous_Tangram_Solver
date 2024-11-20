// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__BUILDER_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tangram_msgs/msg/detail/tangram_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tangram_msgs
{

namespace msg
{

namespace builder
{

class Init_TangramPose_type
{
public:
  explicit Init_TangramPose_type(::tangram_msgs::msg::TangramPose & msg)
  : msg_(msg)
  {}
  ::tangram_msgs::msg::TangramPose type(::tangram_msgs::msg::TangramPose::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tangram_msgs::msg::TangramPose msg_;
};

class Init_TangramPose_flip
{
public:
  explicit Init_TangramPose_flip(::tangram_msgs::msg::TangramPose & msg)
  : msg_(msg)
  {}
  Init_TangramPose_type flip(::tangram_msgs::msg::TangramPose::_flip_type arg)
  {
    msg_.flip = std::move(arg);
    return Init_TangramPose_type(msg_);
  }

private:
  ::tangram_msgs::msg::TangramPose msg_;
};

class Init_TangramPose_pose
{
public:
  Init_TangramPose_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TangramPose_flip pose(::tangram_msgs::msg::TangramPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_TangramPose_flip(msg_);
  }

private:
  ::tangram_msgs::msg::TangramPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tangram_msgs::msg::TangramPose>()
{
  return tangram_msgs::msg::builder::Init_TangramPose_pose();
}

}  // namespace tangram_msgs

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__BUILDER_HPP_
