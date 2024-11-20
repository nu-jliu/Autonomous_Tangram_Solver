// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tangram_msgs/msg/detail/tangram_poses__functions.h"
#include "tangram_msgs/msg/detail/tangram_poses__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tangram_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TangramPoses_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tangram_msgs::msg::TangramPoses(_init);
}

void TangramPoses_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tangram_msgs::msg::TangramPoses *>(message_memory);
  typed_message->~TangramPoses();
}

size_t size_function__TangramPoses__poses(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<tangram_msgs::msg::TangramPose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TangramPoses__poses(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<tangram_msgs::msg::TangramPose> *>(untyped_member);
  return &member[index];
}

void * get_function__TangramPoses__poses(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<tangram_msgs::msg::TangramPose> *>(untyped_member);
  return &member[index];
}

void fetch_function__TangramPoses__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const tangram_msgs::msg::TangramPose *>(
    get_const_function__TangramPoses__poses(untyped_member, index));
  auto & value = *reinterpret_cast<tangram_msgs::msg::TangramPose *>(untyped_value);
  value = item;
}

void assign_function__TangramPoses__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<tangram_msgs::msg::TangramPose *>(
    get_function__TangramPoses__poses(untyped_member, index));
  const auto & value = *reinterpret_cast<const tangram_msgs::msg::TangramPose *>(untyped_value);
  item = value;
}

void resize_function__TangramPoses__poses(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<tangram_msgs::msg::TangramPose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TangramPoses_message_member_array[1] = {
  {
    "poses",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<tangram_msgs::msg::TangramPose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tangram_msgs::msg::TangramPoses, poses),  // bytes offset in struct
    nullptr,  // default value
    size_function__TangramPoses__poses,  // size() function pointer
    get_const_function__TangramPoses__poses,  // get_const(index) function pointer
    get_function__TangramPoses__poses,  // get(index) function pointer
    fetch_function__TangramPoses__poses,  // fetch(index, &value) function pointer
    assign_function__TangramPoses__poses,  // assign(index, value) function pointer
    resize_function__TangramPoses__poses  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TangramPoses_message_members = {
  "tangram_msgs::msg",  // message namespace
  "TangramPoses",  // message name
  1,  // number of fields
  sizeof(tangram_msgs::msg::TangramPoses),
  TangramPoses_message_member_array,  // message members
  TangramPoses_init_function,  // function to initialize message memory (memory has to be allocated)
  TangramPoses_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TangramPoses_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TangramPoses_message_members,
  get_message_typesupport_handle_function,
  &tangram_msgs__msg__TangramPoses__get_type_hash,
  &tangram_msgs__msg__TangramPoses__get_type_description,
  &tangram_msgs__msg__TangramPoses__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tangram_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tangram_msgs::msg::TangramPoses>()
{
  return &::tangram_msgs::msg::rosidl_typesupport_introspection_cpp::TangramPoses_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tangram_msgs, msg, TangramPoses)() {
  return &::tangram_msgs::msg::rosidl_typesupport_introspection_cpp::TangramPoses_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
