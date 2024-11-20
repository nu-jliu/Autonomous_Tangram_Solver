// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tangram_msgs/msg/detail/tangram_poses__rosidl_typesupport_introspection_c.h"
#include "tangram_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tangram_msgs/msg/detail/tangram_poses__functions.h"
#include "tangram_msgs/msg/detail/tangram_poses__struct.h"


// Include directives for member types
// Member `poses`
#include "tangram_msgs/msg/tangram_pose.h"
// Member `poses`
#include "tangram_msgs/msg/detail/tangram_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tangram_msgs__msg__TangramPoses__init(message_memory);
}

void tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_fini_function(void * message_memory)
{
  tangram_msgs__msg__TangramPoses__fini(message_memory);
}

size_t tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__size_function__TangramPoses__poses(
  const void * untyped_member)
{
  const tangram_msgs__msg__TangramPose__Sequence * member =
    (const tangram_msgs__msg__TangramPose__Sequence *)(untyped_member);
  return member->size;
}

const void * tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_const_function__TangramPoses__poses(
  const void * untyped_member, size_t index)
{
  const tangram_msgs__msg__TangramPose__Sequence * member =
    (const tangram_msgs__msg__TangramPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_function__TangramPoses__poses(
  void * untyped_member, size_t index)
{
  tangram_msgs__msg__TangramPose__Sequence * member =
    (tangram_msgs__msg__TangramPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__fetch_function__TangramPoses__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const tangram_msgs__msg__TangramPose * item =
    ((const tangram_msgs__msg__TangramPose *)
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_const_function__TangramPoses__poses(untyped_member, index));
  tangram_msgs__msg__TangramPose * value =
    (tangram_msgs__msg__TangramPose *)(untyped_value);
  *value = *item;
}

void tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__assign_function__TangramPoses__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  tangram_msgs__msg__TangramPose * item =
    ((tangram_msgs__msg__TangramPose *)
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_function__TangramPoses__poses(untyped_member, index));
  const tangram_msgs__msg__TangramPose * value =
    (const tangram_msgs__msg__TangramPose *)(untyped_value);
  *item = *value;
}

bool tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__resize_function__TangramPoses__poses(
  void * untyped_member, size_t size)
{
  tangram_msgs__msg__TangramPose__Sequence * member =
    (tangram_msgs__msg__TangramPose__Sequence *)(untyped_member);
  tangram_msgs__msg__TangramPose__Sequence__fini(member);
  return tangram_msgs__msg__TangramPose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_member_array[1] = {
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tangram_msgs__msg__TangramPoses, poses),  // bytes offset in struct
    NULL,  // default value
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__size_function__TangramPoses__poses,  // size() function pointer
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_const_function__TangramPoses__poses,  // get_const(index) function pointer
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__get_function__TangramPoses__poses,  // get(index) function pointer
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__fetch_function__TangramPoses__poses,  // fetch(index, &value) function pointer
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__assign_function__TangramPoses__poses,  // assign(index, value) function pointer
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__resize_function__TangramPoses__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_members = {
  "tangram_msgs__msg",  // message namespace
  "TangramPoses",  // message name
  1,  // number of fields
  sizeof(tangram_msgs__msg__TangramPoses),
  tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_member_array,  // message members
  tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_init_function,  // function to initialize message memory (memory has to be allocated)
  tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_type_support_handle = {
  0,
  &tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_members,
  get_message_typesupport_handle_function,
  &tangram_msgs__msg__TangramPoses__get_type_hash,
  &tangram_msgs__msg__TangramPoses__get_type_description,
  &tangram_msgs__msg__TangramPoses__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tangram_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tangram_msgs, msg, TangramPoses)() {
  tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tangram_msgs, msg, TangramPose)();
  if (!tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_type_support_handle.typesupport_identifier) {
    tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tangram_msgs__msg__TangramPoses__rosidl_typesupport_introspection_c__TangramPoses_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
