// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tangram_msgs/msg/detail/tangram_pose__rosidl_typesupport_introspection_c.h"
#include "tangram_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tangram_msgs/msg/detail/tangram_pose__functions.h"
#include "tangram_msgs/msg/detail/tangram_pose__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose2_d.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tangram_msgs__msg__TangramPose__init(message_memory);
}

void tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_fini_function(void * message_memory)
{
  tangram_msgs__msg__TangramPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_member_array[3] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tangram_msgs__msg__TangramPose, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flip",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tangram_msgs__msg__TangramPose, flip),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tangram_msgs__msg__TangramPose, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_members = {
  "tangram_msgs__msg",  // message namespace
  "TangramPose",  // message name
  3,  // number of fields
  sizeof(tangram_msgs__msg__TangramPose),
  tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_member_array,  // message members
  tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_init_function,  // function to initialize message memory (memory has to be allocated)
  tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_type_support_handle = {
  0,
  &tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_members,
  get_message_typesupport_handle_function,
  &tangram_msgs__msg__TangramPose__get_type_hash,
  &tangram_msgs__msg__TangramPose__get_type_description,
  &tangram_msgs__msg__TangramPose__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tangram_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tangram_msgs, msg, TangramPose)() {
  tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose2D)();
  if (!tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_type_support_handle.typesupport_identifier) {
    tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tangram_msgs__msg__TangramPose__rosidl_typesupport_introspection_c__TangramPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
