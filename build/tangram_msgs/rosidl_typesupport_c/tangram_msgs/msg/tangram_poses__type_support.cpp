// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "tangram_msgs/msg/detail/tangram_poses__struct.h"
#include "tangram_msgs/msg/detail/tangram_poses__type_support.h"
#include "tangram_msgs/msg/detail/tangram_poses__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace tangram_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _TangramPoses_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TangramPoses_type_support_ids_t;

static const _TangramPoses_type_support_ids_t _TangramPoses_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TangramPoses_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TangramPoses_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TangramPoses_type_support_symbol_names_t _TangramPoses_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tangram_msgs, msg, TangramPoses)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tangram_msgs, msg, TangramPoses)),
  }
};

typedef struct _TangramPoses_type_support_data_t
{
  void * data[2];
} _TangramPoses_type_support_data_t;

static _TangramPoses_type_support_data_t _TangramPoses_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TangramPoses_message_typesupport_map = {
  2,
  "tangram_msgs",
  &_TangramPoses_message_typesupport_ids.typesupport_identifier[0],
  &_TangramPoses_message_typesupport_symbol_names.symbol_name[0],
  &_TangramPoses_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TangramPoses_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TangramPoses_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &tangram_msgs__msg__TangramPoses__get_type_hash,
  &tangram_msgs__msg__TangramPoses__get_type_description,
  &tangram_msgs__msg__TangramPoses__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace tangram_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, tangram_msgs, msg, TangramPoses)() {
  return &::tangram_msgs::msg::rosidl_typesupport_c::TangramPoses_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
