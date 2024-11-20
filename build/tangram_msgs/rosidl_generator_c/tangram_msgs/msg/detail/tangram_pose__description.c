// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#include "tangram_msgs/msg/detail/tangram_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tangram_msgs
const rosidl_type_hash_t *
tangram_msgs__msg__TangramPose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0x7f, 0x43, 0x7c, 0xde, 0xdc, 0x5f, 0xe9,
      0xc5, 0x38, 0x32, 0x9d, 0x97, 0x54, 0x2d, 0xf2,
      0xee, 0x85, 0x6c, 0xb8, 0xe6, 0xe7, 0xa7, 0xde,
      0x7c, 0x16, 0x1e, 0xc4, 0xd4, 0xb0, 0x6d, 0x5f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Pose2D__EXPECTED_HASH = {1, {
    0xd6, 0x8e, 0xfa, 0x5b, 0x46, 0xe7, 0x0f, 0x7b,
    0x16, 0xca, 0x23, 0x08, 0x54, 0x74, 0xfd, 0xac,
    0x5a, 0x44, 0xb6, 0x38, 0x78, 0x3e, 0xc4, 0x2f,
    0x66, 0x1d, 0xa6, 0x4d, 0xa4, 0x72, 0x4c, 0xcc,
  }};
#endif

static char tangram_msgs__msg__TangramPose__TYPE_NAME[] = "tangram_msgs/msg/TangramPose";
static char geometry_msgs__msg__Pose2D__TYPE_NAME[] = "geometry_msgs/msg/Pose2D";

// Define type names, field names, and default values
static char tangram_msgs__msg__TangramPose__FIELD_NAME__pose[] = "pose";
static char tangram_msgs__msg__TangramPose__FIELD_NAME__flip[] = "flip";
static char tangram_msgs__msg__TangramPose__FIELD_NAME__type[] = "type";

static rosidl_runtime_c__type_description__Field tangram_msgs__msg__TangramPose__FIELDS[] = {
  {
    {tangram_msgs__msg__TangramPose__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {tangram_msgs__msg__TangramPose__FIELD_NAME__flip, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tangram_msgs__msg__TangramPose__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tangram_msgs__msg__TangramPose__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tangram_msgs__msg__TangramPose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tangram_msgs__msg__TangramPose__TYPE_NAME, 28, 28},
      {tangram_msgs__msg__TangramPose__FIELDS, 3, 3},
    },
    {tangram_msgs__msg__TangramPose__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 BIG_TRIANGLE=1\n"
  "int32 MEDIUM_TRIANGLE=2\n"
  "int32 SMALL_TRIANGLE=3\n"
  "int32 PARALELOGRAM=4\n"
  "int32 SQUARE=5\n"
  "\n"
  "geometry_msgs/Pose2D pose\n"
  "bool flip\n"
  "int32 type";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tangram_msgs__msg__TangramPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tangram_msgs__msg__TangramPose__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tangram_msgs__msg__TangramPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tangram_msgs__msg__TangramPose__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
