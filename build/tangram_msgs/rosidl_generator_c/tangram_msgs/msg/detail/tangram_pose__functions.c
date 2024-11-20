// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice
#include "tangram_msgs/msg/detail/tangram_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
tangram_msgs__msg__TangramPose__init(tangram_msgs__msg__TangramPose * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose2D__init(&msg->pose)) {
    tangram_msgs__msg__TangramPose__fini(msg);
    return false;
  }
  // flip
  // type
  return true;
}

void
tangram_msgs__msg__TangramPose__fini(tangram_msgs__msg__TangramPose * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose2D__fini(&msg->pose);
  // flip
  // type
}

bool
tangram_msgs__msg__TangramPose__are_equal(const tangram_msgs__msg__TangramPose * lhs, const tangram_msgs__msg__TangramPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // flip
  if (lhs->flip != rhs->flip) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
tangram_msgs__msg__TangramPose__copy(
  const tangram_msgs__msg__TangramPose * input,
  tangram_msgs__msg__TangramPose * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // flip
  output->flip = input->flip;
  // type
  output->type = input->type;
  return true;
}

tangram_msgs__msg__TangramPose *
tangram_msgs__msg__TangramPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPose * msg = (tangram_msgs__msg__TangramPose *)allocator.allocate(sizeof(tangram_msgs__msg__TangramPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tangram_msgs__msg__TangramPose));
  bool success = tangram_msgs__msg__TangramPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tangram_msgs__msg__TangramPose__destroy(tangram_msgs__msg__TangramPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tangram_msgs__msg__TangramPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tangram_msgs__msg__TangramPose__Sequence__init(tangram_msgs__msg__TangramPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPose * data = NULL;

  if (size) {
    data = (tangram_msgs__msg__TangramPose *)allocator.zero_allocate(size, sizeof(tangram_msgs__msg__TangramPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tangram_msgs__msg__TangramPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tangram_msgs__msg__TangramPose__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tangram_msgs__msg__TangramPose__Sequence__fini(tangram_msgs__msg__TangramPose__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tangram_msgs__msg__TangramPose__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tangram_msgs__msg__TangramPose__Sequence *
tangram_msgs__msg__TangramPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPose__Sequence * array = (tangram_msgs__msg__TangramPose__Sequence *)allocator.allocate(sizeof(tangram_msgs__msg__TangramPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tangram_msgs__msg__TangramPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tangram_msgs__msg__TangramPose__Sequence__destroy(tangram_msgs__msg__TangramPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tangram_msgs__msg__TangramPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tangram_msgs__msg__TangramPose__Sequence__are_equal(const tangram_msgs__msg__TangramPose__Sequence * lhs, const tangram_msgs__msg__TangramPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tangram_msgs__msg__TangramPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tangram_msgs__msg__TangramPose__Sequence__copy(
  const tangram_msgs__msg__TangramPose__Sequence * input,
  tangram_msgs__msg__TangramPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tangram_msgs__msg__TangramPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tangram_msgs__msg__TangramPose * data =
      (tangram_msgs__msg__TangramPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tangram_msgs__msg__TangramPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tangram_msgs__msg__TangramPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tangram_msgs__msg__TangramPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
