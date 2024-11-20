// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice
#include "tangram_msgs/msg/detail/tangram_poses__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `poses`
#include "tangram_msgs/msg/detail/tangram_pose__functions.h"

bool
tangram_msgs__msg__TangramPoses__init(tangram_msgs__msg__TangramPoses * msg)
{
  if (!msg) {
    return false;
  }
  // poses
  if (!tangram_msgs__msg__TangramPose__Sequence__init(&msg->poses, 0)) {
    tangram_msgs__msg__TangramPoses__fini(msg);
    return false;
  }
  return true;
}

void
tangram_msgs__msg__TangramPoses__fini(tangram_msgs__msg__TangramPoses * msg)
{
  if (!msg) {
    return;
  }
  // poses
  tangram_msgs__msg__TangramPose__Sequence__fini(&msg->poses);
}

bool
tangram_msgs__msg__TangramPoses__are_equal(const tangram_msgs__msg__TangramPoses * lhs, const tangram_msgs__msg__TangramPoses * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // poses
  if (!tangram_msgs__msg__TangramPose__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
tangram_msgs__msg__TangramPoses__copy(
  const tangram_msgs__msg__TangramPoses * input,
  tangram_msgs__msg__TangramPoses * output)
{
  if (!input || !output) {
    return false;
  }
  // poses
  if (!tangram_msgs__msg__TangramPose__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

tangram_msgs__msg__TangramPoses *
tangram_msgs__msg__TangramPoses__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPoses * msg = (tangram_msgs__msg__TangramPoses *)allocator.allocate(sizeof(tangram_msgs__msg__TangramPoses), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tangram_msgs__msg__TangramPoses));
  bool success = tangram_msgs__msg__TangramPoses__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tangram_msgs__msg__TangramPoses__destroy(tangram_msgs__msg__TangramPoses * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tangram_msgs__msg__TangramPoses__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tangram_msgs__msg__TangramPoses__Sequence__init(tangram_msgs__msg__TangramPoses__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPoses * data = NULL;

  if (size) {
    data = (tangram_msgs__msg__TangramPoses *)allocator.zero_allocate(size, sizeof(tangram_msgs__msg__TangramPoses), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tangram_msgs__msg__TangramPoses__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tangram_msgs__msg__TangramPoses__fini(&data[i - 1]);
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
tangram_msgs__msg__TangramPoses__Sequence__fini(tangram_msgs__msg__TangramPoses__Sequence * array)
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
      tangram_msgs__msg__TangramPoses__fini(&array->data[i]);
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

tangram_msgs__msg__TangramPoses__Sequence *
tangram_msgs__msg__TangramPoses__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tangram_msgs__msg__TangramPoses__Sequence * array = (tangram_msgs__msg__TangramPoses__Sequence *)allocator.allocate(sizeof(tangram_msgs__msg__TangramPoses__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tangram_msgs__msg__TangramPoses__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tangram_msgs__msg__TangramPoses__Sequence__destroy(tangram_msgs__msg__TangramPoses__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tangram_msgs__msg__TangramPoses__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tangram_msgs__msg__TangramPoses__Sequence__are_equal(const tangram_msgs__msg__TangramPoses__Sequence * lhs, const tangram_msgs__msg__TangramPoses__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tangram_msgs__msg__TangramPoses__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tangram_msgs__msg__TangramPoses__Sequence__copy(
  const tangram_msgs__msg__TangramPoses__Sequence * input,
  tangram_msgs__msg__TangramPoses__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tangram_msgs__msg__TangramPoses);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tangram_msgs__msg__TangramPoses * data =
      (tangram_msgs__msg__TangramPoses *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tangram_msgs__msg__TangramPoses__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tangram_msgs__msg__TangramPoses__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tangram_msgs__msg__TangramPoses__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
