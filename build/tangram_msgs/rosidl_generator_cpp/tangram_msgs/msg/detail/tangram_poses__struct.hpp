// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tangram_msgs:msg/TangramPoses.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'poses'
#include "tangram_msgs/msg/detail/tangram_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tangram_msgs__msg__TangramPoses __attribute__((deprecated))
#else
# define DEPRECATED__tangram_msgs__msg__TangramPoses __declspec(deprecated)
#endif

namespace tangram_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TangramPoses_
{
  using Type = TangramPoses_<ContainerAllocator>;

  explicit TangramPoses_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TangramPoses_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _poses_type =
    std::vector<tangram_msgs::msg::TangramPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tangram_msgs::msg::TangramPose_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__poses(
    const std::vector<tangram_msgs::msg::TangramPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tangram_msgs::msg::TangramPose_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tangram_msgs::msg::TangramPoses_<ContainerAllocator> *;
  using ConstRawPtr =
    const tangram_msgs::msg::TangramPoses_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tangram_msgs::msg::TangramPoses_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tangram_msgs::msg::TangramPoses_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tangram_msgs__msg__TangramPoses
    std::shared_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tangram_msgs__msg__TangramPoses
    std::shared_ptr<tangram_msgs::msg::TangramPoses_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TangramPoses_ & other) const
  {
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const TangramPoses_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TangramPoses_

// alias to use template instance with default allocator
using TangramPoses =
  tangram_msgs::msg::TangramPoses_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tangram_msgs

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSES__STRUCT_HPP_
