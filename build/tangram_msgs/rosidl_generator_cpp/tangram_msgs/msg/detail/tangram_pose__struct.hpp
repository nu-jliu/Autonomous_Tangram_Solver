// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tangram_msgs:msg/TangramPose.idl
// generated code does not contain a copyright notice

#ifndef TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_HPP_
#define TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tangram_msgs__msg__TangramPose __attribute__((deprecated))
#else
# define DEPRECATED__tangram_msgs__msg__TangramPose __declspec(deprecated)
#endif

namespace tangram_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TangramPose_
{
  using Type = TangramPose_<ContainerAllocator>;

  explicit TangramPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flip = false;
      this->type = 0l;
    }
  }

  explicit TangramPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flip = false;
      this->type = 0l;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _pose_type pose;
  using _flip_type =
    bool;
  _flip_type flip;
  using _type_type =
    int32_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__flip(
    const bool & _arg)
  {
    this->flip = _arg;
    return *this;
  }
  Type & set__type(
    const int32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t BIG_TRIANGLE =
    1;
  static constexpr int32_t MEDIUM_TRIANGLE =
    2;
  static constexpr int32_t SMALL_TRIANGLE =
    3;
  static constexpr int32_t PARALELOGRAM =
    4;
  static constexpr int32_t SQUARE =
    5;

  // pointer types
  using RawPtr =
    tangram_msgs::msg::TangramPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const tangram_msgs::msg::TangramPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tangram_msgs::msg::TangramPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tangram_msgs::msg::TangramPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tangram_msgs__msg__TangramPose
    std::shared_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tangram_msgs__msg__TangramPose
    std::shared_ptr<tangram_msgs::msg::TangramPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TangramPose_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->flip != other.flip) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const TangramPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TangramPose_

// alias to use template instance with default allocator
using TangramPose =
  tangram_msgs::msg::TangramPose_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t TangramPose_<ContainerAllocator>::BIG_TRIANGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t TangramPose_<ContainerAllocator>::MEDIUM_TRIANGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t TangramPose_<ContainerAllocator>::SMALL_TRIANGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t TangramPose_<ContainerAllocator>::PARALELOGRAM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t TangramPose_<ContainerAllocator>::SQUARE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace tangram_msgs

#endif  // TANGRAM_MSGS__MSG__DETAIL__TANGRAM_POSE__STRUCT_HPP_
