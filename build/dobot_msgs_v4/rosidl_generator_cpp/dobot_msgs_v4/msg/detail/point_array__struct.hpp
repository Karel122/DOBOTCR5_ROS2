// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
#define DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v4__msg__PointArray __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v4__msg__PointArray __declspec(deprecated)
#endif

namespace dobot_msgs_v4
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointArray_
{
  using Type = PointArray_<ContainerAllocator>;

  explicit PointArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PointArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _vrednosti_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _vrednosti_type vrednosti;

  // setters for named parameter idiom
  Type & set__vrednosti(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->vrednosti = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dobot_msgs_v4::msg::PointArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v4::msg::PointArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v4__msg__PointArray
    std::shared_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v4__msg__PointArray
    std::shared_ptr<dobot_msgs_v4::msg::PointArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointArray_ & other) const
  {
    if (this->vrednosti != other.vrednosti) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointArray_

// alias to use template instance with default allocator
using PointArray =
  dobot_msgs_v4::msg::PointArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
