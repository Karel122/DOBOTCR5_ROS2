// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dobot_msgs_v4:srv/SetFloat.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v4__srv__SetFloat_Request __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v4__srv__SetFloat_Request __declspec(deprecated)
#endif

namespace dobot_msgs_v4
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetFloat_Request_
{
  using Type = SetFloat_Request_<ContainerAllocator>;

  explicit SetFloat_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetFloat_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v4__srv__SetFloat_Request
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v4__srv__SetFloat_Request
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetFloat_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetFloat_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetFloat_Request_

// alias to use template instance with default allocator
using SetFloat_Request =
  dobot_msgs_v4::srv::SetFloat_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dobot_msgs_v4


// Include directives for member types
// Member 'data'
#include "dobot_msgs_v4/msg/detail/point_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v4__srv__SetFloat_Response __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v4__srv__SetFloat_Response __declspec(deprecated)
#endif

namespace dobot_msgs_v4
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetFloat_Response_
{
  using Type = SetFloat_Response_<ContainerAllocator>;

  explicit SetFloat_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetFloat_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dobot_msgs_v4::msg::PointArray_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v4__srv__SetFloat_Response
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v4__srv__SetFloat_Response
    std::shared_ptr<dobot_msgs_v4::srv::SetFloat_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetFloat_Response_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetFloat_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetFloat_Response_

// alias to use template instance with default allocator
using SetFloat_Response =
  dobot_msgs_v4::srv::SetFloat_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace dobot_msgs_v4
{

namespace srv
{

struct SetFloat
{
  using Request = dobot_msgs_v4::srv::SetFloat_Request;
  using Response = dobot_msgs_v4::srv::SetFloat_Response;
};

}  // namespace srv

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_HPP_
