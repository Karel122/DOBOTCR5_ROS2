// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dobot_msgs_v4:srv/SendFloat.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__STRUCT_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v4__srv__SendFloat_Request __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v4__srv__SendFloat_Request __declspec(deprecated)
#endif

namespace dobot_msgs_v4
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendFloat_Request_
{
  using Type = SendFloat_Request_<ContainerAllocator>;

  explicit SendFloat_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->zasuk = 0.0;
      this->hitrost = 0.0;
    }
  }

  explicit SendFloat_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->zasuk = 0.0;
      this->hitrost = 0.0;
    }
  }

  // field types and members
  using _zasuk_type =
    double;
  _zasuk_type zasuk;
  using _hitrost_type =
    double;
  _hitrost_type hitrost;

  // setters for named parameter idiom
  Type & set__zasuk(
    const double & _arg)
  {
    this->zasuk = _arg;
    return *this;
  }
  Type & set__hitrost(
    const double & _arg)
  {
    this->hitrost = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v4__srv__SendFloat_Request
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v4__srv__SendFloat_Request
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendFloat_Request_ & other) const
  {
    if (this->zasuk != other.zasuk) {
      return false;
    }
    if (this->hitrost != other.hitrost) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendFloat_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendFloat_Request_

// alias to use template instance with default allocator
using SendFloat_Request =
  dobot_msgs_v4::srv::SendFloat_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dobot_msgs_v4


#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v4__srv__SendFloat_Response __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v4__srv__SendFloat_Response __declspec(deprecated)
#endif

namespace dobot_msgs_v4
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendFloat_Response_
{
  using Type = SendFloat_Response_<ContainerAllocator>;

  explicit SendFloat_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SendFloat_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v4__srv__SendFloat_Response
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v4__srv__SendFloat_Response
    std::shared_ptr<dobot_msgs_v4::srv::SendFloat_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendFloat_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendFloat_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendFloat_Response_

// alias to use template instance with default allocator
using SendFloat_Response =
  dobot_msgs_v4::srv::SendFloat_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace dobot_msgs_v4
{

namespace srv
{

struct SendFloat
{
  using Request = dobot_msgs_v4::srv::SendFloat_Request;
  using Response = dobot_msgs_v4::srv::SendFloat_Response;
};

}  // namespace srv

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__STRUCT_HPP_
