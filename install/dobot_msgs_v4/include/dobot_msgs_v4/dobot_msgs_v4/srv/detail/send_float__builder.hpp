// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SendFloat.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/send_float__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SendFloat_Request_hitrost
{
public:
  explicit Init_SendFloat_Request_hitrost(::dobot_msgs_v4::srv::SendFloat_Request & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SendFloat_Request hitrost(::dobot_msgs_v4::srv::SendFloat_Request::_hitrost_type arg)
  {
    msg_.hitrost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendFloat_Request msg_;
};

class Init_SendFloat_Request_zasuk
{
public:
  Init_SendFloat_Request_zasuk()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendFloat_Request_hitrost zasuk(::dobot_msgs_v4::srv::SendFloat_Request::_zasuk_type arg)
  {
    msg_.zasuk = std::move(arg);
    return Init_SendFloat_Request_hitrost(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendFloat_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SendFloat_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_SendFloat_Request_zasuk();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SendFloat_Response>()
{
  return ::dobot_msgs_v4::srv::SendFloat_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SEND_FLOAT__BUILDER_HPP_
