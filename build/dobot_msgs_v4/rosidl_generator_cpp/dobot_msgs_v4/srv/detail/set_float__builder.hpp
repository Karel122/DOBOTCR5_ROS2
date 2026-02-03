// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SetFloat.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/set_float__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetFloat_Request>()
{
  return ::dobot_msgs_v4::srv::SetFloat_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetFloat_Response_data
{
public:
  Init_SetFloat_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetFloat_Response data(::dobot_msgs_v4::srv::SetFloat_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetFloat_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetFloat_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SetFloat_Response_data();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__BUILDER_HPP_
