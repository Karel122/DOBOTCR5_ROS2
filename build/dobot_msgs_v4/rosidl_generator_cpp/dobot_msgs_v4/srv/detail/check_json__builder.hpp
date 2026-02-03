// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/CheckJson.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__CHECK_JSON__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__CHECK_JSON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/check_json__struct.hpp"
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
auto build<::dobot_msgs_v4::srv::CheckJson_Request>()
{
  return ::dobot_msgs_v4::srv::CheckJson_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CheckJson_Response_vrednost
{
public:
  Init_CheckJson_Response_vrednost()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::CheckJson_Response vrednost(::dobot_msgs_v4::srv::CheckJson_Response::_vrednost_type arg)
  {
    msg_.vrednost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CheckJson_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CheckJson_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_CheckJson_Response_vrednost();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__CHECK_JSON__BUILDER_HPP_
