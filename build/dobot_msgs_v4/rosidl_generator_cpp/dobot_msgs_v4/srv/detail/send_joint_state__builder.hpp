// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SendJointState.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/send_joint_state__struct.hpp"
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
auto build<::dobot_msgs_v4::srv::SendJointState_Request>()
{
  return ::dobot_msgs_v4::srv::SendJointState_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SendJointState_Response_f
{
public:
  explicit Init_SendJointState_Response_f(::dobot_msgs_v4::srv::SendJointState_Response & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SendJointState_Response f(::dobot_msgs_v4::srv::SendJointState_Response::_f_type arg)
  {
    msg_.f = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

class Init_SendJointState_Response_e
{
public:
  explicit Init_SendJointState_Response_e(::dobot_msgs_v4::srv::SendJointState_Response & msg)
  : msg_(msg)
  {}
  Init_SendJointState_Response_f e(::dobot_msgs_v4::srv::SendJointState_Response::_e_type arg)
  {
    msg_.e = std::move(arg);
    return Init_SendJointState_Response_f(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

class Init_SendJointState_Response_d
{
public:
  explicit Init_SendJointState_Response_d(::dobot_msgs_v4::srv::SendJointState_Response & msg)
  : msg_(msg)
  {}
  Init_SendJointState_Response_e d(::dobot_msgs_v4::srv::SendJointState_Response::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_SendJointState_Response_e(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

class Init_SendJointState_Response_c
{
public:
  explicit Init_SendJointState_Response_c(::dobot_msgs_v4::srv::SendJointState_Response & msg)
  : msg_(msg)
  {}
  Init_SendJointState_Response_d c(::dobot_msgs_v4::srv::SendJointState_Response::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_SendJointState_Response_d(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

class Init_SendJointState_Response_b
{
public:
  explicit Init_SendJointState_Response_b(::dobot_msgs_v4::srv::SendJointState_Response & msg)
  : msg_(msg)
  {}
  Init_SendJointState_Response_c b(::dobot_msgs_v4::srv::SendJointState_Response::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_SendJointState_Response_c(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

class Init_SendJointState_Response_a
{
public:
  Init_SendJointState_Response_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendJointState_Response_b a(::dobot_msgs_v4::srv::SendJointState_Response::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_SendJointState_Response_b(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SendJointState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SendJointState_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SendJointState_Response_a();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__BUILDER_HPP_
