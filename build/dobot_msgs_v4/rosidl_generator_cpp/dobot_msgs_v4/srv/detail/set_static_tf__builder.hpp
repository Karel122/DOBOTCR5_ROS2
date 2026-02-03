// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SetStaticTF.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/set_static_tf__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetStaticTF_Request_child_frame
{
public:
  explicit Init_SetStaticTF_Request_child_frame(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SetStaticTF_Request child_frame(::dobot_msgs_v4::srv::SetStaticTF_Request::_child_frame_type arg)
  {
    msg_.child_frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_parent_frame
{
public:
  explicit Init_SetStaticTF_Request_parent_frame(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_child_frame parent_frame(::dobot_msgs_v4::srv::SetStaticTF_Request::_parent_frame_type arg)
  {
    msg_.parent_frame = std::move(arg);
    return Init_SetStaticTF_Request_child_frame(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_yaw
{
public:
  explicit Init_SetStaticTF_Request_yaw(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_parent_frame yaw(::dobot_msgs_v4::srv::SetStaticTF_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SetStaticTF_Request_parent_frame(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_pitch
{
public:
  explicit Init_SetStaticTF_Request_pitch(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_yaw pitch(::dobot_msgs_v4::srv::SetStaticTF_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SetStaticTF_Request_yaw(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_roll
{
public:
  explicit Init_SetStaticTF_Request_roll(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_pitch roll(::dobot_msgs_v4::srv::SetStaticTF_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_SetStaticTF_Request_pitch(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_z
{
public:
  explicit Init_SetStaticTF_Request_z(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_roll z(::dobot_msgs_v4::srv::SetStaticTF_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SetStaticTF_Request_roll(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_y
{
public:
  explicit Init_SetStaticTF_Request_y(::dobot_msgs_v4::srv::SetStaticTF_Request & msg)
  : msg_(msg)
  {}
  Init_SetStaticTF_Request_z y(::dobot_msgs_v4::srv::SetStaticTF_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SetStaticTF_Request_z(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

class Init_SetStaticTF_Request_x
{
public:
  Init_SetStaticTF_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetStaticTF_Request_y x(::dobot_msgs_v4::srv::SetStaticTF_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetStaticTF_Request_y(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetStaticTF_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_SetStaticTF_Request_x();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetStaticTF_Response_success
{
public:
  Init_SetStaticTF_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetStaticTF_Response success(::dobot_msgs_v4::srv::SetStaticTF_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetStaticTF_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetStaticTF_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SetStaticTF_Response_success();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__BUILDER_HPP_
