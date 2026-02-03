// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
#define DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace msg
{

namespace builder
{

class Init_PointArray_vrednosti
{
public:
  Init_PointArray_vrednosti()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::msg::PointArray vrednosti(::dobot_msgs_v4::msg::PointArray::_vrednosti_type arg)
  {
    msg_.vrednosti = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::msg::PointArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::msg::PointArray>()
{
  return dobot_msgs_v4::msg::builder::Init_PointArray_vrednosti();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
