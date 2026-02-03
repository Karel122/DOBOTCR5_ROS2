// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
#define DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dobot_msgs_v4/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dobot_msgs_v4
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: vrednosti
  {
    if (msg.vrednosti.size() == 0) {
      out << "vrednosti: []";
    } else {
      out << "vrednosti: [";
      size_t pending_items = msg.vrednosti.size();
      for (auto item : msg.vrednosti) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vrednosti
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vrednosti.size() == 0) {
      out << "vrednosti: []\n";
    } else {
      out << "vrednosti:\n";
      for (auto item : msg.vrednosti) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dobot_msgs_v4

namespace rosidl_generator_traits
{

[[deprecated("use dobot_msgs_v4::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dobot_msgs_v4::msg::PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::msg::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::msg::PointArray & msg)
{
  return dobot_msgs_v4::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::msg::PointArray>()
{
  return "dobot_msgs_v4::msg::PointArray";
}

template<>
inline const char * name<dobot_msgs_v4::msg::PointArray>()
{
  return "dobot_msgs_v4/msg/PointArray";
}

template<>
struct has_fixed_size<dobot_msgs_v4::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dobot_msgs_v4::msg::PointArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dobot_msgs_v4::msg::PointArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
