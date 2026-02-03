// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dobot_msgs_v4:srv/SendJointState.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__TRAITS_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dobot_msgs_v4/srv/detail/send_joint_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendJointState_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendJointState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendJointState_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_generator_traits
{

[[deprecated("use dobot_msgs_v4::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dobot_msgs_v4::srv::SendJointState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::SendJointState_Request & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::SendJointState_Request>()
{
  return "dobot_msgs_v4::srv::SendJointState_Request";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SendJointState_Request>()
{
  return "dobot_msgs_v4/srv/SendJointState_Request";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SendJointState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SendJointState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dobot_msgs_v4::srv::SendJointState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const SendJointState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << ", ";
  }

  // member: e
  {
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << ", ";
  }

  // member: f
  {
    out << "f: ";
    rosidl_generator_traits::value_to_yaml(msg.f, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendJointState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << "\n";
  }

  // member: f
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f: ";
    rosidl_generator_traits::value_to_yaml(msg.f, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendJointState_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_generator_traits
{

[[deprecated("use dobot_msgs_v4::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dobot_msgs_v4::srv::SendJointState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::SendJointState_Response & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::SendJointState_Response>()
{
  return "dobot_msgs_v4::srv::SendJointState_Response";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SendJointState_Response>()
{
  return "dobot_msgs_v4/srv/SendJointState_Response";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SendJointState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SendJointState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dobot_msgs_v4::srv::SendJointState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dobot_msgs_v4::srv::SendJointState>()
{
  return "dobot_msgs_v4::srv::SendJointState";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SendJointState>()
{
  return "dobot_msgs_v4/srv/SendJointState";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SendJointState>
  : std::integral_constant<
    bool,
    has_fixed_size<dobot_msgs_v4::srv::SendJointState_Request>::value &&
    has_fixed_size<dobot_msgs_v4::srv::SendJointState_Response>::value
  >
{
};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SendJointState>
  : std::integral_constant<
    bool,
    has_bounded_size<dobot_msgs_v4::srv::SendJointState_Request>::value &&
    has_bounded_size<dobot_msgs_v4::srv::SendJointState_Response>::value
  >
{
};

template<>
struct is_service<dobot_msgs_v4::srv::SendJointState>
  : std::true_type
{
};

template<>
struct is_service_request<dobot_msgs_v4::srv::SendJointState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dobot_msgs_v4::srv::SendJointState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SEND_JOINT_STATE__TRAITS_HPP_
