// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dobot_msgs_v4:srv/SetStaticTF.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__TRAITS_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dobot_msgs_v4/srv/detail/set_static_tf__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetStaticTF_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: parent_frame
  {
    out << "parent_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_frame, out);
    out << ", ";
  }

  // member: child_frame
  {
    out << "child_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.child_frame, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetStaticTF_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: parent_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_frame, out);
    out << "\n";
  }

  // member: child_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "child_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.child_frame, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetStaticTF_Request & msg, bool use_flow_style = false)
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
  const dobot_msgs_v4::srv::SetStaticTF_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::SetStaticTF_Request & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::SetStaticTF_Request>()
{
  return "dobot_msgs_v4::srv::SetStaticTF_Request";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SetStaticTF_Request>()
{
  return "dobot_msgs_v4/srv/SetStaticTF_Request";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SetStaticTF_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SetStaticTF_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dobot_msgs_v4::srv::SetStaticTF_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetStaticTF_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetStaticTF_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetStaticTF_Response & msg, bool use_flow_style = false)
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
  const dobot_msgs_v4::srv::SetStaticTF_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::SetStaticTF_Response & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::SetStaticTF_Response>()
{
  return "dobot_msgs_v4::srv::SetStaticTF_Response";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SetStaticTF_Response>()
{
  return "dobot_msgs_v4/srv/SetStaticTF_Response";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SetStaticTF_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SetStaticTF_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dobot_msgs_v4::srv::SetStaticTF_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dobot_msgs_v4::srv::SetStaticTF>()
{
  return "dobot_msgs_v4::srv::SetStaticTF";
}

template<>
inline const char * name<dobot_msgs_v4::srv::SetStaticTF>()
{
  return "dobot_msgs_v4/srv/SetStaticTF";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::SetStaticTF>
  : std::integral_constant<
    bool,
    has_fixed_size<dobot_msgs_v4::srv::SetStaticTF_Request>::value &&
    has_fixed_size<dobot_msgs_v4::srv::SetStaticTF_Response>::value
  >
{
};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::SetStaticTF>
  : std::integral_constant<
    bool,
    has_bounded_size<dobot_msgs_v4::srv::SetStaticTF_Request>::value &&
    has_bounded_size<dobot_msgs_v4::srv::SetStaticTF_Response>::value
  >
{
};

template<>
struct is_service<dobot_msgs_v4::srv::SetStaticTF>
  : std::true_type
{
};

template<>
struct is_service_request<dobot_msgs_v4::srv::SetStaticTF_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dobot_msgs_v4::srv::SetStaticTF_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_STATIC_TF__TRAITS_HPP_
