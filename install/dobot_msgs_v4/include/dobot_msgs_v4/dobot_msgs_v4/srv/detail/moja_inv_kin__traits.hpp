// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dobot_msgs_v4:srv/MojaInvKin.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__TRAITS_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dobot_msgs_v4/srv/detail/moja_inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const MojaInvKin_Request & msg,
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

  // member: rx
  {
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << ", ";
  }

  // member: ry
  {
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
    out << ", ";
  }

  // member: rz
  {
    out << "rz: ";
    rosidl_generator_traits::value_to_yaml(msg.rz, out);
    out << ", ";
  }

  // member: use_joint_near
  {
    out << "use_joint_near: ";
    rosidl_generator_traits::value_to_yaml(msg.use_joint_near, out);
    out << ", ";
  }

  // member: joint_near
  {
    out << "joint_near: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_near, out);
    out << ", ";
  }

  // member: user
  {
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
    out << ", ";
  }

  // member: tool
  {
    out << "tool: ";
    rosidl_generator_traits::value_to_yaml(msg.tool, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MojaInvKin_Request & msg,
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

  // member: rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << "\n";
  }

  // member: ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
    out << "\n";
  }

  // member: rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rz: ";
    rosidl_generator_traits::value_to_yaml(msg.rz, out);
    out << "\n";
  }

  // member: use_joint_near
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_joint_near: ";
    rosidl_generator_traits::value_to_yaml(msg.use_joint_near, out);
    out << "\n";
  }

  // member: joint_near
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_near: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_near, out);
    out << "\n";
  }

  // member: user
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
    out << "\n";
  }

  // member: tool
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool: ";
    rosidl_generator_traits::value_to_yaml(msg.tool, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MojaInvKin_Request & msg, bool use_flow_style = false)
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
  const dobot_msgs_v4::srv::MojaInvKin_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::MojaInvKin_Request & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::MojaInvKin_Request>()
{
  return "dobot_msgs_v4::srv::MojaInvKin_Request";
}

template<>
inline const char * name<dobot_msgs_v4::srv::MojaInvKin_Request>()
{
  return "dobot_msgs_v4/srv/MojaInvKin_Request";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::MojaInvKin_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::MojaInvKin_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dobot_msgs_v4::srv::MojaInvKin_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dobot_msgs_v4
{

namespace srv
{

inline void to_flow_style_yaml(
  const MojaInvKin_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: res
  {
    out << "res: ";
    rosidl_generator_traits::value_to_yaml(msg.res, out);
    out << ", ";
  }

  // member: joints
  {
    if (msg.joints.size() == 0) {
      out << "joints: []";
    } else {
      out << "joints: [";
      size_t pending_items = msg.joints.size();
      for (auto item : msg.joints) {
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
  const MojaInvKin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: res
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res: ";
    rosidl_generator_traits::value_to_yaml(msg.res, out);
    out << "\n";
  }

  // member: joints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints.size() == 0) {
      out << "joints: []\n";
    } else {
      out << "joints:\n";
      for (auto item : msg.joints) {
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

inline std::string to_yaml(const MojaInvKin_Response & msg, bool use_flow_style = false)
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
  const dobot_msgs_v4::srv::MojaInvKin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dobot_msgs_v4::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dobot_msgs_v4::srv::to_yaml() instead")]]
inline std::string to_yaml(const dobot_msgs_v4::srv::MojaInvKin_Response & msg)
{
  return dobot_msgs_v4::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dobot_msgs_v4::srv::MojaInvKin_Response>()
{
  return "dobot_msgs_v4::srv::MojaInvKin_Response";
}

template<>
inline const char * name<dobot_msgs_v4::srv::MojaInvKin_Response>()
{
  return "dobot_msgs_v4/srv/MojaInvKin_Response";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::MojaInvKin_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::MojaInvKin_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dobot_msgs_v4::srv::MojaInvKin_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dobot_msgs_v4::srv::MojaInvKin>()
{
  return "dobot_msgs_v4::srv::MojaInvKin";
}

template<>
inline const char * name<dobot_msgs_v4::srv::MojaInvKin>()
{
  return "dobot_msgs_v4/srv/MojaInvKin";
}

template<>
struct has_fixed_size<dobot_msgs_v4::srv::MojaInvKin>
  : std::integral_constant<
    bool,
    has_fixed_size<dobot_msgs_v4::srv::MojaInvKin_Request>::value &&
    has_fixed_size<dobot_msgs_v4::srv::MojaInvKin_Response>::value
  >
{
};

template<>
struct has_bounded_size<dobot_msgs_v4::srv::MojaInvKin>
  : std::integral_constant<
    bool,
    has_bounded_size<dobot_msgs_v4::srv::MojaInvKin_Request>::value &&
    has_bounded_size<dobot_msgs_v4::srv::MojaInvKin_Response>::value
  >
{
};

template<>
struct is_service<dobot_msgs_v4::srv::MojaInvKin>
  : std::true_type
{
};

template<>
struct is_service_request<dobot_msgs_v4::srv::MojaInvKin_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dobot_msgs_v4::srv::MojaInvKin_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__TRAITS_HPP_
