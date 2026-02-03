// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dobot_msgs_v4/msg/detail/point_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dobot_msgs_v4
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PointArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::msg::PointArray(_init);
}

void PointArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::msg::PointArray *>(message_memory);
  typed_message->~PointArray();
}

size_t size_function__PointArray__vrednosti(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PointArray__vrednosti(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PointArray__vrednosti(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PointArray__vrednosti(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PointArray__vrednosti(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PointArray__vrednosti(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PointArray__vrednosti(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PointArray__vrednosti(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PointArray_message_member_array[1] = {
  {
    "vrednosti",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::msg::PointArray, vrednosti),  // bytes offset in struct
    nullptr,  // default value
    size_function__PointArray__vrednosti,  // size() function pointer
    get_const_function__PointArray__vrednosti,  // get_const(index) function pointer
    get_function__PointArray__vrednosti,  // get(index) function pointer
    fetch_function__PointArray__vrednosti,  // fetch(index, &value) function pointer
    assign_function__PointArray__vrednosti,  // assign(index, value) function pointer
    resize_function__PointArray__vrednosti  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PointArray_message_members = {
  "dobot_msgs_v4::msg",  // message namespace
  "PointArray",  // message name
  1,  // number of fields
  sizeof(dobot_msgs_v4::msg::PointArray),
  PointArray_message_member_array,  // message members
  PointArray_init_function,  // function to initialize message memory (memory has to be allocated)
  PointArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PointArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PointArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::msg::PointArray>()
{
  return &::dobot_msgs_v4::msg::rosidl_typesupport_introspection_cpp::PointArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, msg, PointArray)() {
  return &::dobot_msgs_v4::msg::rosidl_typesupport_introspection_cpp::PointArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
