// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dobot_msgs_v4/msg/detail/point_array__rosidl_typesupport_introspection_c.h"
#include "dobot_msgs_v4/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dobot_msgs_v4/msg/detail/point_array__functions.h"
#include "dobot_msgs_v4/msg/detail/point_array__struct.h"


// Include directives for member types
// Member `vrednosti`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dobot_msgs_v4__msg__PointArray__init(message_memory);
}

void dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_fini_function(void * message_memory)
{
  dobot_msgs_v4__msg__PointArray__fini(message_memory);
}

size_t dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__size_function__PointArray__vrednosti(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_const_function__PointArray__vrednosti(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_function__PointArray__vrednosti(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__fetch_function__PointArray__vrednosti(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_const_function__PointArray__vrednosti(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__assign_function__PointArray__vrednosti(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_function__PointArray__vrednosti(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__resize_function__PointArray__vrednosti(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_member_array[1] = {
  {
    "vrednosti",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__msg__PointArray, vrednosti),  // bytes offset in struct
    NULL,  // default value
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__size_function__PointArray__vrednosti,  // size() function pointer
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_const_function__PointArray__vrednosti,  // get_const(index) function pointer
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__get_function__PointArray__vrednosti,  // get(index) function pointer
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__fetch_function__PointArray__vrednosti,  // fetch(index, &value) function pointer
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__assign_function__PointArray__vrednosti,  // assign(index, value) function pointer
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__resize_function__PointArray__vrednosti  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_members = {
  "dobot_msgs_v4__msg",  // message namespace
  "PointArray",  // message name
  1,  // number of fields
  sizeof(dobot_msgs_v4__msg__PointArray),
  dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_member_array,  // message members
  dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_init_function,  // function to initialize message memory (memory has to be allocated)
  dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_type_support_handle = {
  0,
  &dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dobot_msgs_v4
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, msg, PointArray)() {
  if (!dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_type_support_handle.typesupport_identifier) {
    dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dobot_msgs_v4__msg__PointArray__rosidl_typesupport_introspection_c__PointArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
