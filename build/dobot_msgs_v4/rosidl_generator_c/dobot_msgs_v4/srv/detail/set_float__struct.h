// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dobot_msgs_v4:srv/SetFloat.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_H_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetFloat in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__SetFloat_Request
{
  uint8_t structure_needs_at_least_one_member;
} dobot_msgs_v4__srv__SetFloat_Request;

// Struct for a sequence of dobot_msgs_v4__srv__SetFloat_Request.
typedef struct dobot_msgs_v4__srv__SetFloat_Request__Sequence
{
  dobot_msgs_v4__srv__SetFloat_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__SetFloat_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "dobot_msgs_v4/msg/detail/point_array__struct.h"

/// Struct defined in srv/SetFloat in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__SetFloat_Response
{
  dobot_msgs_v4__msg__PointArray__Sequence data;
} dobot_msgs_v4__srv__SetFloat_Response;

// Struct for a sequence of dobot_msgs_v4__srv__SetFloat_Response.
typedef struct dobot_msgs_v4__srv__SetFloat_Response__Sequence
{
  dobot_msgs_v4__srv__SetFloat_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__SetFloat_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_FLOAT__STRUCT_H_
