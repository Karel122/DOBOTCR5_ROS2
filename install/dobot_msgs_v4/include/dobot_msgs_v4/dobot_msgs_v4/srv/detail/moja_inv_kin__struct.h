// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dobot_msgs_v4:srv/MojaInvKin.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__STRUCT_H_
#define DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'use_joint_near'
// Member 'joint_near'
// Member 'user'
// Member 'tool'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MojaInvKin in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__MojaInvKin_Request
{
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
  rosidl_runtime_c__String use_joint_near;
  rosidl_runtime_c__String joint_near;
  rosidl_runtime_c__String user;
  rosidl_runtime_c__String tool;
} dobot_msgs_v4__srv__MojaInvKin_Request;

// Struct for a sequence of dobot_msgs_v4__srv__MojaInvKin_Request.
typedef struct dobot_msgs_v4__srv__MojaInvKin_Request__Sequence
{
  dobot_msgs_v4__srv__MojaInvKin_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__MojaInvKin_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'joints'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MojaInvKin in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__MojaInvKin_Response
{
  int32_t res;
  rosidl_runtime_c__double__Sequence joints;
} dobot_msgs_v4__srv__MojaInvKin_Response;

// Struct for a sequence of dobot_msgs_v4__srv__MojaInvKin_Response.
typedef struct dobot_msgs_v4__srv__MojaInvKin_Response__Sequence
{
  dobot_msgs_v4__srv__MojaInvKin_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__MojaInvKin_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__MOJA_INV_KIN__STRUCT_H_
