// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dobot_msgs_v4:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
#define DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vrednosti'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PointArray in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__msg__PointArray
{
  rosidl_runtime_c__double__Sequence vrednosti;
} dobot_msgs_v4__msg__PointArray;

// Struct for a sequence of dobot_msgs_v4__msg__PointArray.
typedef struct dobot_msgs_v4__msg__PointArray__Sequence
{
  dobot_msgs_v4__msg__PointArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__msg__PointArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
