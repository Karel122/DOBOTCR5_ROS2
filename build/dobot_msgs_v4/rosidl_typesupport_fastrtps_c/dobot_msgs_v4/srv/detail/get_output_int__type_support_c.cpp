// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dobot_msgs_v4:srv/GetOutputInt.idl
// generated code does not contain a copyright notice
#include "dobot_msgs_v4/srv/detail/get_output_int__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dobot_msgs_v4/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dobot_msgs_v4/srv/detail/get_output_int__struct.h"
#include "dobot_msgs_v4/srv/detail/get_output_int__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _GetOutputInt_Request__ros_msg_type = dobot_msgs_v4__srv__GetOutputInt_Request;

static bool _GetOutputInt_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetOutputInt_Request__ros_msg_type * ros_message = static_cast<const _GetOutputInt_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: address
  {
    cdr << ros_message->address;
  }

  return true;
}

static bool _GetOutputInt_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetOutputInt_Request__ros_msg_type * ros_message = static_cast<_GetOutputInt_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: address
  {
    cdr >> ros_message->address;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t get_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetOutputInt_Request__ros_msg_type * ros_message = static_cast<const _GetOutputInt_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name address
  {
    size_t item_size = sizeof(ros_message->address);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetOutputInt_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t max_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: address
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dobot_msgs_v4__srv__GetOutputInt_Request;
    is_plain =
      (
      offsetof(DataType, address) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetOutputInt_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetOutputInt_Request = {
  "dobot_msgs_v4::srv",
  "GetOutputInt_Request",
  _GetOutputInt_Request__cdr_serialize,
  _GetOutputInt_Request__cdr_deserialize,
  _GetOutputInt_Request__get_serialized_size,
  _GetOutputInt_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetOutputInt_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetOutputInt_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, GetOutputInt_Request)() {
  return &_GetOutputInt_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "dobot_msgs_v4/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_output_int__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_output_int__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // robot_return
#include "rosidl_runtime_c/string_functions.h"  // robot_return

// forward declare type support functions


using _GetOutputInt_Response__ros_msg_type = dobot_msgs_v4__srv__GetOutputInt_Response;

static bool _GetOutputInt_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetOutputInt_Response__ros_msg_type * ros_message = static_cast<const _GetOutputInt_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_return
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_return;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: res
  {
    cdr << ros_message->res;
  }

  return true;
}

static bool _GetOutputInt_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetOutputInt_Response__ros_msg_type * ros_message = static_cast<_GetOutputInt_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_return
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_return.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_return);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_return,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_return'\n");
      return false;
    }
  }

  // Field name: res
  {
    cdr >> ros_message->res;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t get_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetOutputInt_Response__ros_msg_type * ros_message = static_cast<const _GetOutputInt_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_return
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_return.size + 1);
  // field.name res
  {
    size_t item_size = sizeof(ros_message->res);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetOutputInt_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t max_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: robot_return
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: res
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dobot_msgs_v4__srv__GetOutputInt_Response;
    is_plain =
      (
      offsetof(DataType, res) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetOutputInt_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dobot_msgs_v4__srv__GetOutputInt_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetOutputInt_Response = {
  "dobot_msgs_v4::srv",
  "GetOutputInt_Response",
  _GetOutputInt_Response__cdr_serialize,
  _GetOutputInt_Response__cdr_deserialize,
  _GetOutputInt_Response__get_serialized_size,
  _GetOutputInt_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetOutputInt_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetOutputInt_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, GetOutputInt_Response)() {
  return &_GetOutputInt_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "dobot_msgs_v4/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dobot_msgs_v4/srv/get_output_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetOutputInt__callbacks = {
  "dobot_msgs_v4::srv",
  "GetOutputInt",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, GetOutputInt_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, GetOutputInt_Response)(),
};

static rosidl_service_type_support_t GetOutputInt__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetOutputInt__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, GetOutputInt)() {
  return &GetOutputInt__handle;
}

#if defined(__cplusplus)
}
#endif
