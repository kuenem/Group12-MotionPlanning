// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_dashboard_msgs:srv/GetProgramState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ur_dashboard_msgs/srv/get_program_state.h"


#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__STRUCT_H_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetProgramState in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Request
{
  uint8_t structure_needs_at_least_one_member;
} ur_dashboard_msgs__srv__GetProgramState_Request;

// Struct for a sequence of ur_dashboard_msgs__srv__GetProgramState_Request.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Request__Sequence
{
  ur_dashboard_msgs__srv__GetProgramState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__GetProgramState_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "ur_dashboard_msgs/msg/detail/program_state__struct.h"
// Member 'program_name'
// Member 'answer'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetProgramState in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Response
{
  ur_dashboard_msgs__msg__ProgramState state;
  rosidl_runtime_c__String program_name;
  rosidl_runtime_c__String answer;
  bool success;
} ur_dashboard_msgs__srv__GetProgramState_Response;

// Struct for a sequence of ur_dashboard_msgs__srv__GetProgramState_Response.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Response__Sequence
{
  ur_dashboard_msgs__srv__GetProgramState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__GetProgramState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  ur_dashboard_msgs__srv__GetProgramState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  ur_dashboard_msgs__srv__GetProgramState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetProgramState in the package ur_dashboard_msgs.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  ur_dashboard_msgs__srv__GetProgramState_Request__Sequence request;
  ur_dashboard_msgs__srv__GetProgramState_Response__Sequence response;
} ur_dashboard_msgs__srv__GetProgramState_Event;

// Struct for a sequence of ur_dashboard_msgs__srv__GetProgramState_Event.
typedef struct ur_dashboard_msgs__srv__GetProgramState_Event__Sequence
{
  ur_dashboard_msgs__srv__GetProgramState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_dashboard_msgs__srv__GetProgramState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__GET_PROGRAM_STATE__STRUCT_H_
