// generated from
// rosidl_typesupport_introspection_c/resource/rosidl_typesupport_introspection_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
#define AUTO_AIM__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_auto_aim __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_auto_aim __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_INTROSPECTION_C_BUILDING_DLL_auto_aim
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_auto_aim ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_auto_aim ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_auto_aim
  #endif
#else
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_IMPORT_auto_aim
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_auto_aim __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC_auto_aim
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM__MSG__ROSIDL_TYPESUPPORT_INTROSPECTION_C__VISIBILITY_CONTROL_H_
