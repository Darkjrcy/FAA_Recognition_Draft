// generated from
// rosidl_typesupport_c/resource/rosidl_typesupport_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef GNSS_MULTIPATH_PLUGIN__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
#define GNSS_MULTIPATH_PLUGIN__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_C_EXPORT_gnss_multipath_plugin __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_C_IMPORT_gnss_multipath_plugin __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_C_EXPORT_gnss_multipath_plugin __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_C_IMPORT_gnss_multipath_plugin __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_C_BUILDING_DLL_gnss_multipath_plugin
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_gnss_multipath_plugin ROSIDL_TYPESUPPORT_C_EXPORT_gnss_multipath_plugin
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_gnss_multipath_plugin ROSIDL_TYPESUPPORT_C_IMPORT_gnss_multipath_plugin
  #endif
#else
  #define ROSIDL_TYPESUPPORT_C_EXPORT_gnss_multipath_plugin __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_C_IMPORT_gnss_multipath_plugin
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_gnss_multipath_plugin __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_gnss_multipath_plugin
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // GNSS_MULTIPATH_PLUGIN__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
