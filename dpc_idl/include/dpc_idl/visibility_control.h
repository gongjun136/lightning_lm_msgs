#ifndef DPC_IDL__VISIBILITY_CONTROL_H_
#define DPC_IDL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DPC_IDL_EXPORT __attribute__ ((dllexport))
    #define DPC_IDL_IMPORT __attribute__ ((dllimport))
  #else
    #define DPC_IDL_EXPORT __declspec(dllexport)
    #define DPC_IDL_IMPORT __declspec(dllimport)
  #endif
  #ifdef DPC_IDL_BUILDING_LIBRARY
    #define DPC_IDL_PUBLIC DPC_IDL_EXPORT
  #else
    #define DPC_IDL_PUBLIC DPC_IDL_IMPORT
  #endif
  #define DPC_IDL_PUBLIC_TYPE DPC_IDL_PUBLIC
  #define DPC_IDL_LOCAL
#else
  #define DPC_IDL_EXPORT __attribute__ ((visibility("default")))
  #define DPC_IDL_IMPORT
  #if __GNUC__ >= 4
    #define DPC_IDL_PUBLIC __attribute__ ((visibility("default")))
    #define DPC_IDL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DPC_IDL_PUBLIC
    #define DPC_IDL_LOCAL
  #endif
  #define DPC_IDL_PUBLIC_TYPE
#endif
#endif  // DPC_IDL__VISIBILITY_CONTROL_H_
// Generated 18-Sep-2026 09:52:30
 