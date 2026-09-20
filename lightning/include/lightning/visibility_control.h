#ifndef LIGHTNING__VISIBILITY_CONTROL_H_
#define LIGHTNING__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIGHTNING_EXPORT __attribute__ ((dllexport))
    #define LIGHTNING_IMPORT __attribute__ ((dllimport))
  #else
    #define LIGHTNING_EXPORT __declspec(dllexport)
    #define LIGHTNING_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIGHTNING_BUILDING_LIBRARY
    #define LIGHTNING_PUBLIC LIGHTNING_EXPORT
  #else
    #define LIGHTNING_PUBLIC LIGHTNING_IMPORT
  #endif
  #define LIGHTNING_PUBLIC_TYPE LIGHTNING_PUBLIC
  #define LIGHTNING_LOCAL
#else
  #define LIGHTNING_EXPORT __attribute__ ((visibility("default")))
  #define LIGHTNING_IMPORT
  #if __GNUC__ >= 4
    #define LIGHTNING_PUBLIC __attribute__ ((visibility("default")))
    #define LIGHTNING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIGHTNING_PUBLIC
    #define LIGHTNING_LOCAL
  #endif
  #define LIGHTNING_PUBLIC_TYPE
#endif
#endif  // LIGHTNING__VISIBILITY_CONTROL_H_
// Generated 18-Sep-2026 09:52:30
 