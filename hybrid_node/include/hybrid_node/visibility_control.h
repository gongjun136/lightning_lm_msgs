#ifndef HYBRID_NODE__VISIBILITY_CONTROL_H_
#define HYBRID_NODE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HYBRID_NODE_EXPORT __attribute__ ((dllexport))
    #define HYBRID_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define HYBRID_NODE_EXPORT __declspec(dllexport)
    #define HYBRID_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef HYBRID_NODE_BUILDING_LIBRARY
    #define HYBRID_NODE_PUBLIC HYBRID_NODE_EXPORT
  #else
    #define HYBRID_NODE_PUBLIC HYBRID_NODE_IMPORT
  #endif
  #define HYBRID_NODE_PUBLIC_TYPE HYBRID_NODE_PUBLIC
  #define HYBRID_NODE_LOCAL
#else
  #define HYBRID_NODE_EXPORT __attribute__ ((visibility("default")))
  #define HYBRID_NODE_IMPORT
  #if __GNUC__ >= 4
    #define HYBRID_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define HYBRID_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HYBRID_NODE_PUBLIC
    #define HYBRID_NODE_LOCAL
  #endif
  #define HYBRID_NODE_PUBLIC_TYPE
#endif
#endif  // HYBRID_NODE__VISIBILITY_CONTROL_H_
// Generated 04-Feb-2026 16:03:24
 