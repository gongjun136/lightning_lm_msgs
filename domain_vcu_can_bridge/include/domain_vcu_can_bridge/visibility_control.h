#ifndef DOMAIN_VCU_CAN_BRIDGE__VISIBILITY_CONTROL_H_
#define DOMAIN_VCU_CAN_BRIDGE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DOMAIN_VCU_CAN_BRIDGE_EXPORT __attribute__ ((dllexport))
    #define DOMAIN_VCU_CAN_BRIDGE_IMPORT __attribute__ ((dllimport))
  #else
    #define DOMAIN_VCU_CAN_BRIDGE_EXPORT __declspec(dllexport)
    #define DOMAIN_VCU_CAN_BRIDGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DOMAIN_VCU_CAN_BRIDGE_BUILDING_LIBRARY
    #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC DOMAIN_VCU_CAN_BRIDGE_EXPORT
  #else
    #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC DOMAIN_VCU_CAN_BRIDGE_IMPORT
  #endif
  #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC_TYPE DOMAIN_VCU_CAN_BRIDGE_PUBLIC
  #define DOMAIN_VCU_CAN_BRIDGE_LOCAL
#else
  #define DOMAIN_VCU_CAN_BRIDGE_EXPORT __attribute__ ((visibility("default")))
  #define DOMAIN_VCU_CAN_BRIDGE_IMPORT
  #if __GNUC__ >= 4
    #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC __attribute__ ((visibility("default")))
    #define DOMAIN_VCU_CAN_BRIDGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC
    #define DOMAIN_VCU_CAN_BRIDGE_LOCAL
  #endif
  #define DOMAIN_VCU_CAN_BRIDGE_PUBLIC_TYPE
#endif
#endif  // DOMAIN_VCU_CAN_BRIDGE__VISIBILITY_CONTROL_H_
// Generated 18-Sep-2026 09:52:30
 