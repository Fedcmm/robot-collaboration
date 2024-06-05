#ifndef JOIN_PLUGIN__VISIBILITY_CONTROL_H_
#define JOIN_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JOIN_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define JOIN_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define JOIN_PLUGIN_EXPORT __declspec(dllexport)
    #define JOIN_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef JOIN_PLUGIN_BUILDING_LIBRARY
    #define JOIN_PLUGIN_PUBLIC JOIN_PLUGIN_EXPORT
  #else
    #define JOIN_PLUGIN_PUBLIC JOIN_PLUGIN_IMPORT
  #endif
  #define JOIN_PLUGIN_PUBLIC_TYPE JOIN_PLUGIN_PUBLIC
  #define JOIN_PLUGIN_LOCAL
#else
  #define JOIN_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define JOIN_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define JOIN_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define JOIN_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JOIN_PLUGIN_PUBLIC
    #define JOIN_PLUGIN_LOCAL
  #endif
  #define JOIN_PLUGIN_PUBLIC_TYPE
#endif

#endif  // JOIN_PLUGIN__VISIBILITY_CONTROL_H_
