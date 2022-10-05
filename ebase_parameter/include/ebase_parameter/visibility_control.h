#ifndef EBASE_PARAMETER__VISIBILITY_CONTROL_H_
#define EBASE_PARAMETER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EBASE_PARAMETER_EXPORT __attribute__ ((dllexport))
    #define EBASE_PARAMETER_IMPORT __attribute__ ((dllimport))
  #else
    #define EBASE_PARAMETER_EXPORT __declspec(dllexport)
    #define EBASE_PARAMETER_IMPORT __declspec(dllimport)
  #endif
  #ifdef EBASE_PARAMETER_BUILDING_LIBRARY
    #define EBASE_PARAMETER_PUBLIC EBASE_PARAMETER_EXPORT
  #else
    #define EBASE_PARAMETER_PUBLIC EBASE_PARAMETER_IMPORT
  #endif
  #define EBASE_PARAMETER_PUBLIC_TYPE EBASE_PARAMETER_PUBLIC
  #define EBASE_PARAMETER_LOCAL
#else
  #define EBASE_PARAMETER_EXPORT __attribute__ ((visibility("default")))
  #define EBASE_PARAMETER_IMPORT
  #if __GNUC__ >= 4
    #define EBASE_PARAMETER_PUBLIC __attribute__ ((visibility("default")))
    #define EBASE_PARAMETER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EBASE_PARAMETER_PUBLIC
    #define EBASE_PARAMETER_LOCAL
  #endif
  #define EBASE_PARAMETER_PUBLIC_TYPE
#endif

#endif  // EBASE_PARAMETER__VISIBILITY_CONTROL_H_
