#ifndef PLATFORMTYPES_H_
#define PLATFORMTYPES_H_

//// Platform ------------------------------------------------------------
#if defined(_WIN32) || defined(WIN32)
  #include <windows.h>
#endif

//// Types ---------------------------------------------------------------
#if defined(_WIN32) || defined(WIN32)

  #include <stdint.h>

  typedef unsigned char uchar8_t;

  #ifndef _SIZE_T_DEFINED
    #ifdef  _WIN64
      typedef unsigned __int64 size_t;
    #else
      typedef uint32_t size_t;
    #endif
    #define _SIZE_T_DEFINED
  #endif

  #ifndef _INTPTR_T_DEFINED
    #ifdef  _WIN64
      typedef __int64 intptr_t;
    #else
      typedef int intptr_t;
    #endif
    #define _INTPTR_T_DEFINED
  #endif
#else // Linux
    typedef unsigned char uchar8_t;
#   define  __STDC_LIMIT_MACROS
#   define  __STDC_CONSTANT_MACROS
#   include <stddef.h>
#   include <inttypes.h>
#   include <stdint.h>
#endif

//UtilityTypes
#if defined(__cplusplus)
  typedef bool              bool8_t;
#else
  typedef uint8_t           bool8_t;
#endif
  typedef const char *      pcchar8_t;
  typedef char *            pchar8_t;
  typedef float             float32_t;
  typedef double            float64_t;

#endif
