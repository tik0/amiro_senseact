
#ifndef SDK4_TIMING_CONTROL_H_
#define SDK4_TIMING_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

  //ITiming
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetPixelClock(CTRL_HANDLE hCtrl,uint32_t* puPixelClock);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetReadoutTime(CTRL_HANDLE hCtrl,uint32_t* puReadoutTime);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetReadoutTime(CTRL_HANDLE hCtrl,uint32_t uReadoutTime);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetFrameDuration(CTRL_HANDLE hCtrl,uint32_t* puFrameDuration);

#ifdef __cplusplus
}
#endif

#endif
