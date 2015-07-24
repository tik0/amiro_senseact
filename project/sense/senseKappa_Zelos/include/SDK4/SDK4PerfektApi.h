#ifndef SDK4_PERFEKT_CONTROL_H_
#define SDK4_PERFEKT_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4ControlTypes.h>

//Shared Interfaces


#ifdef __cplusplus
extern "C" {
#endif

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE PerfektSetCCDControl(HANDLE hCtrl,uint32_t uControl);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE PerfektGetCCDControl(HANDLE hCtrl,uint32_t* puControl);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE PerfektWriteCOM(HANDLE hCtrl,uint32_t uValue);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE PerfektReadCOM(HANDLE hCtrl,uint32_t* puvalue);


#ifdef __cplusplus
}
#endif



#endif
