
#ifndef SDK4_GAIN_CONTROL_H_
#define SDK4_GAIN_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

	//IGain
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetGain(CTRL_HANDLE hCtrl,uint32_t uGain);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetGain(CTRL_HANDLE hCtrl,uint32_t* puGain);
  
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetAGC(CTRL_HANDLE hCtrl,SDK4_ENUM_AGC iSelect);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetAGC(CTRL_HANDLE hCtrl,SDK4_ENUM_AGC* piSelect);

#ifdef __cplusplus
}
#endif

#endif
