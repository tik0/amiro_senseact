
#ifndef SDK4_MEMORY_CONTROL_H_
#define SDK4_MEMORY_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>
#include <SDK4/SDK4FDC10Types.h>

#ifdef __cplusplus
extern "C" {
#endif

	//IMemory
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetVerticalFlip(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetVerticalFlip(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetHorizontalFlip(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetHorizontalFlip(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetLastImageHold(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetLastImageHold(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetImageSubtraction(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetImageSubtraction(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4StoreImage(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetRecursiveFilter(CTRL_HANDLE hCtrl,SDK4_ENUM_RECURSIVEFILTER uFactor);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetRecursiveFilter(CTRL_HANDLE hCtrl,SDK4_ENUM_RECURSIVEFILTER* puFactor);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetAutoMotionDetection(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetAutoMotionDetection(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);
 
#ifdef __cplusplus
}
#endif

#endif
