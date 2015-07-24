
#ifndef SDK4_GUFI_CONTROL_H_
#define SDK4_GUFI_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif
  //IGufi
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetDeviceName(CTRL_HANDLE hCtrl,char * psDeviceName, int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetSerialNumber(CTRL_HANDLE hCtrl,char * psSerialNumber, int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetFunctionModelID(CTRL_HANDLE hCtrl,uint32_t * puModelID);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetFunctionModelVersion(CTRL_HANDLE hCtrl,SDK4_KVERSION * pkVersion);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetRevision(CTRL_HANDLE hCtrl,SDK4_KREVISION * pkRevision);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetSoftwareBuild(CTRL_HANDLE hCtrl,char * psSoftwareBuild, int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetSoftwareVersion(CTRL_HANDLE hCtrl,SDK4_KVERSION * pkVersion);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetHardwareVersion(CTRL_HANDLE hCtrl,SDK4_KVERSION * pkVersion);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetFirmwareVersion(CTRL_HANDLE hCtrl,SDK4_KVERSION * pkVersion);

#ifdef __cplusplus
}
#endif

#endif
